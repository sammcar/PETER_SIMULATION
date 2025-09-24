
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2
import time
import serial
import threading
import torch
import torch.nn as nn
import joblib

# === Parámetros de estancamiento ===
delta_x_min = 0.8
tiempo_umbral = 1.0
ultima_pos_x = None
ultimo_cambio_tiempo = time.time()
estancado = 0
position = 0.0
mlp_output = np.zeros(2)
ganglios = np.zeros(15)
response = np.zeros(3)

count = 0
C = 0; M = 0; Y = 0.0
K = 0; W = 0
mode = 'k'
past = 'k'
# Comunicación serial
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
running = True
capturing = False
cam_ok = False
buffer_lines = []
ser = None
ventana = []
mapa_ = {1: 1, 2: 2, 3: 5, 4: 3, 5: 0, 6: 6, 7: 4}

class MLP(nn.Module):
    def __init__(self, input_size, num_classes):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 200),
            nn.ReLU(),
            nn.Linear(200, 100),
            nn.ReLU(),
            nn.Linear(100, num_classes)
        )
    def forward(self, x): return self.net(x)

model = MLP(input_size=135, num_classes=2)
model.load_state_dict(torch.load("MLP_final_final/modelo_mlp.pth"))
model.eval()
scaler = joblib.load("MLP_final_final/escalador.pkl")

def verificar_estancamiento(pos_actual_x):
    global ultima_pos_x, ultimo_cambio_tiempo, estancado
    ahora = time.time()
    if ultima_pos_x is None:
        ultima_pos_x = pos_actual_x
        ultimo_cambio_tiempo = ahora
        return 0
    delta_x = abs(pos_actual_x - ultima_pos_x)
    if delta_x > delta_x_min:
        ultima_pos_x = pos_actual_x
        ultimo_cambio_tiempo = ahora
        estancado = 0
    else:
        if ahora - ultimo_cambio_tiempo >= tiempo_umbral:
            estancado = 1*cam_ok
        else:
            estancado = 0
    return estancado

def preprocess_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([110, 50, 40])
    upper = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    filtered = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    processed = cv2.dilate(binary, kernel, iterations=1)
    image = cv2.resize(processed, (500, 375))
    return image

def camera_thread():
    global position, count, cam_ok
    cap = cv2.VideoCapture(2)
    owo = np.tile(np.arange(500), (375, 1))
    while running:
        ret, frame = cap.read()
        if not ret:
            print("Error: no se pudo capturar frame.")
            break
        frame_bin = preprocess_frame(frame)
        position = np.mean(np.multiply(frame_bin, owo)) / 375
        if np.count_nonzero(frame_bin) > 3000:
            cam_ok = True
        else:
            cam_ok = False
        # if count%30 == 0:
        #     print(np.count_nonzero(frame_bin))
        # count += 1
        #verificar_estancamiento(position)
        cv2.imshow("Campo Visual", frame_bin)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def test_mlp(linea_test):
    global mlp_output
    ventana.append(linea_test)
    if len(ventana) == 15:
        entrada_mlp = np.array(ventana).flatten()
        ventana.clear()
        entrada_escalada = scaler.transform([entrada_mlp])
        X_tensor = torch.tensor(entrada_escalada, dtype=torch.float32)
        with torch.no_grad():
            logits = model(X_tensor)
            probs = torch.softmax(logits, dim=1)[0].numpy()
            mlp_output = np.array(probs)
            #print(f"[MLP] Probabilidades: Plano={probs[0]:.2f}, Rampa={probs[1]:.2f}")

def read_serial():
    global running, ser, capturing, buffer_lines, position
    while running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                if line == "--END--":
                    if capturing and buffer_lines:
                        try:
                            packet = np.array([list(map(float, l.split(','))) for l in buffer_lines])
                            for fila in packet:
                                ax, ay, az, gx, gy, gz, direccion, modo = fila
                                direccion_mapeada = mapa_.get(int(direccion), -1)
                                bits = format(direccion_mapeada, '03b')
                                bit0, bit1, bit2 = int(bits[0]), int(bits[1]), int(bits[2])
                                linea_test = [ax, ay, az, gx, gy, bit0, bit1, bit2, modo]
                                test_mlp(linea_test)
                                verificar_estancamiento(position)

                        except Exception as e:
                            print(f"[ERROR] Procesando paquete: {e}")
                    buffer_lines = []
                else:
                    if capturing:
                        buffer_lines.append(line)
        except Exception as e:
            print(f"[ERROR] Lectura serial: {e}")

def consola_serial():
    global capturing,past
    while running:
        cmd = input("[SERIAL] > ").strip()
        if cmd == 's':
            capturing = not capturing
            ser.write(b's\n')
            print(f"[INFO] Captura {'activada' if capturing else 'detenida'}")
        elif len(cmd) == 1:
            ser.write(cmd.encode())
            past = cmd
        elif cmd.lower() == 'exit':
            print("[INFO] Cerrando...")
            break

def resend_last():
    while True:
        ser.write(past.encode())
        time.sleep(0.5)



def arbitramiento(response, Ganglia):
    mult = 0.6
    weights_Gan = np.ones((3, 3)) - np.eye(3)
    condSTR = np.array([Y, C * M, W * K])
    TaoGPi = 5 * mult
    TaoGPe = 10 * mult
    TaoSubTN = 10 * mult
    TaoStR = 5 * mult
    GPi = np.zeros(3)
    GPe = np.zeros(3)
    SubTN = np.zeros(3)
    StR = np.zeros(6)
    for m in range(3):
        SubTN[m] = np.clip(Ganglia[m] + (1/TaoSubTN)*(-Ganglia[m] + response[m] - Ganglia[m+3] + np.dot(-weights_Gan[m, :], Ganglia[6:9])), 0, None)
        GPi[m] = np.clip(Ganglia[m+3] + (1/TaoGPi)*(-Ganglia[m+3] + np.dot(weights_Gan[m, :], Ganglia[0:3]) - Ganglia[m+6] - Ganglia[m+9] + 3*Ganglia[12]), 0, 1)
        GPe[m] = np.clip(Ganglia[m+6] + (1/TaoGPe)*(-Ganglia[m+6] + Ganglia[m]), 0, None)
        StR[m] = np.clip(Ganglia[m+9] + (1/TaoStR)*(-Ganglia[m+9] + Ganglia[m]), 0, None)
        StR[m+3] = np.clip(Ganglia[m+12] + (1/TaoStR)*(-Ganglia[m+12] + condSTR[m]), 0, None)
    return np.hstack([SubTN, GPi, GPe, StR])

def animacion_thread():
    global ganglios,past
    ganglia_labels = [
    "STN Irregular", "STN Inclinado", "STN Plano",
    "GPi Irregular", "GPi Inclinado", "GPi Plano",
    "GPe Irregular", "GPe Inclinado", "GPe Plano",
    "STR Irregular", "STR Inclinado", "STR Plano",
    "STR Yellow", "STR CyanMagenta", "STR WhiteBlack"
    ]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig2, ax2 = plt.subplots(figsize=(8, 4))
    axs[0, 0].set_title("SubTN Neuronas")
    axs[0, 1].set_title("GPi Neuronas")
    axs[1, 0].set_title("GPe Neuronas")
    axs[1, 1].set_title("StR Neuronas")
    

    axs[0, 0].set_ylabel("Hz")
    axs[1, 0].set_ylabel("Hz")
    axs[1, 0].set_xlabel("Tiempo (ms)")
    axs[1, 1].set_xlabel("Tiempo (ms)")

    ax2.set_title("Selección de Modo de Locomoción")
    ax2.set_xlabel("Tiempo (ms)")
    ax2.set_ylabel("Nivel de Activación")
    colores = ['b', 'r', 'g', 'y', 'c', 'k']
    #lines_SubTN, lines_GPi, lines_GPe, lines_StR = [], [], [], []
    lines_SubTN = [axs[0, 0].plot([], [], lw=2, color=colores[i], label=ganglia_labels[i])[0] for i in range(3)]
    lines_GPi = [axs[0, 1].plot([], [], lw=2, color=colores[i], label=ganglia_labels[i+3])[0] for i in range(3)]
    lines_GPe = [axs[1, 0].plot([], [], lw=2, color=colores[i], label=ganglia_labels[i+6])[0] for i in range(3)]
    lines_StR = [axs[1, 1].plot([], [], lw=2, color=colores[i], label=ganglia_labels[i+9])[0] for i in range(4)]
    axs[0, 0].legend(loc="upper right")
    axs[0, 1].legend(loc="upper right")
    axs[1, 0].legend(loc="upper right")
    axs[1, 1].legend(loc="upper right")
    # for i in range(3):
    #     lines_SubTN.append(axs[0, 0].plot([], [], lw=2, color=colores[i])[0])
    #     lines_GPi.append(axs[0, 1].plot([], [], lw=2, color=colores[i])[0])
    #     lines_GPe.append(axs[1, 0].plot([], [], lw=2, color=colores[i])[0])
    # for i in range(6):
    #     lines_StR.append(axs[1, 1].plot([], [], lw=2, color=colores[i%3])[0])
    line_quad, = ax2.plot([], [], lw=2, color='m')
    line_diff, = ax2.plot([], [], lw=2, color='orange')
    ax2.legend(["Modo Cuadrúpedo", "Modo Ruedas"])
    
    for ax in axs.flat:
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 1)
        ax.grid()
    ax2.set_xlim(0, 100)
    ax2.set_ylim(0, 2)
    ax2.grid()
    xdata = []
    ydata_SubTN, ydata_GPi, ydata_GPe = [[] for _ in range(3)], [[] for _ in range(3)], [[] for _ in range(3)]
    ydata_StR = [[] for _ in range(6)]
    ydata_quad, ydata_diff = [], []
    quad, diff, tau = 0, 0, 3#15
    bias = 1
    base_frame = 0

    def update(frame):
        global mlp_output, estancado, response, mode, past
        nonlocal quad, diff, base_frame, tau
        # if frame% 20 == 0:
        #     print(estancado,mlp_output[1],mlp_output[0])
        #     print("Irregular",response[0])
        #     print("Inclinado",response[1])
        #     print("Plano",response[2])
        #     print("Cuadrúpedo:",quad)
        #     print("Diferencial:",diff)
        if frame % 100 == 0 and frame != 0:
            #print(mlp_output,estancado)
            xdata.clear()
            for lst in ydata_SubTN + ydata_GPi + ydata_GPe + ydata_StR:
                lst.clear()
            ydata_quad.clear()
            ydata_diff.clear()
            base_frame = frame
        x = frame - base_frame
        xdata.append(x)
        
        Irregular = response[0] + (1/tau)*(-response[0] + estancado - 0.5*response[1] - 0.5*response[2])
        Inclinado = response[1] + (1/tau)*(-response[1] + mlp_output[1] - response[0] - 0.5*cam_ok)
        Plano = response[2] + (1/tau)*(-response[2] + mlp_output[0] - response[0] - 0.5*cam_ok)
        response = np.array([Irregular, Inclinado, Plano])
        response = np.maximum(0,response) #/ (np.max(response) if np.max(response) > 0 else 1))
        ganglios[:] = arbitramiento(response, ganglios)
        GPi = ganglios[3:6]
        quad = np.maximum(0.0,(quad + (1 / tau) * (-quad + bias - GPi[0])))
        diff = np.maximum(0.0,(diff + (1 / tau) * (-diff + bias - GPi[1]*GPi[2])))
        if quad-diff > 0.85 and mode != 'a':
            mode = ('a').strip()
            ser.write(mode.encode())
            time.sleep(3)
            ser.write(past.encode())
        elif diff-quad > 0.85 and mode != 'c':
            msg = ('k').strip()
            ser.write(msg.encode())
            time.sleep(5)
            mode = ('c').strip()
            ser.write(mode.encode())
            time.sleep(3)
            ser.write(past.encode())
            

        for i in range(3):
            ydata_SubTN[i].append(ganglios[i])
            lines_SubTN[i].set_data(xdata, ydata_SubTN[i])
            ydata_GPi[i].append(ganglios[i+3])
            lines_GPi[i].set_data(xdata, ydata_GPi[i])
            ydata_GPe[i].append(ganglios[i+6])
            lines_GPe[i].set_data(xdata, ydata_GPe[i])
        for i in range(4):
            ydata_StR[i].append(ganglios[i+9])
            lines_StR[i].set_data(xdata, ydata_StR[i])
        ydata_quad.append(quad)
        ydata_diff.append(diff)
        line_quad.set_data(xdata, ydata_quad)
        line_diff.set_data(xdata, ydata_diff)
        return lines_SubTN + lines_GPi + lines_GPe + lines_StR + [line_quad, line_diff]

    ani = FuncAnimation(fig, update, frames=np.arange(0, 1000), blit=True, interval=100)
    plt.tight_layout()
    plt.show()

def main():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        print(f"[INFO] Conectado a {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir el puerto: {e}")
        return
    
    threading.Thread(target=read_serial, daemon=True).start()
    threading.Thread(target=camera_thread, daemon=True).start()
    threading.Thread(target=consola_serial, daemon=True).start()
    # threading.Thread(target=resend_last, daemon=True).start()
    animacion_thread()

if __name__ == "__main__":
    main()

import time
import serial
import numpy as np
import torch
import torch.nn as nn
import joblib

# ----------------- Config -----------------
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

VENTANA_TAM = 15           # 15 filas
INPUT_SIZE  = 135          # 15*9
NUM_CLASSES = 2            # [Plano, Rampa]

# Mapa de direccion -> índice compacto y luego a 3 bits
MAPA_DIR = {1: 1, 2: 2, 3: 5, 4: 3, 5: 0, 6: 6, 7: 4}

# ----------------- Modelo -----------------
class MLP(nn.Module):
    def __init__(self, input_size, num_classes):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 200),
            nn.ReLU(),
            nn.Linear(200, 100),
            nn.ReLU(),
            nn.Linear(100, num_classes),
        )
    def forward(self, x): 
        return self.net(x)

# Cargar modelo y scaler
model = MLP(INPUT_SIZE, NUM_CLASSES)
model.load_state_dict(torch.load("/home/sebas/Desktop/Peter_RED_Paper/IMU/modelo_mlp.pth", map_location="cpu"))
model.eval()
scaler = joblib.load("IMU/escalador.pkl")

# ----------------- Lógica MLP -----------------
ventana = []  # acumula 15 filas de 9 features

def linea_a_features(tokens):
    """
    tokens: lista de strings con al menos:
      ax, ay, az, gx, gy, gz, direccion, modo, [stuck? ...]
    Devuelve lista de 9 features: ax, ay, az, gx, gy, bit0, bit1, bit2, modo
    """
    # Toma seguro por índice (ignora extras)
    ax = float(tokens[0]); ay = float(tokens[1]); az = float(tokens[2])
    gx = float(tokens[3]); gy = float(tokens[4])
    # gz = float(tokens[5])  # NO lo usamos en la MLP de 9 features

    direccion = int(float(tokens[6]))  # por si llega como "1.00"
    modo      = float(tokens[7])
    stuck = float(tokens[8])
    # mapear y codificar 3 bits
    direc_map = MAPA_DIR.get(direccion, -1)
    if direc_map < 0:
        # dirección desconocida: usa 000
        bit0 = bit1 = bit2 = 0
    else:
        bits = format(direc_map, "03b")
        bit0, bit1, bit2 = int(bits[0]), int(bits[1]), int(bits[2])

    print("Stuck: ", stuck)
    return [ax, ay, az, gx, gy, bit0, bit1, bit2, modo, stuck]

def test_mlp_con_linea(linea_tokens):
    """
    Agrega una fila (9 features). Cuando se juntan 15 filas:
    - escala
    - infiere con la MLP
    - imprime probabilidades [Plano, Rampa]
    """
    global ventana
    feats = linea_a_features(linea_tokens)
    ventana.append(feats[0:len(feats)-1])
    if len(ventana) == VENTANA_TAM:
        entrada = np.array(ventana, dtype=np.float32).flatten()  # (135,)
        ventana.clear()

        # Escalar → tensor
        entrada_escalada = scaler.transform([entrada])           # (1,135)
        X = torch.tensor(entrada_escalada, dtype=torch.float32)  # (1,135)

        with torch.no_grad():
            logits = model(X)
            probs  = torch.softmax(logits, dim=1)[0].cpu().numpy()

        # Imprime limpio: Plano, Rampa
        print(f"[MLP] Plano={probs[0]:.3f}, Rampa={probs[1]:.3f}")

# ----------------- Serie -----------------
def leer_paquetes_serie():
    """
    Lee líneas del puerto serie. Acumula hasta '--END--'.
    Cada paquete se parsea a filas y se alimenta a la MLP.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        print(f"[INFO] Conectado a {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir el puerto: {e}")
        return

    buffer_lines = []
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == "--END--":
                # procesar paquete
                if buffer_lines:
                    for raw in buffer_lines:
                        try:
                            tokens = raw.split(",")
                            if len(tokens) < 8:
                                continue  # línea incompleta
                            # Alimentar línea a la ventana/MLP
                            test_mlp_con_linea(tokens)
                        except Exception as e:
                            print(f"[WARN] Línea inválida: {e}")
                buffer_lines = []
            else:
                buffer_lines.append(line)

    except KeyboardInterrupt:
        print("\n[INFO] Saliendo...")
    finally:
        ser.close()

# ----------------- Main -----------------
if __name__ == "__main__":
    leer_paquetes_serie()

import numpy as np
from collections import deque 
import time, math
from math import inf  # para usar 'np.inf' en la lista literal
import struct
import mmap
import os
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial import Serial
from mlp_imu import test_mlp_con_linea
from queue import Queue

# === NUEVO: parámetros serial ===
PORT = "/dev/ttyACM0"
BAUD = 115200
END_TOKEN = "--END--"
READ_TIMEOUT = 1.0

# ====== MODO SIMULACIÓN ======
USE_LIDAR_SHM   = True   # si False, usa LIDAR simulado
USE_VISION_SHM  = True # si False, usa detecciones simuladas
USE_IMU = False       # si False, usa IMU simulado
MOVEMENT = True
MIN_HOLD_ROCOSO_S = 5.0    # Mínimo tiempo en rocoso antes de permitir volver a liso
SER = True
DEBUG = False

# ====== LIDAR SIM CONFIG (objeto fijo o nada) ======
LIDAR_SIM_HAS_OBJECT = False    # False -> ningún eco (todo np.inf)
LIDAR_SIM_ANGLE_DEG  = 60.0    # ángulo fijo [0..360)
LIDAR_SIM_DISTANCE_M = 0.50    # distancia fija (m)
LIDAR_SIM_WIDTH_DEG  = 8       # ancho del eco (grados, ±half)

# ====== DETECCIONES (SHM) ======
DET_PATH   = "/dev/shm/det_vector"
DET_ROWS   = 2         # 0=ROJO, 1=AZUL
DET_COLS   = 3         # [pos(0..180), area, class_id]
DET_DTYPE  = np.int32
DET_BYTES  = DET_ROWS * DET_COLS * np.dtype(DET_DTYPE).itemsize

# === NEURON SHM (85 floats + seqlock de 8 bytes) ===
NEURON_SHM_PATH = "/dev/shm/neuron_activity"
NEURON_COUNT    = 85
NEURON_SHM_SIZE = 8 + NEURON_COUNT * 4  # 8 bytes de seq + 85 float32

# ====== LIDAR (SHM opcional) ======
MAX_LIDAR_POINTS   = 500
LIDAR_STRUCT_FORMAT = "Qdii"
LIDAR_STRUCT_SIZE   = struct.calcsize(LIDAR_STRUCT_FORMAT)
SHM_NAME_LIDAR      = "/dev/shm/shared_lidar"
ANGLE_IS_DEGREES    = True
ANGLE_BIN_WIDTH     = math.radians(1.0)
NUM_BINS            = int(round(2*math.pi / ANGLE_BIN_WIDTH))
LIM_DIST_M          = 2.0

# ====== TERRAIN ======
STUCK_DEBOUNCE_S     = 3.0   # estancado debe durar al menos 3 s para disparar
MIN_HOLD_ROCOSO_S    = 10.0  # mantener rocoso mínimo 10 s antes de poder salir

#------------------------- Funciones Auxiliares --------------------------------------#

class MiniTX(tk.Tk):
    def __init__(self, SER: Serial, on_quit):
        super().__init__()
        self.title("TX Serial")
        self.geometry("360x140")
        self.SER = SER
        self._on_quit = on_quit

        frm = ttk.Frame(self, padding=10); frm.pack(fill="both", expand=True)
        ttk.Label(frm, text=f"Puerto: {SER.port} @ {SER.baudrate}").grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(frm, text="Enviar:").grid(row=1, column=0, sticky="e", padx=5, pady=8)
        self.var = tk.StringVar()
        ent = ttk.Entry(frm, textvariable=self.var, width=24); ent.grid(row=1, column=1, sticky="we", padx=5, pady=8); ent.focus()

        ttk.Button(frm, text="Enviar", command=self.send).grid(row=1, column=2, sticky="w", padx=5, pady=8)
        ttk.Button(frm, text="Salir", command=self.on_close).grid(row=2, column=2, sticky="e")

        self.bind("<Return>", lambda _: self.send())
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def send(self):
        if not self.SER or not self.SER.is_open:
            messagebox.showwarning("Sin conexión", "El puerto no está abierto."); return
        text = self.var.get()
        if not text: return
        if not text.endswith("\n") and not text.endswith("\r\n"):
            text += "\n"
        try:
            self.SER.write(text.encode("utf-8")); self.SER.flush()
        except Exception as e:
            messagebox.showerror("Error enviando", str(e))

    def on_close(self):
        try:
            if callable(self._on_quit): self._on_quit()
        finally:
            self.destroy()

def gui_thread_fn(SER: Serial, stop_evt: threading.Event):
    def _quit(): stop_evt.set()
    app = MiniTX(SER, on_quit=_quit)

    def poll_stop():
        if stop_evt.is_set():
            try: app.destroy()
            except Exception: pass
            return
        app.after(100, poll_stop)

    app.after(100, poll_stop)
    app.mainloop()

def limit(value, max_abs):
    if value >  max_abs: return  max_abs
    if value < -max_abs: return -max_abs
    return value

def neuron_shm_setup(path=NEURON_SHM_PATH, size=NEURON_SHM_SIZE):
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    os.ftruncate(fd, size)
    mm = mmap.mmap(fd, size, access=mmap.ACCESS_WRITE)
    # inicializa seq en 0 (par = estable)
    struct.pack_into("Q", mm, 0, 0)
    return fd, mm

def neuron_shm_close(fd, mm):
    try:
        if mm: mm.close()
    finally:
        try:
            if fd: os.close(fd)
        except Exception:
            pass

def neuron_shm_write(mm, vec: np.ndarray):
    """
    Escribe vec[0..84] (float32) con un seqlock muy simple:
    - pone seq impar (en escritura), escribe datos, luego pone seq par (estable).
    """
    assert vec.size == NEURON_COUNT, f"Se esperaban {NEURON_COUNT} floats; llegaron {vec.size}"
    # lee seq actual
    seq = struct.unpack_from("Q", mm, 0)[0]
    # entra a sección crítica (impar)
    seq = seq + 1 if seq % 2 == 0 else seq + 2
    struct.pack_into("Q", mm, 0, seq)
    # escribe datos
    mm.seek(8)
    mm.write(vec.astype(np.float32, copy=False).tobytes(order="C"))
    # sale de sección crítica (par)
    struct.pack_into("Q", mm, 0, seq + 1)
    mm.flush()

def open_det_shm():
    """Abre la memoria compartida de detecciones en modo lectura."""
    if not os.path.exists(DET_PATH):
        raise FileNotFoundError(f"No existe {DET_PATH}. ¿Está corriendo el productor de detecciones?")
    fd = os.open(DET_PATH, os.O_RDONLY)
    mm = mmap.mmap(fd, DET_BYTES, access=mmap.ACCESS_READ)
    return fd, mm

def det_view_from_mm(mm):
    """Devuelve una vista numpy (2,3) sobre el mmap."""
    return np.ndarray((DET_ROWS, DET_COLS), dtype=DET_DTYPE, buffer=mm)

def read_det_snapshot(det_view):
    """
    Lee una foto atómica (copia) del vector de detecciones para evitar condiciones de carrera.
    Retorna ndarray shape (2,3) int32.
    """
    # Copia defensiva (el productor puede escribir en caliente)
    return det_view.copy()

def clamp_pos_deg(v):
    """Asegura rango [0, 180] para la posición angular."""
    return float(max(0, min(180, int(v))))

def _to_rad(angle_value: float) -> float:
    """Convierte el ángulo del productor a radianes si viene en grados."""
    return math.radians(angle_value) if ANGLE_IS_DEGREES else float(angle_value)

def _rad_to_bin_idx(theta_rad: float) -> int:
    """Mapea un ángulo en radianes a un índice [0..NUM_BINS-1] usando bins de 1°."""
    theta = theta_rad % (2*math.pi)                  # [0, 2π)
    idx = int(theta / ANGLE_BIN_WIDTH)               # tamaño de bin = 1°
    # Seguridad por acumulación de error flotante
    return min(idx, NUM_BINS - 1)

def print_lidar_distances(lidar_points, max_to_show=12):
    """
    Muestra conteo, punto más cercano y algunos (ángulo en radianes, distancia en metros).
    'lidar_points' es un vector de tamaño NUM_BINS con distancias (m) o +inf.
    """
    idx_valid = np.flatnonzero(np.isfinite(lidar_points))
    if idx_valid.size == 0:
        print("LIDAR: sin distancias válidas.")
        return

    nearest_idx = idx_valid[np.argmin(lidar_points[idx_valid])]
    nearest_dist = float(lidar_points[nearest_idx])
    nearest_theta = nearest_idx * ANGLE_BIN_WIDTH   # rad en [0, 2π)

    # print(f"LIDAR: {idx_valid.size} puntos válidos | "
    #       f"más cercano = {nearest_dist:.3f} m @ {nearest_theta:.3f} rad")

    for i in idx_valid[:max_to_show]:
        theta = i * ANGLE_BIN_WIDTH
        #print(f"θ={theta:6.3f} rad → {float(lidar_points[i]):.3f} m")

def lidar_process(lidar_points):
    """
    Lee un snapshot de la SHM y deposita en bins angulares (rad) las distancias en METROS.
    Si hay múltiples impactos en el mismo bin, se conserva la **menor** distancia (más cercana).
    """
    # Recorre los slots posibles
    for i in range(MAX_LIDAR_POINTS):
        offset = i * LIDAR_STRUCT_SIZE
        data = shm[offset:offset + LIDAR_STRUCT_SIZE]
        if len(data) < LIDAR_STRUCT_SIZE:
            break  # fin real de datos

        timestamp, angle_val, distance_mm, intensity = struct.unpack(LIDAR_STRUCT_FORMAT, data)

        if distance_mm is None or distance_mm <= 0:
            continue

        # Convertir a unidades correctas
        theta_rad = _to_rad(angle_val)                     # ahora en rad
        dist_m    = float(distance_mm) / 1000.0            # mm → m

        # Filtro por distancia
        if not (0.0 < dist_m <= LIM_DIST_M):
            continue

        # Bin por radianes
        idx = _rad_to_bin_idx(theta_rad)

        # Mantener el más cercano por bin
        if not np.isfinite(lidar_points[idx]) or dist_m < lidar_points[idx]:
            lidar_points[idx] = dist_m

    return lidar_points

def vision_fake_step(t, have_red=True, have_blue=True):
    """
    Devuelve (posR, areaR, posB, areaB).
    Pos en [0..180] y áreas enteras.
    """
    posR = 90.0
    areaR = 0
    posB = 90.0
    areaB = 0

    if have_red:
        # rojo oscilando a la izquierda
        posR = 120 + 40*math.sin(0.5*t)   # ~[80..160]
        areaR = 2000 + int(1500*max(0.0, math.sin(0.7*t)))
    if have_blue:
        # azul oscilando a la derecha
        posB = 60 + 40*math.sin(0.4*t + 1.0)   # ~[20..100]
        areaB = 2500 + int(1200*max(0.0, math.sin(0.6*t + 0.5)))
    return float(np.clip(posR,0,180)), max(0,int(areaR)), float(np.clip(posB,0,180)), max(0,int(areaB))

def lidar_clear(num_bins: int):
    """Vector de bins en metros: +inf = sin detección."""
    return np.full(num_bins, np.inf, dtype=np.float32)

def lidar_fake_step(lidar_points, t, angle_width_deg=None, dist_m=None, speed_deg_s=None):
    """
    Sim único: objeto a ÁNGULO FIJO o nada.
    - Se mantiene la firma para no tocar el resto del código.
    - 't' y 'speed_deg_s' se IGNORAN (no hay giro).
    - Si LIDAR_SIM_HAS_OBJECT=False => devuelve todo np.inf.
    - 'angle_width_deg' y 'dist_m' pueden sobrescribir la config global si se pasan.
    """
    num_bins = len(lidar_points)
    pts = lidar_clear(num_bins)

    if not LIDAR_SIM_HAS_OBJECT:
        return pts  # sin detecciones

    # Parámetros efectivos (prioriza argumentos si se pasan)
    width_deg = LIDAR_SIM_WIDTH_DEG if angle_width_deg is None else angle_width_deg
    dist      = LIDAR_SIM_DISTANCE_M if dist_m is None else dist_m
    center_deg = LIDAR_SIM_ANGLE_DEG  # siempre fijo, NO depende de 't' ni 'speed_deg_s'

    # Pintar el eco fijo
    center_idx = int(round(math.radians(center_deg) / ANGLE_BIN_WIDTH)) % num_bins
    half = max(1, int(round(width_deg / 2.0)))
    for k in range(-half, half + 1):
        idx = (center_idx + k) % num_bins
        pts[idx] = dist

    return pts

def lidar_callback_python(ranges, angle_min, angle_increment):
    """
    Entradas:
      - ranges: array-like de distancias en metros (len N)
      - angle_min: float (rad) primer angulo mapeado en radianes
      - angle_increment: float (rad) incremento angular por índice
    Salidas:
      - activaciones_totales: (n_neuronas,) máximo por neurona
    """
    # ====== Parámetros y estructura EXACTOS del original ======
    
    # ====== Ventana de distancia (editable dentro de la función) ======
    r_min = 0.2  # Distancia mínima (m) — mismo valor que tu callback original
    r_max = 0.6  # Distancia máxima (m) — mismo valor que tu callback original

    # ====== Cálculo ======
    ranges = np.asarray(ranges, dtype=float)
    activaciones_totales = np.zeros(n_neuronas, dtype=float)
    detected = False

    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_increment

        # Filtrar valores no válidos o fuera del rango permitido
        if np.isnan(r) or np.isinf(r) or (r < r_min) or (r > r_max):
            continue

        detected = True
        # vector_input = [cos(angle), sin(angle)]
        c, s = np.cos(angle), np.sin(angle)

        # Producto punto con todos los omegas a la vez
        dots = Om[0, :] * c + Om[1, :] * s  # (n_neuronas,)

        # Gaussiana + escala 130/(1000*r)
        gauss = np.exp((dots - 1.0) / (2.0 * (sigma ** 2)))  # (n_neuronas,)
        acts  = gauss * (130.0 / (1000.0 * r))               # (n_neuronas,)

        # Tomar el máximo por neurona (idéntico a np.maximum del original)
        activaciones_totales = np.maximum(activaciones_totales, acts)

    return activaciones_totales

def reader_loop(SER: Serial, stop_evt: threading.Event, print_lock: threading.Lock, q: Queue) -> None:
    buffer_lines = []
    while not stop_evt.is_set() and SER.is_open:
        try:
            raw = SER.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == END_TOKEN:
                # procesa paquete completo (ejemplo con tu MLP)
                for raw_line in buffer_lines:
                    try:
                        tokens = [t.strip() for t in raw_line.split(",")]
                        if len(tokens) < 8:
                            continue
                        out = test_mlp_con_linea(tokens)
                        if out is not None:
                            probs, stuck = out
                            q.put((probs, stuck))   # <-- guardas en la cola
                            # with print_lock:
                            #     print(f"[SER] Pred: {probs}  Stuck: {stuck}")
                    except Exception as ex:
                        with print_lock:
                            print(f"[SER-parse] {ex} :: {raw_line}")
                buffer_lines.clear()
            else:
                buffer_lines.append(line)

        except Exception as e:
            with print_lock:
                print(f"[SER-error] {e}")

shm = None
mlp_queue = Queue()
# --- Abrir SHM de detecciones (pos/área para ROJO/AZUL) ---
det_fd = det_mm = det_view = None

#-------------------------E S T I M U L O -------------------------------------#
areaBoundingBoxR = 0
areaBoundingBoxG = 0
areaBoundingBoxB = 0
posR = 0.0
posG = 0.0
posB = 0.0

#-------------------------N E U R O N A S--------------------------------------#
Gpi = np.zeros((3, 2)) # Globo Pálido Interno
Gpe = np.zeros((3, 2)) # Globo Pálido Externo
StN = np.zeros((3, 2)) # Subtalámico
StR = np.zeros((3, 2)) # Estriado
z = np.zeros((20, 2)) # Red Sam/Espitia
lidar = np.zeros((5,2)) # Wta Lidar
Response = np.zeros((16,2))
Aux = np.zeros((16,2))

#------------------------- C O N S T A N T E S --------------------------------------#
HZ = 10.0
period = 1.0 / HZ          # 0.2 s, como tu timer de ROS
modo_time = 2.0
salida_modo = time.time()
modo_ant = ""
salida = "None"
modo = "None"
command_char = { #MAPA del Serial 
    "Cuadrupedo": 'a',
    "Movil H": 'c',
    "Movil X": 'b',
    "Stop": 'k',
    "Avanza": 'i',
    "Retrocede": ',',
    "Giro Izquierda": 'u', # Cuadrupedo Mover Izquierda
    "Giro Derecha": 'o',  # Cuadrupedo Mover Derecha
    "Desp. Izquierda": 'l', #Girar izquierda cuadrupedo
    "Desp. Derecha":  'j', #Girar Derecha cuadrupedo
    "A": 'a',
    "B": 'b',
    "C": 'c',
}

#--------------IMU --------------
terrainchanger = False
terrain_timer  = 0.0
stuck_since    = None  # timestamp cuando estancado pasa a True (para el debounce)
ignore_imu = False
ignore_timer = time.time()
ignore_duration = 4.2
inclinado = 0
Uprob_inclinado = 0.6
Uprob_plano = 0.85
plano = 1
estancado = False

# Antes de tu callback, inicializa buffers y filtros:
accel_buffer = deque(maxlen=50)     # Ventana de 50 muestras
accel_std = 0.0
accel_std2 = 0.0

low_accel_counter = 0
low_accel_threshold = 0.5
low_accel_limit = 100
low_accel_flag = False

ang_p = 90 # Posicion Frente del Robot
ang_s = 90 # Posicion del estimulo
epsilem = 0.01 # Tolerancia
dt = 1.0 # EXACTO a ROS
cte = 3 # Constante de Avance
Area = 30 # Area Limite Tuneada segun iluminacion

roll = 0.0
pitch = 0.0
std_dev_accel_z = 0.0
accel_z_history = []
w = 10 # Peso Sinaptico
j = 2 # Peso Sinaptico

A = 5 # Parametro Saturacion Naka
Sigma = 0.3 # Parametro Inhibicion Naka
SigmaIMU = 1.5 # Parametro Inhibicion Naka

tau = 1 # Tao Neuronas Z
tauMotor = 2 # Tao Neuronas Z
TaoGpi = 1 # Tao Ganglios
TaoGpe = 2 # Tao Ganglios
TaoSTN = 2 # Tao Ganglios
TaoSTR = 1 # Tao Ganglios

#Usigma_az = 3.27 #PARA CASO PLANO-RUGOSO-PLANO
Usigma_az = 3.7 #PARA CASO PLANO-INLINADO
Upitch = 7.8 #Umbral pitch
Uroll = 270 #Umbral roll

# 1) Pesos para Input -> Response (inverso)
W_input_to_response = np.fliplr(np.eye(16))
# 2) Inhibición lateral en Response
weights_r_r = -np.ones((16,16)) + np.eye(16)
# 3) Conexiones triangulares Response -> Aux
W_response_to_aux = np.triu(np.ones((16,16)))

#------------------------- LIDAR --------------------------------------#
n_neuronas = 16
n_por_cuadrante = n_neuronas // 4

# Cuadrantes en grados (endpoint=False para replicar tu malla)
cuadrante_1 = np.linspace(0,   90,  n_por_cuadrante, endpoint=False)
cuadrante_2 = np.linspace(90,  180, n_por_cuadrante, endpoint=False)
cuadrante_3 = np.linspace(180, 270, n_por_cuadrante, endpoint=False)
cuadrante_4 = np.linspace(270, 360, n_por_cuadrante, endpoint=False)

Directions_deg = np.concatenate([cuadrante_1, cuadrante_2, cuadrante_3, cuadrante_4])
Directions_rad = np.radians(Directions_deg)

# Vectores unitarios preferidos Om (2 x n_neuronas)
Om = np.vstack((np.cos(Directions_rad), np.sin(Directions_rad)))

# Parámetros de la gaussiana (exacto al original)
sigma = 0.06

# --- STUB DE PRUEBA (quítalo cuando uses LIDAR real) ---
angle_min = -3.140000104904175
angle_increment = 0.01749303564429283  # ~1° por muestra
ranges = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 11.928791046142578, 11.859428405761719, 11.730998992919922, 11.671629905700684, 11.561872482299805, 11.511222839355469, 11.417759895324707, 11.37471866607666, 11.295480728149414, 11.259085655212402, 11.19227123260498, 11.132852554321289, 11.105716705322266, 11.056218147277832, 11.012625694274902, 10.974431991577148, 10.94117546081543, 10.912430763244629, 10.887810707092285, 10.866957664489746, 10.82585334777832, 10.659710884094238, 10.55108642578125, 10.444369316101074, 10.339734077453613, 10.237374305725098, 10.187108993530273, 10.088534355163574, 9.992751121520996, 9.899985313415527, 9.854808807373047, 9.76701831817627, 9.682860374450684, 9.642226219177246, 9.56401538848877, 9.526510238647461, 9.454876899719238, 9.420822143554688, 9.356430053710938, 9.326166152954102, 9.29724407196045, 9.24357795715332, 9.218915939331055, 9.195752143859863, 9.154088020324707, 9.13566780090332, 1.310181975364685, 1.2410608530044556, 1.1974314451217651, 1.1410187482833862, 1.11784029006958, 1.0968273878097534, 1.0824755430221558, 1.0686348676681519, 1.0424387454986572, 1.0321404933929443, 1.0244622230529785, 1.0170994997024536, 1.0100492238998413, 1.0033082962036133, 0.9968730211257935, 0.9906423091888428, 0.9882991909980774, 0.9862038493156433, 0.9843555688858032, 0.9827532172203064, 0.9827808737754822, 0.9846906661987305, 0.9868459701538086, 0.9918953776359558, 0.9947900176048279, 0.9984793663024902, 1.0052378177642822, 1.01229989528656, 1.0196688175201416, 1.0273470878601074, 1.0457245111465454, 1.0589585304260254, 1.0726786851882935, 1.086896538734436, 1.101623773574829, 1.1468082666397095, 1.1716538667678833, 1.205284595489502, 1.2506428956985474, 1.3308942317962646, 10.815502166748047, 10.912272453308105, 11.011664390563965, 11.218291282653809, 11.325514793395996, 11.435338020324707, 11.66275691986084, 11.780338287353516, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf], dtype=float)


# Buffers (metros) con +inf = sin dato
lidar_points = np.full(NUM_BINS, np.inf, dtype=np.float32)
time_lidar_reset = time.monotonic()

# === Ajustes para tu red: ángulos en RADIANES ===
angle_min = np.pi/2               
angle_increment = ANGLE_BIN_WIDTH    # π/180 rad (~1°)

assert ranges.ndim == 1 and len(ranges) == 360, f"Esperaba 360 lecturas, tengo {ranges.shape}"

if USE_LIDAR_SHM and os.path.exists(SHM_NAME_LIDAR):
    try:
        with open(SHM_NAME_LIDAR, "r+b") as f:
            shm = mmap.mmap(f.fileno(), 0, mmap.MAP_SHARED, mmap.PROT_READ)
        print("[LIDAR] usando SHM")
    except Exception as e:
        print(f"[LIDAR] no se pudo abrir SHM, uso simulado: {e}")
        shm = None
else:
    print("[LIDAR] SHM deshabilitada o no existe; uso simulado")

if USE_VISION_SHM:
    try:
        det_fd, det_mm = open_det_shm()
        det_view = det_view_from_mm(det_mm)
        print("[VISION] usando SHM")
    except Exception as e:
        print(f"[VISION] no pude abrir {DET_PATH}, uso simulado: {e}")
else:
    print("[VISION] SHM deshabilitada; uso simulado")

try:
    if (SER==True):
        SER = serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT)
        stop_evt = threading.Event()
        print_lock = threading.Lock()

        t_reader = threading.Thread(target=reader_loop, args=(SER, stop_evt, print_lock, mlp_queue), daemon=True)
        t_reader.start()

        # hilo GUI
        t_gui = threading.Thread(target=gui_thread_fn, args=(SER, stop_evt), daemon=True)
        t_gui.start()
except Exception as e:
    print(f"No se pudo abrir {PORT}: {e}")

#------------------------- D I N A M I C A --------------------------------------#
try:
    while not stop_evt.is_set():
        if USE_IMU:
            try:
                probs, stuck = mlp_queue.get_nowait()
                inclinado =probs[1]
                plano = probs[0]
                estancado = stuck

                if (DEBUG==True):
                    print(f"MLP → probs={probs}  stuck={stuck}")

            except Exception as e:
                pass
                #print(f"No se pudo abrir la MLP: {e}")
        else:
            pass

        # === SHM de neuronas ===
        neuron_fd = None
        neuron_mm = None
        try:
            neuron_fd, neuron_mm = neuron_shm_setup()
            #print(f"[NEURON-SHM] listo en {NEURON_SHM_PATH} ({NEURON_SHM_SIZE} bytes)")
        except Exception as e:
            print(f"[NEURON-SHM] error al crear SHM: {e}")

        loop_t0 = time.monotonic()
        now = loop_t0

        # === LIDAR ===
        if (now - time_lidar_reset) > 1:
            lidar_points = np.full(NUM_BINS, np.inf, dtype=np.float32)
            time_lidar_reset = now

        if shm is not None:
            print_lidar_distances(lidar_points, max_to_show=12)
            lidar_points = lidar_process(lidar_points)  # lee SHM real
        else:
            # sim: 1 “eco” giratorio a 0.5 m
            lidar_points = lidar_fake_step(lidar_points, t=time.time(), angle_width_deg=8, dist_m=0.5, speed_deg_s=60.0)

        activaciones_totales = lidar_callback_python(
            lidar_points,
            angle_min=angle_min,
            angle_increment=-angle_increment
        )

        # === Estímulos (visión) ===
        if det_view is not None:
            det_snapshot = read_det_snapshot(det_view)   # shape (2,3)
            posR  = clamp_pos_deg(det_snapshot[0, 0]); areaBoundingBoxR = int(det_snapshot[0, 1])
            posB  = clamp_pos_deg(det_snapshot[1, 0]); areaBoundingBoxB = int(det_snapshot[1, 1])
        else:
            posR, areaBoundingBoxR, posB, areaBoundingBoxB = vision_fake_step(time.time(), have_red=False, have_blue=False)
    
   #------------------------E S T I M U L O -------------------------------------#

        # Comportamiento esperado: Huye del depredador (Red) en movil H. Caza a la presa (Blue) en cuadrupedo.
        # Evita al obstaculo (Green) en modo omnidireccional. Si no hay estimulo avanza en modo movil H.
        # Estimulo en frente es posicion entre 70 y 110. Estimulo a la izquierda son valores entre 111 y 180.
        # Estimulo a la derecha son valores entre 0 y 69

        cmd_lineal = 0.0 # Comando de Velocidad Lineal
        cmd_ang = 0.0 # Comando de Velocidad Angular
        cmd_lateral = 0.0 # Comando de Velocidad Lateral

        #------------------------- D I N A M I C A --------------------------------------#
        lidar[0, 1] = lidar[0, 0] + (dt / 10) * (-lidar[0, 0] + (np.sum(activaciones_totales[0:4]) + np.sum(activaciones_totales[12:16]) - lidar[1, 0])) # Frente
        lidar[1, 1] = lidar[1, 0] + (dt / 10) * (-lidar[1, 0] + (np.sum(activaciones_totales[4:12]) - lidar[0, 0])) # Atras
        lidar[2, 1] = lidar[2, 0] + (dt / 10) * (-lidar[2, 0] + (np.sum(activaciones_totales[0:8]) - lidar[3, 0])) # Izquierda
        lidar[3, 1] = lidar[3, 0] + (dt / 10) * (-lidar[3, 0] + (np.sum(activaciones_totales[8:16]) - lidar[2, 0])) # Derecha

        for r in range(16):
            Response[r, 1] = Response[r, 0] + (dt/5) * (-Response[r, 0] + max(0, (W_input_to_response @ activaciones_totales)[r] + weights_r_r[r, :] @ Response[:, 0]))
            Aux[r, 1] = Aux[r, 0] + (dt/5) * (-Aux[r, 0] + max(0, W_response_to_aux[r, :] @ Response[:, 1]))

        lidar[4,1] = lidar[4, 0] + (dt / tau) * (-lidar[4, 0]*1.1 + max(0, (np.sum(Aux[:,0]))))

        if areaBoundingBoxR/400 > 10: R = areaBoundingBoxR/400
        else: R = 0

        #R = 3.626 #Descomentar para probar inclinacion
        if lidar[4,0]*15 > 0.2: G = lidar[4,0]*15
        else: G = 0
        B = areaBoundingBoxB/500

        StN[0, 1] = np.clip((StN[0, 0] + (1/TaoSTN)*(-StN[0, 0] + R - Gpi[0,0] - Gpe[1,0] - Gpe[2,0] -1.0)),0, None)
        StN[1, 1] = np.clip((StN[1, 0] + (1/TaoSTN)*(-StN[1, 0] + G - Gpi[1,0] - Gpe[0,0] - Gpe[2,0] -1.0)),0, None)
        StN[2, 1] = np.clip((StN[2, 0] + (1/TaoSTN)*(-StN[2, 0] + B*0.7 - Gpi[2,0] - Gpe[0,0] - Gpe[1,0] -1.0)),0, None)
        
        Gpi[0, 1] = np.clip((Gpi[0, 0] + (1/TaoGpi)*(-Gpi[0, 0] + StN[1,0] + StN[2,0] - Gpe[0,0] - StR[0,0])),0, None)
        Gpi[1, 1] = np.clip((Gpi[1, 0] + (1/TaoGpi)*(-Gpi[1, 0] + StN[0,0] + StN[2,0] - Gpe[1,0] - StR[1,0])),0, None)
        Gpi[2, 1] = np.clip((Gpi[2, 0] + (1/TaoGpi)*(-Gpi[2, 0] + StN[0,0] + StN[1,0] - Gpe[2,0] - StR[2,0])),0, None)
        
        Gpe[0, 1] = np.clip((Gpe[0, 0] + (1/TaoGpe)*(-Gpe[0, 0] + StN[0,0])),0, None)
        Gpe[1, 1] = np.clip((Gpe[1, 0] + (1/TaoGpe)*(-Gpe[1, 0] + StN[1,0]*5)),0, None)
        Gpe[2, 1] = np.clip((Gpe[2, 0] + (1/TaoGpe)*(-Gpe[2, 0] + StN[2,0]*0.5)),0, None)
        
        StR[0, 1] = np.clip((StR[0, 0] + (1/TaoSTR)*(-StR[0, 0] + StN[0,0])),0, None)
        StR[1, 1] = np.clip((StR[1, 0] + (1/TaoSTR)*(-StR[1, 0] + StN[1,0])),0, None)
        StR[2, 1] = np.clip((StR[2, 0] + (1/TaoSTR)*(-StR[2, 0] + StN[2,0])),0, None)

        if Gpe[0,1] > 1.5 and R > 0.5:
            ang_s = posR
        # elif Gpe[1,1] > 0.5 and G > 0.5:
        #     ang_s = 180*(lidar[2,1] > 0.1) + 90*(lidar[0,1] > 0.1) + ang_s*(lidar[4,1]<0.1)
        elif Gpe[2,1] > 1.5 and B > 0.5:
            ang_s = posB
        else:
            ang_s = 90*(lidar[4,1]<0.3)

        # ang_s = 90 #Descomentar para probar terreno inclinado

        # ------IMPLEMENTACIÒN MÒDULO IMU ----------
        z[0, 1] = z[0, 0] + (dt / tau) * (-z[0, 0] + (A * max(0, (std_dev_accel_z - Usigma_az ))**2) / (SigmaIMU**2 + (-z[0,0] + std_dev_accel_z - Usigma_az)**2))
        z[1, 1] = z[1, 0] + (dt / tau) * (-z[1, 0] + (A * max(0, (pitch - Upitch))**2) / (SigmaIMU**2 + (-z[1,0] + pitch - Upitch)**2))
        z[2, 1] = z[2, 0] + (dt / tau) * (-z[2, 0] + (A * max(0, (roll - Uroll ))**2) / (SigmaIMU**2 + (-z[2,0] + roll - Uroll )**2))
        z[3, 1] = z[3, 0] + (dt / tau) * (-z[3, 0] + max(0, (Gpe[2,0] )))
        z[4, 1] = z[4, 0] + (dt / tau) * (-z[4, 0] + max(0, (Gpe[1, 0] + j * Gpe[0, 0])))

        z[5, 1] = z[5, 0] + (dt / tau) * (-z[5, 0] + max(0, lidar[2,0]*160 + (lidar[4,1]<0.3)*((ang_s - ang_p) - 20)))
        z[6, 1] = z[6, 0] + (dt / tau) * (-z[6, 0] + max(0, lidar[3,0]*160 + (lidar[4,1]<0.3)*((ang_p - ang_s) - 20)))

        z[7, 1] = z[7, 0] + (dt / tau) * (-z[7, 0] + max(0, (z[5, 0] + z[3, 0] - w*z[4, 0])))
        z[8, 1] = z[8, 0] + (dt / tau) * (-z[8, 0] + max(0, (z[5, 0] + z[4, 0] - w*z[3, 0])))
        z[9, 1] = z[9, 0] + (dt / tau) * (-z[9, 0] + max(0, (z[4, 0] + z[6, 0] - w*z[3, 0])))
        z[10, 1] = z[10, 0] + (dt / tau) * (-z[10, 0] + max(0, (z[3, 0] + z[6, 0] - w*z[4, 0])))

        z[11, 1] = z[11, 0] + (dt / tau) * (-z[11, 0] + max(0, (z[7, 0] + z[9, 0])))
        z[12, 1] = z[12, 0] + (dt / tau) * (-z[12, 0] + max(0, (z[10, 0] + z[8, 0])))
        z[13, 1] = z[13, 0] + (dt / tau) * (-z[13, 0] + max(0, (-w*abs(cmd_ang)*z[11, 0] - w*abs(cmd_ang)*z[12, 0] -2*w*z[17,0] + cte + Gpe[2,0])))

        z[14, 1] = z[14, 0] + (dt / tau) * (-z[14, 0] + (A * max(0, (100*Gpe[1,0] - w*Gpi[0, 0] - w*Gpi[2, 0] - w*z[15, 0] - w*z[16, 0] ))**2) / (Sigma**2 + (100*Gpe[1,0] - w*Gpi[0, 0] - w*Gpi[2, 0] - w*z[15, 0] - w*z[16, 0] )**2))
        z[15, 1] = z[15, 0] + (dt / tau) * (-z[15, 0] + (A * max(0, (-Gpe[1,0]*100 -cte + z[4, 0]*w + 2*z[0,0] - w*z[14, 0]*1.5 - w*z[16, 0] ))**2) / (Sigma**2 + (-Gpe[1,0]*100 -cte + z[4, 0]*w + 2*z[0,0] - w*z[14, 0]*1.5 - w*z[16, 0] )**2))
        z[16, 1] = z[16, 0] + (dt / tau) * (-z[16, 0] + (A * max(0, (z[3, 0] - 100*Gpe[1,0] - w*z[14, 0]*1.5 - w*z[15, 0]*1.5 + 5*z[1,0] + 5*z[2,0] + cte ))**2) / (Sigma**2 + (z[3, 0] - 100*Gpe[1,0] - w*z[14, 0]*1.5 - w*z[15, 0]*1.5 + 5*z[1,0] + 5*z[2,0] + cte )**2))
        
        z[17, 1] = z[17, 0] + (dt / tau) * (-z[17, 0] + max(0, (Gpe[2,0] - Area)))

        cmd_ang = (z[11,0]*(lidar[4,0] < 0.3)) - (z[12,0]*(lidar[4,0]<0.3))
        cmd_lateral = (lidar[2,0]*1.5 + lidar[3,0]*1.5 + z[11,0]*((Gpe[1,0] > 0.5)and(Gpe[2,0]<0.5))) - (z[12,0]*((Gpe[1,0] > 0.5)and(Gpe[2,0]<0.5)))
        cmd_lineal = lidar[0,0]*1.5 - lidar[1,0]*1.5 + z[13,0] -j*z[4,0]*(z[5,0] < epsilem and z[6,0] < epsilem)

        for i in range(len(z)): z[i, 0] = z[i,1]*(z[i,1]>epsilem)
        for i in range(len(lidar)): lidar[i, 0] = lidar[i,1]*(lidar[i,1]>epsilem)
        for i in range(len(Response)): Response[i, 0] = Response[i,1]*(Response[i,1]>epsilem)
        for i in range(len(Aux)): Aux[i, 0] = Aux[i,1]*(Aux[i,1]>epsilem)
        for i in range(len(StN)): StN[i, 0] = StN[i,1]*(StN[i,1]>epsilem)
        for i in range(len(Gpi)): Gpi[i, 0] = Gpi[i,1]*(Gpi[i,1]>epsilem)
        for i in range(len(Gpe)): Gpe[i, 0] = Gpe[i,1]*(Gpe[i,1]>epsilem)
        for i in range(len(StR)): StR[i, 0] = StR[i,1]*(StR[i,1]>epsilem)

        # ------------------- PRINTS -------------------------
        if (DEBUG==False):
            # if USE_IMU: print(f"MLP → probs={probs}  stuck={stuck}")
            # print("estancado: ", str(estancado))
            # print("inclinado: ", str(inclinado))
            # print("plano: ", str(plano))
            print("R: ", str(R))
            print("G: ", str(G))
            print("B: ", str(B))

            # print("cmd_ang: ", str(cmd_ang))
            # print("cmd_lineal: ", str(cmd_lineal))
            # print("cmd_lateral: ", str(cmd_lateral))

            # print("lidar frente: ", str(lidar[0,0]))
            # print("lidar atras: ", str(lidar[1,0]))
            # print("lidar izquierda: ", str(lidar[2,0]))
            # print("lidar derecha:", str(lidar[3,0]))
            # print("lidar 4:", str(lidar[4,0]))

            # # Imprimir los 16 valores de Response
            # for i, val in enumerate(Response[:, 0]):
            #     print(f"Response {i}: {val}")

            # # Imprimir los 16 valores de Aux
            # for i, val in enumerate(Aux[:, 0]):
            #     print(f"Aux {i}: {val}")

            # print(
            #                 f"GpeR: {Gpe[0,1]}\n"
            #                 f"GpeG: {Gpe[1,1]}\n"
            #                 f"GpeB: {Gpe[2,1]}\n"
            #                 f"ang_p: {ang_p}\n"
            #                 f"ang_s: {ang_s}\n"
            # #                 f"3: {z[3,1]}\n"
            # #                 f"4: {z[4,1]}\n"
            # #                 f"5: {z[5,1]}\n"
            # #                 f"6: {z[6,1]}\n"
            # #                 f"7: {z[7,1]}\n"
            # #                 f"8: {z[8,1]}\n"
            # #                 f"9: {z[9,1]}\n"
            # #                 f"10: {z[10,1]}\n"
            # #                 f"11: {z[11,1]}\n"
            # #                 f"12: {z[12,1]}\n"
            # #                 f"13: {z[13,1]}\n"
            # #                 f"14: {z[14,1]}\n"
            # #                 f"15: {z[15,1]}\n"
            # #                 f"16: {z[16,1]}\n"
            # #                 f"17: {z[17,1]}\n"
            # #                 f"0: {z[0,1]}\n"
            # #                 f"1: {z[1,1]}\n"
            # #                 f"2: {z[2,1]}\n"
            # #                 f"roll: {roll}\n"
            # #                 f"pitch: {pitch}\n"
            # #                 f"STD total: {accel_std:.3f}"
            # )

        cmd_ang = limit(cmd_ang, 1)
        cmd_lineal = limit(cmd_lineal, 5)
        cmd_lateral = limit(cmd_lateral, 5)

        now = time.time()
        if estancado:
            if stuck_since is None:
                stuck_since = now
        else:
            stuck_since = None

    #------------------------- TERRAIN CHANGER -----------------------------
        pitch = 8 if (inclinado > Uprob_inclinado) else 0

        # 1) ENTRADA a rocoso: exige estancado True durante ≥ 3 s (debounce) + plano alto
        if (not terrainchanger):
            if (stuck_since is not None) and ((now - stuck_since) >= STUCK_DEBOUNCE_S) and (plano > Uprob_plano):
                terrainchanger = True
                std_dev_accel_z = 9
                terrain_timer = now  # arranca el hold mínimo
                # print("Terreno rocoso detectado 🚧 (debounce OK)")

        # 2) SI YA ESTÁ EN ROCOSO: mantener mínimo 10 s; cumplido el mínimo, salir si estancado=False
        if terrainchanger:
            elapsed = now - terrain_timer

            if elapsed < MIN_HOLD_ROCOSO_S:
                # En ventana mínima: siempre rocoso, no permitir salida
                std_dev_accel_z = 9
            else:
                # Cumplido el mínimo: salir en cuanto deje de estar estancado
                if not estancado:
                    terrainchanger = False
                    std_dev_accel_z = 0
                    # print("Terreno liso 🛣️ (salida tras hold mínimo)")
                else:
                    std_dev_accel_z = 9  # sigue rocoso
        else:
            # Estado liso
            std_dev_accel_z = 0
        
        #------------------------- P U B L I C A C I O N --------------------------------------#
        if MOVEMENT:
            # 1) calcular magnitudes filtradas por umbral
            ang = cmd_ang     if abs(cmd_ang)     > epsilem else 0.0
            lat = cmd_lateral if abs(cmd_lateral) > epsilem else 0.0
            lin = cmd_lineal  if abs(cmd_lineal)  > epsilem else 0.0

            # 2) resolver prioridad y construir UN solo Twist
            tw_ang, tw_lat, tw_lin = 0.0, 0.0, 0.0

            if z[17,1] > 0.25:
                # STOP: todo a cero
                salida = "Stop"
                if not DEBUG: print("Stop")
            elif ang != 0.0:
                # prioridad 1: giro
                tw_ang = ang
                if not DEBUG: print("Giro Izquierda" if ang > 0 else "Giro Derecha")
                salida = "Giro Izquierda" if ang > 0 else "Giro Derecha"
            elif lat != 0.0 and z[16,1] < 0.5:
                # prioridad 2: desplazamiento lateral
                tw_lat = lat
                if not DEBUG: print("Desp. Izquierda" if lat > 0 else "Desp. Derecha")
                salida = "Desp. Izquierda" if lat > 0 else "Desp. Derecha"
            elif lin != 0.0:
                # prioridad 3: avance/retroceso
                tw_lin = lin
                if not DEBUG: print("Avanza" if lin > 0 else "Retrocede")
                salida = "Avanza" if lin > 0 else "Retrocede"
            else:
                # sin comando significativo → queda todo en 0
                salida = "None"
                pass

            # 3) publicar una sola vez
           # publish_twist(linear_x=tw_lin, linear_y=tw_lat, angular_z=tw_ang)
            # ------------------------------------------------------------------------------

            # modos
            if z[15,1] > 0.5:
                #publish_mode('C');
                if not DEBUG: print("Cuadrupedo") 
                modo = "Cuadrupedo"
            elif z[16,1] > 0.5:
                #publish_mode('H');
                if not DEBUG: print("Móvil H") 
                modo = "Movil H"
            elif z[14,1] > 0.5:
                #publish_mode('X');
                if not DEBUG: print("Móvil X") 
                modo = "Movil X"

            # Ejemplo de consulta
            #publish_data()
            #publish_imu()

        # === PUBLICAR 85 NEURONAS EN SHM ===
        try:
            parts = [
                np.asarray(Gpi)[:, 1],           # 3
                np.asarray(Gpe)[:, 1],           # 3
                np.asarray(StN)[:, 1],           # 3
                np.asarray(StR)[:, 1],           # 3   → 12
                np.asarray(z)[:, 1],             # 20  → 32
                np.asarray(lidar)[:, 1],         # 5   → 37
                np.asarray(activaciones_totales),# 16  → 53 (INPUT)
                np.asarray(Response)[:, 1],      # 16  → 69
                np.asarray(Aux)[:, 1],           # 16  → 85
            ]
            vec85 = np.concatenate([p.ravel() for p in parts]).astype(np.float32)
            assert vec85.size == 85
            if neuron_mm is not None:
                neuron_shm_write(neuron_mm, vec85)
        except Exception as e:
            # no interrumpas el control por un fallo de SHM
            if DEBUG: print(f"[NEURON-SHM] write error: {e}")

        if (command_char.get(modo) != None):
            cmd = command_char.get(modo)
            if (modo_ant != cmd):
                cmd_t0 = time.monotonic()
                parada = "k"
                print('cambio de modo')
                SER.write(parada.encode())   # Envía el comando
                time.sleep(2) #4
                modo_ant = cmd
                SER.write(cmd.encode())   # Envía el comando
                if DEBUG: print(f"Enviado: {cmd!r}")
                modo_time = time.time()
                time.sleep(3) #5

        if (command_char.get(salida) != None):
            cmd_salida = command_char.get(salida)
            if (time.time()-salida_modo > 0.5 ):
                SER.write(cmd_salida.encode())   # Envía el comando
                if DEBUG: print(f"Enviado: {cmd_salida!r}")
                salida_modo = time.time()

        time.sleep(max(0.0, period - (time.monotonic() - loop_t0)))

except KeyboardInterrupt:
        print("\n[main] Interrupción del usuario.")
        stop_evt.set()
finally:
        # cierre ordenado
        parada = "k"
        SER.write(parada.encode())   # Envía el comando
        stop_evt.set()
        if SER and SER.is_open:
            try: SER.close()
            except Exception: pass
        if t_reader: t_reader.join(timeout=1.0)
        t_gui.join(timeout=1.0)
        print("[main] Listo.")
        neuron_shm_close(neuron_fd, neuron_mm)



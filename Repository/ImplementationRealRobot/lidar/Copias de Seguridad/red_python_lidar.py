import numpy as np
from collections import deque 
import time, math

#------------------------- Funciones Auxiliares --------------------------------------#

def limit(value, max_abs):
    if value >  max_abs: return  max_abs
    if value < -max_abs: return -max_abs
    return value


#------------------------- IMU SIMULADO --------------------------------------#

# Nota: Estas funciones (DEL IMU) no representan el comportamiento real de los sensores ni de Ros ni de la vida real,
# solo deberian usarse para probar comportamientos de la red pero NO tunear la red para que respondan a estas funciones.
# Se recomienda reemplazar estas funciones por funciones de sensores reales y tunear la red acorde. En ROS ya esta tuneado.

def imu_step(state, dt, terrain_on=False, params=None):
    """
    Simula una lectura IMU y replica la lógica de tu imu_callback (ROS):
      - roll/pitch desde cuaterniones con las mismas fórmulas
      - accel_mag -> accel_buffer -> accel_std/accel_std2
      - ignore_imu con ventana temporal
      - low_accel_counter / low_accel_flag
    Sin clases: todo el estado va en 'state' (dict).
    Devuelve: (roll, pitch, accel_std, accel_std2, low_accel_flag)
    """
    # ---- Inicialización perezosa del estado ----
    if not state.get("_init", False):
        state["_init"] = True
        state["t"] = 0.0
        state["accel_buffer"] = deque(maxlen=state.get("buffer_len", 50))
        state["ignore_imu"] = state.get("ignore_imu", False)
        state["ignore_timer"] = state.get("ignore_timer", 0.0)
        state["ignore_duration"] = state.get("ignore_duration", 4.2)
        state["low_accel_threshold"] = state.get("low_accel_threshold", 0.5)
        state["low_accel_limit"] = state.get("low_accel_limit", 100)
        state["low_accel_counter"] = state.get("low_accel_counter", 0)
        state["low_accel_flag"] = False
        # Señales publicadas
        state["roll"] = 0.0
        state["pitch"] = 0.0
        state["accel_std"] = 0.0
        state["accel_std2"] = 0.0
        # RNG
        seed = state.get("seed", None)
        state["_rng"] = np.random.default_rng(seed)

    # ---- Parámetros por defecto ----
    P = {
        "pitch_amp_deg": 5.0, "pitch_freq_hz": 0.20,
        "roll_amp_deg":  1.0, "roll_freq_hz":  0.05,
        "yaw_deg_const": 0.0,
        "pitch_offset_deg": 0.0,
        "roll_offset_deg":  0.0,
        "vib_base_std": 0.40, "vib_rough_std": 3.50,
        "ax_bias": 0.0, "ay_bias": 0.0, "az_bias": 0.0,
    }
    if params:
        P.update(params)  # <- aplica overrides externos

    # ---- Avance de tiempo ----
    state["t"] += float(dt)

    # ---- Euler sintéticos (con offsets) ----
    pitch_deg = P["pitch_offset_deg"] + P["pitch_amp_deg"] * math.sin(2*math.pi*P["pitch_freq_hz"] * state["t"])
    roll_deg  = P["roll_offset_deg"]  + P["roll_amp_deg"]  * math.sin(2*math.pi*P["roll_freq_hz"]  * state["t"] + 1.0)
    yaw_deg   = P["yaw_deg_const"]

    # ---- Euler -> quaternion (ZYX) ----
    r = math.radians(roll_deg); p = math.radians(pitch_deg); y = math.radians(yaw_deg)
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy

    # ---- Tus fórmulas EXACTAS de roll/pitch desde quat ----
    roll  = 180.0 - abs(np.degrees(np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))))
    pitch = abs(np.degrees(np.arcsin(2*(qw*qy - qz*qx))))

    # ---- Aceleraciones sintéticas (ruido gaussiano) ----
    vib = P["vib_base_std"] + (P["vib_rough_std"] if terrain_on else 0.0)
    rng = state["_rng"]
    ax = P["ax_bias"] + rng.normal(0.0, vib)
    ay = P["ay_bias"] + rng.normal(0.0, vib)
    az = P["az_bias"] + rng.normal(0.0, vib)

    # ---- Magnitud y buffer ----
    accel_mag = float(np.sqrt(ax*ax + ay*ay + az*az))
    state["accel_buffer"].append(accel_mag)

    # ---- ignore_imu (idéntico) ----
    if state["ignore_imu"]:
        if time.time() - state["ignore_timer"] < state["ignore_duration"]:
            state["accel_std"]  = 0.0
            state["accel_std2"] = 0.0
            state["roll"]  = roll
            state["pitch"] = pitch
            return roll, pitch, 0.0, 0.0, state["low_accel_flag"]
        else:
            state["ignore_imu"] = False  # termina ventana

    # ---- STD como en tu callback ----
    if len(state["accel_buffer"]) > 1:
        std_val = float(np.std(state["accel_buffer"]))
    else:
        std_val = 0.0

    state["accel_std"]  = std_val
    state["accel_std2"] = std_val
    state["roll"]  = roll
    state["pitch"] = pitch

    # ---- low_accel_counter / flag ----
    if state["accel_std"] < state["low_accel_threshold"]:
        state["low_accel_counter"] += 1
    else:
        state["low_accel_counter"] = 0
    state["low_accel_flag"] = (state["low_accel_counter"] >= state["low_accel_limit"])

    return roll, pitch, state["accel_std"], state["accel_std2"], state["low_accel_flag"]

#-------------------------E S T I M U L O -------------------------------------#

# Aqui definimos el comportamiento simulado que ve el robot con el objetivo de probar la red, por eso
# se definen valores estaticos.

areaBoundingBoxR = 0
areaBoundingBoxG = 0
areaBoundingBoxB = 0
posR = 0.0
posG = 0.0
posB = 0.0

# ====== CONFIGURACIÓN IMU (estática, fuera del loop) ======
IMU_SCENARIO = "liso"       # opciones: "liso", "rugoso", "inclinado"

imu_state = {
    "buffer_len": 50,
    "ignore_duration": 4.2,
    "low_accel_threshold": 0.5,
    "low_accel_limit": 100,
    "seed": 42,
}

# Parám. base comunes
imu_params = {
    "pitch_amp_deg": 5.0,
    "roll_amp_deg":  1.0,
    "pitch_offset_deg": 0.0,
    "roll_offset_deg":  0.0,
    "vib_base_std": 0.40,
    "vib_rough_std": 3.50,   # se aplica solo si terrain_on=True
    "ax_bias": 0.0, "ay_bias": 0.0, "az_bias": 0.0,
}

# Selección del modo (estático)
if IMU_SCENARIO == "liso":
    terrain_on = False                     # vibración baja
    # amplitudes pequeñas para no cruzar Upitch
    imu_params["pitch_amp_deg"] = 4.0
    imu_params["roll_amp_deg"]  = 1.0

elif IMU_SCENARIO == "rugoso":
    terrain_on = True                      # añade vib_rough_std a la std (rocoso)
    # ángulos similares al liso, el trigger viene por vibración (accel_std)
    imu_params["pitch_amp_deg"] = 4.0
    imu_params["roll_amp_deg"]  = 1.5

elif IMU_SCENARIO == "inclinado":
    terrain_on = False
    # offset para asegurar pitch > Upitch (tu Upitch=7.8°). Pon ~10–12°:
    imu_params["pitch_offset_deg"] = 10.0  # inclinación estática
    imu_params["pitch_amp_deg"]    = 1.0   # pequeña oscilación sobre el offset
    imu_params["roll_amp_deg"]     = 1.0

else:
    raise ValueError("IMU_SCENARIO inválido")


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

MOVEMENT = True

#--------------IMU --------------

ignore_imu = False
ignore_timer = time.time()
ignore_duration = 4.2

# Antes de tu callback, inicializa buffers y filtros:
accel_buffer = deque(maxlen=50)     # Ventana de 50 muestras
accel_std = 0.0
accel_std2 = 0.0

#Logica que será neuronal
terrainchanger = False
terrain_timer = 0.0  # Guarda el tiempo de inicio

low_accel_counter = 0
low_accel_threshold = 0.5
low_accel_limit = 100
low_accel_flag = False

ang_p = 90 # Posicion Frente del Robot
ang_s = 90 # Posicion del estimulo
epsilem = 0.01 # Tolerancia
dt = 1.0 # EXACTO a ROS
cte = 3 # Constante de Avance
Area = 28 # Area Limite Tuneada segun iluminacion

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
# umbral = 0.95  # (no se usa en el cálculo de activaciones_totales) 

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
    r_min = 0.3  # Distancia mínima (m) — mismo valor que tu callback original
    r_max = 1.0  # Distancia máxima (m) — mismo valor que tu callback original

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

#------------------------- D I N A M I C A --------------------------------------#

# --- STUB DE PRUEBA (quítalo cuando uses LIDAR real) ---
from math import inf  # para usar 'np.inf' en la lista literal

# Metadatos EXACTOS del escaneo que enviaste
angle_min = -3.140000104904175
angle_increment = 0.01749303564429283  # ~1° por muestra

ranges = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 11.928791046142578, 11.859428405761719, 11.730998992919922, 11.671629905700684, 11.561872482299805, 11.511222839355469, 11.417759895324707, 11.37471866607666, 11.295480728149414, 11.259085655212402, 11.19227123260498, 11.132852554321289, 11.105716705322266, 11.056218147277832, 11.012625694274902, 10.974431991577148, 10.94117546081543, 10.912430763244629, 10.887810707092285, 10.866957664489746, 10.82585334777832, 10.659710884094238, 10.55108642578125, 10.444369316101074, 10.339734077453613, 10.237374305725098, 10.187108993530273, 10.088534355163574, 9.992751121520996, 9.899985313415527, 9.854808807373047, 9.76701831817627, 9.682860374450684, 9.642226219177246, 9.56401538848877, 9.526510238647461, 9.454876899719238, 9.420822143554688, 9.356430053710938, 9.326166152954102, 9.29724407196045, 9.24357795715332, 9.218915939331055, 9.195752143859863, 9.154088020324707, 9.13566780090332, 1.310181975364685, 1.2410608530044556, 1.1974314451217651, 1.1410187482833862, 1.11784029006958, 1.0968273878097534, 1.0824755430221558, 1.0686348676681519, 1.0424387454986572, 1.0321404933929443, 1.0244622230529785, 1.0170994997024536, 1.0100492238998413, 1.0033082962036133, 0.9968730211257935, 0.9906423091888428, 0.9882991909980774, 0.9862038493156433, 0.9843555688858032, 0.9827532172203064, 0.9827808737754822, 0.9846906661987305, 0.9868459701538086, 0.9918953776359558, 0.9947900176048279, 0.9984793663024902, 1.0052378177642822, 1.01229989528656, 1.0196688175201416, 1.0273470878601074, 1.0457245111465454, 1.0589585304260254, 1.0726786851882935, 1.086896538734436, 1.101623773574829, 1.1468082666397095, 1.1716538667678833, 1.205284595489502, 1.2506428956985474, 1.3308942317962646, 10.815502166748047, 10.912272453308105, 11.011664390563965, 11.218291282653809, 11.325514793395996, 11.435338020324707, 11.66275691986084, 11.780338287353516, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
                  , dtype=float)

# Sugerencia defensiva (opcional)
assert ranges.ndim == 1 and len(ranges) == 360, f"Esperaba 360 lecturas, tengo {ranges.shape}"

# -------------------------------------------------------

while(True):
        t0 = time.time()

        # 1) IMU SIMULADO SIN ROS
        roll, pitch, accel_std, accel_std2, low_flag = imu_step(
            imu_state, dt,
            terrain_on=terrain_on,   # fijo por escenario
            params=imu_params        # fijo por escenario
        )
        std_dev_accel_z = accel_std

        # LIDAR
        # Esto debe ejecutarse cada 10HZ:
        # ranges es el arreglo de 360 datos del lidar, en metros, angle min el primer angulo que escanea, angle increment cada cuanto escanea
        # El 0 del lidar debe estar frente al robot.
        activaciones_totales = lidar_callback_python(ranges,angle_min,angle_increment) # Llamar cada 10 HZ
    
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


        R = areaBoundingBoxR/500
        #R = 2 #Descomentar para probar inclinacion
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

        #ang_s = 90 #Descomentar para probar terreno inclinado

        # ------IMPLEMENTACIÒN MÒDULO IMU ----------

        z[0, 1] = z[0, 0] + (dt / tau) * (-z[0, 0] + (A * max(0, (std_dev_accel_z - Usigma_az ))**2) / (SigmaIMU**2 + (-z[0,0] + std_dev_accel_z - Usigma_az )**2))
        z[1, 1] = z[1, 0] + (dt / tau) * (-z[1, 0] + (A * max(0, (pitch - Upitch ))**2) / (SigmaIMU**2 + (-z[1,0] + pitch - Upitch )**2))
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
        
        # print("R: ", str(R))
        print("G: ", str(G))
        # print("B: ", str(B)) 

        # print(
        #                 f"GpeR: {Gpe[0,1]}\n"
        #                 f"GpeG: {Gpe[1,1]}\n"
        #                 f"GpeB: {Gpe[2,1]}\n"
        #                 f"ang_p: {ang_p}\n"
        #                 f"ang_s: {ang_s}\n"
        #                 f"3: {z[3,1]}\n"
        #                 f"4: {z[4,1]}\n"
        #                 f"5: {z[5,1]}\n"
        #                 f"6: {z[6,1]}\n"
        #                 f"7: {z[7,1]}\n"
        #                 f"8: {z[8,1]}\n"
        #                 f"9: {z[9,1]}\n"
        #                 f"10: {z[10,1]}\n"
        #                 f"11: {z[11,1]}\n"
        #                 f"12: {z[12,1]}\n"
        #                 f"13: {z[13,1]}\n"
        #                 f"14: {z[14,1]}\n"
        #                 f"15: {z[15,1]}\n"
        #                 f"16: {z[16,1]}\n"
        #                 f"17: {z[17,1]}\n"
        #                 f"0: {z[0,1]}\n"
        #                 f"1: {z[1,1]}\n"
        #                 f"2: {z[2,1]}\n"
        #                 f"roll: {roll}\n"
        #                 f"pitch: {pitch}\n"
        #                 f"STD total: {accel_std:.3f}"
        #                 )
        
        # print("cmd_ang: ", str(cmd_ang))
        # print("cmd_lineal: ", str(cmd_lineal))
        # print("cmd_lateral: ", str(cmd_lateral))

        print("lidar frente: ", str(lidar[0,0]))
        print("lidar atras: ", str(lidar[1,0]))
        print("lidar izquierda: ", str(lidar[2,0]))
        print("lidar derecha:", str(lidar[3,0]))
        print("lidar 4:", str(lidar[4,0]))

        # Imprimir los 16 valores de Response
        for i, val in enumerate(Response[:, 0]):
            print(f"Response {i}: {val}")

        # Imprimir los 16 valores de Aux
        for i, val in enumerate(Aux[:, 0]):
            print(f"Aux {i}: {val}")

        cmd_ang = limit(cmd_ang, 1)
        cmd_lineal = limit(cmd_lineal, 5)
        cmd_lateral = limit(cmd_lateral, 5)

    #------------------------- TERRAIN CHANGER -----------------------------

        if accel_std > Usigma_az and not terrainchanger:
        #if (accel_std > Usigma_az or low_accel_flag)and not terrainchanger:  # Descomentar si el robot se queda atascado mucho
            print("Terreno rocoso detectado 🚧")
            terrainchanger = True
            std_dev_accel_z = 6
            terrain_timer = time.time()  # Guardar momento de activación

        # Si está activo, verificar si pasaron 8 segundos
        if terrainchanger:
            elapsed = time.time() - terrain_timer
            if elapsed < 16:
                print("Terreno rocoso detectado 🚧")
                std_dev_accel_z = 6
            else:
                terrainchanger = False  # Volver a estado normal
                print("Terreno liso 🛣️")
                std_dev_accel_z = 0

        # Si no está activo y no hay vibración, mensaje normal
        elif not terrainchanger:
            std_dev_accel_z = 0
            print("Terreno liso 🛣️")

        if pitch > Upitch: print("Terreno inclinado")
        else: print("Terreno NO inclinado")

        #------------------------- P U B L I C A C I O N --------------------------------------#
        
        #publicarGPE(Gpe[0,1], Gpe[1,1], Gpe[2,1])
        
        # -------------------- PUBLICACIÓN (una sola vez por ciclo) --------------------
        
        if MOVEMENT:
            # 1) calcular magnitudes filtradas por umbral
            ang = cmd_ang     if abs(cmd_ang)     > epsilem else 0.0
            lat = cmd_lateral if abs(cmd_lateral) > epsilem else 0.0
            lin = cmd_lineal  if abs(cmd_lineal)  > epsilem else 0.0

            # 2) resolver prioridad y construir UN solo Twist
            tw_ang, tw_lat, tw_lin = 0.0, 0.0, 0.0

            if z[17,1] > 0.25:
                # STOP: todo a cero
                print("Stop")
            elif ang != 0.0:
                # prioridad 1: giro
                tw_ang = ang
                print("Giro Izquierda" if ang > 0 else "Giro Derecha")
            elif lat != 0.0 and z[16,1] < 0.5:
                # prioridad 2: desplazamiento lateral
                tw_lat = lat
                print("Desp. Izquierda" if lat > 0 else "Desp. Derecha")
            elif lin != 0.0:
                # prioridad 3: avance/retroceso
                tw_lin = lin
                print("Avanza" if lin > 0 else "Retrocede")
            else:
                # sin comando significativo → queda todo en 0
                pass

            # 3) publicar una sola vez
           # publish_twist(linear_x=tw_lin, linear_y=tw_lat, angular_z=tw_ang)
            # ------------------------------------------------------------------------------

            # modos
            if z[15,1] > 0.5:
                #publish_mode('C'); 
                print("Cuadrupedo")
            elif z[16,1] > 0.5:
                #publish_mode('H'); 
                print("Móvil H")
            elif z[14,1] > 0.5:
                #publish_mode('X'); 
                print("Móvil X")

            #publish_data()
            #publish_imu()
        time.sleep(max(0.0, period - (time.time() - t0)))
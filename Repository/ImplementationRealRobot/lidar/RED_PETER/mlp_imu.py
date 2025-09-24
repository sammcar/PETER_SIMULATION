# mlp_imu_module.py
import time
import serial
import numpy as np
import torch
import torch.nn as nn
import joblib
from typing import Optional, Tuple, Iterator, List

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

# Cargar modelo y scaler (tal cual el script original)
model = MLP(INPUT_SIZE, NUM_CLASSES)
model.load_state_dict(torch.load("/home/sebas/Desktop/Peter_RED_Paper/IMU/modelo_mlp.pth",
                                 map_location="cpu"))
model.eval()
scaler = joblib.load("IMU/escalador.pkl")

# ----------------- Lógica MLP -----------------
ventana: List[List[float]] = []  # acumula 15 filas de 9 features

def linea_a_features(tokens: List[str]) -> List[float]:
    """
    tokens: lista de strings con al menos:
      ax, ay, az, gx, gy, gz, direccion, modo, [stuck? ...]
    Devuelve lista de 10 valores: (9 features + stuck al final)
      features (9): ax, ay, az, gx, gy, bit0, bit1, bit2, modo
      stuck (1)   : float (0/1)
    """
    ax = float(tokens[0]); ay = float(tokens[1]); az = float(tokens[2])
    gx = float(tokens[3]); gy = float(tokens[4])
    # gz = float(tokens[5])  # NO lo usamos en la MLP de 9 features

    direccion = int(float(tokens[6]))  # por si llega como "1.00"
    modo      = float(tokens[7])
    # stuck opcional; si no viene, usa 0.0
    stuck     = float(tokens[8]) if len(tokens) > 8 else 0.0

    # mapear y codificar 3 bits
    direc_map = MAPA_DIR.get(direccion, -1)
    if direc_map < 0:
        bit0 = bit1 = bit2 = 0
    else:
        bits = format(direc_map, "03b")
        bit0, bit1, bit2 = int(bits[0]), int(bits[1]), int(bits[2])

    # print("Stuck: ", stuck)  # (si lo quieres ver)
    return [ax, ay, az, gx, gy, bit0, bit1, bit2, modo, stuck]

def test_mlp_con_linea(linea_tokens: List[str]) -> Optional[Tuple[np.ndarray, float]]:
    """
    Agrega una fila (de tokens). Cuando se juntan 15 filas:
      - escala
      - infiere con la MLP
      - devuelve (probs [Plano, Rampa], stuck_ultimo)
    Si aún no se completa la ventana, devuelve None.
    """
    global ventana
    feats = linea_a_features(linea_tokens)     # 10 valores: 9 features + stuck
    feats9, stuck = feats[:-1], float(feats[-1])

    ventana.append(feats9)                     # solo 9 features van al modelo
    if len(ventana) < VENTANA_TAM:
        return None

    entrada = np.array(ventana, dtype=np.float32).flatten()  # (135,)
    ventana.clear()

    # Escalar → tensor
    entrada_escalada = scaler.transform([entrada])           # (1,135)
    X = torch.tensor(entrada_escalada, dtype=torch.float32)  # (1,135)

    with torch.no_grad():
        logits = model(X)
        probs  = torch.softmax(logits, dim=1)[0].cpu().numpy()

    # Devuelve predicción + el valor de stuck (float)
    return probs, stuck

# ----------------- Serie -----------------
def iter_resultados_serie(
    port: str = SERIAL_PORT,
    baud: int = BAUD_RATE,
    end_token: str = "--END--",
    timeout: float = 1.0
) -> Iterator[Tuple[np.ndarray, float]]:
    """
    Generador: lee del puerto serie y, cada vez que se completa la ventana,
    YIELD (probs [Plano, Rampa], stuck_ultimo).
    """
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.2)
    except serial.SerialException as e:
        raise RuntimeError(f"No se pudo abrir el puerto {port}: {e}")

    buffer_lines: List[str] = []
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == end_token:
                # procesar paquete
                if buffer_lines:
                    for raw in buffer_lines:
                        try:
                            tokens = raw.split(",")
                            if len(tokens) < 8:
                                continue  # línea incompleta
                            out = test_mlp_con_linea(tokens)
                            if out is not None:
                                yield out  # (probs, stuck)
                        except Exception:
                            # omite líneas inválidas, continúa
                            pass
                buffer_lines = []
            else:
                buffer_lines.append(line)

    finally:
        ser.close()

__all__ = ["MLP", "test_mlp_con_linea", "iter_resultados_serie",
           "SERIAL_PORT", "BAUD_RATE", "VENTANA_TAM", "INPUT_SIZE", "NUM_CLASSES"]

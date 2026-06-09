#!/usr/bin/env python3
"""
camera_node.py — Recepción de cámara + detección de color + publicación ZMQ.

Recibe frames JPEG de la Pi por TCP, detecta objetos rojo/azul usando los
filtros guardados en color_config.json, y publica los bounding boxes al
broker ZMQ para que los consuma neural_recorder.py y la red neuronal.

Mensaje publicado (Float32MultiArray, igual que el nodo ROS2 original):
  data[0] = x mapeado (0–180), basado en centroide_x / ancho_frame * 180 (invertido)
  data[1] = área en píxeles (w * h)
  [0.0, 0.0] cuando no hay detección válida

Uso:
  python camera_node.py [--show] [--config ruta/color_config.json]

  --show    muestra ventana de debug con bboxes dibujados
  --config  ruta al archivo de configuración (por defecto: color_config.json
            en la misma carpeta que este script)
"""

import argparse
import json
import os
import queue
import socket
import struct
import sys
import threading

import cv2
import numpy as np

# Agregar el directorio de zmq_metrics al path para importar transport
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, spin, Float32MultiArray
from topics import TOPIC_BBOX_RED, TOPIC_BBOX_BLUE

# ── Defaults ──────────────────────────────────────────────────────────────────
PI_HOST      = '0.0.0.0'
PI_PORT      = 8080
CONFIG_PATH  = os.path.join(os.path.dirname(__file__), 'color_config.json')
AREA_MAX     = 300000

FALLBACK_CFG = {
    'red': {
        'lower1': [0,   65,  70],
        'upper1': [10, 255, 215],
        'lower2': [160, 65,  70],
        'upper2': [180, 255, 215],
        'use_range2': 1,
    },
    'blue': {
        'lower': [80,   0,  40],
        'upper': [179, 255, 255],
    },
    'area_min': 500,
    'area_max': 300000,
}


# ── Config ────────────────────────────────────────────────────────────────────

def cargar_config(path):
    if os.path.exists(path):
        with open(path) as f:
            cfg = json.load(f)
        print(f'[camera_node] Config cargado: {path}')
        return cfg
    print(f'[camera_node] AVISO: {path} no encontrado, usando valores por defecto.')
    return json.loads(json.dumps(FALLBACK_CFG))


# ── Recepción TCP ─────────────────────────────────────────────────────────────

def recibir_frame(conn):
    raw_size = b''
    while len(raw_size) < 4:
        chunk = conn.recv(4 - len(raw_size))
        if not chunk:
            return None
        raw_size += chunk
    total_size = struct.unpack('!I', raw_size)[0]
    data = b''
    while len(data) < total_size:
        chunk = conn.recv(min(4096, total_size - len(data)))
        if not chunk:
            return None
        data += chunk
    np_arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def hilo_recepcion(conn, frame_queue):
    """Lee frames del socket y los pone en una queue (descarta si llena)."""
    while True:
        frame = recibir_frame(conn)
        if frame is None:
            frame_queue.put(None)   # señal de cierre
            break
        if not frame_queue.full():
            frame_queue.put(frame)


# ── Detección de color ────────────────────────────────────────────────────────

def _morph(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.erode(mask,  kernel, iterations=1)
    mask   = cv2.dilate(mask, kernel, iterations=2)
    return mask


def detectar_rojo(frame_hsv, red_cfg, area_min, area_max):
    lower1 = np.array(red_cfg['lower1'], dtype=np.uint8)
    upper1 = np.array(red_cfg['upper1'], dtype=np.uint8)
    mask   = cv2.inRange(frame_hsv, lower1, upper1)

    if red_cfg.get('use_range2', 0):
        lower2 = np.array(red_cfg['lower2'], dtype=np.uint8)
        upper2 = np.array(red_cfg['upper2'], dtype=np.uint8)
        mask   = cv2.bitwise_or(mask, cv2.inRange(frame_hsv, lower2, upper2))

    mask = _morph(mask)
    return _mejor_contorno(mask, area_min, area_max)


def detectar_azul(frame_hsv, blue_cfg, area_min, area_max):
    lower = np.array(blue_cfg['lower'], dtype=np.uint8)
    upper = np.array(blue_cfg['upper'], dtype=np.uint8)
    mask  = _morph(cv2.inRange(frame_hsv, lower, upper))
    return _mejor_contorno(mask, area_min, area_max)


def _mejor_contorno(mask, area_min, area_max):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    validos = [
        c for c in cnts
        if area_min <= cv2.contourArea(c) <= area_max
        and 30 <= cv2.boundingRect(c)[3] <= 500   # filtro de altura
    ]
    return max(validos, key=cv2.contourArea) if validos else None


# ── Cálculo del mensaje (igual que el camera_node original de la simulación) ──

def contorno_a_mensaje(contorno, frame_w):
    """Devuelve [x_mapeado, area] o [0.0, 0.0] si contorno es None."""
    if contorno is None:
        return [0.0, 0.0]
    x, y, w, h = cv2.boundingRect(contorno)
    cx   = x + w // 2
    area = float(w * h)
    x_mapped = 180.0 - (cx / (frame_w - 1)) * 180.0
    return [x_mapped, area]


# ── Nodo ZMQ ──────────────────────────────────────────────────────────────────

class CameraNode(Node):

    def __init__(self, cfg, show_debug=False):
        super().__init__('camera_node')
        self.cfg        = cfg
        self.show_debug = show_debug
        self.area_min   = cfg.get('area_min', 500)
        self.area_max   = cfg.get('area_max', AREA_MAX)

        self._pub_red  = self.create_publisher(Float32MultiArray, TOPIC_BBOX_RED,  10)
        self._pub_blue = self.create_publisher(Float32MultiArray, TOPIC_BBOX_BLUE, 10)

        self.get_logger().info(
            f'[camera_node] Publicando en {TOPIC_BBOX_RED} y {TOPIC_BBOX_BLUE}'
        )

    def procesar_y_publicar(self, frame):
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_w   = frame.shape[1]

        cnt_r = detectar_rojo(frame_hsv, self.cfg['red'],  self.area_min, self.area_max)
        cnt_b = detectar_azul(frame_hsv, self.cfg['blue'], self.area_min, self.area_max)

        msg_r      = Float32MultiArray(contorno_a_mensaje(cnt_r, frame_w))
        msg_b      = Float32MultiArray(contorno_a_mensaje(cnt_b, frame_w))
        self._pub_red.publish(msg_r)
        self._pub_blue.publish(msg_b)

        if self.show_debug:
            _mostrar_debug(frame, cnt_r, cnt_b, msg_r.data, msg_b.data)

    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


# ── Debug visual ──────────────────────────────────────────────────────────────

def _mostrar_debug(frame, cnt_r, cnt_b, data_r, data_b):
    debug = frame.copy()
    if cnt_r is not None:
        x, y, w, h = cv2.boundingRect(cnt_r)
        cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(debug, f'R x={data_r[0]:.0f} A={data_r[1]:.0f}',
                    (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    if cnt_b is not None:
        x, y, w, h = cv2.boundingRect(cnt_b)
        cv2.rectangle(debug, (x, y), (x+w, y+h), (255, 100, 0), 2)
        cv2.putText(debug, f'B x={data_b[0]:.0f} A={data_b[1]:.0f}',
                    (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
    cv2.imshow('camera_node debug', debug)
    cv2.waitKey(1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Camera Node — detección + ZMQ')
    parser.add_argument('--show',   action='store_true', help='Mostrar ventana de debug')
    parser.add_argument('--config', default=CONFIG_PATH, help='Ruta a color_config.json')
    args = parser.parse_args()

    cfg  = cargar_config(args.config)
    node = CameraNode(cfg, show_debug=args.show)

    # Conectar al servidor TCP de la Pi
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((PI_HOST, PI_PORT))
    server.listen(1)
    print(f'[camera_node] Esperando conexión de la Pi en {PI_HOST}:{PI_PORT}...')
    conn, addr = server.accept()
    print(f'[camera_node] Conectado desde {addr}')

    frame_queue = queue.Queue(maxsize=2)
    recv_thread = threading.Thread(
        target=hilo_recepcion, args=(conn, frame_queue), daemon=True
    )
    recv_thread.start()

    frame_count = 0
    try:
        while True:
            frame = frame_queue.get(timeout=5.0)
            if frame is None:
                print('[camera_node] Conexión cerrada por la Pi.')
                break
            node.procesar_y_publicar(frame)
            frame_count += 1
            if frame_count % 30 == 0:
                print(f'[camera_node] Frames procesados: {frame_count}')
    except queue.Empty:
        print('[camera_node] Timeout esperando frame.')
    except KeyboardInterrupt:
        print('\n[camera_node] Detenido.')
    finally:
        conn.close()
        server.close()
        node.destroy_node()


if __name__ == '__main__':
    main()

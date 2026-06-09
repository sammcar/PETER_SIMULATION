#!/usr/bin/env python3
"""
color_calibrator.py — Calibración de filtros HSV en una sola ventana.

Todo en una ventana:
  • Vista previa (frame original + máscara)
  • Botones ROJO / AZUL para seleccionar qué filtro editar
  • Sliders HSV (dos rangos para rojo, uno para azul)
  • Botón GUARDAR

Controles de teclado:
  r / b  → cambiar color activo
  s      → guardar   (igual que el botón)
  q/ESC  → salir
"""

import json
import os
import socket
import struct
import time

import cv2
import numpy as np

# ── Red ───────────────────────────────────────────────────────────────────────
PI_HOST     = '0.0.0.0'
PI_PORT     = 8080
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'color_config.json')

DEFAULTS = {
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

# ── Layout ────────────────────────────────────────────────────────────────────
WIN_W   = 1100
PREV_H  = 300        # altura zona de imágenes
PANEL_H = 320        # altura zona de controles
WIN_H   = PREV_H + PANEL_H

IMG_W   = WIN_W // 2  # 550  cada preview

# Fila de botones de color + guardar
BTN_ROW_Y = PREV_H + 28

# Zona de sliders: dos columnas
#   Columna izquierda  (rango principal / rango único azul)
COL1_LABEL_X  = 12
COL1_TRACK_X0 = 105
COL1_TRACK_X1 = 510
COL1_VAL_X    = 518

#   Columna derecha (rango 2, solo cuando color == rojo)
COL2_LABEL_X  = 590
COL2_TRACK_X0 = 680
COL2_TRACK_X1 = 1060
COL2_VAL_X    = 1068

SLIDER_ROW_START = PREV_H + 110   # y del primer slider
SLIDER_ROW_STEP  = 33

# ── Paleta ────────────────────────────────────────────────────────────────────
BG       = (30,  30,  30)
PANEL_BG = (22,  22,  22)
TRACK_C  = (70,  70,  70)
FILL_C   = (200, 150,  0)
THUMB_C  = (0,  200, 255)
TXT_C    = (190, 190, 190)
TXT_W    = (255, 255, 255)
SEP_C    = (55,  55,  55)

BTN_RED_OFF  = (40,  40, 120)
BTN_RED_ON   = (0,   40, 220)
BTN_BLUE_OFF = (100,  40,  0)
BTN_BLUE_ON  = (210,  80,  0)
BTN_SAVE_C   = (40, 120,  40)
BTN_SAVE_FL  = (60, 200,  60)   # flash al guardar
TOG_ON       = (30, 150,  60)
TOG_OFF      = (55,  55,  55)


# ═══════════════════════════════════════════════════════════════════════════════
# Clases UI
# ═══════════════════════════════════════════════════════════════════════════════

class Slider:
    """Slider horizontal dibujado en el canvas."""

    def __init__(self, label, vmin, vmax, value, tx0, tx1, ty, label_x, val_x):
        self.label   = label
        self.vmin    = int(vmin)
        self.vmax    = int(vmax)
        self.value   = int(value)
        self.tx0     = tx0   # inicio del track
        self.tx1     = tx1   # fin del track
        self.ty      = ty    # y centro
        self.label_x = label_x
        self.val_x   = val_x
        self.dragging = False
        self.R        = 8    # radio del thumb

    @property
    def thumb_x(self):
        t = (self.value - self.vmin) / max(1, self.vmax - self.vmin)
        return int(self.tx0 + t * (self.tx1 - self.tx0))

    def hit(self, px, py):
        return abs(px - self.thumb_x) <= self.R + 4 and abs(py - self.ty) <= self.R + 4

    def update_from_x(self, px):
        t = (px - self.tx0) / max(1, self.tx1 - self.tx0)
        t = max(0.0, min(1.0, t))
        self.value = int(round(self.vmin + t * (self.vmax - self.vmin)))

    def draw(self, canvas, active=True):
        alpha = 1.0 if active else 0.4
        tx = self.thumb_x

        # track fondo
        cv2.line(canvas, (self.tx0, self.ty), (self.tx1, self.ty), TRACK_C, 4)
        # track relleno
        fill_col = tuple(int(c * alpha) for c in FILL_C)
        cv2.line(canvas, (self.tx0, self.ty), (tx, self.ty), fill_col, 4)
        # thumb
        thumb_col = THUMB_C if active else (100, 100, 100)
        cv2.circle(canvas, (tx, self.ty), self.R, thumb_col, -1)
        cv2.circle(canvas, (tx, self.ty), self.R, (255, 255, 255) if active else (80,80,80), 1)
        # label
        txt_col = TXT_W if active else (100, 100, 100)
        cv2.putText(canvas, self.label,
                    (self.label_x, self.ty + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, txt_col, 1, cv2.LINE_AA)
        # valor
        cv2.putText(canvas, str(self.value),
                    (self.val_x, self.ty + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, txt_col, 1, cv2.LINE_AA)


class Button:
    """Botón rectangular dibujado en el canvas."""

    def __init__(self, label, x, y, w, h, color, color_active=None, toggle=False):
        self.label         = label
        self.x, self.y     = x, y
        self.w, self.h     = w, h
        self.color         = color
        self.color_active  = color_active or color
        self.toggle        = toggle
        self.active        = False   # estado ON/OFF (solo toggle)
        self._flash_until  = 0

    def hit(self, px, py):
        return self.x <= px <= self.x + self.w and self.y <= py <= self.y + self.h

    def flash(self, duration=0.6):
        self._flash_until = time.time() + duration

    def draw(self, canvas):
        now = time.time()
        if now < self._flash_until:
            col = BTN_SAVE_FL
        elif self.active:
            col = self.color_active
        else:
            col = self.color

        cv2.rectangle(canvas, (self.x, self.y),
                      (self.x + self.w, self.y + self.h), col, -1)
        cv2.rectangle(canvas, (self.x, self.y),
                      (self.x + self.w, self.y + self.h), (80, 80, 80), 1)

        # texto centrado
        (tw, th), _ = cv2.getTextSize(self.label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        tx = self.x + (self.w - tw) // 2
        ty = self.y + (self.h + th) // 2
        cv2.putText(canvas, self.label, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, TXT_W, 1, cv2.LINE_AA)


# ═══════════════════════════════════════════════════════════════════════════════
# Calibrador principal
# ═══════════════════════════════════════════════════════════════════════════════

class Calibrador:

    def __init__(self, cfg):
        self.cfg   = cfg
        self.color = 'r'         # 'r' | 'b'
        self._drag_slider = None
        self._saved_msg   = ''
        self._saved_until = 0

        # ── Botones ────────────────────────────────────────────────────────
        self.btn_rojo  = Button('ROJO',    20,  BTN_ROW_Y, 100, 34, BTN_RED_OFF,  BTN_RED_ON,  toggle=True)
        self.btn_azul  = Button('AZUL',   135,  BTN_ROW_Y, 100, 34, BTN_BLUE_OFF, BTN_BLUE_ON, toggle=True)
        self.btn_save  = Button('GUARDAR', WIN_W - 130, BTN_ROW_Y, 115, 34, BTN_SAVE_C)
        self.btn_r2    = Button('R2: OFF', COL2_TRACK_X0, PREV_H + 75, 100, 26, TOG_OFF, TOG_ON, toggle=True)

        self.btn_rojo.active = True

        # ── Sliders rojo ───────────────────────────────────────────────────
        r = cfg['red']
        rows = [SLIDER_ROW_START + i * SLIDER_ROW_STEP for i in range(6)]

        self.sliders_red_c1 = [
            Slider('H1 min', 0, 30,  r['lower1'][0], COL1_TRACK_X0, COL1_TRACK_X1, rows[0], COL1_LABEL_X, COL1_VAL_X),
            Slider('H1 max', 0, 30,  r['upper1'][0], COL1_TRACK_X0, COL1_TRACK_X1, rows[1], COL1_LABEL_X, COL1_VAL_X),
            Slider('S  min', 0, 255, r['lower1'][1], COL1_TRACK_X0, COL1_TRACK_X1, rows[2], COL1_LABEL_X, COL1_VAL_X),
            Slider('S  max', 0, 255, r['upper1'][1], COL1_TRACK_X0, COL1_TRACK_X1, rows[3], COL1_LABEL_X, COL1_VAL_X),
            Slider('V  min', 0, 255, r['lower1'][2], COL1_TRACK_X0, COL1_TRACK_X1, rows[4], COL1_LABEL_X, COL1_VAL_X),
            Slider('V  max', 0, 255, r['upper1'][2], COL1_TRACK_X0, COL1_TRACK_X1, rows[5], COL1_LABEL_X, COL1_VAL_X),
        ]
        self.sliders_red_c2 = [
            Slider('H2 min', 150, 180, r['lower2'][0], COL2_TRACK_X0, COL2_TRACK_X1, rows[0], COL2_LABEL_X, COL2_VAL_X),
            Slider('H2 max', 150, 180, r['upper2'][0], COL2_TRACK_X0, COL2_TRACK_X1, rows[1], COL2_LABEL_X, COL2_VAL_X),
        ]
        self.btn_r2.active = bool(r.get('use_range2', 1))
        self._sync_r2_label()

        # ── Sliders azul ───────────────────────────────────────────────────
        b = cfg['blue']
        self.sliders_blue = [
            Slider('H  min', 0, 179, b['lower'][0], COL1_TRACK_X0, COL1_TRACK_X1, rows[0], COL1_LABEL_X, COL1_VAL_X),
            Slider('H  max', 0, 179, b['upper'][0], COL1_TRACK_X0, COL1_TRACK_X1, rows[1], COL1_LABEL_X, COL1_VAL_X),
            Slider('S  min', 0, 255, b['lower'][1], COL1_TRACK_X0, COL1_TRACK_X1, rows[2], COL1_LABEL_X, COL1_VAL_X),
            Slider('S  max', 0, 255, b['upper'][1], COL1_TRACK_X0, COL1_TRACK_X1, rows[3], COL1_LABEL_X, COL1_VAL_X),
            Slider('V  min', 0, 255, b['lower'][2], COL1_TRACK_X0, COL1_TRACK_X1, rows[4], COL1_LABEL_X, COL1_VAL_X),
            Slider('V  max', 0, 255, b['upper'][2], COL1_TRACK_X0, COL1_TRACK_X1, rows[5], COL1_LABEL_X, COL1_VAL_X),
        ]

    # ── Helpers de estado ──────────────────────────────────────────────────

    def _sync_r2_label(self):
        self.btn_r2.label = 'R2: ON ' if self.btn_r2.active else 'R2: OFF'
        self.btn_r2.color = TOG_ON if self.btn_r2.active else TOG_OFF

    def _active_sliders(self):
        """Sliders visibles según color activo."""
        if self.color == 'r':
            return self.sliders_red_c1 + self.sliders_red_c2
        return self.sliders_blue

    def _read_sliders_to_cfg(self):
        """Vuelca los valores actuales de los sliders al dict cfg."""
        s = self.sliders_red_c1
        self.cfg['red']['lower1'] = [s[0].value, s[2].value, s[4].value]
        self.cfg['red']['upper1'] = [s[1].value, s[3].value, s[5].value]
        s2 = self.sliders_red_c2
        self.cfg['red']['lower2'] = [s2[0].value, s[2].value, s[4].value]
        self.cfg['red']['upper2'] = [s2[1].value, s[3].value, s[5].value]
        self.cfg['red']['use_range2'] = int(self.btn_r2.active)

        b = self.sliders_blue
        self.cfg['blue']['lower'] = [b[0].value, b[2].value, b[4].value]
        self.cfg['blue']['upper'] = [b[1].value, b[3].value, b[5].value]

    # ── Detección ──────────────────────────────────────────────────────────

    def _mask_rojo(self, hsv):
        s = self.sliders_red_c1
        lower1 = np.array([s[0].value, s[2].value, s[4].value], dtype=np.uint8)
        upper1 = np.array([s[1].value, s[3].value, s[5].value], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1)
        if self.btn_r2.active:
            s2 = self.sliders_red_c2
            lower2 = np.array([s2[0].value, s[2].value, s[4].value], dtype=np.uint8)
            upper2 = np.array([s2[1].value, s[3].value, s[5].value], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))
        return _morph(mask)

    def _mask_azul(self, hsv):
        b = self.sliders_blue
        lower = np.array([b[0].value, b[2].value, b[4].value], dtype=np.uint8)
        upper = np.array([b[1].value, b[3].value, b[5].value], dtype=np.uint8)
        return _morph(cv2.inRange(hsv, lower, upper))

    def get_masks(self, frame):
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_r = self._mask_rojo(hsv)
        mask_b = self._mask_azul(hsv)
        return mask_r, mask_b

    # ── Renderizado ────────────────────────────────────────────────────────

    def render(self, frame):
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
        canvas[:] = BG

        # ── Panel fondo ────────────────────────────────────────────────────
        canvas[PREV_H:, :] = PANEL_BG
        cv2.line(canvas, (0, PREV_H), (WIN_W, PREV_H), SEP_C, 2)

        # ── Imágenes ───────────────────────────────────────────────────────
        mask_r, mask_b = self.get_masks(frame)

        # Izquierda: frame original con contornos de ambos colores
        left = _resize(frame, IMG_W, PREV_H)
        _draw_contours(left, _resize_mask(mask_r, IMG_W, PREV_H), (0, 60, 220), 'R')
        _draw_contours(left, _resize_mask(mask_b, IMG_W, PREV_H), (200, 80,   0), 'B')

        # Derecha: máscara del color activo (+ overlay de color)
        active_mask  = mask_r if self.color == 'r' else mask_b
        overlay_bgr  = (0, 60, 220) if self.color == 'r' else (200, 80, 0)
        right_base   = _resize(frame, IMG_W, PREV_H)
        right_mask   = _resize_mask(active_mask, IMG_W, PREV_H)
        right_color  = np.zeros_like(right_base)
        right_color[right_mask > 0] = overlay_bgr
        right = cv2.addWeighted(right_base, 0.45, right_color, 0.55, 0)

        canvas[:PREV_H, :IMG_W]      = left
        canvas[:PREV_H, IMG_W:WIN_W] = right

        # Etiquetas de imagen
        _label(canvas, 'Original',                             8,  18)
        _label(canvas, f'Mascara  {"ROJO" if self.color == "r" else "AZUL"}',
               IMG_W + 8, 18)

        # ── Botones de color ───────────────────────────────────────────────
        self.btn_rojo.draw(canvas)
        self.btn_azul.draw(canvas)
        self.btn_save.draw(canvas)

        # ── Encabezados de columnas ────────────────────────────────────────
        hdr_y = PREV_H + 76
        cv2.putText(canvas, 'Rango principal',
                    (COL1_TRACK_X0, hdr_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TXT_C, 1, cv2.LINE_AA)
        if self.color == 'r':
            cv2.putText(canvas, 'Rango 2  (hue alto)',
                        (COL2_TRACK_X0, hdr_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, TXT_C, 1, cv2.LINE_AA)
            self.btn_r2.draw(canvas)
        else:
            cv2.putText(canvas, 'Rango unico (azul)',
                        (COL2_TRACK_X0, hdr_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1, cv2.LINE_AA)
            cv2.putText(canvas, 'S y V se comparten',
                        (COL2_TRACK_X0, hdr_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (70, 70, 70), 1, cv2.LINE_AA)

        # ── Separador vertical entre columnas ──────────────────────────────
        cv2.line(canvas, (575, PREV_H + 60), (575, WIN_H - 10), SEP_C, 1)

        # ── Sliders ────────────────────────────────────────────────────────
        if self.color == 'r':
            for sl in self.sliders_red_c1:
                sl.draw(canvas, active=True)
            r2_active = self.btn_r2.active
            for sl in self.sliders_red_c2:
                sl.draw(canvas, active=r2_active)
            # Indicar que S/V del rango 2 se heredan del rango 1
            note_y = SLIDER_ROW_START + 2 * SLIDER_ROW_STEP
            for i in range(2, 6):
                y = SLIDER_ROW_START + i * SLIDER_ROW_STEP
                cv2.putText(canvas, '← igual rango 1',
                            (COL2_TRACK_X0, y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (70, 70, 70), 1, cv2.LINE_AA)
        else:
            for sl in self.sliders_blue:
                sl.draw(canvas, active=True)

        # ── Mensaje de guardado ────────────────────────────────────────────
        if time.time() < self._saved_until:
            cv2.putText(canvas, self._saved_msg,
                        (BTN_ROW_Y, BTN_ROW_Y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (80, 220, 80), 1, cv2.LINE_AA)

        # ── Hint teclas ────────────────────────────────────────────────────
        cv2.putText(canvas, 'r/b: cambiar color   s: guardar   q: salir',
                    (20, WIN_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (70, 70, 70), 1, cv2.LINE_AA)

        return canvas

    # ── Mouse ──────────────────────────────────────────────────────────────

    def on_mouse(self, event, px, py, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Botones de color
            if self.btn_rojo.hit(px, py):
                self.color = 'r'
                self.btn_rojo.active = True
                self.btn_azul.active = False
                return
            if self.btn_azul.hit(px, py):
                self.color = 'b'
                self.btn_rojo.active = False
                self.btn_azul.active = True
                return
            # Botón guardar
            if self.btn_save.hit(px, py):
                self._guardar()
                return
            # Toggle R2
            if self.color == 'r' and self.btn_r2.hit(px, py):
                self.btn_r2.active = not self.btn_r2.active
                self._sync_r2_label()
                return
            # Sliders — iniciar drag
            for sl in self._active_sliders():
                if sl.hit(px, py):
                    self._drag_slider = sl
                    sl.dragging = True
                    sl.update_from_x(px)
                    break

        elif event == cv2.EVENT_MOUSEMOVE and self._drag_slider:
            self._drag_slider.update_from_x(px)

        elif event == cv2.EVENT_LBUTTONUP and self._drag_slider:
            self._drag_slider.dragging = False
            self._drag_slider = None

    # ── Guardar ────────────────────────────────────────────────────────────

    def _guardar(self):
        self._read_sliders_to_cfg()

        def to_native(obj):
            if isinstance(obj, dict):  return {k: to_native(v) for k, v in obj.items()}
            if isinstance(obj, list):  return [to_native(v) for v in obj]
            if isinstance(obj, (np.integer, np.int_)): return int(obj)
            return obj

        with open(CONFIG_PATH, 'w') as f:
            json.dump(to_native(self.cfg), f, indent=2)

        self._saved_msg   = f'Guardado en {os.path.basename(CONFIG_PATH)}'
        self._saved_until = time.time() + 2.0
        self.btn_save.flash(0.5)
        print(f'[calibrador] Config guardado: {CONFIG_PATH}')

    def set_color(self, c):
        self.color = c
        self.btn_rojo.active = (c == 'r')
        self.btn_azul.active = (c == 'b')


# ── Helpers de imagen ─────────────────────────────────────────────────────────

def _morph(mask):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.dilate(cv2.erode(mask, k, iterations=1), k, iterations=2)


def _resize(img, w, h):
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


def _resize_mask(mask, w, h):
    return cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)


def _draw_contours(img, mask, color_bgr, label):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return
    best = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(best) < 300:
        return
    x, y, w, h = cv2.boundingRect(best)
    cv2.rectangle(img, (x, y), (x+w, y+h), color_bgr, 2)
    area = w * h
    cv2.putText(img, f'{label}  A={area}', (x, max(y-6, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, color_bgr, 1, cv2.LINE_AA)


def _label(canvas, txt, x, y):
    cv2.putText(canvas, txt, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (160, 160, 160), 1, cv2.LINE_AA)


# ── Red TCP ───────────────────────────────────────────────────────────────────

def recibir_frame(conn):
    raw = b''
    while len(raw) < 4:
        chunk = conn.recv(4 - len(raw))
        if not chunk: return None
        raw += chunk
    total = struct.unpack('!I', raw)[0]
    data = b''
    while len(data) < total:
        chunk = conn.recv(min(4096, total - len(data)))
        if not chunk: return None
        data += chunk
    arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def cargar_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
        print(f'[calibrador] Config cargado: {CONFIG_PATH}')
        return cfg
    print('[calibrador] Usando valores por defecto.')
    return json.loads(json.dumps(DEFAULTS))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    cfg  = cargar_config()
    cal  = Calibrador(cfg)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((PI_HOST, PI_PORT))
    server.listen(1)
    print(f'[calibrador] Esperando conexión en {PI_HOST}:{PI_PORT}...')
    conn, addr = server.accept()
    print(f'[calibrador] Conectado desde {addr}')

    WINDOW = 'Calibracion HSV'
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, cal.on_mouse)

    last_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    try:
        while True:
            frame = recibir_frame(conn)
            if frame is None:
                print('[calibrador] Conexión cerrada.')
                break
            last_frame = frame

            canvas = cal.render(last_frame)
            cv2.imshow(WINDOW, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('r'):
                cal.set_color('r')
            elif key == ord('b'):
                cal.set_color('b')
            elif key == ord('s'):
                cal._guardar()

    except KeyboardInterrupt:
        pass
    finally:
        conn.close()
        server.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

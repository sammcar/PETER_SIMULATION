import numpy as np
import cv2
import mmap
import os
import time

# ================== SALIDA COMPARTIDA (DETECCIONES) ==================
DET_PATH  = "/dev/shm/det_vector"   # archivo compartido (OUT)
DET_ROWS  = 2                       # A y B
DET_COLS  = 3                       # [cx, area, class_id]
DET_DTYPE = np.int32
DET_SIZE  = DET_ROWS * DET_COLS * np.dtype(DET_DTYPE).itemsize

def open_det_shared():
    """Crea/trunca el archivo de salida y lo mapea para ESCRITURA."""
    fd = os.open(DET_PATH, os.O_RDWR | os.O_CREAT)
    os.ftruncate(fd, DET_SIZE)
    mm_out = mmap.mmap(fd, DET_SIZE, access=mmap.ACCESS_WRITE)
    return fd, mm_out

def det_array_from_mm(mm_out):
    """Devuelve una vista numpy sobre el mmap: shape (2,3), dtype int32."""
    return np.ndarray((DET_ROWS, DET_COLS), dtype=DET_DTYPE, buffer=mm_out)

# ================== CONFIGURA AQUÍ ==================
SHM_NAME       = "/dev/shm/shared_image"  # ruta del productor de imagen
IMAGE_WIDTH    = 640
IMAGE_HEIGHT   = 480
IMAGE_CHANNELS = 3

# ---- Valores iniciales (se pueden recalibrar con sliders) ----
# Filtro A (bbox ROJA)
LOWER_HSV_A_INIT = (135, 112, 78)
UPPER_HSV_A_INIT = (179, 255, 255)
OPEN_K_A_INIT    = 0
CLOSE_K_A_INIT   = 0
MIN_AREA_A_INIT  = 500

# Filtro B (bbox AZUL)
LOWER_HSV_B_INIT = (98, 155, 158)
UPPER_HSV_B_INIT = (113, 239, 201)
OPEN_K_B_INIT    = 2
CLOSE_K_B_INIT   = 2
MIN_AREA_B_INIT  = 800

# Preprocesado común
BLUR_ODD_INIT = 13                # 1 (sin blur), 3,5,7... impar
TARGET_FPS    = 30.0              # limita la UI

# Ventanas opcionales (también se puede alternar con tecla 'm')
SHOW_MASKS    = False
# ====================================================

FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS
PERIOD     = 1.0 / max(TARGET_FPS, 1e-6)

def open_shared():
    if not os.path.exists(SHM_NAME):
        raise FileNotFoundError(f"No existe {SHM_NAME}. ¿Está corriendo el productor?")
    f = open(SHM_NAME, "rb")  # mantener handler vivo
    mm = mmap.mmap(f.fileno(), FRAME_SIZE, access=mmap.ACCESS_READ)
    return f, mm

def read_frame(mm):
    mm.seek(0)
    data = mm.read(FRAME_SIZE)
    if len(data) != FRAME_SIZE:
        return None
    frame = np.frombuffer(data, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS))
    return frame

def make_mask(hsv, lower, upper, open_k=0, close_k=0):
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    if open_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if close_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def biggest_bbox(mask, min_area):
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]
    if not contours:
        return None
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(biggest)
    return (x, y, w, h)

def put_text(img, txt, xy=(10, 24), color=(255,255,255)):
    cv2.putText(img, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

def _odd_or_one(v: int) -> int:
    """Asegura impar >=1 para el blur gaussiano."""
    if v <= 1:
        return 1
    return v if (v % 2 == 1) else v + 1

def setup_calibration_ui():
    """Crea la ventana de calibración y todos los sliders."""
    cv2.namedWindow("Calibracion", cv2.WINDOW_NORMAL)

    # ----- Sliders comunes -----
    cv2.createTrackbar("BLUR (odd)", "Calibracion", BLUR_ODD_INIT, 31, lambda x: None)  # 0..31

    # ----- Sliders Filtro A -----
    cv2.createTrackbar("A H min", "Calibracion", LOWER_HSV_A_INIT[0], 179, lambda x: None)
    cv2.createTrackbar("A H max", "Calibracion", UPPER_HSV_A_INIT[0], 179, lambda x: None)
    cv2.createTrackbar("A S min", "Calibracion", LOWER_HSV_A_INIT[1], 255, lambda x: None)
    cv2.createTrackbar("A S max", "Calibracion", UPPER_HSV_A_INIT[1], 255, lambda x: None)
    cv2.createTrackbar("A V min", "Calibracion", LOWER_HSV_A_INIT[2], 255, lambda x: None)
    cv2.createTrackbar("A V max", "Calibracion", UPPER_HSV_A_INIT[2], 255, lambda x: None)
    cv2.createTrackbar("A OPEN k", "Calibracion", OPEN_K_A_INIT, 21, lambda x: None)
    cv2.createTrackbar("A CLOSE k","Calibracion", CLOSE_K_A_INIT, 21, lambda x: None)
    cv2.createTrackbar("A MIN area","Calibracion", MIN_AREA_A_INIT, 20000, lambda x: None)

    # ----- Sliders Filtro B -----
    cv2.createTrackbar("B H min", "Calibracion", LOWER_HSV_B_INIT[0], 179, lambda x: None)
    cv2.createTrackbar("B H max", "Calibracion", UPPER_HSV_B_INIT[0], 179, lambda x: None)
    cv2.createTrackbar("B S min", "Calibracion", LOWER_HSV_B_INIT[1], 255, lambda x: None)
    cv2.createTrackbar("B S max", "Calibracion", UPPER_HSV_B_INIT[1], 255, lambda x: None)
    cv2.createTrackbar("B V min", "Calibracion", LOWER_HSV_B_INIT[2], 255, lambda x: None)
    cv2.createTrackbar("B V max", "Calibracion", UPPER_HSV_B_INIT[2], 255, lambda x: None)
    cv2.createTrackbar("B OPEN k", "Calibracion", OPEN_K_B_INIT, 21, lambda x: None)
    cv2.createTrackbar("B CLOSE k","Calibracion", CLOSE_K_B_INIT, 21, lambda x: None)
    cv2.createTrackbar("B MIN area","Calibracion", MIN_AREA_B_INIT, 20000, lambda x: None)

def read_calibration_ui():
    """Lee los valores actuales de sliders y devuelve dict con parámetros A/B + blur."""
    blur_val = cv2.getTrackbarPos("BLUR (odd)", "Calibracion")
    blur_odd = _odd_or_one(blur_val)

    # --- A ---
    a_hmin = cv2.getTrackbarPos("A H min", "Calibracion")
    a_hmax = cv2.getTrackbarPos("A H max", "Calibracion")
    a_smin = cv2.getTrackbarPos("A S min", "Calibracion")
    a_smax = cv2.getTrackbarPos("A S max", "Calibracion")
    a_vmin = cv2.getTrackbarPos("A V min", "Calibracion")
    a_vmax = cv2.getTrackbarPos("A V max", "Calibracion")
    # garantizar coherencia
    if a_hmax < a_hmin: a_hmax = a_hmin
    if a_smax < a_smin: a_smax = a_smin
    if a_vmax < a_vmin: a_vmax = a_vmin
    a_open = cv2.getTrackbarPos("A OPEN k", "Calibracion")
    a_close = cv2.getTrackbarPos("A CLOSE k","Calibracion")
    a_area  = cv2.getTrackbarPos("A MIN area","Calibracion")

    # --- B ---
    b_hmin = cv2.getTrackbarPos("B H min", "Calibracion")
    b_hmax = cv2.getTrackbarPos("B H max", "Calibracion")
    b_smin = cv2.getTrackbarPos("B S min", "Calibracion")
    b_smax = cv2.getTrackbarPos("B S max", "Calibracion")
    b_vmin = cv2.getTrackbarPos("B V min", "Calibracion")
    b_vmax = cv2.getTrackbarPos("B V max", "Calibracion")
    if b_hmax < b_hmin: b_hmax = b_hmin
    if b_smax < b_smin: b_smax = b_smin
    if b_vmax < b_vmin: b_vmax = b_vmin
    b_open = cv2.getTrackbarPos("B OPEN k", "Calibracion")
    b_close= cv2.getTrackbarPos("B CLOSE k","Calibracion")
    b_area = cv2.getTrackbarPos("B MIN area","Calibracion")

    params = {
        "blur_odd": blur_odd,
        "A": {
            "lower": (a_hmin, a_smin, a_vmin),
            "upper": (a_hmax, a_smax, a_vmax),
            "open_k": a_open,
            "close_k": a_close,
            "min_area": max(0, a_area),
        },
        "B": {
            "lower": (b_hmin, b_smin, b_vmin),
            "upper": (b_hmax, b_smax, b_vmax),
            "open_k": b_open,
            "close_k": b_close,
            "min_area": max(0, b_area),
        }
    }
    return params

def main():
    global SHOW_MASKS

    # Ventanas de vista
    cv2.namedWindow("Original",  cv2.WINDOW_NORMAL)
    cv2.namedWindow("Filtrada",  cv2.WINDOW_NORMAL)
    if SHOW_MASKS:
        cv2.namedWindow("Mask A", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask B", cv2.WINDOW_NORMAL)

    # Panel de calibración
    setup_calibration_ui()

    try:
        # mmap de entrada (imagen) y salida (detecciones)
        fhandle, mm = open_shared()
        fd_out, mm_out = open_det_shared()
        det_arr = det_array_from_mm(mm_out)

        # Inicial: sin detecciones (cx=-1, area=-1), clases fijas 0 y 1
        det_arr[:] = np.array([[-1, -1, 0],
                               [-1, -1, 1]], dtype=DET_DTYPE)

    except Exception as e:
        print("❌ Error:", e)
        return

    t0, frames, fps = time.time(), 0, 0.0

    try:
        while True:
            loop_t = time.time()

            frame = read_frame(mm)
            if frame is None:
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                time.sleep(0.005)
                continue

            # Lee parámetros actuales del panel
            p = read_calibration_ui()
            BLUR_ODD    = p["blur_odd"]
            LOWER_HSV_A = p["A"]["lower"]
            UPPER_HSV_A = p["A"]["upper"]
            OPEN_K_A    = p["A"]["open_k"]
            CLOSE_K_A   = p["A"]["close_k"]
            MIN_AREA_A  = p["A"]["min_area"]

            LOWER_HSV_B = p["B"]["lower"]
            UPPER_HSV_B = p["B"]["upper"]
            OPEN_K_B    = p["B"]["open_k"]
            CLOSE_K_B   = p["B"]["close_k"]
            MIN_AREA_B  = p["B"]["min_area"]

            # Pre-blur y HSV una sola vez (más eficiente)
            proc = frame
            if BLUR_ODD > 1:
                k = _odd_or_one(BLUR_ODD)
                proc = cv2.GaussianBlur(proc, (k, k), 0)
            hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)

            # Máscaras A y B
            maskA = make_mask(hsv, LOWER_HSV_A, UPPER_HSV_A, OPEN_K_A, CLOSE_K_A)
            maskB = make_mask(hsv, LOWER_HSV_B, UPPER_HSV_B, OPEN_K_B, CLOSE_K_B)

            # Segmentación para vista "Filtrada"
            segA = cv2.bitwise_and(frame, frame, mask=maskA)
            segB = cv2.bitwise_and(frame, frame, mask=maskB)
            seg_combined = cv2.addWeighted(segA, 1.0, segB, 1.0, 0)

            # Bounding boxes independientes
            bboxA = biggest_bbox(maskA, MIN_AREA_A)
            bboxB = biggest_bbox(maskB, MIN_AREA_B)

            view_orig = frame.copy()
            view_seg  = seg_combined.copy()

            # Por defecto, marca “sin detección” en este frame (-1, -1)
            det_arr[0, :] = (-1, -1, 0)  # A
            det_arr[1, :] = (-1, -1, 1)  # B

            # ----- Dibuja y PUBLICA bbox A (ROJO)
            if bboxA is not None:
                x, y, w, h = bboxA
                cx = x + w // 2
                area = int(w * h)

                cv2.rectangle(view_orig, (x, y), (x+w, y+h), (0, 0, 255), 3)
                cv2.rectangle(view_seg,  (x, y), (x+w, y+h), (0, 0, 255), 3)
                cv2.putText(view_orig, f"A W={w} H={h}", (x, max(0, y-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                det_arr[0, 0] = cx
                det_arr[0, 1] = area
                det_arr[0, 2] = 0

            # ----- Dibuja y PUBLICA bbox B (AZUL)
            if bboxB is not None:
                x, y, w, h = bboxB
                cx = x + w // 2
                area = int(w * h)

                cv2.rectangle(view_orig, (x, y), (x+w, y+h), (255, 0, 0), 3)
                cv2.rectangle(view_seg,  (x, y), (x+w, y+h), (255, 0, 0), 3)
                cv2.putText(view_orig, f"B W={w} H={h}", (x, max(0, y-22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                det_arr[1, 0] = cx
                det_arr[1, 1] = area
                det_arr[1, 2] = 1

            # FPS
            frames += 1
            t1 = time.time()
            if t1 - t0 >= 0.5:
                fps = frames / (t1 - t0)
                frames = 0
                t0 = t1

            put_text(view_orig, f"{fps:.1f} FPS", (10,24), (0,255,0))
            put_text(view_orig, f"A: {LOWER_HSV_A}->{UPPER_HSV_A}  |  B: {LOWER_HSV_B}->{UPPER_HSV_B}",
                     (10,50), (0,255,255))
            put_text(view_seg,  "Filtrada (A + B) + bbox", (10,24), (0,255,0))

            cv2.imshow("Original", view_orig)
            cv2.imshow("Filtrada", view_seg)
            if SHOW_MASKS:
                cv2.imshow("Mask A", maskA)
                cv2.imshow("Mask B", maskB)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('s'):
                ts = int(time.time())
                cv2.imwrite(f"orig_{ts}.png", frame)
                cv2.imwrite(f"seg_{ts}.png", view_seg)
                cv2.imwrite(f"maskA_{ts}.png", maskA)
                cv2.imwrite(f"maskB_{ts}.png", maskB)
                cv2.imwrite(f"orig_anno_{ts}.png", view_orig)
                print(f"[i] Guardado orig/seg/maskA/maskB y anotadas ({ts})")
            elif key == ord('m'):
                SHOW_MASKS = not SHOW_MASKS
                if SHOW_MASKS:
                    cv2.namedWindow("Mask A", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Mask B", cv2.WINDOW_NORMAL)
                else:
                    try:
                        cv2.destroyWindow("Mask A")
                        cv2.destroyWindow("Mask B")
                    except Exception:
                        pass

            # Limitar FPS
            dt = time.time() - loop_t
            if dt < PERIOD:
                time.sleep(PERIOD - dt)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            mm.close(); fhandle.close()
        except Exception:
            pass
        try:
            mm_out.close(); os.close(fd_out)
        except Exception:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

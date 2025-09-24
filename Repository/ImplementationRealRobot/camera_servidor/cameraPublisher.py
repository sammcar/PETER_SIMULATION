import numpy as np
import cv2
import mmap
import os
import time

# ================== SALIDA COMPARTIDA (DETECCIONES) ==================
# Formato por fila: [pos_mapeada(0..180), area_bbox, class_id]
#   fila 0 -> ROJO (class_id=0)
#   fila 1 -> AZ                )UL (class_id=1)
DET_PATH  = "/dev/shm/det_vector"   # archivo compartido (OUT)
DET_ROWS  = 2                       # A=rojo, B=azul
DET_COLS  = 3                       # [pos(0..180), area, class_id]
DET_DTYPE = np.int32                # compatibilidad con lectores existentes
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

def _map_x_to_angle(x_center: float, img_width: int) -> int:
    """
    Mapea el centroide X de la bbox (px) a rango [0..180]:
      0   -> borde derecho
      90  -> centro
      180 -> borde izquierdo
    """
    if img_width <= 1:
        return 0
    # Normaliza x a [0..1] y luego invierte para que derecha=0, izquierda=1
    pos = 1.0 - (x_center / float(img_width - 1))
    angle = 180.0 * np.clip(pos, 0.0, 1.0)
    return int(round(angle))

def write_detection(det_arr, row: int, detected: bool, x: int, w: int, h: int,
                    img_width: int, class_id: int):
    """
    Escribe en la fila 'row' del vector compartido:
      - pos_mapeada (0..180, int)
      - area_bbox (int)
      - class_id (int)
    Si detected=False, publica [0, 0, class_id].
    """
    if detected and w > 0 and h > 0:
        x_center = x + (w / 2.0)
        pos_mapped = _map_x_to_angle(x_center, img_width)
        area = int(w * h)
        det_arr[row, 0] = int(pos_mapped)
        det_arr[row, 1] = int(area)
        det_arr[row, 2] = int(class_id)
    else:
        det_arr[row, 0] = 0
        det_arr[row, 1] = 0
        det_arr[row, 2] = int(class_id)

# ================== CONFIGURA AQUÍ ==================
SHM_NAME       = "/dev/shm/shared_image"  # ruta del productor de imagen
IMAGE_WIDTH    = 640
IMAGE_HEIGHT   = 480
IMAGE_CHANNELS = 3

# ---- Filtro A (bbox ROJA) ----
LOWER_HSV_A    = (8, 112, 78)   # (Hmin, Smin, Vmin) (135, 112, 78) 
UPPER_HSV_A    = (11, 255, 255)  # (Hmax, Smax, Vmax) (179, 255, 255) 
OPEN_K_A       = 0                # 0=off, >0 kernel elíptico
CLOSE_K_A      = 0                # 0=off, >0 kernel elíptico
MIN_AREA_A     = 500

# ---- Filtro B (bbox AZUL) ----
LOWER_HSV_B    = (98, 155, 120)
UPPER_HSV_B    = (113, 239, 255)
OPEN_K_B       = 2
CLOSE_K_B      = 2
MIN_AREA_B     = 800

# Preprocesado común
BLUR_ODD       = 13               # 1 (sin blur), 3,5,7... impar
TARGET_FPS     = 30.0             # limita la UI

# Ventanas opcionales
SHOW_MASKS     = False            # mostrar máscaras A y B en ventanas
# ====================================================

FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS
PERIOD = 1.0 / max(TARGET_FPS, 1e-6)

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

def main():
    cv2.namedWindow("Original",  cv2.WINDOW_NORMAL)
    cv2.namedWindow("Filtrada",  cv2.WINDOW_NORMAL)
    if SHOW_MASKS:
        cv2.namedWindow("Mask A", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask B", cv2.WINDOW_NORMAL)

    try:
        # mmap de entrada (imagen) y salida (detecciones)
        fhandle, mm = open_shared()
        fd_out, mm_out = open_det_shared()
        det_arr = det_array_from_mm(mm_out)

        # Inicial: sin detecciones (pos=0, area=0) y class_id fijos 0 y 1
        det_arr[:] = np.array([[0, 0, 0],
                            [0, 0, 1]], dtype=DET_DTYPE)


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

            # Pre-blur y HSV una sola vez (más eficiente)
            proc = frame
            if BLUR_ODD > 1:
                k = max(1, BLUR_ODD | 1)
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

            # Por defecto, “sin detección” en este frame
            det_arr[0, :] = (0, 0, 0)  # ROJO
            det_arr[1, :] = (0, 0, 1)  # AZUL


            # ----- Dibuja y PUBLICA bbox A (ROJO)
            if bboxA is not None:
                x, y, w, h = bboxA
                cv2.rectangle(view_orig, (x, y), (x+w, y+h), (0, 0, 255), 3)
                cv2.rectangle(view_seg,  (x, y), (x+w, y+h), (0, 0, 255), 3)
                cv2.putText(view_orig, f"A W={w} H={h}", (x, max(0, y-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                write_detection(det_arr, row=0, detected=True, x=x, w=w, h=h,
                                img_width=IMAGE_WIDTH, class_id=0)
            else:
                write_detection(det_arr, row=0, detected=False, x=0, w=0, h=0,
                                img_width=IMAGE_WIDTH, class_id=0)


            # ----- Dibuja y PUBLICA bbox B (AZUL)
            # ----- Dibuja y PUBLICA bbox B (AZUL)
            if bboxB is not None:
                x, y, w, h = bboxB
                cv2.rectangle(view_orig, (x, y), (x+w, y+h), (255, 0, 0), 3)
                cv2.rectangle(view_seg,  (x, y), (x+w, y+h), (255, 0, 0), 3)
                cv2.putText(view_orig, f"B W={w} H={h}", (x, max(0, y-22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                write_detection(det_arr, row=1, detected=True, x=x, w=w, h=h,
                                img_width=IMAGE_WIDTH, class_id=1)
            else:
                write_detection(det_arr, row=1, detected=False, x=0, w=0, h=0,
                                img_width=IMAGE_WIDTH, class_id=1)

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

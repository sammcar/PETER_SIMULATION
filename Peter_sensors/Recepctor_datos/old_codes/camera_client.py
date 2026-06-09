import socket
import struct
import numpy as np
import cv2

HOST = '0.0.0.0'
PORT = 8080

def recibir_frame(conn):
    # Recibir los 4 bytes del tamaño
    raw_size = b''
    while len(raw_size) < 4:
        chunk = conn.recv(4 - len(raw_size))
        if not chunk:
            return None
        raw_size += chunk

    total_size = struct.unpack('!I', raw_size)[0]

    # Recibir los bytes del frame
    data = b''
    while len(data) < total_size:
        chunk = conn.recv(min(4096, total_size - len(data)))
        if not chunk:
            return None
        data += chunk

    # Decodificar JPEG
    np_arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return frame

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Esperando conexión en {HOST}:{PORT}...")

    conn, addr = server.accept()
    print(f"Conectado desde {addr}")

    frame_count = 0
    try:
        while True:
            frame = recibir_frame(conn)
            if frame is None:
                print("Conexión cerrada.")
                break

            cv2.imshow("Stream Pi", frame)
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"Frames recibidos: {frame_count}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Detenido.")
    finally:
        conn.close()
        server.close()
        cv2.destroyAllWindows()


main()
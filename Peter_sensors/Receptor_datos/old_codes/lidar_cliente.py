import socket
import struct
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 8888
SIZE_DATOS = 500

# struct LidarData: uint64_t timestamp, double angle, int distance, int intensity
# tamaños: 8 + 8 + 4 + 4 = 24 bytes por punto
LIDAR_DATA_FORMAT = "=Qdii"
LIDAR_DATA_SIZE = struct.calcsize(LIDAR_DATA_FORMAT)  # 24 bytes
PACKET_SIZE = LIDAR_DATA_SIZE * SIZE_DATOS

def parsear_paquete(data):
    puntos = []
    for i in range(SIZE_DATOS):
        offset = i * LIDAR_DATA_SIZE
        chunk = data[offset:offset + LIDAR_DATA_SIZE]
        if len(chunk) < LIDAR_DATA_SIZE:
            break
        timestamp, angle, distance, intensity = struct.unpack(LIDAR_DATA_FORMAT, chunk)
        puntos.append({
            "timestamp": timestamp,
            "angle": angle,
            "distance": distance,
            "intensity": intensity
        })
    return puntos

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Escuchando en {UDP_IP}:{UDP_PORT}...")

    paquetes = 0
    try:
        while True:
            data, addr = sock.recvfrom(PACKET_SIZE + 512)
            paquetes += 1
            puntos = parsear_paquete(data)

            print(f"\nPaquete {paquetes} desde {addr} — {len(puntos)} puntos")
            for p in puntos[:5]:  # Muestra solo los primeros 5
                print(f"  angle: {p['angle']:.2f}° | dist: {p['distance']}mm | intensity: {p['intensity']}")

    except KeyboardInterrupt:
        print("\nDetenido.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
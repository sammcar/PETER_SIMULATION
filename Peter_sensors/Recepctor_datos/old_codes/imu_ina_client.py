import socket
import struct

UDP_IP   = "0.0.0.0"
UDP_PORT = 9999

# timestamp_ns(Q) roll pitch ax ay az gx gy gz current voltage
FORMAT = "=Qffffffffff"
SIZE   = struct.calcsize(FORMAT)  # 48 bytes

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Escuchando IMU+INA226 en {UDP_PORT}...")

while True:
    data, _ = sock.recvfrom(SIZE)
    ts, roll, pitch, ax, ay, az, gx, gy, gz, current, voltage = struct.unpack(FORMAT, data)
    print(f"roll={roll:.2f}° pitch={pitch:.2f}° | "
          f"acc=({ax:.2f},{ay:.2f},{az:.2f}) | "
          f"gyr=({gx:.2f},{gy:.2f},{gz:.2f}) | "
          f"I={current:.3f}A V={voltage:.3f}V")
#!/usr/bin/env python3
"""
peter_control.py — Interfaz serial para el robot cuadrúpedo Peter

Uso interactivo:
    python3 peter_control.py [puerto] [baudrate]
    python3 peter_control.py /dev/ttyUSB0 115200

Si no se especifica puerto, busca automáticamente el primer Arduino disponible.

Comandos de marcha (una sola tecla + Enter):
    i  Adelante       ,  Atrás         k  Stop
    j  Izquierda      l  Derecha
    u  Adelante+Izq   o  Adelante+Der
    m  Atrás+Izq      .  Atrás+Der
    c  Calibrar IMU   (poner robot nivelado y presionar c)

Comando de articulación directa:
    leg,joint,angulo   (ej: 0,1,30  →  pata FL, fémur, 30°)

Comandos especiales:
    help   Muestra esta ayuda
    quit / exit / q   Cierra la conexión y sale
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

# ── Detección automática de puerto ────────────────────────────────────────────

def find_arduino_port() -> str | None:
    """Devuelve el primer puerto que parece un Arduino/CH340/CP210x."""
    keywords = ("arduino", "ch340", "cp210", "ftdi", "usb serial", "acm", "usbmodem")
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()
        if any(k in desc or k in hwid for k in keywords):
            return port.device
    # fallback: primer puerto disponible
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ── Hilo lector ───────────────────────────────────────────────────────────────

def reader_thread(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Lee líneas del Arduino e imprime las respuestas con prefijo <<."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"<< {line}")
        except serial.SerialException:
            break
        time.sleep(0.01)


# ── Envío de comando ──────────────────────────────────────────────────────────

def send_command(ser: serial.Serial, cmd: str) -> None:
    """Envía un comando al Arduino (agrega \\n automáticamente)."""
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # ── Argumentos ────────────────────────────────────────────────────────
    port     = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    # ── Conexión ──────────────────────────────────────────────────────────
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR al abrir {port}: {e}")
        sys.exit(1)

    print(f"Conectado a {port} @ {baudrate} baud")
    print("Escribe un comando y presiona Enter. 'help' para ayuda, 'q' para salir.\n")

    # Espera a que el Arduino reinicie (reset por DTR)
    time.sleep(2)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    # ── Bucle de entrada ──────────────────────────────────────────────────
    teleop_keys = set("i,jkluom.c")

    try:
        while True:
            try:
                raw = input(">> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            if raw.lower() in ("quit", "exit", "q"):
                break

            if raw.lower() == "help":
                print(__doc__)
                continue

            # Tecla teleop de un solo carácter
            if len(raw) == 1 and raw in teleop_keys:
                print(f">> enviando tecla: '{raw}'")
                send_command(ser, raw)
                continue

            # Comando de articulación: leg,joint,ang
            if "," in raw:
                parts = raw.split(",")
                if len(parts) == 3:
                    try:
                        leg   = int(parts[0])
                        joint = int(parts[1])
                        ang   = float(parts[2])
                        if not (0 <= leg <= 3):
                            print("ERROR: pata debe ser 0-3")
                            continue
                        if not (0 <= joint <= 2):
                            print("ERROR: articulación debe ser 0-2")
                            continue
                        if not (-90 <= ang <= 90):
                            print("ERROR: ángulo debe ser -90 a 90")
                            continue
                        send_command(ser, raw)
                        continue
                    except ValueError:
                        pass
                print("ERROR: formato incorrecto. Usa: leg,joint,angulo  (ej: 0,1,30)")
                continue

            print(f"Comando desconocido: '{raw}'. Escribe 'help' para ver los comandos.")

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()
        print("\nConexión cerrada.")


if __name__ == "__main__":
    main()

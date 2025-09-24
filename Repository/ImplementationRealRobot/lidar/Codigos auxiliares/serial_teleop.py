import serial
import time

# Configuración del puerto serie
SERIAL_PORT = "/dev/ttyACM0"   # Ajusta según tu sistema
BAUD_RATE = 115200

def send_command(command):
    """ Envía un carácter y muestra lo que responde el ESP32 """
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.write(command.encode())   # Envía el comando
            print(f"Enviado: {command!r}")

            time.sleep(0.05)  # Pequeña pausa

            # Lee la respuesta que imprime el ESP32
            response = ser.read(100)  # Hasta 100 bytes
            if response:
                print("Respuesta ESP32:", response.decode(errors="ignore").strip())
            else:
                print("Sin respuesta del ESP32 (timeout)")
    except serial.SerialException as e:
        print("Error:", e)

def main():
    print("ESP32 Serial Command Sender")
    print("Escribe un carácter ('u','j','h','k', etc.) o 'exit' para salir.")

    while True:
        command = input("Enter command: ").strip()
        if command.lower() == "exit":
            print("Exiting...")
            break
        elif len(command) == 1:
            send_command(command)
        else:
            print("Entrada inválida. Solo un carácter.")

if __name__ == "__main__":
    main()

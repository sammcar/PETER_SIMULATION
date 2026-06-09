# from mlp_imu import iter_resultados_serie

# for probs, stuck in iter_resultados_serie("/dev/ttyACM0", 115200):
#     print("Pred:", probs, "Stuck:", stuck)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial import Serial

# Usa TU parser/MLP ya entrenado (mismo que emplea iter_resultados_serie)
from mlp_imu import test_mlp_con_linea

# --------- Ajusta aquí si quieres ----------
PORT = "/dev/ttyACM0"
BAUD = 115200
END_TOKEN = "--END--"
READ_TIMEOUT = 1.0
# -------------------------------------------

def reader_loop(ser: Serial) -> None:
    """Lee del serial, arma paquetes por END_TOKEN y alimenta test_mlp_con_linea.
    Imprime en la terminal cuando hay predicción."""
    buffer_lines: List[str] = []
    # breve pausa por estabilidad
    time.sleep(0.2)
    while ser.is_open:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == END_TOKEN:
                # procesa paquete completo
                for raw_line in buffer_lines:
                    try:
                        tokens = [t.strip() for t in raw_line.split(",")]
                        if len(tokens) < 8:
                            continue  # línea incompleta
                        out: Optional[Tuple] = test_mlp_con_linea(tokens)
                        if out is not None:
                            probs, stuck = out
                            print(f"Pred: {probs}  Stuck: {stuck}")
                    except Exception as ex:
                        print(f"[parse-error] {ex} :: {raw_line}")
                buffer_lines.clear()
            else:
                buffer_lines.append(line)

        except Exception as e:
            print(f"[serial-error] {e}")
            time.sleep(0.1)

# ---------------- GUI mínima ----------------
class MiniTX(tk.Tk):
    def __init__(self, ser: Serial):
        super().__init__()
        self.title("TX Serial (mínimo)")
        self.geometry("360x120")
        self.ser = ser

        frm = ttk.Frame(self, padding=10)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text=f"Puerto: {ser.port} @ {ser.baudrate}").grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(frm, text="Enviar:").grid(row=1, column=0, sticky="e", padx=5, pady=10)
        self.var = tk.StringVar()
        ent = ttk.Entry(frm, textvariable=self.var, width=24)
        ent.grid(row=1, column=1, sticky="we", padx=5, pady=10)
        ent.focus()

        btn = ttk.Button(frm, text="Enviar", command=self.send)
        btn.grid(row=1, column=2, sticky="w", padx=5, pady=10)

        self.bind("<Return>", lambda _: self.send())

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def send(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Sin conexión", "El puerto no está abierto.")
            return
        text = self.var.get()
        if not text:
            return
        # añade salto de línea (muy común en firmwares)
        if not text.endswith("\n") and not text.endswith("\r\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("utf-8"))
            self.ser.flush()
        except Exception as e:
            messagebox.showerror("Error enviando", str(e))

    def on_close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.destroy()

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT)
    except Exception as e:
        print(f"No se pudo abrir {PORT}: {e}")
        return

    # Hilo lector que imprime resultados en la terminal
    t = threading.Thread(target=reader_loop, args=(ser,), daemon=True)
    t.start()

    # Ventana mínima para transmitir
    app = MiniTX(ser)
    app.mainloop()

if __name__ == "__main__":
    main()

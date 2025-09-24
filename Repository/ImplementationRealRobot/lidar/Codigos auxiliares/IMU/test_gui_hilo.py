#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial import Serial

# Usa tu parser/MLP
from mlp_imu import test_mlp_con_linea

# --------- Config ----------
PORT = "/dev/ttyACM0"
BAUD = 115200
END_TOKEN = "--END--"
READ_TIMEOUT = 1.0
# ---------------------------

# ---------- Hilo lector ----------
def reader_loop(ser: Serial, stop_evt: threading.Event, print_lock: threading.Lock) -> None:
    """Lee del serial, arma paquetes por END_TOKEN y alimenta test_mlp_con_linea."""
    buffer_lines: List[str] = []
    time.sleep(0.2)  # breve pausa por estabilidad
    while not stop_evt.is_set() and ser.is_open:
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
                            # evita solapado de prints desde múltiples hilos
                            with print_lock:
                                print(f"Pred: {probs}  Stuck: {stuck}")
                    except Exception as ex:
                        with print_lock:
                            print(f"[parse-error] {ex} :: {raw_line}")
                buffer_lines.clear()
            else:
                buffer_lines.append(line)

        except Exception as e:
            with print_lock:
                print(f"[serial-error] {e}")
            time.sleep(0.1)

# -------------- GUI (hilo aparte) --------------
class MiniTX(tk.Tk):
    def __init__(self, ser: Serial, on_quit):
        super().__init__()
        self.title("TX Serial (mínimo)")
        self.geometry("360x140")
        self.ser = ser
        self._on_quit = on_quit

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

        # Botón salir (opcional): señaliza parada al hilo principal
        ttk.Button(frm, text="Salir", command=self.on_close).grid(row=2, column=2, sticky="e")

        self.bind("<Return>", lambda _: self.send())
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def send(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Sin conexión", "El puerto no está abierto.")
            return
        text = self.var.get()
        if not text:
            return
        if not text.endswith("\n") and not text.endswith("\r\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("utf-8"))
            self.ser.flush()
        except Exception as e:
            messagebox.showerror("Error enviando", str(e))

    def on_close(self):
        # No cierres el serial aquí; sólo notifica y cierra la GUI.
        try:
            if callable(self._on_quit):
                self._on_quit()
        finally:
            # destruye ventana; el hilo principal hará cleanup del serial
            self.destroy()

def gui_thread_fn(ser: Serial, stop_evt: threading.Event):
    """Función que corre en el hilo de la GUI."""
    # callback para avisar parada al hilo principal
    def _quit():
        stop_evt.set()
    app = MiniTX(ser, on_quit=_quit)

    # Poll opcional del stop_evt por si quieres autocerrar GUI al recibir señal
    def poll_stop():
        if stop_evt.is_set():
            try:
                app.destroy()
            except Exception:
                pass
            return
        app.after(100, poll_stop)

    app.after(100, poll_stop)
    app.mainloop()

# -------------- Orquestación --------------
def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT)
    except Exception as e:
        print(f"No se pudo abrir {PORT}: {e}")
        return

    stop_evt = threading.Event()
    print_lock = threading.Lock()

    # Hilo lector (trabajador)
    t_reader = threading.Thread(target=reader_loop, args=(ser, stop_evt, print_lock), daemon=True)
    t_reader.start()

    # Hilo GUI (¡toda la GUI vive aquí!)
    t_gui = threading.Thread(target=gui_thread_fn, args=(ser, stop_evt), daemon=True)
    t_gui.start()

    # Hilo principal queda libre; espera señal de parada (Ctrl+C o cierre de GUI)
    try:
        while not stop_evt.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[main] Interrupción del usuario, cerrando…")
        stop_evt.set()
    finally:
        # Cierre ordenado
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass
        # Espera breve a que terminen los hilos
        t_reader.join(timeout=1.0)
        t_gui.join(timeout=1.0)
        print("[main] Listo.")

if __name__ == "__main__":
    main()

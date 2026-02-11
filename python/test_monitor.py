import tkinter as tk
from tkinter import ttk, messagebox
import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import threading
import time
from datetime import datetime

# --- CONFIGURACIÓN ---
BROKER = "broker.emqx.io"
PORT = 1883

# Topics
TOPIC_ENV_SUB   = "bridge/env/+"
TOPIC_ACCEL_SUB = "bridge/accel/+"
TOPIC_CMD_SUB   = "bridge/cmd/+"  
DESTINOS_CMD    = ["bridge/cmd/1", "bridge/cmd/2"]

# Parámetros de Muestreo
FS_ACCEL = 52.0  # Hz (Acelerómetro)
FS_ENV   = 1.0   # Hz (Asumimos 1 muestra/seg para Temp/Hum para poder pintar su FFT)
MAX_SAMPLES = 200

class BridgeCommanderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Centro de Control - Puente Eduardo Torroja")
        self.root.geometry("1500x950") # Un poco más ancho para que quepan las 3 columnas
        
        # --- BUFFERS DE DATOS ---
        # Temp y Hum necesitan historial suficiente para una FFT decente
        self.temp_data = deque(maxlen=256) 
        self.hum_data = deque(maxlen=256)
        
        # Aceleración
        self.accel_display = deque(maxlen=MAX_SAMPLES) # Solo para pintar tiempo real
        self.accel_fft_buffer = deque(maxlen=1024)     # Buffer más largo para FFT precisa
        
        self.alarm_active = False

        # MQTT
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # GUI Layout
        self.create_layout()
        
        # Hilos
        threading.Thread(target=self.mqtt_thread, daemon=True).start()
        self.update_plots_loop()

    def create_layout(self):
        # 1. HEADER
        header = tk.Frame(self.root, bg="#2c3e50", pady=15)
        header.pack(fill=tk.X)
        tk.Label(header, text="MONITORIZACIÓN Y CONTROL ESTRUCTURAL", 
                 font=("Segoe UI", 20, "bold"), fg="white", bg="#2c3e50").pack()

        # 2. CONTENEDOR PRINCIPAL
        main_container = tk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- ZONA GRÁFICAS (Izquierda - Ocupa más espacio ahora) ---
        # Usamos weight para que las gráficas se expandan más que el panel de control
        main_container.columnconfigure(0, weight=4) 
        main_container.columnconfigure(1, weight=1)
        
        plot_frame = tk.Frame(main_container)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.setup_plots(plot_frame)

        # --- ZONA CONTROL (Derecha - Panel lateral fijo) ---
        ctrl_panel = tk.Frame(main_container, bg="#ecf0f1", width=280, relief=tk.RIDGE, bd=2)
        ctrl_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        ctrl_panel.pack_propagate(False)

        self.create_control_panel(ctrl_panel)

        # 3. BARRA ESTADO
        self.status_bar = tk.Label(self.root, text="Iniciando sistema...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def create_control_panel(self, parent):
        pad_opts = {'padx': 10, 'pady': 5, 'fill': tk.X}
        
        tk.Label(parent, text="PANEL DE COMANDOS", font=("Arial", 14, "bold"), bg="#ecf0f1").pack(pady=10)
        
        # SECCIÓN 1: ALARMA
        fr_alarm = tk.LabelFrame(parent, text="Estado Alarma", bg="#ecf0f1", font=("Arial", 10, "bold"))
        fr_alarm.pack(**pad_opts)
        
        self.canvas_sem = tk.Canvas(fr_alarm, height=60, bg="#ecf0f1", highlightthickness=0)
        self.canvas_sem.pack()
        self.light = self.canvas_sem.create_oval(110, 10, 150, 50, fill="gray", outline="black")
        self.lbl_alarm = tk.Label(fr_alarm, text="NORMAL", font=("Arial", 12, "bold"), fg="green", bg="#ecf0f1")
        self.lbl_alarm.pack()

        # SECCIÓN 2: CONTROL GLOBAL
        fr_acc = tk.LabelFrame(parent, text="Control Global", bg="#ecf0f1")
        fr_acc.pack(**pad_opts)
        
        tk.Button(fr_acc, text="▶ ACTIVAR ALARMA", bg="#27ae60", fg="white",
                  command=lambda: self.send_global_command("CONT ON")).pack(pady=5, fill=tk.X, padx=5)
        
        tk.Button(fr_acc, text="⏹ DESACTIVAR ALARMA", bg="#c0392b", fg="white",
                  command=lambda: self.send_global_command("CONT OFF")).pack(pady=5, fill=tk.X, padx=5)
        
        tk.Button(fr_acc, text="⚡ FORZAR LECTURA", bg="#f39c12",
                  command=lambda: self.send_global_command("READ")).pack(pady=5, fill=tk.X, padx=5)

        # SECCIÓN 3: RTC
        fr_rtc = tk.LabelFrame(parent, text="Sincronización", bg="#ecf0f1")
        fr_rtc.pack(**pad_opts)
        
        tk.Button(fr_rtc, text="🕒 Sincronizar Relojes", 
                  command=self.sync_rtc_global).pack(pady=5, padx=5, fill=tk.X)

    def setup_plots(self, parent):
        # CREACIÓN DE LA CUADRÍCULA 2x3
        # Fila 0: Tiempo (Temp, Hum, Accel)
        # Fila 1: Frecuencia (FFT Temp, FFT Hum, FFT Accel)
        self.fig, self.axs = plt.subplots(2, 3, figsize=(10, 8), dpi=100, constrained_layout=True)
        self.fig.patch.set_facecolor('#f0f0f0') 
        
        # Configuración inicial de títulos
        titulos = ["Temperatura (ºC)", "Humedad (%)", "Aceleración Z (mg)"]
        colores = ['r', 'b', 'g']
        
        for i in range(3):
            # Fila Superior (Tiempo)
            self.axs[0, i].set_title(titulos[i])
            self.axs[0, i].grid(True, linestyle='--', alpha=0.6)
            
            # Fila Inferior (FFT)
            self.axs[1, i].set_title(f"FFT - {titulos[i]}")
            self.axs[1, i].set_xlabel("Hz")
            self.axs[1, i].grid(True, linestyle='--', alpha=0.6)

        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas_plot.draw()
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # --- LÓGICA DE CONTROL ---
    def send_global_command(self, cmd_str):
        if not self.client.is_connected():
            return
        for topic in DESTINOS_CMD:
            self.client.publish(topic, cmd_str)
        self.status_bar.config(text=f"Enviado global: {cmd_str}")

    def sync_rtc_global(self):
        now = datetime.now()
        cmd = f"RTC {now.year} {now.month} {now.day} {now.hour} {now.minute} {now.second}"
        self.send_global_command(cmd)

    # --- LÓGICA MQTT ---
    def mqtt_thread(self):
        try:
            self.client.connect(BROKER, PORT, 60)
            self.client.loop_forever()
        except Exception as e:
            self.status_bar.config(text=f"Error MQTT: {e}", fg="red")

    def on_connect(self, client, userdata, flags, rc, props=None):
        if rc == 0:
            self.status_bar.config(text=f"Conectado a {BROKER}", fg="green")
            client.subscribe([(TOPIC_ENV_SUB, 0), (TOPIC_ACCEL_SUB, 0), (TOPIC_CMD_SUB, 0)])

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode()

            if "bridge/env" in topic:
                data = json.loads(payload)
                self.temp_data.append(float(data.get("temp", 0))/10.0)
                self.hum_data.append(float(data.get("hum", 0)))

            elif "bridge/accel" in topic:
                data = json.loads(payload)
                vals = data.get("z", [])
                self.accel_display.extend(vals)
                self.accel_fft_buffer.extend(vals)

            elif "bridge/cmd" in topic:
                if "CONT ON" in payload:
                    self.set_alarm_state(True)
                elif "CONT OFF" in payload:
                    self.set_alarm_state(False)

        except Exception as e:
            pass

    def set_alarm_state(self, is_active):
        self.alarm_active = is_active
        color = "red" if is_active else "green"
        text = "ALARMA ACTIVA" if is_active else "NORMAL"
        self.canvas_sem.itemconfig(self.light, fill=color)
        self.lbl_alarm.config(text=text, fg=color)

    # --- CÁLCULO DE FFT ---
    def calculate_fft(self, data, fs):
        """ Función auxiliar para calcular FFT de cualquier señal """
        if len(data) < 10: return [], []
        
        sig = np.array(data)
        # Eliminamos la media (Componente DC) para ver mejor las fluctuaciones
        sig = sig - np.mean(sig)
        
        n = len(sig)
        freqs = np.fft.rfftfreq(n, d=1/fs)
        mag = np.abs(np.fft.rfft(sig)) / n
        return freqs, mag

    def update_plots_loop(self):
        try:
            # --- FILA 1: TIEMPO ---
            # 1. Temperatura
            self.axs[0, 0].cla()
            self.axs[0, 0].set_title("Temperatura (ºC)")
            self.axs[0, 0].grid(True)
            if self.temp_data: self.axs[0, 0].plot(self.temp_data, 'r.-')

            # 2. Humedad
            self.axs[0, 1].cla()
            self.axs[0, 1].set_title("Humedad (%)")
            self.axs[0, 1].grid(True)
            if self.hum_data: self.axs[0, 1].plot(self.hum_data, 'b.-')

            # 3. Aceleración
            self.axs[0, 2].cla()
            self.axs[0, 2].set_title("Aceleración Z (mg)")
            # Fija el límite Y dinámico para que no baile demasiado
            if self.accel_display: 
                self.axs[0, 2].plot(self.accel_display, 'g-')
                mini, maxi = min(self.accel_display), max(self.accel_display)
                self.axs[0, 2].set_ylim(mini-50, maxi+50)

            # --- FILA 2: FRECUENCIA (FFTs) ---
            # 4. FFT Temp
            self.axs[1, 0].cla()
            self.axs[1, 0].set_title("Espectro Temp")
            self.axs[1, 0].grid(True)
            f, m = self.calculate_fft(self.temp_data, FS_ENV)
            if len(f)>0: self.axs[1, 0].plot(f, m, 'r-')

            # 5. FFT Hum
            self.axs[1, 1].cla()
            self.axs[1, 1].set_title("Espectro Hum")
            self.axs[1, 1].grid(True)
            f, m = self.calculate_fft(self.hum_data, FS_ENV)
            if len(f)>0: self.axs[1, 1].plot(f, m, 'b-')

            # 6. FFT Aceleración
            self.axs[1, 2].cla()
            self.axs[1, 2].set_title("Espectro Accel")
            self.axs[1, 2].set_xlabel("Hz")
            self.axs[1, 2].grid(True)
            self.axs[1, 2].set_xlim(0, FS_ACCEL/2)
            
            f, m = self.calculate_fft(self.accel_fft_buffer, FS_ACCEL)
            if len(f)>0: 
                self.axs[1, 2].plot(f, m, 'k-')
                # Marcar pico dominante si supera el ruido
                idx = np.argmax(m)
                if m[idx] > 5:
                    self.axs[1, 2].annotate(f"{f[idx]:.1f}Hz", xy=(f[idx], m[idx]), 
                                            xytext=(f[idx]+2, m[idx]), arrowprops=dict(facecolor='black', shrink=0.05))

            self.canvas_plot.draw()
        except Exception as e:
            # print(f"Plot error: {e}") # Descomentar para debug
            pass
        
        self.root.after(500, self.update_plots_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = BridgeCommanderApp(root)
    root.mainloop()
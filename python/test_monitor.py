import tkinter as tk
from tkinter import ttk, messagebox
import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import threading
import time
from datetime import datetime, timedelta

# --- CONFIGURACIÓN ---
BROKER = "broker.emqx.io"
PORT = 1883

# Topics
TOPIC_ENV_SUB   = "bridge/env/+"
TOPIC_ACCEL_SUB = "bridge/accel/+"
TOPIC_CMD_SUB   = "bridge/cmd/+"  
DESTINOS_CMD    = ["bridge/cmd/1", "bridge/cmd/2"]

# Parámetros de Muestreo
FS_ACCEL = 52.0  # Hz
MAX_SAMPLES_ACCEL = 200 # Ventana de visualización accel

class BridgeCommanderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Centro de Control - Puente Eduardo Torroja")
        self.root.geometry("1400x900")
        
        # --- BUFFERS DE DATOS (TIEMPO Y VALOR) ---
        # Ahora guardamos tuplas o listas paralelas: Times y Values
        self.temp_vals = deque(maxlen=100)
        self.temp_times = deque(maxlen=100)
        
        self.hum_vals = deque(maxlen=100)
        self.hum_times = deque(maxlen=100)
        
        # Aceleración (Visualización Tiempo)
        self.accel_vals = deque(maxlen=MAX_SAMPLES_ACCEL)
        self.accel_times = deque(maxlen=MAX_SAMPLES_ACCEL)
        
        # Aceleración (Buffer para FFT - Solo valores)
        self.accel_fft_buffer = deque(maxlen=1024)
        
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
        tk.Label(header, text="SISTEMA DE MONITORIZACIÓN ESTRUCTURAL", 
                 font=("Segoe UI", 20, "bold"), fg="white", bg="#2c3e50").pack()

        # 2. CONTENEDOR PRINCIPAL
        main_container = tk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- ZONA GRÁFICAS (Izquierda) ---
        plot_frame = tk.Frame(main_container)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.setup_plots(plot_frame)

        # --- ZONA CONTROL (Derecha) ---
        ctrl_panel = tk.Frame(main_container, bg="#ecf0f1", width=250, relief=tk.RIDGE, bd=2)
        ctrl_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        ctrl_panel.pack_propagate(False)

        self.create_control_panel(ctrl_panel)

        # 3. BARRA ESTADO
        self.status_bar = tk.Label(self.root, text="Iniciando sistema...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def create_control_panel(self, parent):
        pad_opts = {'padx': 10, 'pady': 5, 'fill': tk.X}
        
        tk.Label(parent, text="COMANDOS", font=("Arial", 14, "bold"), bg="#ecf0f1").pack(pady=10)
        
        # SECCIÓN ALARMA
        fr_alarm = tk.LabelFrame(parent, text="Estado Alarma", bg="#ecf0f1")
        fr_alarm.pack(**pad_opts)
        
        self.canvas_sem = tk.Canvas(fr_alarm, height=60, bg="#ecf0f1", highlightthickness=0)
        self.canvas_sem.pack()
        self.light = self.canvas_sem.create_oval(105, 10, 145, 50, fill="gray", outline="black")
        self.lbl_alarm = tk.Label(fr_alarm, text="NORMAL", font=("Arial", 12, "bold"), fg="green", bg="#ecf0f1")
        self.lbl_alarm.pack()

        # BOTONES
        fr_btns = tk.LabelFrame(parent, text="Acciones", bg="#ecf0f1")
        fr_btns.pack(**pad_opts)
        
        tk.Button(fr_btns, text="▶ ACTIVAR ALARMA", bg="#27ae60", fg="white",
                  command=lambda: self.send_global_command("CONT ON")).pack(pady=5, fill=tk.X, padx=5)
        
        tk.Button(fr_btns, text="⏹ DESACTIVAR ALARMA", bg="#c0392b", fg="white",
                  command=lambda: self.send_global_command("CONT OFF")).pack(pady=5, fill=tk.X, padx=5)
        
        tk.Button(fr_btns, text="⚡ FORZAR LECTURA", bg="#f39c12",
                  command=lambda: self.send_global_command("READ")).pack(pady=5, fill=tk.X, padx=5)
        
        tk.Button(fr_btns, text="🕒 Sincronizar Relojes", 
                  command=self.sync_rtc_global).pack(pady=5, padx=5, fill=tk.X)

    def setup_plots(self, parent):
        # AHORA 2x2: 
        # (0,0) Temp vs Time   | (0,1) Hum vs Time
        # (1,0) Accel vs Time  | (1,1) Accel FFT
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8), dpi=100, constrained_layout=True)
        self.fig.patch.set_facecolor('#f0f0f0') 
        
        # Configuramos formateador de fecha para eje X
        self.date_fmt = mdates.DateFormatter('%H:%M:%S')

        # Estilos iniciales
        # Temp
        self.axs[0, 0].set_title("Temperatura (ºC)")
        self.axs[0, 0].grid(True, linestyle='--', alpha=0.6)
        
        # Hum
        self.axs[0, 1].set_title("Humedad (%)")
        self.axs[0, 1].grid(True, linestyle='--', alpha=0.6)
        
        # Accel Time
        self.axs[1, 0].set_title("Aceleración Z (mg)")
        self.axs[1, 0].grid(True, linestyle='--', alpha=0.6)

        # Accel FFT (Este mantiene eje X en Hz, no Time)
        self.axs[1, 1].set_title("Espectro Frecuencia Aceleración")
        self.axs[1, 1].set_xlabel("Frecuencia (Hz)")
        self.axs[1, 1].grid(True, linestyle='--', alpha=0.6)

        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas_plot.draw()
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # --- LÓGICA DE CONTROL ---
    def send_global_command(self, cmd_str):
        if not self.client.is_connected():
            return
        for topic in DESTINOS_CMD:
            self.client.publish(topic, cmd_str)
        self.status_bar.config(text=f"Enviado: {cmd_str}")

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
                
                # 1. Parsear Valor
                val_temp = float(data.get("temp", 0))/10.0
                val_hum = float(data.get("hum", 0))
                
                # 2. Parsear Timestamp ("2026-02-16 12:35:00")
                ts_str = data.get("ts")
                if ts_str:
                    try:
                        dt = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
                    except ValueError:
                        dt = datetime.now()
                else:
                    dt = datetime.now()

                # 3. Guardar en Deques
                self.temp_times.append(dt)
                self.temp_vals.append(val_temp)
                
                self.hum_times.append(dt)
                self.hum_vals.append(val_hum)

            elif "bridge/accel" in topic:
                data = json.loads(payload)
                vals = data.get("z", [])
                
                # Para la FFT guardamos todo
                self.accel_fft_buffer.extend(vals)
                
                # Para la gráfica en tiempo, generamos timestamps simulados
                # basados en la hora actual de llegada (aprox)
                now = datetime.now()
                # Retrocedemos en el tiempo para que el último punto sea 'now'
                # dt entre muestras = 1 / FS
                dt_sample = 1.0 / FS_ACCEL
                
                start_time = now - timedelta(seconds=len(vals)*dt_sample)
                
                for i, v in enumerate(vals):
                    t = start_time + timedelta(seconds=i*dt_sample)
                    self.accel_times.append(t)
                    self.accel_vals.append(v)

            elif "bridge/cmd" in topic:
                if "CONT ON" in payload:
                    self.set_alarm_state(True)
                elif "CONT OFF" in payload:
                    self.set_alarm_state(False)

        except Exception as e:
            print(f"Error parseando: {e}")

    def set_alarm_state(self, is_active):
        self.alarm_active = is_active
        color = "red" if is_active else "green"
        text = "ALARMA ACTIVA" if is_active else "NORMAL"
        self.canvas_sem.itemconfig(self.light, fill=color)
        self.lbl_alarm.config(text=text, fg=color)

    # --- CÁLCULO DE FFT ---
    def calculate_fft(self, data, fs):
        if len(data) < 10: return [], []
        sig = np.array(data)
        sig = sig - np.mean(sig) # Quitar DC
        n = len(sig)
        freqs = np.fft.rfftfreq(n, d=1/fs)
        mag = np.abs(np.fft.rfft(sig)) / n
        return freqs, mag

    def update_plots_loop(self):
        try:
            # 1. TEMPERATURA (Arriba Izq)
            ax_t = self.axs[0, 0]
            ax_t.cla()
            ax_t.set_title("Temperatura (ºC)")
            ax_t.grid(True)
            if self.temp_times:
                ax_t.plot(self.temp_times, self.temp_vals, 'r.-')
                ax_t.xaxis.set_major_formatter(self.date_fmt)

            # 2. HUMEDAD (Arriba Der)
            ax_h = self.axs[0, 1]
            ax_h.cla()
            ax_h.set_title("Humedad (%)")
            ax_h.grid(True)
            if self.hum_times:
                ax_h.plot(self.hum_times, self.hum_vals, 'b.-')
                ax_h.xaxis.set_major_formatter(self.date_fmt)

            # 3. ACELERACIÓN TIME (Abajo Izq)
            ax_a = self.axs[1, 0]
            ax_a.cla()
            ax_a.set_title("Aceleración Z (mg)")
            ax_a.grid(True)
            if self.accel_times:
                ax_a.plot(self.accel_times, self.accel_vals, 'g-')
                ax_a.xaxis.set_major_formatter(self.date_fmt)
                # Escala dinámica suave
                if len(self.accel_vals) > 0:
                    mini, maxi = min(self.accel_vals), max(self.accel_vals)
                    ax_a.set_ylim(mini-50, maxi+50)

            # 4. ACELERACIÓN FFT (Abajo Der)
            ax_f = self.axs[1, 1]
            ax_f.cla()
            ax_f.set_title("Espectro Aceleración")
            ax_f.set_xlabel("Frecuencia (Hz)")
            ax_f.grid(True)
            ax_f.set_xlim(0, FS_ACCEL/2) # Nyquist
            
            f, m = self.calculate_fft(self.accel_fft_buffer, FS_ACCEL)
            if len(f) > 0: 
                ax_f.plot(f, m, 'k-')
                # Marcar pico máximo
                idx = np.argmax(m)
                if m[idx] > 5: # Umbral de ruido visual
                    ax_f.annotate(f"{f[idx]:.1f}Hz", xy=(f[idx], m[idx]), 
                                  xytext=(f[idx]+2, m[idx]), 
                                  arrowprops=dict(facecolor='black', shrink=0.05))

            # Auto-rotar las fechas para que no se pisen en las gráficas de tiempo
            self.fig.autofmt_xdate()
            self.canvas_plot.draw()
            
        except Exception as e:
            print(f"Error plot: {e}")
        
        self.root.after(500, self.update_plots_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = BridgeCommanderApp(root)
    root.mainloop()
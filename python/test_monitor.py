import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt
import paho.mqtt.enums as mqtt_enums
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import threading
import time

# --- CONFIGURACIÓN ---
BROKER = "broker.emqx.io" # O usa "localhost" si tienes Mosquitto local
PORT = 1883

# Topics específicos del Nodo 1
TOPIC_ENV = "bridge/env/1"
TOPIC_ACCEL = "bridge/accel/1"
TOPIC_CMD = "bridge/cmd/1" # Necesario para saber si está en modo continuo

# Parámetros de Muestreo
FS_ACCEL = 52.0  # Frecuencia muestreo Acelerómetro
FS_ENV = 1.0     # Frecuencia asumida para Temp/Hum (solo para pintar la FFT)

class MonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitorización Estructural - Nodo 1")
        self.root.geometry("1400x900")
        
        # --- BUFFERS DE DATOS ---
        # Usamos deque para mantener un historial deslizante
        self.len_env = 50
        self.len_acc = 1024 # Potencia de 2 para mejor FFT
        
        self.temp_data = deque(maxlen=self.len_env)
        self.hum_data = deque(maxlen=self.len_env)
        self.accel_data = deque(maxlen=self.len_acc)
        
        # Buffer visual para aceleración (para que no se vea tan denso el tiempo real)
        self.accel_display = deque(maxlen=200) 

        # Estado del sistema
        self.continuous_mode = False

        # --- INTERFAZ GRÁFICA ---
        self.create_header()
        self.create_plots()

        # --- MQTT CONFIG ---
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Iniciar MQTT en hilo secundario
        threading.Thread(target=self.mqtt_thread, daemon=True).start()

        # Iniciar bucle de refresco de gráficas (500ms)
        self.update_gui_loop()

    def create_header(self):
        header_frame = tk.Frame(self.root, bg="#ddd", pady=10)
        header_frame.pack(side=tk.TOP, fill=tk.X)
        
        tk.Label(header_frame, text="Monitorización Puente (Nodo 1)", 
                 font=("Arial", 16, "bold"), bg="#ddd").pack(side=tk.LEFT, padx=20)

        # Indicador de Modo Continuo (Luz Roja)
        ctrl_frame = tk.Frame(header_frame, bg="#ddd")
        ctrl_frame.pack(side=tk.RIGHT, padx=20)
        
        self.canvas_light = tk.Canvas(ctrl_frame, width=40, height=40, bg="#ddd", highlightthickness=0)
        self.canvas_light.pack(side=tk.RIGHT)
        self.light = self.canvas_light.create_oval(5, 5, 35, 35, fill="gray", outline="black", width=2)
        
        tk.Label(ctrl_frame, text="MODO CONTINUO:", bg="#ddd", font=("Arial", 10, "bold")).pack(side=tk.RIGHT, padx=5)
        
        # Barra de estado inferior
        self.status_label = tk.Label(self.root, text="Desconectado", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)

    def create_plots(self):
        # Creamos una figura con 2 filas y 3 columnas
        # Fila 1: Series Temporales (Temp, Hum, Accel)
        # Fila 2: FFTs (Temp, Hum, Accel)
        self.fig, self.axs = plt.subplots(2, 3, figsize=(12, 8), constrained_layout=True)
        self.fig.patch.set_facecolor('#f0f0f0')
        
        # Títulos de las gráficas
        titles = ["Temperatura (ºC)", "Humedad (%)", "Aceleración Z (mg)"]
        for i in range(3):
            self.axs[0, i].set_title(titles[i] + " - Tiempo Real")
            self.axs[0, i].grid(True, linestyle='--', alpha=0.6)
            
            self.axs[1, i].set_title("FFT - " + titles[i])
            self.axs[1, i].set_xlabel("Frecuencia (Hz)")
            self.axs[1, i].grid(True, linestyle='--', alpha=0.6)

        # Integrar Matplotlib en Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # --- LÓGICA MQTT ---
    def mqtt_thread(self):
        try:
            print(f"Conectando a {BROKER}...")
            self.client.connect(BROKER, PORT, 60)
            self.client.loop_forever()
        except Exception as e:
            err = f"Error conexión MQTT: {e}"
            print(err)
            self.root.after(0, lambda: self.status_label.config(text=err, fg="red"))

    def on_connect(self, client, userdata, flags, rc, props=None):
        if rc == 0:
            msg = f"Conectado a {BROKER}"
            print(msg)
            self.root.after(0, lambda: self.status_label.config(text=msg, fg="green"))
            # Suscribirse a los 3 topics necesarios
            client.subscribe([(TOPIC_ENV, 0), (TOPIC_ACCEL, 0), (TOPIC_CMD, 0)])
        else:
            self.root.after(0, lambda: self.status_label.config(text=f"Error conexión: {rc}", fg="red"))

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            
            # 1. TEMPERATURA Y HUMEDAD
            if topic == TOPIC_ENV:
                data = json.loads(payload)
                # Según main.c, temp viene multiplicada por 10
                t_val = float(data.get("temp", 0)) / 10.0
                h_val = float(data.get("hum", 0))
                
                self.temp_data.append(t_val)
                self.hum_data.append(h_val)

            # 2. ACELERACIÓN
            elif topic == TOPIC_ACCEL:
                data = json.loads(payload)
                # Puede venir un bloque "z" (array) o un dato suelto
                valores = data.get("z", [])
                
                self.accel_data.extend(valores)
                self.accel_display.extend(valores)

            # 3. COMANDOS (Detectar Modo Continuo)
            elif topic == TOPIC_CMD:
                # El STM32 envía texto plano "CONT ON" o "CONT OFF"
                content = payload.upper()
                if "CONT ON" in content:
                    self.continuous_mode = True
                elif "CONT OFF" in content:
                    self.continuous_mode = False
                
                # Actualizar luz roja inmediatamente
                self.root.after(0, self.update_light)

        except Exception as e:
            print(f"Error procesando mensaje: {e}")

    def update_light(self):
        color = "red" if self.continuous_mode else "gray"
        self.canvas_light.itemconfig(self.light, fill=color)
        # Efecto de brillo si está encendido
        outline = "red" if self.continuous_mode else "black"
        self.canvas_light.itemconfig(self.light, outline=outline)

    # --- PROCESAMIENTO MATEMÁTICO (FFT) ---
    def calculate_fft(self, data, fs):
        if len(data) < 10: # Mínimo de muestras para calcular
            return [], []
        
        signal = np.array(data)
        n = len(signal)
        # Eliminar componente DC (offset) para ver mejor los picos
        signal = signal - np.mean(signal)
        
        freqs = np.fft.rfftfreq(n, d=1/fs)
        mag = np.abs(np.fft.rfft(signal)) / n
        return freqs, mag

    # --- BUCLE DE VISUALIZACIÓN ---
    def update_gui_loop(self):
        try:
            # 1. Gráficas Temporales (Fila Superior)
            # Temp
            self.axs[0, 0].cla()
            self.axs[0, 0].set_title("Temperatura (ºC)")
            self.axs[0, 0].grid(True)
            if self.temp_data:
                self.axs[0, 0].plot(self.temp_data, 'r.-')

            # Hum
            self.axs[0, 1].cla()
            self.axs[0, 1].set_title("Humedad (%)")
            self.axs[0, 1].grid(True)
            if self.hum_data:
                self.axs[0, 1].plot(self.hum_data, 'b.-')

            # Accel
            self.axs[0, 2].cla()
            self.axs[0, 2].set_title("Aceleración Z (mg)")
            if self.accel_display:
                self.axs[0, 2].plot(self.accel_display, 'g-')
                # Autoescala dinámica pero con margen
                mini, maxi = min(self.accel_display), max(self.accel_display)
                self.axs[0, 2].set_ylim(mini-50, maxi+50)

            # 2. Gráficas FFT (Fila Inferior)
            # FFT Temp
            self.axs[1, 0].cla()
            self.axs[1, 0].set_title("FFT Temperatura")
            self.axs[1, 0].grid(True)
            f_t, m_t = self.calculate_fft(self.temp_data, FS_ENV)
            if len(f_t) > 0: self.axs[1, 0].plot(f_t, m_t, 'r-')

            # FFT Hum
            self.axs[1, 1].cla()
            self.axs[1, 1].set_title("FFT Humedad")
            self.axs[1, 1].grid(True)
            f_h, m_h = self.calculate_fft(self.hum_data, FS_ENV)
            if len(f_h) > 0: self.axs[1, 1].plot(f_h, m_h, 'b-')

            # FFT Accel (La más importante)
            self.axs[1, 2].cla()
            self.axs[1, 2].set_title("FFT Aceleración")
            self.axs[1, 2].set_xlabel("Hz")
            self.axs[1, 2].grid(True)
            self.axs[1, 2].set_xlim(0, FS_ACCEL/2) # Nyquist
            
            f_a, m_a = self.calculate_fft(self.accel_data, FS_ACCEL)
            if len(f_a) > 0:
                self.axs[1, 2].plot(f_a, m_a, 'k-')
                # Marcar pico máximo
                idx = np.argmax(m_a)
                if m_a[idx] > 10: # Umbral de ruido
                    self.axs[1, 2].annotate(f"{f_a[idx]:.1f}Hz", xy=(f_a[idx], m_a[idx]))

            self.canvas.draw()
            
        except Exception as e:
            print(f"Error en update gráfico: {e}")
        
        # Repetir cada 500ms
        self.root.after(500, self.update_gui_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = MonitorApp(root)
    root.mainloop()
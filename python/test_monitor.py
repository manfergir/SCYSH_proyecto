import tkinter as tk
import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import threading
from datetime import datetime, timedelta

BROKER = "broker.emqx.io"
PORT = 1883

TOPIC_ENV_SUB   = "bridge/env/+"
TOPIC_ACCEL_SUB = "bridge/accel/+"
TOPIC_CMD_SUB   = "bridge/cmd/+"
DESTINOS_CMD    = ["bridge/cmd/1", "bridge/cmd/2"]

FS_ACCEL = 52.0
MAX_SAMPLES_ACCEL = 200


class BridgeCommanderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Centro de Control - Puente Eduardo Torroja")
        self.root.geometry("1400x900")

        self.temp_vals  = deque(maxlen=100)
        self.temp_times = deque(maxlen=100)

        self.accel_fft_buffer = deque(maxlen=1024)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.hum_vals   = deque(maxlen=100)
        self.hum_times  = deque(maxlen=100)
        self.accel_vals  = deque(maxlen=MAX_SAMPLES_ACCEL)
        self.accel_times = deque(maxlen=MAX_SAMPLES_ACCEL)

        self.alarm_active = False

        self.create_layout()
        threading.Thread(target=self.mqtt_thread, daemon=True).start()
        self.update_plots_loop()

    def mqtt_thread(self):
        try:
            self.client.connect(BROKER, PORT, 60)
            self.client.loop_forever()
        except Exception as e:
            self.status_bar.config(text=f"Error MQTT: {e}", fg="red")

    def create_control_panel(self, parent):
        pad_opts = {'padx': 10, 'pady': 5, 'fill': tk.X}
        tk.Label(parent, text="COMANDOS", font=("Arial", 14, "bold"), bg="#ecf0f1").pack(pady=10)

        fr_alarm = tk.LabelFrame(parent, text="Estado Alarma", bg="#ecf0f1")
        fr_alarm.pack(**pad_opts)
        self.canvas_sem = tk.Canvas(fr_alarm, height=60, bg="#ecf0f1", highlightthickness=0)
        self.canvas_sem.pack()
        self.light = self.canvas_sem.create_oval(105, 10, 145, 50, fill="gray", outline="black")
        self.lbl_alarm = tk.Label(fr_alarm, text="NORMAL", font=("Arial", 12, "bold"), fg="green", bg="#ecf0f1")
        self.lbl_alarm.pack()

        fr_btns = tk.LabelFrame(parent, text="Acciones", bg="#ecf0f1")
        fr_btns.pack(**pad_opts)
        tk.Button(fr_btns, text="ACTIVAR ALARMA", bg="#27ae60", fg="white",
                  command=lambda: self.send_global_command("CONT ON")).pack(pady=5, fill=tk.X, padx=5)
        tk.Button(fr_btns, text="DESACTIVAR ALARMA", bg="#c0392b", fg="white",
                  command=lambda: self.send_global_command("CONT OFF")).pack(pady=5, fill=tk.X, padx=5)
        tk.Button(fr_btns, text="FORZAR LECTURA", bg="#f39c12",
                  command=lambda: self.send_global_command("READ")).pack(pady=5, fill=tk.X, padx=5)
        tk.Button(fr_btns, text="Sincronizar Relojes",
                  command=self.sync_rtc_global).pack(pady=5, padx=5, fill=tk.X)

    def calculate_fft(self, data, fs):
        if len(data) < 10:
            return [], []
        sig = np.array(data)
        sig = sig - np.mean(sig)
        n = len(sig)
        freqs = np.fft.rfftfreq(n, d=1/fs)
        mag = np.abs(np.fft.rfft(sig)) / n
        return freqs, mag

    def on_connect(self, client, userdata, flags, rc, props=None):
        if rc == 0:
            self.status_bar.config(text=f"Conectado a {BROKER}", fg="green")
            client.subscribe([(TOPIC_ENV_SUB, 0), (TOPIC_ACCEL_SUB, 0), (TOPIC_CMD_SUB, 0)])

    def create_layout(self):
        header = tk.Frame(self.root, bg="#2c3e50", pady=15)
        header.pack(fill=tk.X)
        tk.Label(header, text="SISTEMA DE MONITORIZACIÓN ESTRUCTURAL",
                 font=("Segoe UI", 20, "bold"), fg="white", bg="#2c3e50").pack()

        self.status_bar = tk.Label(self.root, text="Desconectado", fg="red", anchor=tk.W, relief=tk.SUNKEN, bd=1)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        main_container = tk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        plot_frame = tk.Frame(main_container)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.setup_plots(plot_frame)

        ctrl_panel = tk.Frame(main_container, bg="#ecf0f1", width=250, relief=tk.RIDGE, bd=2)
        ctrl_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        ctrl_panel.pack_propagate(False)
        self.create_control_panel(ctrl_panel)

    def set_alarm_state(self, active):
        self.alarm_active = active
        c = "red" if active else "green"
        self.canvas_sem.itemconfig(self.light, fill=c)
        self.lbl_alarm.config(text="ALARMA ACTIVA" if active else "NORMAL", fg=c)

    def setup_plots(self, parent):
        self.fig = plt.figure(figsize=(10, 8), dpi=100)
        self.fig.patch.set_facecolor('#f0f0f0')
        self.axs = [self.fig.add_subplot(2, 2, i) for i in range(1, 5)]

        titles = ["Temperatura (ºC)", "Humedad (%)", "Aceleración Z (mg)", "Espectro Frecuencia Aceleración"]
        for ax, t in zip(self.axs, titles):
            ax.set_title(t)
            ax.grid(True, linestyle='--', alpha=0.6)
        self.axs[3].set_xlabel("Frecuencia (Hz)")

        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas_plot.draw()
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def sync_rtc_global(self):
        n = datetime.now()
        self.send_global_command(f"RTC {n.year} {n.month} {n.day} {n.hour} {n.minute} {n.second}")

    def on_message(self, client, userdata, msg):
        try:
            topic   = msg.topic
            payload = msg.payload.decode()

            if "bridge/env" in topic:
                data = json.loads(payload)
                val_temp = float(data.get("temp", 0)) / 10.0
                val_hum  = float(data.get("hum", 0))
                ts_str = data.get("ts")
                if ts_str:
                    try:
                        dt = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
                    except ValueError:
                        dt = datetime.now()
                else:
                    dt = datetime.now()
                self.temp_times.append(dt)
                self.temp_vals.append(val_temp)
                self.hum_times.append(dt)
                self.hum_vals.append(val_hum)

            elif "bridge/accel" in topic:
                data = json.loads(payload)
                vals = data.get("z", [])
                self.accel_fft_buffer.extend(vals)
                now = datetime.now()
                dt_sample  = 1.0 / FS_ACCEL
                start_time = now - timedelta(seconds=len(vals) * dt_sample)
                for i, v in enumerate(vals):
                    self.accel_times.append(start_time + timedelta(seconds=i * dt_sample))
                    self.accel_vals.append(v)

            elif "bridge/cmd" in topic:
                if "CONT ON" in payload:   self.set_alarm_state(True)
                elif "CONT OFF" in payload: self.set_alarm_state(False)

        except Exception as e:
            print(f"Error parseando: {e}")

    def send_global_command(self, cmd_str):
        if not self.client.is_connected():
            return
        for t in DESTINOS_CMD:
            self.client.publish(t, cmd_str)
        self.status_bar.config(text=f"Enviado: {cmd_str}")

    def _fmt_time_axis(self, ax):
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        ax.xaxis.set_major_locator(mdates.AutoDateLocator())
        plt.setp(ax.get_xticklabels(), rotation=30, ha='right', fontsize=7)

    def update_plots_loop(self):
        try:
            datasets = [
                (self.axs[0], self.temp_times,  self.temp_vals,  "Temperatura (ºC)",   'r-'),
                (self.axs[1], self.hum_times,   self.hum_vals,   "Humedad (%)",         'b-'),
                (self.axs[2], self.accel_times, self.accel_vals, "Aceleración Z (mg)",  'g-'),
            ]
            for ax, times, vals, title, color in datasets:
                ax.cla()
                ax.set_title(title)
                ax.grid(True, linestyle='--', alpha=0.5)
                if times:
                    ax.plot(list(times), list(vals), color)
                    self._fmt_time_axis(ax)

            ax_f = self.axs[3]
            ax_f.cla()
            ax_f.set_title("Espectro Acelerómetro")
            ax_f.set_xlabel("Frecuencia (Hz)")
            ax_f.grid(True, linestyle='--', alpha=0.5)
            f, m = self.calculate_fft(list(self.accel_fft_buffer), FS_ACCEL)
            if len(f) > 0:
                ax_f.plot(f, m, 'k-')

            self.fig.tight_layout(pad=1.5)
            self.canvas_plot.draw()

        except Exception as e:
            print(f"Error plot: {e}")

        self.root.after(500, self.update_plots_loop)


if __name__ == "__main__":
    root = tk.Tk()
    app  = BridgeCommanderApp(root)
    root.mainloop()
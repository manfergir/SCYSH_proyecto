import paho.mqtt.client as mqtt
import json
import random
import sys

# --- CONFIGURACIÓN PARA MANOLO ---
# La IP que tienes en tu .h (test.mosquitto.org)
BROKER = "54.36.178.49" 
PORT = 1883

# EL TOPIC EXACTO (Valor de pcTempTopic en tu .h)
TOPIC_TO_LISTEN = "SCF" 

# ID Aleatorio
CLIENT_ID = f"PC_Manolo_Listener_{random.randint(0, 10000)}"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"--- [PC] CONECTADO A MOSQUITTO ({BROKER}) ---")
        print(f"--- [PC] Escuchando en el topic: '{TOPIC_TO_LISTEN}' ---")
        client.subscribe(TOPIC_TO_LISTEN)
    else:
        print(f"--- [PC] Error de conexión: {rc} ---")

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode('utf-8')
        print(f"\n[RECIBIDO] 📩 Topic: {msg.topic}")
        print(f"Payload: {payload_str}")
        
    except Exception as e:
        print(f"Error procesando mensaje: {e}")

# Configuración del cliente
client = mqtt.Client(client_id=CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message

print(f"Conectando a {BROKER}...")

try:
    client.connect(BROKER, PORT, 60)
    client.loop_forever()
except Exception as e:
    print(f"Error de conexión: {e}")
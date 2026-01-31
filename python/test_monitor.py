import paho.mqtt.client as mqtt
import json
import random

# --- CONFIGURACIÓN (Coincidiendo con tu STM32) ---
# Usamos la IP de broker.emqx.io para evitar DNS lookup, igual que en el C
BROKER = "35.172.255.228" 
PORT = 1883

# El topic que aparece en tu log: [MQTT TX] T: SCF/test/sim
TOPIC_TO_LISTEN = "SCF/test/sim" 

# Generamos un ID aleatorio para el PC para no chocar con el STM32
CLIENT_ID = f"PC_Monitor_{random.randint(0, 1000)}"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"--- [PC] CONECTADO AL BROKER (EMQX) ---")
        print(f"--- [PC] Escuchando en: {TOPIC_TO_LISTEN} ---")
        client.subscribe(TOPIC_TO_LISTEN)
    else:
        print(f"--- [PC] Error de conexión: {rc} ---")

def on_message(client, userdata, msg):
    try:
        # Decodificamos el mensaje
        payload_str = msg.payload.decode('utf-8')
        print(f"\n[RECIBIDO] Topic: {msg.topic}")
        print(f"Raw Payload: {payload_str}")
        
        # Intentamos parsear JSON
        # Tu JSON es: {"id": 0, "val": 25.5, "status": "RUN"}
        data = json.loads(payload_str)
        
        id_dato = data.get('id')
        val_dato = data.get('val')
        print(f"-> ID: {id_dato} | Valor: {val_dato}")
        
    except Exception as e:
        print(f"Error decodificando: {e}")

# Configuración del cliente
client = mqtt.Client(client_id=CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message

print(f"Conectando a {BROKER}...")
try:
    client.connect(BROKER, PORT, 60)
    # Bucle infinito
    client.loop_forever()
except Exception as e:
    print(f"Error al conectar desde el PC: {e}")
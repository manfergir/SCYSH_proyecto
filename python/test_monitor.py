import paho.mqtt.client as mqtt
import json

# --- CONFIGURACIÓN (Debe coincidir con mqtt_priv_config.h del STM32) ---
BROKER = "test.mosquitto.org"
PORT = 1883
TOPIC_TO_LISTEN = "bridge/test/sim"  # El mismo que pusimos en el C

def on_connect(client, userdata, flags, rc):
    print(f"--- CONECTADO AL BROKER (Código: {rc}) ---")
    print(f"Escuchando en: {TOPIC_TO_LISTEN}")
    client.subscribe(TOPIC_TO_LISTEN)

def on_message(client, userdata, msg):
    try:
        # Decodificamos el mensaje
        payload_str = msg.payload.decode('utf-8')
        print(f"\n[RECIBIDO] Topic: {msg.topic}")
        print(f"Datos: {payload_str}")
        
        # Intentamos parsear JSON para ver si está bien formado
        data = json.loads(payload_str)
        print(f"-> Contador: {data.get('cnt')}")
        
    except Exception as e:
        print(f"Error decodificando: {e}")

# Configuración del cliente
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(f"Conectando a {BROKER}...")
client.connect(BROKER, PORT, 60)

# Bucle infinito
client.loop_forever()
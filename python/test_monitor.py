import paho.mqtt.client as mqtt

# --- CONFIGURACIÓN ---
BROKER = "test.mosquitto.org"
PORT = 1883
TOPIC = "SCF"

# --- FUNCIONES CALLBACK ---

# 1. Qué hacer cuando nos conectamos
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"✅ Conectado al Broker. Escuchando en: {TOPIC}")
        # IMPORTANTE: Nos suscribimos DENTRO de on_connect.
        # Si la conexión se cae y vuelve, esto nos resuscribe automáticamente.
        client.subscribe(TOPIC)
    else:
        print(f"❌ Error de conexión: {rc}")

# 2. Qué hacer cuando llega un mensaje
def on_message(client, userdata, msg):
    # El mensaje viene en bytes, hay que decodificarlo a texto
    mensaje = msg.payload.decode()
    topic = msg.topic
    print(f"📩 DATO RECIBIDO -> Tema: {topic} | Valor: {mensaje}")

# --- CONFIGURACIÓN DEL CLIENTE ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# --- ARRANQUE ---
print("Conectando al servidor...")
client.connect(BROKER, PORT, 60)

# Usamos loop_forever() para un script que SOLO recibe datos.
# Bloquea el programa aquí y mantiene la conexión viva para siempre
# (hasta que pulses Ctrl + C)
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nDesconectando...")
    client.disconnect()
import paho.mqtt.client as mqtt

BROKER = "test.mosquitto.org"
PORT = 1883
TOPIC = "bridge/cmd/1"

payload = "READ"   # o "MODO::NORMAL" o "ACC::READ"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, PORT, 60)

# publicar
result = client.publish(TOPIC, payload, qos=0, retain=False)
result.wait_for_publish()

print(f"Publicado en {TOPIC}: {payload}")
client.disconnect()

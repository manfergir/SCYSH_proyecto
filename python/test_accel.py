import paho.mqtt.client as mqtt
import json
import time

# --- CONFIGURACIÓN ---
BROKER = "test.mosquitto.org"
PORT = 1883
# Asegúrate de que este topic coincide con TOPIC_PUB_ACCEL_PREFIX en tu C
# En tu código C veo: TOPIC_PUB_ACCEL_PREFIX. 
# Si en tu common_def.h es "SCF/Accel", pon eso aquí.
# Voy a asumir uno genérico basado en tu ejemplo, CAMBIALO si es distinto.
TOPIC = "bridge/accel/" 

# --- VARIABLES GLOBALES PARA RECONSTRUCCIÓN (Opcional) ---
current_buffer = []
expected_chunks = 0
received_chunks = 0

# --- FUNCIONES CALLBACK ---

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"✅ Conectado al Broker {BROKER}. Suscrito a: {TOPIC}")
        client.subscribe(TOPIC)
    else:
        print(f"❌ Error de conexión: {rc}")

def on_message(client, userdata, msg):
    global current_buffer, expected_chunks, received_chunks
    
    try:
        # 1. Decodificar payload
        payload_str = msg.payload.decode()
        data = json.loads(payload_str)
        
        # 2. Extraer metadatos del protocolo que definiste en C
        # {"t0":123, "fs":52, "s":0, "n":16, "z":[...]}
        ts_inicio = data.get("t0", 0)
        fs_hz = data.get("fs", 52)
        seq_num = data.get("s", 0)      # Número de secuencia del chunk
        total_chunks = data.get("n", 1) # Total de chunks esperados
        muestras_z = data.get("z", [])
        
        num_muestras = len(muestras_z)
        
        # 3. Lógica de visualización
        print(f"\n📦 [RECV] T0:{ts_inicio}ms | Chunk {seq_num + 1}/{total_chunks} | {num_muestras} muestras")
        print(f"   Datos Z (mg): {muestras_z}")

        # 4. (Opcional) Lógica simple de reconstrucción
        # Si es el primer paquete (seq=0), reiniciamos el buffer
        if seq_num == 0:
            current_buffer = []
            expected_chunks = total_chunks
            received_chunks = 0
            print("   🔄 Inicio de nuevo bloque de captura...")

        # Añadimos datos al buffer
        current_buffer.extend(muestras_z)
        received_chunks += 1

        # Si hemos completado el bloque
        if received_chunks == expected_chunks:
            print(f"   ✅ ¡BLOQUE COMPLETO RECIBIDO! Total {len(current_buffer)} muestras.")
            # Aquí podrías guardar en CSV o graficar
            print("-" * 60)

    except json.JSONDecodeError:
        print(f"⚠️ Error: Mensaje recibido no es un JSON válido: {msg.payload}")
    except Exception as e:
        print(f"⚠️ Error procesando datos: {e}")

# --- CONFIGURACIÓN DEL CLIENTE ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# --- ARRANQUE ---
print("Conectando al servidor...")
try:
    client.connect(BROKER, PORT, 60)
    client.loop_forever()
except KeyboardInterrupt:
    print("\nDesconectando...")
    client.disconnect()
except Exception as e:
    print(f"Error fatal: {e}")
/*
 * common_def.h
 *
 * DEFINICIONES COMUNES PARA EL SISTEMA DISTRIBUIDO (PROYECTO FINAL)
 * Este archivo debe ser idéntico en el Nodo 1 (Acelerómetro) y Nodo 2 (Ambiente).
 */

#ifndef INC_COMMON_DEF_H_
#define INC_COMMON_DEF_H_

#include <stdint.h>

/* ==============================================================================
 * 1. DEFINICIONES DE TAMAÑOS Y LÍMITES
 * ============================================================================== */
// Tamaño máximo para el nombre del topic (ej: "bridge/accel/1")
#define MSG_TOPIC_SIZE    32

// Tamaño máximo de la carga útil (JSON).
// Calculado para que quepa un bloque de 64 muestras int16 (128 bytes)
// o un JSON de temperatura/humedad extenso.
#define MSG_PAYLOAD_SIZE  256

// Tamaño total del elemento de la cola (Debe coincidir con "Item Size" en el .ioc)
// 32 + 128 = 160 bytes.
#define QUEUE_TX_ITEM_SIZE 160


/* ==============================================================================
 * 2. ESTRUCTURA DE DATOS (COLA DE ENVÍO)
 * ============================================================================== */
/**
 * @brief Estructura que viaja por la cola qMqttTx hacia la tarea de comunicaciones.
 * Todos los nodos (Accel, Env, Console) deben rellenar esto para enviar.
 */
typedef struct {
    char topic[MSG_TOPIC_SIZE];      // Canal MQTT de destino
    char payload[MSG_PAYLOAD_SIZE];  // Datos formateados (JSON o Raw bytes)
} MqttMsg_t;


/* ==============================================================================
 * 3. COMANDOS DEL SISTEMA (COLA DE RECEPCIÓN)
 * ============================================================================== */
/**
 * @brief Órdenes que pueden llegar desde el PC (MQTT) o Consola (UART)
 * hacia las tareas de control de sensores.
 */
typedef enum {
    CMD_NOP = 0,            // No operation
    CMD_START_CONTINUOUS,   // Orden para activar modo ráfaga (Solo Nodo 1)
    CMD_STOP_CONTINUOUS,    // Orden para volver a modo normal (Solo Nodo 1)
    CMD_FORCE_READ          // Forzar lectura inmediata (Debug / Botón)
} SystemCommand_t;


/* ==============================================================================
 * 4. NOTIFICACIONES DE TAREAS (BIT FLAGS)
 * ============================================================================== */
// Usar con osThreadFlagsSet() y osThreadFlagsWait() para despertar a las tareas.
// Sustituyen a los semáforos binarios por eficiencia.

#define NOTE_RTC_WAKEUP  (1 << 0)  // (0x01) Salta alarma RTC (cada 30 min)
#define NOTE_FIFO_FULL   (1 << 1)  // (0x02) Interrupción del Sensor (FIFO llena / Data Ready)
#define NOTE_CMD_RX      (1 << 2)  // (0x04) Ha llegado una orden por la cola qCmdRx
#define NOTE_BUTTON_IRQ  (1 << 3)  // (0x08) Pulsación del botón de usuario


/* ==============================================================================
 * 5. DEFINICIÓN DE TOPICS MQTT (JERARQUÍA)
 * ============================================================================== */
// Identificadores de los Nodos
#define NODE_ID_ACCEL  "1"
#define NODE_ID_ENV    "2"

// Prefijos para construir los topics
// Uso: sprintf(msg.topic, "%s%s", TOPIC_PUB_ACCEL_PREFIX, NODE_ID_ACCEL);
#define TOPIC_PUB_ACCEL_PREFIX  "bridge/accel/"  // + ID -> "bridge/accel/1"
#define TOPIC_PUB_ENV_PREFIX    "bridge/env/"    // + ID -> "bridge/env/2"
#define TOPIC_SUB_CMD_PREFIX    "bridge/cmd/"    // + ID -> "bridge/cmd/1" o "bridge/cmd/+"


/* ==============================================================================
 * 6. VARIABLES GLOBALES COMPARTIDAS (EXTERN)
 * ============================================================================== */
// Estas variables se definen en main.c, pero se usan en otras tareas.

// Estado de la conexión WiFi (0: Desconectado, 1: Conectado)
extern volatile uint8_t WIFI_IS_CONNECTED;

// Estado del servicio MQTT (0: No listo, 1: Listo para enviar)
// Si es 0, las tareas de sensor no deberían intentar escribir en la cola.
extern volatile uint8_t NET_MQTT_OK;

#endif /* INC_COMMON_DEF_H_ */

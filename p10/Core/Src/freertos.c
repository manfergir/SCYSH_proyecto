/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for WifiTask */
osThreadId_t WifiTaskHandle;
const osThreadAttr_t WifiTask_attributes = {
  .name = "WifiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MQTT_Task */
osThreadId_t MQTT_TaskHandle;
const osThreadAttr_t MQTT_Task_attributes = {
  .name = "MQTT_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task_envRead */
osThreadId_t task_envReadHandle;
const osThreadAttr_t task_envRead_attributes = {
  .name = "task_envRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Accel_Task */
osThreadId_t Accel_TaskHandle;
const osThreadAttr_t Accel_Task_attributes = {
  .name = "Accel_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Uart_Task */
osThreadId_t Uart_TaskHandle;
const osThreadAttr_t Uart_Task_attributes = {
  .name = "Uart_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for qMqttTx */
osMessageQueueId_t qMqttTxHandle;
const osMessageQueueAttr_t qMqttTx_attributes = {
  .name = "qMqttTx"
};
/* Definitions for qCmdRx */
osMessageQueueId_t qCmdRxHandle;
const osMessageQueueAttr_t qCmdRx_attributes = {
  .name = "qCmdRx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartWifiTask(void *argument);
void MQTT_TaskFun(void *argument);
void task_envReadFunc(void *argument);
void Accel_Task_fun(void *argument);
void Uart_Task_Fun(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qMqttTx */
  qMqttTxHandle = osMessageQueueNew (16, 160, &qMqttTx_attributes);

  /* creation of qCmdRx */
  qCmdRxHandle = osMessageQueueNew (5, sizeof(uint8_t), &qCmdRx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of WifiTask */
  WifiTaskHandle = osThreadNew(StartWifiTask, NULL, &WifiTask_attributes);

  /* creation of MQTT_Task */
  MQTT_TaskHandle = osThreadNew(MQTT_TaskFun, NULL, &MQTT_Task_attributes);

  /* creation of task_envRead */
  task_envReadHandle = osThreadNew(task_envReadFunc, NULL, &task_envRead_attributes);

  /* creation of Accel_Task */
  Accel_TaskHandle = osThreadNew(Accel_Task_fun, NULL, &Accel_Task_attributes);

  /* creation of Uart_Task */
  Uart_TaskHandle = osThreadNew(Uart_Task_Fun, NULL, &Uart_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartWifiTask */
/**
  * @brief  Function implementing the WifiTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartWifiTask */
void StartWifiTask(void *argument)
{
  /* USER CODE BEGIN StartWifiTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartWifiTask */
}

/* USER CODE BEGIN Header_MQTT_TaskFun */
/**
* @brief Function implementing the MQTT_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MQTT_TaskFun */
void MQTT_TaskFun(void *argument)
{
  /* USER CODE BEGIN MQTT_TaskFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MQTT_TaskFun */
}

/* USER CODE BEGIN Header_task_envReadFunc */
/**
* @brief Function implementing the task_envRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_envReadFunc */
void task_envReadFunc(void *argument)
{
  /* USER CODE BEGIN task_envReadFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_envReadFunc */
}

/* USER CODE BEGIN Header_Accel_Task_fun */
/**
* @brief Function implementing the Accel_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Accel_Task_fun */
void Accel_Task_fun(void *argument)
{
  /* USER CODE BEGIN Accel_Task_fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Accel_Task_fun */
}

/* USER CODE BEGIN Header_Uart_Task_Fun */
/**
* @brief Function implementing the Uart_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart_Task_Fun */
void Uart_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Uart_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Uart_Task_Fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


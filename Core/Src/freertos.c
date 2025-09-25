/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "queue.h"
#include "stdio.h"
#include "PID.h" 
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
extern QueueHandle_t receiveDataQueue;
// PID控制器实例
extern PID_Controller pid_pitch;
extern PID_Controller pid_yaw;
extern uint8_t uartTxReady;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes = {
  .name = "PID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendData */
osThreadId_t SendDataHandle;
const osThreadAttr_t SendData_attributes = {
  .name = "SendData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sendDataQueue */
osMessageQueueId_t sendDataQueueHandle;
const osMessageQueueAttr_t sendDataQueue_attributes = {
  .name = "sendDataQueue"
};
/* Definitions for htim_gimbal_ctrl */
osTimerId_t htim_gimbal_ctrlHandle;
const osTimerAttr_t htim_gimbal_ctrl_attributes = {
  .name = "htim_gimbal_ctrl"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void send_motor_cmd(uint8_t direction, uint16_t speed, uint8_t accel_level,
                   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void Callback01(void *argument);

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

  /* Create the timer(s) */
  /* creation of htim_gimbal_ctrl */
  htim_gimbal_ctrlHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &htim_gimbal_ctrl_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(htim_gimbal_ctrlHandle, 100);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sendDataQueue */
  sendDataQueueHandle = osMessageQueueNew (6, sizeof(AngleData_t), &sendDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PID */
  PIDHandle = osThreadNew(StartTask02, NULL, &PID_attributes);

  /* creation of SendData */
  SendDataHandle = osThreadNew(StartTask03, NULL, &SendData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	printf("FreeRTOS is running...\r\n");
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  /* Infinite loop */
  for(;;)
  {
	  SensorData_t RecvData;
	  AngleData_t SendData;
	  int16_t target_yaw_deg = 0;
	  int16_t target_pitch_deg = 0;
	  if (xQueueReceive(receiveDataQueue, &RecvData, portMAX_DELAY) == pdPASS) {
		  	if (RecvData.x < 10 && RecvData.x > -10) RecvData.x = 0;
			if (RecvData.y < 10 && RecvData.y > -10) RecvData.y = 0;

			target_yaw_deg = RecvData.x / 1;
			target_pitch_deg = RecvData.y / 1;

			//printf("x: %d, y: %d\r\n", target_yaw_deg, target_pitch_deg);
	  }
	  SendData.yaw = PID_Compute(&pid_yaw, target_yaw_deg);
	  SendData.pitch = PID_Compute(&pid_pitch, target_pitch_deg);

	  // 发送计算后的角度数据到发送队列
	  // 放数据前判断队列是否已满，满了就先取出一个
	  if (osMessageQueueGetCount(sendDataQueueHandle) == osMessageQueueGetCapacity(sendDataQueueHandle)) {
		  AngleData_t dummy;
		  osMessageQueueGet(sendDataQueueHandle, &dummy, NULL, 0); // 丢弃最旧数据
	  }
	  osMessageQueuePut(sendDataQueueHandle, &SendData, 0, 0);
	osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the SendData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	// 从发送队列中接收数据
	AngleData_t SendData;
	if (osMessageQueueGet(sendDataQueueHandle, &SendData, NULL, osWaitForever) == osOK) {
		// 发送数据处理，例如通过UART发送
		uint16_t speed = 5000;
		if(SendData.yaw == 0){
			uint8_t stop[5] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
			HAL_UART_Transmit_DMA(&huart2, stop, 5);
		}
		else if(SendData.yaw > 0){//正转
				uint32_t outyaw = (uint32_t)(142.0f * SendData.yaw * 6.0f);
				send_motor_cmd(0x00, speed, 0xDD, outyaw, 0x00, 0x00);
		}
		else if(SendData.yaw < 0){//反转
				SendData.yaw = -SendData.yaw;
				uint32_t outyaw = (uint32_t)(142.0f * SendData.yaw * 6.0f);
				send_motor_cmd(0x01, speed, 0xDD, outyaw, 0x00, 0x00);
		}
		printf("Yaw: %.2f, Pitch: %.2f\r\n", SendData.yaw, SendData.pitch);
	}
    osDelay(20);
  }
  /* USER CODE END StartTask03 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void send_motor_cmd(uint8_t direction, uint16_t speed, uint8_t accel_level,
                   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable) {
    if(uartTxReady == 0) { // 只有在上次发送完成后才发送新指令
        static uint8_t cmd[13];
        cmd[0] = 0x01;
        cmd[1] = 0xFD;
        cmd[2] = direction;
        cmd[3] = (speed >> 8) & 0xFF;
        cmd[4] = speed & 0xFF;
        cmd[5] = accel_level;
        cmd[6] = (pulse_count >> 24) & 0xFF;
        cmd[7] = (pulse_count >> 16) & 0xFF;
        cmd[8] = (pulse_count >> 8) & 0xFF;
        cmd[9] = pulse_count & 0xFF;
        cmd[10] = pos_mode;
        cmd[11] = sync_enable;
        cmd[12] = 0x6B;
        HAL_UART_Transmit_DMA(&huart2, cmd, 13);
        uartTxReady = 1; // 标记正在发送，等待发送完成
    }
}
/* USER CODE END Application */


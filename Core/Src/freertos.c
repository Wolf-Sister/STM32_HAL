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
// 事件组标志位
#define EVENT_BIT_0 (1 << 0) // 偏航云台急停
#define EVENT_BIT_1 (1 << 1) // 俯仰云台急停
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
/* Definitions for SendyawData */
osThreadId_t SendyawDataHandle;
const osThreadAttr_t SendyawData_attributes = {
  .name = "SendyawData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendpitchData */
osThreadId_t SendpitchDataHandle;
const osThreadAttr_t SendpitchData_attributes = {
  .name = "SendpitchData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sendyawDataQueue */
osMessageQueueId_t sendyawDataQueueHandle;
const osMessageQueueAttr_t sendyawDataQueue_attributes = {
  .name = "sendyawDataQueue"
};
/* Definitions for sendpitchDataQueue */
osMessageQueueId_t sendpitchDataQueueHandle;
const osMessageQueueAttr_t sendpitchDataQueue_attributes = {
  .name = "sendpitchDataQueue"
};
/* Definitions for htim_gimbal_ctrl */
osTimerId_t htim_gimbal_ctrlHandle;
const osTimerAttr_t htim_gimbal_ctrl_attributes = {
  .name = "htim_gimbal_ctrl"
};
/* Definitions for UART6 */
osMutexId_t UART6Handle;
const osMutexAttr_t UART6_attributes = {
  .name = "UART6"
};
/* Definitions for UART2 */
osMutexId_t UART2Handle;
const osMutexAttr_t UART2_attributes = {
  .name = "UART2"
};
/* Definitions for sendyawSemHandle */
osSemaphoreId_t sendyawSemHandleHandle;
const osSemaphoreAttr_t sendyawSemHandle_attributes = {
  .name = "sendyawSemHandle"
};
/* Definitions for sendpitchSemHandle */
osSemaphoreId_t sendpitchSemHandleHandle;
const osSemaphoreAttr_t sendpitchSemHandle_attributes = {
  .name = "sendpitchSemHandle"
};
/* Definitions for motor_stop */
osEventFlagsId_t motor_stopHandle;
const osEventFlagsAttr_t motor_stop_attributes = {
  .name = "motor_stop"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void send_gimbal_motor_cmd(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t accel_level,
				   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable);
void send_gimbal_motor_cmd(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t accel_level,
				   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
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
  /* Create the mutex(es) */
  /* creation of UART6 */
  UART6Handle = osMutexNew(&UART6_attributes);

  /* creation of UART2 */
  UART2Handle = osMutexNew(&UART2_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sendyawSemHandle */
  sendyawSemHandleHandle = osSemaphoreNew(1, 1, &sendyawSemHandle_attributes);

  /* creation of sendpitchSemHandle */
  sendpitchSemHandleHandle = osSemaphoreNew(1, 1, &sendpitchSemHandle_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of htim_gimbal_ctrl */
  htim_gimbal_ctrlHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &htim_gimbal_ctrl_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(htim_gimbal_ctrlHandle, 50);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sendyawDataQueue */
  sendyawDataQueueHandle = osMessageQueueNew (6, sizeof(float), &sendyawDataQueue_attributes);

  /* creation of sendpitchDataQueue */
  sendpitchDataQueueHandle = osMessageQueueNew (6, sizeof(float), &sendpitchDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PID */
  PIDHandle = osThreadNew(StartTask02, NULL, &PID_attributes);

  /* creation of SendyawData */
  SendyawDataHandle = osThreadNew(StartTask03, NULL, &SendyawData_attributes);

  /* creation of SendpitchData */
  SendpitchDataHandle = osThreadNew(StartTask04, NULL, &SendpitchData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of motor_stop */
  motor_stopHandle = osEventFlagsNew(&motor_stop_attributes);

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
	//获取互斥量，获取成功后打印调试信息，然后释放互斥量
	if (osMutexAcquire(UART6Handle, osWaitForever) == osOK) {
		printf("FreeRTOS is running...\r\n");
		osMutexRelease(UART6Handle);
	}
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
	  float target_yaw_deg = 0;
	  float target_pitch_deg = 0;
	  if (xQueueReceive(receiveDataQueue, &RecvData, portMAX_DELAY) == pdPASS) {
		  	if (RecvData.x < 10 && RecvData.x > -10){
				RecvData.x = 0;
				//标志位motor_stopHandle，表示云台急停
				osEventFlagsSet(motor_stopHandle, EVENT_BIT_0);
			}
			if (RecvData.y < 10 && RecvData.y > -10){
				RecvData.y = 0;
				//标志位motor_stopHandle，表示云台急停
				osEventFlagsSet(motor_stopHandle, EVENT_BIT_1);
			}

			target_yaw_deg = RecvData.x / 1.0;
			target_pitch_deg = RecvData.y / 1.0;

			//printf("x: %d, y: %d\r\n", target_yaw_deg, target_pitch_deg);
	  }
	  float yaw_out = PID_Compute(&pid_yaw, target_yaw_deg);
	  float pitch_out = PID_Compute(&pid_pitch, target_pitch_deg);

	  // 发送计算后的角度数据到发送队列
	  // yaw放数据前判断队列是否已满，满了就先取出一个
	  if (osMessageQueueGetCount(sendyawDataQueueHandle) == osMessageQueueGetCapacity(sendyawDataQueueHandle)) {
		  float dummy;
		  osMessageQueueGet(sendyawDataQueueHandle, &dummy, NULL, 0); // 丢弃最旧数据
	  }
	  osMessageQueuePut(sendyawDataQueueHandle, &yaw_out, 0, 0);
	  // pitch放数据前判断队列是否已满，满了就先取出一个
	  if (osMessageQueueGetCount(sendpitchDataQueueHandle) == osMessageQueueGetCapacity(sendpitchDataQueueHandle)) {
		  float dummy;
		  osMessageQueueGet(sendpitchDataQueueHandle, &dummy, NULL, 0); // 丢弃最旧数据
	  }
	  osMessageQueuePut(sendpitchDataQueueHandle, &pitch_out, 0, 0);
	osDelay(10);
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
    // 等待信号量，确保定时器周期到达后再发送数据
    if(osSemaphoreAcquire(sendyawSemHandleHandle, osWaitForever) == osOK){
        // 从发送队列中接收数据
	    float yaw;
	    if (osMessageQueueGet(sendyawDataQueueHandle, &yaw, NULL, osWaitForever) == osOK) {
			//判断云台状态
			uint32_t event_flags = osEventFlagsWait(motor_stopHandle, EVENT_BIT_0, osFlagsWaitAny, 0);
		    // 发送数据处理，例如通过UART发送
		    uint16_t speed = 500;
		    if(event_flags == EVENT_BIT_0){
			    send_gimbal_motor_cmd(0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00);
		    }
			else {
		    	if(yaw > 0){//正转
					uint32_t outyaw = (uint32_t)(142.0f * yaw * 6.0f);
					send_gimbal_motor_cmd(0x01, 0x00, speed, 0xDD, outyaw, 0x00, 0x00);
		    	}
		    	else{//反转
					yaw = -yaw;
					uint32_t outyaw = (uint32_t)(142.0f * yaw * 6.0f);
					send_gimbal_motor_cmd(0x01, 0x01, speed, 0xDD, outyaw, 0x00, 0x00);
	        	}
			}
			//获取互斥量，获取成功后打印调试信息，然后释放互斥量
			if (osMutexAcquire(UART6Handle, osWaitForever) == osOK) {
			    printf("Yaw: %.2f\r\n", yaw);
			    osMutexRelease(UART6Handle);
		    }
	    }
    }
    osDelay(5);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the SendpitchData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
	// 等待信号量，确保定时器周期到达后再发送数据
    if(osSemaphoreAcquire(sendpitchSemHandleHandle, osWaitForever) == osOK){
        // 从发送队列中接收数据
	    float pitch;
	    if (osMessageQueueGet(sendpitchDataQueueHandle, &pitch, NULL, osWaitForever) == osOK) {
			//判断云台状态
			uint32_t event_flags = osEventFlagsWait(motor_stopHandle, EVENT_BIT_1, osFlagsWaitAny, 0);
		    // 发送数据处理，例如通过UART发送
		    uint16_t speed = 500;
		    if(event_flags == EVENT_BIT_1){
			    send_gimbal_motor_cmd(0x02, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00);
		    }
			else {
		    	if(pitch > 0){//正转
					uint32_t outpitch = (uint32_t)(142.0f * pitch * 6.0f);
					send_gimbal_motor_cmd(0x02, 0x01, speed, 0xDD, outpitch, 0x00, 0x00);
		    	}
		    	else{//反转
					pitch = -pitch;
					uint32_t outpitch = (uint32_t)(142.0f * pitch * 6.0f);
					send_gimbal_motor_cmd(0x02, 0x00, speed, 0xDD, outpitch, 0x00, 0x00);
	        	}
			}
			//获取互斥量，获取成功后打印调试信息，然后释放互斥量
			if (osMutexAcquire(UART6Handle, osWaitForever) == osOK) {
			    printf("Pitch: %.2f\r\n", pitch);
			    osMutexRelease(UART6Handle);
		    }
	    }
    }
    osDelay(5);
  }
  /* USER CODE END StartTask04 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
    //释放sendSemHandleHandle信号量
    osSemaphoreRelease(sendyawSemHandleHandle);
	//释放sendpitchSemHandle信号量
	osSemaphoreRelease(sendpitchSemHandleHandle);
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void send_motor_cmd(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t accel_level,
                   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable) {
    if(!uartTxReady) { // 只有在上次发送完成后才发送新指令
        static uint8_t cmd[13];
        cmd[0] = motor_id;
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
		if (osMutexAcquire(UART2Handle, osWaitForever) == osOK) {
        	HAL_UART_Transmit_DMA(&huart2, cmd, 13);
			osMutexRelease(UART2Handle);
		}
        uartTxReady = 1; // 标记正在发送，等待发送完成
    }
}

//云台电机控制发送函数
/**
* @brief 云台电机控制发送函数（带模式参数）
* @param motor_id: 电机ID，0x01为偏航云台，0x02为俯仰云台
* @param direction: 方向，0x00为正转，0x01为反转 0xFF为停止
* @param speed: 速度，单位为脉冲每秒（PPS）
* @param accel_level: 加速度等级
* @param pulse_count: 脉冲数，控制转动角度
* @param pos_mode: 位置模式
* @param sync_enable: 同步使能
* @retval None
*/
void send_gimbal_motor_cmd(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t accel_level,
				   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable) {
	if(direction != 0xFF) { // 启动控制
		send_motor_cmd(motor_id, direction, speed, accel_level, pulse_count, pos_mode, sync_enable);
	} 
	else {// 急停控制
		uint8_t stop[5] = {motor_id, 0xFE, 0x98, 0x00, 0x6B};
		if(uartTxReady == 0) { // 只有在上次发送完成后才发送新指令
			if (osMutexAcquire(UART2Handle, osWaitForever) == osOK) {
				HAL_UART_Transmit_DMA(&huart2, stop, 5);
				osMutexRelease(UART2Handle);
			}
			uartTxReady = 1; // 标记正在发送，等待发送完成
		}
	}
}
/* USER CODE END Application */


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
#include "queue.h"
#include "stdio.h"
#include "PID.h" 

/* Private variables ---------------------------------------------------------*/
extern QueueHandle_t receiveDataQueue;
extern PID_Controller pid_pitch;
extern PID_Controller pid_yaw;
extern uint8_t uartTxReady;

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
void send_motor_cmd(uint8_t direction, uint16_t speed, uint8_t accel_level,
                   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* Create the timer(s) */
  htim_gimbal_ctrlHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &htim_gimbal_ctrl_attributes);
  osTimerStart(htim_gimbal_ctrlHandle, 100);

  /* Create the queue(s) */
  sendDataQueueHandle = osMessageQueueNew(6, sizeof(AngleData_t), &sendDataQueue_attributes);

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  PIDHandle = osThreadNew(StartTask02, NULL, &PID_attributes);
  SendDataHandle = osThreadNew(StartTask03, NULL, &SendData_attributes);
}

/* Function implementing the defaultTask thread. */
void StartDefaultTask(void *argument) {
  for(;;) {
    printf("FreeRTOS is running...\r\n");
    osDelay(1000);
  }
}

/* Function implementing the PID thread. */
void StartTask02(void *argument) {
  for(;;) {
    SensorData_t RecvData;
    AngleData_t SendData;
    int16_t target_yaw_deg = 0;
    int16_t target_pitch_deg = 0;
    if (xQueueReceive(receiveDataQueue, &RecvData, portMAX_DELAY) == pdPASS) {
      if (RecvData.x < 10 && RecvData.x > -10) RecvData.x = 0;
      if (RecvData.y < 10 && RecvData.y > -10) RecvData.y = 0;

      target_yaw_deg = RecvData.x / 1;
      target_pitch_deg = RecvData.y / 1;
    }
    SendData.yaw = PID_Compute(&pid_yaw, target_yaw_deg);
    SendData.pitch = PID_Compute(&pid_pitch, target_pitch_deg);

    if (osMessageQueueGetCount(sendDataQueueHandle) == osMessageQueueGetCapacity(sendDataQueueHandle)) {
      AngleData_t dummy;
      osMessageQueueGet(sendDataQueueHandle, &dummy, NULL, 0);
    }
    osMessageQueuePut(sendDataQueueHandle, &SendData, 0, 0);
    osDelay(20);
  }
}

/* Function implementing the SendData thread. */
void StartTask03(void *argument) {
  for(;;) {
    AngleData_t SendData;
    if (osMessageQueueGet(sendDataQueueHandle, &SendData, NULL, osWaitForever) == osOK) {
      uint16_t speed = 5000;
      if(SendData.yaw == 0) {
        uint8_t stop[5] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
        HAL_UART_Transmit_DMA(&huart2, stop, 5);
      } else if(SendData.yaw > 0) {
        uint32_t outyaw = (uint32_t)(142.0f * SendData.yaw * 6.0f);
        send_motor_cmd(0x00, speed, 0xDD, outyaw, 0x00, 0x00);
      } else if(SendData.yaw < 0) {
        SendData.yaw = -SendData.yaw;
        uint32_t outyaw = (uint32_t)(142.0f * SendData.yaw * 6.0f);
        send_motor_cmd(0x01, speed, 0xDD, outyaw, 0x00, 0x00);
      }
      printf("Yaw: %.2f, Pitch: %.2f\r\n", SendData.yaw, SendData.pitch);
    }
    osDelay(20);
  }
}

/* Callback01 function */
void Callback01(void *argument) {
}

/* Private application code --------------------------------------------------*/
void send_motor_cmd(uint8_t direction, uint16_t speed, uint8_t accel_level,
                   uint32_t pulse_count, uint8_t pos_mode, uint8_t sync_enable) {
    if(uartTxReady == 0) {
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
        uartTxReady = 1;
    }
}
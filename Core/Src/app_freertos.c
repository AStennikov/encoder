/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "usart.h"
#include "semphr.h"
#include "queue.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "hall_sensor.h"
#include <stdio.h>
#include <string.h>
#include "motor.h"
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
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE END Variables */
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes = {
  .name = "PID",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for greenLED_Blink */
osThreadId_t greenLED_BlinkHandle;
const osThreadAttr_t greenLED_Blink_attributes = {
  .name = "greenLED_Blink",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void thread3(void *argument);
/* USER CODE END FunctionPrototypes */

void threadPID_Loop(void *argument);
void thread2(void *argument);
void threadGreenLED(void *argument);

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
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

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
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (100, sizeof(uint8_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PID */
  PIDHandle = osThreadNew(threadPID_Loop, NULL, &PID_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(thread2, NULL, &myTask02_attributes);

  /* creation of greenLED_Blink */
  greenLED_BlinkHandle = osThreadNew(threadGreenLED, NULL, &greenLED_Blink_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  myTask03Handle = osThreadNew(thread3, NULL, &myTask03_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_threadPID_Loop */
/**
  * @brief  Function implementing the PID thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_threadPID_Loop */
void threadPID_Loop(void *argument)
{
  /* USER CODE BEGIN threadPID_Loop */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;	// TEMPORARY, set to 10ms after no UART interaction is needed
	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	int16_t hallSensorValues[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	motorEnable();
	motorSet(0);


	/* Infinite loop */
	for(;;)
	{
		// rough pseudocode:
		// 		get hall sensor readings
		// 		calculate position
		// 		calculate PID parameters
		// 		set PWM
		// 		wait until next cycle
		updateHallSensorValues(hallSensorValues);

		// print number through uart
		char str[130] = "";
		for (uint8_t i=0; i<20; ++i) {
			char number[8] = "";
			sprintf(number, "%4.4d", hallSensorValues[i]);
			strncat(str, number, 6);
			if (i < 19) {
				strncat(str, ",", 2);
			} else {
				strncat(str, "\n\r", 5);
			}
		}
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 1);
		xSemaphoreGive(uartMutexHandle);



		//uint8_t str[102];
		//sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d\n\r", hallSensorValues[0], hallSensorValues[1], hallSensorValues[2], hallSensorValues[3]);
		/*sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d %4.4d\n\r",
				hallSensorValues[0],
				hallSensorValues[1],
				hallSensorValues[2],
				hallSensorValues[3],
				hallSensorValues[4],
				hallSensorValues[5],
				hallSensorValues[6],
				hallSensorValues[7],
				hallSensorValues[8],
				hallSensorValues[9],
				hallSensorValues[10],
				hallSensorValues[11],
				hallSensorValues[12],
				hallSensorValues[13],
				hallSensorValues[14],
				hallSensorValues[15],
				hallSensorValues[16],
				hallSensorValues[17],
				hallSensorValues[18],
				hallSensorValues[19]);*/
		/*xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		//HAL_UART_Transmit(&huart1, str, 101, 100);
		HAL_UART_Transmit(&huart1, str, 21, 100);
		xSemaphoreGive(uartMutexHandle);
		sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d\n\r", hallSensorValues[4], hallSensorValues[5], hallSensorValues[6], hallSensorValues[7]);
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, str, 21, 100);
		xSemaphoreGive(uartMutexHandle);
		sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d\n\r", hallSensorValues[8], hallSensorValues[9], hallSensorValues[10], hallSensorValues[11]);
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, str, 21, 100);
		xSemaphoreGive(uartMutexHandle);
		sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d\n\r", hallSensorValues[12], hallSensorValues[13], hallSensorValues[14], hallSensorValues[15]);
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, str, 21, 100);
		xSemaphoreGive(uartMutexHandle);
		sprintf((char*) str, "%4.4d %4.4d %4.4d %4.4d\n\r", hallSensorValues[16], hallSensorValues[17], hallSensorValues[18], hallSensorValues[19]);
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, str, 21, 100);
		xSemaphoreGive(uartMutexHandle);
		xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) "--------\n\r", 10, 100);
		xSemaphoreGive(uartMutexHandle);*/

		vTaskDelayUntil(&xLastWakeTime, xFrequency);	// Wait for the next cycle.
	}
  /* USER CODE END threadPID_Loop */
}

/* USER CODE BEGIN Header_thread2 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thread2 */
void thread2(void *argument)
{
  /* USER CODE BEGIN thread2 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END thread2 */
}

/* USER CODE BEGIN Header_threadGreenLED */
/**
* @brief Function implementing the greenLED_Blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_threadGreenLED */
void threadGreenLED(void *argument)
{
  /* USER CODE BEGIN threadGreenLED */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);	// Wait for the next cycle.
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		osDelay(50);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		osDelay(350);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	}
  /* USER CODE END threadGreenLED */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void thread3(void *argument)
{
  /* Infinite loop */
	uint32_t notifValue;

	uint8_t msg;
	osStatus_t status;

  for(;;)
  {
	  xTaskNotifyWait(pdFALSE, 0xFF, &notifValue, portMAX_DELAY);	// waits for notification from another thread
	  if ((notifValue & 0x01) != 0x00) {	// if notification value is 0x01
		  xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		  HAL_UART_Transmit(&huart1, (uint8_t*) "thread3\n\r", 9, 1);
		  xSemaphoreGive(uartMutexHandle);

		  status = osMessageQueueGet(myQueue01Handle, &msg, NULL, 0U);   // wait for message
		  if (status == osOK) {
			  xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
			  HAL_UART_Transmit(&huart1, &msg, 1, 10);
			  xSemaphoreGive(uartMutexHandle);


		  }

		  /*uint8_t buffer;
		  xQueueReceive(Global_Queue_Handle, &buffer, 100);
		  xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		  HAL_UART_Transmit(&huart1, &buffer, 1, 10);
		  xSemaphoreGive(uartMutexHandle);*/
	  }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
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

void thread1(void *argument);
void thread2(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(thread1, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(thread2, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  myTask03Handle = osThreadNew(thread3, NULL, &myTask03_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_thread1 */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_thread1 */
void thread1(void *argument)
{
  /* USER CODE BEGIN thread1 */
	TickType_t xLastWakeTime;
	 const TickType_t xFrequency = 1000;
	 // Initialize the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  // Wait for the next cycle.
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );

	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	  osDelay(50);
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	  osDelay(100);
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	  osDelay(350);
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	  //osDelay(500);
	  xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
	  HAL_UART_Transmit(&huart1, (uint8_t*) "thread1\n\r", 9, 1);
	  xSemaphoreGive(uartMutexHandle);


	  uint8_t data = 0x41;
	  osMessageQueuePut (myQueue01Handle, &data, 0U, 10);

	  /*uint32_t data = 0x00000041;
	  xQueueSend(Global_Queue_Handle, &data, 100);*/

	  xTaskNotify(myTask03Handle, 0x01, eSetBits );

  }
  /* USER CODE END thread1 */
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

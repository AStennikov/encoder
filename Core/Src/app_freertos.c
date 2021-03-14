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
#include "fdcan.h"
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

	int16_t hallSensorValues[SENSOR_COUNT] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int32_t interpolatedHallSensorValues[INTERPOLATED_SENSOR_ARRAY_LENGTH];

	motorEnable();
	motorSetPWM(0);


	/*// CAN message for transmission
	FDCAN_TxHeaderTypeDef txh;
	txh.Identifier = 0x7ff;
	txh.IdType = FDCAN_STANDARD_ID;
	txh.TxFrameType = FDCAN_DATA_FRAME;
	txh.DataLength = FDCAN_DLC_BYTES_8;
	txh.FDFormat = FDCAN_CLASSIC_CAN;
	uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);*/


	double kp = 35;
	double ki = 0.07;
	double kd = 0;
	double kpe = 0;
	double kpi = 0;
	double kpd = 0;
	double error = 0;
	double error_i = 0;
	double error_d = 0;
	double last_error = 0;
	double pwm_d = 0;
	int16_t pwm_16 = 0;
	int16_t motor_direction = -1;

	TickType_t previous_time = xTaskGetTickCount();
	TickType_t current_time = xTaskGetTickCount();
	TickType_t elapsedTime = current_time - previous_time;



	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

		// rough pseudocode:
		// 		get hall sensor readings
		// 		calculate position
		// 		calculate PID parameters
		// 		set PWM
		// 		wait until next cycle

		getSensorValues(hallSensorValues);
		offsetSensorValues(hallSensorValues);
		interpolateSensorValues(hallSensorValues, interpolatedHallSensorValues);
		uint16_t currentPosition = calculateSensorPosition(interpolatedHallSensorValues);

		// pid loop
		current_time = xTaskGetTickCount();
		elapsedTime = current_time - previous_time;

		error = (double) (motorGetTarget() - currentPosition);
		error_i += (double) error*elapsedTime;
		//if (error < 4 && error > -4) {error_i = 0;}
		error_i = error_i*0.9;	// slow decay of integral component, needed to stop power to motor after it has reached its target
		if (elapsedTime != 0) {error_d = (error - last_error)/elapsedTime;}

		kpe = kp*error;
		// trimming error_i so that it does not go too far
		if (error_i > 255/ki) {error_i = 255/ki;}
		if (error_i < -255/ki) {error_i = -255/ki;}
		// zeroing error_i when error crosses zero
		if (error > 0 && error_i < 0) {error_i = 0;}
		if (error < 0 && error_i > 0) {error_i = 0;}
		kpi = ki*error_i;
		//if (kpi > 255) {kpi = 255;}
		//if (kpi < -255) {kpi = -255;}
		kpd = kd*error_d;
		pwm_d = kpe + kpi + kpd;
		// trimming
		if (pwm_d > 255) {pwm_d = 255;}
		if (pwm_d < -255) {pwm_d = -255;}
		pwm_16 = (int16_t) pwm_d;


		last_error = error;
		previous_time = current_time;

		motorSetPWM(motor_direction*pwm_16);




		/*if (currentPosition < motorGetTarget()-50) {
			motorSetPWM(-255);
		} else if (currentPosition > motorGetTarget()+50) {
			motorSetPWM(255);
		} else {
			motorSetPWM(0);
		}*/

		/*// print number through uart
		//char str[140] = "";
		for (uint8_t i=0; i<20; ++i) {
			char number[8] = "";
			sprintf(number, "%5.4d", hallSensorValues[i]);
			strncat(str, number, 6);
			if (i < 19) {
				strncat(str, ",", 2);
			} else {
				strncat(str, "\n\r", 5);
			}
		}*/
		//sprintf(str, "%d %d %d %d %d %d %d %d\r\n", hallSensorValues[0], hallSensorValues[1], hallSensorValues[2], hallSensorValues[3], hallSensorValues[4], hallSensorValues[5], hallSensorValues[6], hallSensorValues[7]);



		//uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
		// transmit telemetry data via CAN
		/*uint8_t data[8];
		data[0] = (uint8_t) (hallSensorValues[0] >> 8);
		data[1] = (uint8_t) (hallSensorValues[0] >> 0);
		data[2] = (uint8_t) (hallSensorValues[1] >> 8);
		data[3] = (uint8_t) (hallSensorValues[1] >> 0);
		data[4] = (uint8_t) (hallSensorValues[2] >> 8);
		data[5] = (uint8_t) (hallSensorValues[2] >> 0);
		data[6] = (uint8_t) (hallSensorValues[3] >> 8);
		data[7] = (uint8_t) (hallSensorValues[3] >> 0);
		CAN_SendSimple(CAN_SENSOR_GROUP_1_MSG_ID, 8, data);
		*/

		uint8_t data[8];
		int16_t kpi_16 = (int16_t) kpi;
		data[0] = (uint8_t) (motorGetTarget()>>0);
		data[1] = (uint8_t) (motorGetTarget()>>8);
		data[2] = (uint8_t) (currentPosition>>0);
		data[3] = (uint8_t) (currentPosition>>8);
		data[4] = (uint8_t) (pwm_16>>0);
		data[5] = (uint8_t) (pwm_16>>8);
		data[6] = (uint8_t) (kpi_16>>0);
		data[7] = (uint8_t) (kpi_16>>8);
		CAN_SendSimple(CAN_STATUS_MESSAGE_ID, 8, data);

		CAN_SendSimple(CAN_SENSOR_GROUP_1_MSG_ID, 8, (uint8_t*) &hallSensorValues[0]);
		CAN_SendSimple(CAN_SENSOR_GROUP_2_MSG_ID, 8, (uint8_t*) &hallSensorValues[4]);
		CAN_SendSimple(CAN_SENSOR_GROUP_3_MSG_ID, 8, (uint8_t*) &hallSensorValues[8]);
		CAN_SendSimple(CAN_SENSOR_GROUP_4_MSG_ID, 8, (uint8_t*) &hallSensorValues[12]);
		CAN_SendSimple(CAN_SENSOR_GROUP_5_MSG_ID, 8, (uint8_t*) &hallSensorValues[16]);


		// temporary, control motor from keys
		/*char input = 'w';
		HAL_UART_Receive(&huart1, (uint8_t*) &input, 1, 0);
		if (input == 'a') {
			motorSet(-150);
		} else if (input == 'd') {
			motorSet(150);
		} else if (input == '\n') {

		} else if (input == '\r') {

		} else if (input == '\0') {

		} else {
			motorSet(0);
		}*/

		/*xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 10);
		xSemaphoreGive(uartMutexHandle);*/


		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

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
		/*HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		osDelay(50);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		osDelay(350);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);*/
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

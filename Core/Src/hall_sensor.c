/*
 * hall_sensor.c
 *
 *  Created on: Jan 23, 2021
 *      Author: asten
 */


#include "hall_sensor.h"
#include "stm32g4xx_hal_conf.h"
#include "adc.h"
//#include "stm32g4xx_hal_adc.h"
//#include "stm32g4xx_hal_gpio.h"


/*
#define GROUP1_Pin GPIO_PIN_12
#define GROUP1_GPIO_Port GPIOB
#define GROUP2_Pin GPIO_PIN_13
#define GROUP2_GPIO_Port GPIOB
#define GROUP3_Pin GPIO_PIN_14
#define GROUP3_GPIO_Port GPIOB
#define GROUP4_Pin GPIO_PIN_15
#define GROUP4_GPIO_Port GPIOB
#define GROUP5_Pin GPIO_PIN_6
#define GROUP5_GPIO_Port GPIOC
*/

#define SENSOR_COUNT 20

// activates a certain sensor group
void activateSensorGroup(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay){
	// sets all pins low
	HAL_GPIO_WritePin(GROUP1_GPIO_Port, GROUP1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP2_GPIO_Port, GROUP2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP3_GPIO_Port, GROUP3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP4_GPIO_Port, GROUP4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP5_GPIO_Port, GROUP5_Pin, GPIO_PIN_RESET);

	// short delay
	for (volatile uint32_t i=0; i<delay; ++i){}

	// sets one specific group high
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	// short delay
	for (volatile uint32_t i=0; i<delay; ++i){}
}

// sensors are not located in a nice incremental order, so the readings are rearranged with the help of a look-up table
const uint32_t sensorOrderLUT[SENSOR_COUNT] = {0,1,2,3,7,6,5,4,8,9,10,11,15,14,13,12,16,17,18,19};

// sensors' offset voltage, obtained with sensorArrayOffset.py
const int16_t sensorOffsets[SENSOR_COUNT] = {1955, 1945, 1934, 1946, 1945, 1938, 1923, 1914, 1958, 1925, 1958, 1921,
		 1954, 1938, 1956, 1961, 1916, 1958, 1935, 1975};



void removeOffsets(int16_t* values) {
	for (uint16_t i=0; i<SENSOR_COUNT; ++i) {
			values[i]-= sensorOffsets[i];
	}
}

// reads ADC values and stores them into the provided array
void updateHallSensorValues(int16_t* values){
	int16_t buffer[SENSOR_COUNT];

	activateSensorGroup(GROUP1_GPIO_Port, GROUP1_Pin, 1000);	// enable group 1
	HAL_ADC_Start_DMA(&hadc2, (uint32_t* ) &(buffer[0]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP2_GPIO_Port, GROUP2_Pin, 1000);	// enable group 2
	HAL_ADC_Start_DMA(&hadc2, (uint32_t* ) &(buffer[4]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP3_GPIO_Port, GROUP3_Pin, 1000);	// enable group 3
	HAL_ADC_Start_DMA(&hadc2, (uint32_t* ) &(buffer[8]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP4_GPIO_Port, GROUP4_Pin, 1000);	// enable group 4
	HAL_ADC_Start_DMA(&hadc2, (uint32_t* ) &(buffer[12]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP5_GPIO_Port, GROUP5_Pin, 1000);	// enable group 5
	HAL_ADC_Start_DMA(&hadc2, (uint32_t* ) &(buffer[16]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	// rearranging the values according to the look-up table
	for (uint16_t i=0; i<SENSOR_COUNT; ++i) {
		values[i] = buffer[sensorOrderLUT[i]];
	}

	// removing offsets
	removeOffsets(values);



	//HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)

	//HAL_ADC_Start(ADC_HandleTypeDef *hadc);
	//HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout)
}






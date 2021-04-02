/*
 * hall_sensor.c
 *
 *  Created on: Jan 23, 2021
 *      Author: asten
 */


#include "hall_sensor.h"
#include "stm32g4xx_hal_conf.h"
#include "adc.h"
#include "sensor_positions.h"

// A139x sensors have a sleep mode with power-on time of 60us max and power off-time of 1us. These constants allow fine tuning delays between GPIO action
#define T_PON	1000
#define T_POFF	50

// for each position, stores difference between sensorValueTable[POSITION_COUNT][5] and the latest sensor reading
uint32_t diff[POSITION_COUNT];


// activates a certain sensor group
void activateSensorGroup(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	// sets all pins low
	HAL_GPIO_WritePin(GROUP1_GPIO_Port, GROUP1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP2_GPIO_Port, GROUP2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP3_GPIO_Port, GROUP3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP4_GPIO_Port, GROUP4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GROUP5_GPIO_Port, GROUP5_Pin, GPIO_PIN_RESET);

	// power-off delay
	for (volatile uint32_t i=0; i<T_POFF; ++i){}

	// sets one specific group high
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	// power-on delay
	for (volatile uint32_t i=0; i<T_PON; ++i){}
}




// reads hall sensor values
void updateSensorValues(uint32_t* sensorValues) {
	activateSensorGroup(GROUP1_GPIO_Port, GROUP1_Pin);	// enable group 1
	HAL_ADC_Start_DMA(&hadc2, &(sensorValues[0]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP2_GPIO_Port, GROUP2_Pin);	// enable group 2
	HAL_ADC_Start_DMA(&hadc2, &(sensorValues[1]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP3_GPIO_Port, GROUP3_Pin);	// enable group 3
	HAL_ADC_Start_DMA(&hadc2, &(sensorValues[2]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP4_GPIO_Port, GROUP4_Pin);	// enable group 4
	HAL_ADC_Start_DMA(&hadc2, &(sensorValues[3]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	activateSensorGroup(GROUP5_GPIO_Port, GROUP5_Pin);	// enable group 5
	HAL_ADC_Start_DMA(&hadc2, &(sensorValues[4]), 4);
	HAL_ADC_PollForConversion(&hadc2, 5);
	HAL_ADC_Stop_DMA(&hadc2);

	// Changing sensor value order for sensor groups 2 and 4. The sensors in these groups have reverse order because of PCB layout.
	sensorValues[1] = __REV(sensorValues[1]);
	sensorValues[3] = __REV(sensorValues[3]);
}

// calculates and returns encoder position within provided range
// offset: starting value in sensorValueTable
// length: how many values in a row are evaluated
// automatically loops if offset + length exceeds POSITION_COUNT
uint8_t calculateSensorPosition(uint32_t* sensorValues, uint8_t offset, uint8_t length) {
	uint32_t minimum = 0xFFFFFFFF;
	uint8_t minimumPos = offset;

	uint8_t position = offset;		// index of a 20-sensor snapshot in sensorValueTable[][] currently processed

	for (int32_t i=0; i<=length; ++i) {
		diff[position] = 0;
		diff[position] = __USADA8(sensorValues[0], sensorValueTable[position][0], diff[position]);
		diff[position] = __USADA8(sensorValues[1], sensorValueTable[position][1], diff[position]);
		diff[position] = __USADA8(sensorValues[2], sensorValueTable[position][2], diff[position]);
		diff[position] = __USADA8(sensorValues[3], sensorValueTable[position][3], diff[position]);
		diff[position] = __USADA8(sensorValues[4], sensorValueTable[position][4], diff[position]);


		// finding minimum
		if (diff[position] < minimum) {
			minimum = diff[position];
			minimumPos = position;
		}

		// incrementing position
		position += 1;
	}

	return minimumPos;
}







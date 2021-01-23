/*
 * hall_sensor.c
 *
 *  Created on: Jan 23, 2021
 *      Author: asten
 */


#include "hall_sensor.h"
#include "stm32g4xx_hal_conf.h"
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

// reads ADC values and stores them into the provided array
void updateHallSensorValues(uint16_t* values){



	HAL_ADC_Start(ADC_HandleTypeDef *hadc);

}
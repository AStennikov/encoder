/*
 * motor.c
 *
 *  Created on: Feb 13, 2021
 *      Author: asten
 */

#include "motor.h"
#include "stm32g4xx_hal_conf.h"
#include "tim.h"


// global variables
uint16_t motorTargetUpdated = 0;	// is set to 1 when motor target is updated
uint8_t motorTarget = 0;			// shows where rotary joint must be. Matches array index.
int8_t direction = -1;				// +1 or -1 sets motor rotation direction


// turns motor on, starts PWM
void motorEnable(){
	// set PMODE
	HAL_GPIO_WritePin(PMODE_GPIO_Port, PMODE_Pin, GPIO_PIN_SET);
	// set PWM values to 0
	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;

	// set nSLEEP
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_SET);

	// start the pwm
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

}

// due to friction losses, low pwm values are useless. This function translates them to start with 50%.
int32_t pwm_compensate_for_friction(int8_t pwmValue) {
	int32_t result = 0;

	if (pwmValue > 0) {
		result = (pwmValue) + 63;
	} else if (pwmValue < 0) {
		result = (pwmValue) - 64;
	} else {
		result = 0;
	}
	/*if (pwmValue > 0) {result = 127;}
	if (pwmValue < 0) {result = -128;}
	if (pwmValue == 0) {result = 0;}*/

	// clamp values
	if (result > 127) {result = 127;}
	if (result < -128) {result = -128;}

	return result;
}

// sets PWM to the provided value. Sign sets direction, (+) being forward. Range: -255 to 255
void motorSetPWM(int8_t pwmValue) {

	// find direction, set PWM channels (CH1 = IN2, CH2 = IN1)
	int32_t pwm = pwm_compensate_for_friction(pwmValue*direction);
	if (pwm > 0) {				// forward
		htim3.Instance->CCR1 = 0;
		htim3.Instance->CCR2 = pwm;
	} else if(pwm < 0) {		// reverse
		htim3.Instance->CCR2 = 0;
		htim3.Instance->CCR1 = pwm*(-1);
	} else {						// brake
		htim3.Instance->CCR1 = 255;
		htim3.Instance->CCR2 = 255;
	}

}

// sets target where rotary joint must be
void motorSetTarget(uint8_t newTargetValue) {
	motorTargetUpdated = 1;
	motorTarget = newTargetValue;
}

uint8_t motorGetTarget() {
	return motorTarget;
}


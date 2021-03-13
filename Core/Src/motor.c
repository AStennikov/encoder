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
uint16_t motorTarget = 0;			// shows where rotary joint must be. Matches array index.

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

// sets PWM to the provided value. Sign sets direction, (+) being forward. Range: -255 to 255
void motorSetPWM(int32_t pwmValue) {
	// clamp values within range
	if(pwmValue < -255) {pwmValue = -255;}
	if(pwmValue > 255) {pwmValue = 255;}

	// find direction, set PWM channels (CH1 = IN2, CH2 = IN1)
	if (pwmValue >= 0) {	// forward or no movement
		htim3.Instance->CCR1 = 0;
		htim3.Instance->CCR2 = pwmValue;
	} else {				// reverse
		htim3.Instance->CCR2 = 0;
		htim3.Instance->CCR1 = pwmValue*(-1);
	}

}

// sets target where rotary joint must be
void motorSetTarget(uint16_t *newTargetValue) {
	motorTargetUpdated = 1;
	motorTarget = *newTargetValue;
}

uint16_t motorGetTarget() {
	return motorTarget;
}


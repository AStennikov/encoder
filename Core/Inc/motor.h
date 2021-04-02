/*
 * motor.h
 *
 *  Created on: Feb 13, 2021
 *      Author: asten
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "main.h"


// turns motor on, starts PWM
void motorEnable();

// sets PWM to the provided value. Sign sets direction, (+) being forward.
void motorSetPWM(int8_t pwmValue);

// sets target where rotary joint must be
void motorSetTarget(uint8_t newTargetValue);

uint8_t motorGetTarget();

#endif /* INC_MOTOR_H_ */

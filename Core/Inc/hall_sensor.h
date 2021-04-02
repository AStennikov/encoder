/*
 * hall_sensor.h
 *
 *  Created on: Jan 23, 2021
 *      Author: asten
 */

#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include <stdint.h>
#include "main.h"

#define SENSOR_GROUP_COUNT 						5

// reads hall sensor values
void updateSensorValues(uint32_t* sensorValues);

// calculates and returns encoder position within provided range
// offset: starting value in sensorValueTable
// length: how many values in a row are evaluated
// automatically loops if offset + length exceeds POSITION_COUNT
uint8_t calculateSensorPosition(uint32_t* sensorValues, uint8_t offset, uint8_t length);




#endif /* INC_HALL_SENSOR_H_ */

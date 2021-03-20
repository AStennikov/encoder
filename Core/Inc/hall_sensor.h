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

/*#define SENSOR_COUNT 						20
#define INTERPOLATED_SENSOR_ARRAY_LENGTH 	SENSOR_COUNT*4-3
#define MAGNETIC_PATTERN_LENGTH				576

// reads hall sensor values
void getSensorValues(int16_t* values);

// applies offset to the sensor values to make average zero
void offsetSensorValues(int16_t* values);

// interpolates sensor values
void interpolateSensorValues(int16_t* originalValues, int32_t* interpolatedValues);

// finds sensor position
uint16_t calculateSensorPosition(int32_t* interpolatedValues);

//void updateHallSensorValues(int16_t* values);*/

// reads hall sensor values
void updateSensorValues();

// returns sensor reading
uint16_t sensorValue(uint32_t sensorNumber);

// writes sensor readings into an array
void sensorValues(uint16_t * sensorValues);

// calculates and returns encoder position within provided range
// offset: starting value in sensorValueTable
// length: how many values in a row are evaluated
// automatically loops if offset + length exceeds POSITION_COUNT
uint16_t calculateSensorPosition(uint16_t offset, uint16_t length);




#endif /* INC_HALL_SENSOR_H_ */

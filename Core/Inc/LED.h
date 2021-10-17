/*
 * LED.h
 *
 *  Created on: Oct 17, 2021
 *      Author: asten
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdint.h>
#include "main.h"

typedef struct {
  uint32_t pattern;
  uint32_t position;
}LED_t;

void LED_Init(LED_t *led, uint32_t pattern);

// shifts through LED pattern and returns whether GPIO should be set or reset
GPIO_PinState LED_NextState(LED_t *led);

#endif /* INC_LED_H_ */

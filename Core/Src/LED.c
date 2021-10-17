/*
 * LED.c
 *
 *  Created on: Oct 17, 2021
 *      Author: asten
 */

#include "LED.h"


void LED_Init(LED_t *led, uint32_t pattern){
  led->pattern = pattern;
  led->position = 0x80000000;
}

// shifts through LED pattern and returns whether GPIO should be set or reset
GPIO_PinState LED_NextState(LED_t *led){
  GPIO_PinState new_state = GPIO_PIN_RESET;

  led->position = led->position >> 1;
  if(led->position == 0) {
    led->position = 0x80000000;
  }

  if ((led->pattern & led->position) != 0) {
    new_state = GPIO_PIN_SET;
  }

  return new_state;
}




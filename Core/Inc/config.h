/*
 * config.h
 *
 *  Created on: Oct 17, 2021
 *      Author: asten
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdint.h>
#include "main.h"

#define ENTRY_COUNT   256 // how many steps are in full turn
#define ENTRY_LENGTH  5   // how many uint32's are needed to store all sensor data for that particular step

typedef struct {
  uint32_t CAN_address;
  uint32_t LUT[ENTRY_COUNT][ENTRY_LENGTH];
  uint32_t XOR_checksum;
} config_t;


void loadFromFlash(config_t *config);
void saveToFlash(config_t *config);

uint32_t isValid(config_t *config); // returns 1 if checksum is valid

static uint32_t checksum(config_t *config);



#endif /* INC_CONFIG_H_ */

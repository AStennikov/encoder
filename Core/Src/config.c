/*
 * config.c
 *
 *  Created on: Oct 17, 2021
 *      Author: asten
 */

// https://stackoverflow.com/questions/28503808/allocating-memory-in-flash-for-user-data-stm32f4-hal

// FLASH: category 2 device
// https://www.st.com/resource/en/reference_manual/dm00355726-stm32g4-series-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

#include "config.h"

uint32_t* config_block_in_flash = (uint32_t*) 0x0801f800;

void loadFromFlash(config_t *config){
    uint32_t *p = (uint32_t*) config;
    for (uint32_t i=0; i<sizeof(config_t)/4; ++i) {
        *p++ = config_block_in_flash[i];
    }
}

void saveToFlash(config_t *config) {

}

uint32_t isValid(config_t *config) {
  if (checksum(config) == config->XOR_checksum) {
    return 1;
  }
  return 0;
}


static uint32_t checksum(config_t *config){
    uint32_t result = 0;
    uint32_t *p = (uint32_t*) config;

    for (uint32_t i=0; i<sizeof(config_t)/4-1; ++i) {
        result ^= *p++;
    }
    return result;
}

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_util_i3c.h"

int32_t i3c_rstdaa(I3C_HandleTypeDef *handle);
int32_t i3c_set_bus_frequency(I3C_HandleTypeDef *handle, uint32_t i3c_freq);
int32_t i3c_setdasa(I3C_HandleTypeDef *handle, uint8_t addr, uint8_t *cccdata, uint16_t len);
int32_t i3c_write(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
int32_t i3c_read(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);

void i3c_set_addr(uint16_t addr);
int32_t write(uint16_t reg, uint8_t pdata);
int32_t read(uint16_t reg, void *pdata, uint16_t len);

#endif

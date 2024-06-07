/*
 ******************************************************************************
 * @file    read_data_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to extract data from the sensor in
 *          polling mode.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-
 * - NUCLEO_F401RE + STEVAL-
 * - DISCOVERY_SPC584B + STEVAL-
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */


#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "steng1ax_reg.h"

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

#define NUM_FIFO_ENTRY  33 /* 32 samples + timestamp */

static stmdev_ctx_t dev_ctx;
static steng1ax_md_t md;
static steng1ax_fifo_mode_t fifo_mode;
static uint8_t fifo_wtm_event = 0;
static steng1ax_ah_eng_mode_t mode;

void steng1ax_read_fifo_handler(void)
{
  uint8_t wmflag = 0;

  steng1ax_fifo_wtm_flag_get(&dev_ctx, &wmflag);

  if (wmflag > 0)
    fifo_wtm_event = 1;
}

/* Main Example --------------------------------------------------------------*/
void steng1ax_read_fifo(void)
{
  steng1ax_pin_int_route_t int_route;
  steng1ax_status_t status;
  uint8_t id;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  steng1ax_power_up(&dev_ctx);

  /* Check device ID */
  steng1ax_device_id_get(&dev_ctx, &id);
  if (id != STENG1AX_ID)
    while(1);

  /* Restore default configuration */
  steng1ax_init_set(&dev_ctx, STENG1AX_RESET);
  do {
    steng1ax_all_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set FIFO watermark to 32 sample(s) */
  fifo_mode.watermark = NUM_FIFO_ENTRY;
  fifo_mode.operation = STENG1AX_STREAM_MODE;
  fifo_mode.batch.dec_ts = STENG1AX_DEC_TS_32;
  fifo_mode.batch.bdr = STENG1AX_BDR_ODR_DIV_4;
  steng1ax_fifo_mode_set(&dev_ctx, fifo_mode);

  /* Enable ENG (fully differential, gain 2, Zin 100Mohm) */
  mode.ah_eng_en = PROPERTY_ENABLE;
  steng1ax_ah_eng_mode_set(&dev_ctx, mode);

  /* Set bdu and if_inc recommended for driver usage */
  steng1ax_init_set(&dev_ctx, STENG1AX_SENSOR_ONLY_ON);

  steng1ax_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Configure interrupt pins */
  int_route.fifo_th   = PROPERTY_ENABLE;
  steng1ax_pin_int_route_set(&dev_ctx, &int_route);

  /* Set Output Data Rate */
  md.odr = STENG1AX_800Hz;
  md.bw = STENG1AX_ODR_div_2;
  steng1ax_mode_set(&dev_ctx, &md);

  /* wait forever (ENG samples read with drdy irq) */
  while (1) {
    if (fifo_wtm_event) {
      uint16_t num = 0;
      uint32_t ts;
      steng1ax_fifo_data_t fdata;

      fifo_wtm_event = 0;

      /* Read number of samples in FIFO */
      steng1ax_fifo_data_level_get(&dev_ctx, &num);

      sprintf((char *)tx_buffer,"-- %d in FIFO\r\n", num);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      while (num--) {
        steng1ax_fifo_data_get(&dev_ctx, &md, &fdata);

        switch (fdata.tag) {
        case STENG1AX_AH_ENG_DATA_TAG:
          sprintf((char *)tx_buffer, "%02d: QVAR [LSB]: %d\r\n", NUM_FIFO_ENTRY - num, fdata.ah_eng.raw);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case STENG1AX_TIMESTAMP_CFG_CHG_TAG:
          ts = fdata.cfg_chg.timestamp / 100;
          sprintf((char*)tx_buffer, "Timestamp:\t%lu ms\t\r\n", ts);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          break;
       default:
          sprintf((char*)tx_buffer, "unknown TAG (%02x)\t\r\n", fdata.tag);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          break;
        }
      }
      sprintf((char *)tx_buffer,"\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, STENG1AX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  STENG1AX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Read(handle, STENG1AX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, STENG1AX_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}

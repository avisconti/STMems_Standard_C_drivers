/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI208V1K
 * - DISCOVERY_SPC584B + STEVAL-MKI208V1K
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
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
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define ASM330AB1_VDD 1.8f
#define ASM330AB1_VDDIO 1.8f

#elif defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS NULL
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "asm330ab1_reg.h"

#if defined(NUCLEO_F411RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        25 //ms

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

/* Main Example --------------------------------------------------------------*/
static asm330ab1_priv_t asm330ab1_config =
  {
    .use_safespi_bus = 0,
  };

void asm330ab1_self_test(void)
{
  stmdev_ctx_t dev_ctx;
  uint8_t whoamI;
  asm330ab1_device_status_t dev_status;
  asm330ab1_st_auto_sum_status_t sum_status;
  asm330ab1_status_t status;
  asm330ab1_data_n_dump_t ndump = { 0 };
  asm330ab1_pin_int_route_t int_route = {0};
  uint8_t val;
  int ret;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  dev_ctx.priv_data = &asm330ab1_config;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  asm330ab1_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ASM330AB1_ID)
    while (1);

  /* Power On Reset */
  ret = asm330ab1_sw_por(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

#if 1
  /* step 1: turn-off sensors */
  ret = asm330ab1_sensor_power_down(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* temporary setting */
  val = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  val = 0x34;
  ret += asm330ab1_write_reg(&dev_ctx, 0x4a, &val, 1);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  /* sensor configuration */
  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x62, &val, 1);

  /* Set GY interrupt */
  int_route.drdy_g = 1;
  asm330ab1_pin_int1_route_set(&dev_ctx, &int_route);

  //val = 0x84;
  //ret += asm330ab1_write_reg(&dev_ctx, 0x15, &val, 1);
  /* Set GY full scale */
  asm330ab1_gy_full_scale_set(&dev_ctx, ASM330AB1_2000dps);

  val = 0x05;
  ret += asm330ab1_write_reg(&dev_ctx, 0x16, &val, 1);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x19, &val, 1);

  /* set GY ndump */
  ndump.gy_data_n_dump = 4;
  asm330ab1_data_n_dump_set(&dev_ctx, ndump);

  /* step 3: turn-on gyroscope */
  ret = asm330ab1_sensor_start_up(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* GY on with ODR=480Hz */
  val = 0x18;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL2, &val, 1);

  /* wait for GY start up */
  do {
    asm330ab1_device_status_get(&dev_ctx, &dev_status);
  } while (dev_status.gy_startup != 0);

  /* wait for GY valid data */
  do {
    asm330ab1_status_get(&dev_ctx, &status);
  } while (status.gda != 1);

  /* GY Turn on automatic self-test */
  val = 1;
  asm330ab1_st_auto_gy_start(&dev_ctx, val);

  platform_delay(100);

  /* wait for GY auto self-test completion */
  do {
    asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);
  } while (sum_status.st_auto_gy_done != 1);

  asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);

  if (sum_status.st_auto_ok_neg_gy == 1 && sum_status.st_auto_ok_pos_gy == 1)
  {
    sprintf((char *)tx_buffer, "GY self-test PASS\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    sprintf((char *)tx_buffer, "GY self-test FAIL\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
#endif

#if 1
  /* step 1: turn-off sensors */
  ret = asm330ab1_sensor_power_down(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* temporary setting */
  val = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  val = 0x34;
  ret += asm330ab1_write_reg(&dev_ctx, 0x4a, &val, 1);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  val = 0x05;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  val = 0x08;
  ret += asm330ab1_write_reg(&dev_ctx, 0x34, &val, 1);

  val = 0x55;
  ret += asm330ab1_write_reg(&dev_ctx, 0x35, &val, 1);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  /* sensor configuration */
  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x62, &val, 1);

  /* Set XL interrupt */
  int_route.drdy_xl = 1;
  asm330ab1_pin_int1_route_set(&dev_ctx, &int_route);

  /* Set XL full scale */
  asm330ab1_xl_full_scale_set(&dev_ctx, ASM330AB1_16g);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x19, &val, 1);

  /* step 3: turn-on accelerometer */
  ret = asm330ab1_sensor_start_up(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* XL on with ODR=480Hz */
  val = 0x18;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL1, &val, 1);

  /* wait for XL start up */
  do {
    asm330ab1_device_status_get(&dev_ctx, &dev_status);
  } while (dev_status.xl_startup != 0);

  /* wait for XL valid data */
  do {
    asm330ab1_status_get(&dev_ctx, &status);
  } while (status.xlda != 1);

  /* XL Turn on automatic self-test */
  val = 1;
  asm330ab1_st_auto_xl_start(&dev_ctx, val);

  platform_delay(100);

  /* wait for XL auto self-test completion */
  do {
    asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);
  } while (sum_status.st_auto_xl_done != 1);

  asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);

  if (sum_status.st_auto_ok_neg_xl == 1 && sum_status.st_auto_ok_pos_xl == 1)
  {
    sprintf((char *)tx_buffer, "XL self-test PASS\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    sprintf((char *)tx_buffer, "XL self-test FAIL\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
#endif
  /* step 1: turn-off sensors */
  ret = asm330ab1_sensor_power_down(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* temporary setting */
  val = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);
  val = 0x34;
  ret += asm330ab1_write_reg(&dev_ctx, 0x4a, &val, 1);
  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &val, 1);

  /* step 2: initialize the sensor */
  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_HAODR_CFG, &val, 1);

  val = 0x02;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_INT1_CTRL, &val, 1);

  val = 0x84;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL6, &val, 1);

  val = 0x05;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL7, &val, 1);

  val = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL10, &val, 1);

  /* set GY ndump */
  ndump.gy_data_n_dump = 4;
  asm330ab1_data_n_dump_set(&dev_ctx, ndump);

  /* step 3: turn-on gyroscope */
  ret = asm330ab1_sensor_start_up(&dev_ctx);
  if (ret != 0) {
    goto error;
  }

  /* GY on with ODR=480Hz */
  val = 0x18;
  ret += asm330ab1_write_reg(&dev_ctx, ASM330AB1_CTRL2, &val, 1);

  /* wait for GY start up */
  do {
    asm330ab1_device_status_get(&dev_ctx, &dev_status);
  } while (dev_status.gy_startup != 0);

  /* wait for GY valid data */
  do {
    asm330ab1_status_get(&dev_ctx, &status);
  } while (status.gda != 1);

  /* GY Turn on automatic self-test */
  val = 1;
  asm330ab1_st_auto_gy_start(&dev_ctx, val);

  platform_delay(100); /* wait 70 ms */

  /* wait for GY auto self-test completion */
  do {
    asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);
  } while (sum_status.st_auto_gy_done != 1);

  asm330ab1_st_auto_sum_status_get(&dev_ctx, &sum_status);
  if (sum_status.st_auto_ok_neg_gy == 1 && sum_status.st_auto_ok_pos_gy == 1)
  {
    sprintf((char *)tx_buffer, "GY self-test PASS\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    sprintf((char *)tx_buffer, "GY self-test FAIL\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
#endif

#if 0
  static uint8_t peppe;

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x10, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x11, &peppe, 1);

  peppe = 0xC3;
  ret += asm330ab1_write_reg(&dev_ctx, 0x06, &peppe, 1);

  peppe = 0x75;
  ret += asm330ab1_write_reg(&dev_ctx, 0x36, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x34;
  ret += asm330ab1_write_reg(&dev_ctx, 0x4a, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0xC8;
  ret += asm330ab1_write_reg(&dev_ctx, 0x76, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x60, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x62, &peppe, 1);

  peppe = 0x02;
  ret += asm330ab1_write_reg(&dev_ctx, 0x0D, &peppe, 1);

  peppe = 0x84;
  ret += asm330ab1_write_reg(&dev_ctx, 0x15, &peppe, 1);

  peppe = 0x05;
  ret += asm330ab1_write_reg(&dev_ctx, 0x16, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x19, &peppe, 1);

  peppe = 0x07;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x40;
  ret += asm330ab1_write_reg(&dev_ctx, 0x5D, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x80;
  ret += asm330ab1_write_reg(&dev_ctx, 0x60, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x76, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x06, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x36, &peppe, 1);

  peppe = 0x18;
  ret += asm330ab1_write_reg(&dev_ctx, 0x11, &peppe, 1);

  peppe = 0x07;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x1a, &peppe, 1);
  } while ((peppe & 0x2) != 0x00);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x1e, &peppe, 1);
  } while ((peppe & 0x2) != 0x2);

  peppe = 0x07;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0xC0;
  ret += asm330ab1_write_reg(&dev_ctx, 0x5d, &peppe, 1);

  platform_delay(100);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x5e, &peppe, 1);
  } while ((peppe &= 0x80) != 0x80);
#endif

#if 0
  static uint8_t peppe;

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x10, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x11, &peppe, 1);

  peppe = 0xC3;
  ret += asm330ab1_write_reg(&dev_ctx, 0x06, &peppe, 1);

  peppe = 0x75;
  ret += asm330ab1_write_reg(&dev_ctx, 0x36, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x34;
  ret += asm330ab1_write_reg(&dev_ctx, 0x4a, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0xC8;
  ret += asm330ab1_write_reg(&dev_ctx, 0x76, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x60, &peppe, 1);



  peppe = 0x05;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x08;
  ret += asm330ab1_write_reg(&dev_ctx, 0x34, &peppe, 1);

  peppe = 0x55;
  ret += asm330ab1_write_reg(&dev_ctx, 0x35, &peppe, 1);



  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x62, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x0D, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x11, &peppe, 1);

  peppe = 0x03;
  ret += asm330ab1_write_reg(&dev_ctx, 0x17, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x19, &peppe, 1);

  peppe = 0x01;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x80;
  ret += asm330ab1_write_reg(&dev_ctx, 0x60, &peppe, 1);

  peppe = 0x10;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x76, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x06, &peppe, 1);

  peppe = 0x00;
  ret += asm330ab1_write_reg(&dev_ctx, 0x36, &peppe, 1);

  peppe = 0x18;
  ret += asm330ab1_write_reg(&dev_ctx, 0x10, &peppe, 1);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x1a, &peppe, 1);
  } while ((peppe & 0x20) != 0x00);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x1e, &peppe, 1);
  } while ((peppe & 0x1) != 0x1);

  peppe = 0x07;
  ret += asm330ab1_write_reg(&dev_ctx, 0x00, &peppe, 1);

  peppe = 0x08;
  ret += asm330ab1_write_reg(&dev_ctx, 0x5d, &peppe, 1);

  platform_delay(100);

  peppe = 0;
  do {
    ret += asm330ab1_read_reg(&dev_ctx, 0x5e, &peppe, 1);
  } while ((peppe &= 0x08) != 0x08);
#endif

error:
  return;
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  IIS3DWB_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
#if defined(STEVAL_MKI109D)
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, IIS3DWB_I2C_ADD_H & 0xFE, reg, bufp, len);
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
#if defined(STEVAL_MKI109D)
  CDC_Transmit_FS(tx_buffer, len);
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
#if defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(STEVAL_MKI109D)
  delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4;
  spi_init(&spi_conf);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(ASM330AB1_VDD);
  set_vddio(ASM330AB1_VDDIO);
  delay(100);

#elif defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
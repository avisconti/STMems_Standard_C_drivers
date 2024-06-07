/**
  ******************************************************************************
  * @file    lis2dlc12_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2dlc12_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS2DLC12_REGS_H
#define LIS2DLC12_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LIS2DLC12
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LIS2DLC12_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 0x3D if SA0=1 -> 0x3B **/
#define LIS2DLC12_I2C_ADD_L     0x3DU
#define LIS2DLC12_I2C_ADD_H     0x3BU

/** Device Identification (Who am I) **/
#define LIS2DLC12_ID            0x43U

/**
  * @}
  *
  */

#define LIS2DLC12_MODULE_8BIT           0x0CU
#define LIS2DLC12_WHO_AM_I              0x0FU
#define LIS2DLC12_CTRL1                 0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdu                 : 1;
  uint8_t hf_odr              : 1;
  uint8_t fs                  : 2;
  uint8_t odr                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr                 : 4;
  uint8_t fs                  : 2;
  uint8_t hf_odr              : 1;
  uint8_t bdu                 : 1;
#endif /* DRV_BYTE_ORDER */

} lis2dlc12_ctrl1_t;

#define LIS2DLC12_CTRL2                 0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t not_used_01         : 2;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                : 1;
  uint8_t soft_reset          : 1;
  uint8_t not_used_01         : 2;
  uint8_t fds_slope           : 1;
  uint8_t if_add_inc          : 1;
  uint8_t i2c_disable         : 1;
  uint8_t sim                 : 1;
#endif /* DRV_BYTE_ORDER */

} lis2dlc12_ctrl2_t;

#define LIS2DLC12_CTRL3                 0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pp_od               : 1;
  uint8_t h_lactive           : 1;
  uint8_t lir                 : 1;
  uint8_t tap_z_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_x_en            : 1;
  uint8_t st                  : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t st                  : 2;
  uint8_t tap_x_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_z_en            : 1;
  uint8_t lir                 : 1;
  uint8_t h_lactive           : 1;
  uint8_t pp_od               : 1;
#endif /* DRV_BYTE_ORDER */

} lis2dlc12_ctrl3_t;

#define LIS2DLC12_CTRL4                 0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy           : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t int1_master_drdy    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_master_drdy    : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_drdy           : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_ctrl4_t;

#define LIS2DLC12_CTRL5                 0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy           : 1;
  uint8_t int2_fth            : 1;
  uint8_t not_used_01         : 2;
  uint8_t int2_tilt           : 1;
  uint8_t int2_on_int1        : 1;
  uint8_t int2_boot           : 1;
  uint8_t drdy_pulsed         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_pulsed         : 1;
  uint8_t int2_boot           : 1;
  uint8_t int2_on_int1        : 1;
  uint8_t int2_tilt           : 1;
  uint8_t not_used_01         : 2;
  uint8_t int2_fth            : 1;
  uint8_t int2_drdy           : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_ctrl5_t;

#define LIS2DLC12_FIFO_CTRL             0x25U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t if_cs_pu_dis        : 1;
  uint8_t not_used_01         : 2;
  uint8_t module_to_fifo      : 1;
  uint8_t not_used_02         : 1;
  uint8_t fmode               : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode               : 3;
  uint8_t not_used_02         : 1;
  uint8_t module_to_fifo      : 1;
  uint8_t not_used_01         : 2;
  uint8_t if_cs_pu_dis        : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_fifo_ctrl_t;

#define LIS2DLC12_OUT_T                 0x26U
#define LIS2DLC12_STATUS                0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t fifo_ths            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_ths            : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state         : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t _6d_ia              : 1;
  uint8_t ff_ia               : 1;
  uint8_t drdy                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_status_t;

#define LIS2DLC12_OUT_X_L               0x28U
#define LIS2DLC12_OUT_X_H               0x29U
#define LIS2DLC12_OUT_Y_L               0x2AU
#define LIS2DLC12_OUT_Y_H               0x2BU
#define LIS2DLC12_OUT_Z_L               0x2CU
#define LIS2DLC12_OUT_Z_H               0x2DU
#define LIS2DLC12_FIFO_THS              0x2EU
typedef struct
{
  uint8_t fth                 : 8;
} lis2dlc12_fifo_ths_t;

#define LIS2DLC12_FIFO_SRC              0x2FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01         : 5;
  uint8_t diff                : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t fth                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fth                 : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t diff                : 1;
  uint8_t not_used_01         : 5;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_fifo_src_t;

#define LIS2DLC12_FIFO_SAMPLES          0x30U
#define LIS2DLC12_TAP_6D_THS            0x31U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths             : 5;
  uint8_t _6d_ths             : 2;
  uint8_t _4d_en              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t _4d_en              : 1;
  uint8_t _6d_ths             : 2;
  uint8_t tap_ths             : 5;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_tap_6d_ths_t;

#define LIS2DLC12_INT_DUR               0x32U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock               : 2;
  uint8_t quiet               : 2;
  uint8_t lat                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lat                 : 4;
  uint8_t quiet               : 2;
  uint8_t shock               : 2;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_int_dur_t;

#define LIS2DLC12_WAKE_UP_THS           0x33U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wu_ths              : 6;
  uint8_t sleep_on            : 1;
  uint8_t single_double_tap   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap   : 1;
  uint8_t sleep_on            : 1;
  uint8_t wu_ths              : 6;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_wake_up_ths_t;

#define LIS2DLC12_WAKE_UP_DUR           0x34U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur           : 4;
  uint8_t int1_fss7           : 1;
  uint8_t wu_dur              : 2;
  uint8_t ff_dur              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur              : 1;
  uint8_t wu_dur              : 2;
  uint8_t int1_fss7           : 1;
  uint8_t sleep_dur           : 4;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_wake_up_dur_t;

#define LIS2DLC12_FREE_FALL             0x35U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths              : 3;
  uint8_t ff_dur              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur              : 5;
  uint8_t ff_ths              : 3;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_free_fall_t;

#define LIS2DLC12_STATUS_DUP            0x36U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t ovr                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ovr                 : 1;
  uint8_t ovr                 : 1;
  uint8_t sleep_state         : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t _6d_ia              : 1;
  uint8_t ff_ia               : 1;
  uint8_t drdy                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_status_dup_t;

#define LIS2DLC12_WAKE_UP_SRC           0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t x_wu                : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t ff_ia               : 1;
  uint8_t not_used_01         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 2;
  uint8_t ff_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t wu_ia               : 1;
  uint8_t x_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t z_wu                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_wake_up_src_t;

#define LIS2DLC12_TAP_SRC               0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t x_tap               : 1;
  uint8_t tap_sign            : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t tap_ia              : 1;
  uint8_t not_used_01         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t tap_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t tap_sign            : 1;
  uint8_t x_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t z_tap               : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_tap_src_t;

#define LIS2DLC12_6D_SRC                0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                  : 1;
  uint8_t xh                  : 1;
  uint8_t yl                  : 1;
  uint8_t yh                  : 1;
  uint8_t zl                  : 1;
  uint8_t zh                  : 1;
  uint8_t _6d_ia              : 1;
  uint8_t not_used_01         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t _6d_ia              : 1;
  uint8_t zh                  : 1;
  uint8_t zl                  : 1;
  uint8_t yh                  : 1;
  uint8_t yl                  : 1;
  uint8_t xh                  : 1;
  uint8_t xl                  : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_6d_src_t;

#define LIS2DLC12_FUNC_CK_GATE          0x3DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ck_gate_func        : 1;
  uint8_t not_used_01         : 4;
  uint8_t fs_src              : 2;
  uint8_t tilt_int            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tilt_int            : 1;
  uint8_t fs_src              : 2;
  uint8_t not_used_01         : 4;
  uint8_t ck_gate_func        : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_func_ck_gate_t;

#define LIS2DLC12_FUNC_SRC              0x3EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_02         : 1;
  uint8_t module_ready        : 1;
  uint8_t rst_tilt            : 1;
  uint8_t not_used_01         : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 5;
  uint8_t rst_tilt            : 1;
  uint8_t module_ready        : 1;
  uint8_t not_used_02         : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_func_src_t;

#define LIS2DLC12_FUNC_CTRL             0x3FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_02         : 3;
  uint8_t tud_en              : 1;
  uint8_t tilt_on             : 1;
  uint8_t module_on           : 1;
  uint8_t not_used_01         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 2;
  uint8_t module_on           : 1;
  uint8_t tilt_on             : 1;
  uint8_t tud_en              : 1;
  uint8_t not_used_02         : 3;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_func_ctrl_t;

/* Advanced Configuration registers */

#define LIS2DLC12_CTRL2_ADV             0x3FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t not_used_01         : 2;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                : 1;
  uint8_t soft_reset          : 1;
  uint8_t not_used_01         : 2;
  uint8_t fds_slope           : 1;
  uint8_t if_add_inc          : 1;
  uint8_t i2c_disable         : 1;
  uint8_t sim                 : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dlc12_ctrl2_adv_t;

/**
  * @defgroup LIS2DLC12_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lis2dlc12_ctrl1_t                     ctrl1;
  lis2dlc12_ctrl2_t                     ctrl2;
  lis2dlc12_ctrl3_t                     ctrl3;
  lis2dlc12_ctrl4_t                     ctrl4;
  lis2dlc12_ctrl5_t                     ctrl5;
  lis2dlc12_fifo_ctrl_t                 fifo_ctrl;
  lis2dlc12_status_t                    status;
  lis2dlc12_fifo_src_t                  fifo_src;
  lis2dlc12_tap_6d_ths_t                tap_6d_ths;
  lis2dlc12_int_dur_t                   int_dur;
  lis2dlc12_wake_up_ths_t               wake_up_ths;
  lis2dlc12_wake_up_dur_t               wake_up_dur;
  lis2dlc12_free_fall_t                 free_fall;
  lis2dlc12_status_dup_t                status_dup;
  lis2dlc12_wake_up_src_t               wake_up_src;
  lis2dlc12_tap_src_t                   tap_src;
  lis2dlc12_6d_src_t                    _6d_src;
  lis2dlc12_func_ck_gate_t              func_ck_gate;
  lis2dlc12_func_src_t                  func_src;
  lis2dlc12_func_ctrl_t                 func_ctrl;
  lis2dlc12_ctrl2_adv_t                 ctrl2_adv;
  bitwise_t                            bitwise;
  uint8_t                              byte;
} lis2dlc12_reg_t;

/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */

int32_t lis2dlc12_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t lis2dlc12_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

float_t lis2dlc12_from_fs2g_to_mg(int16_t lsb);
float_t lis2dlc12_from_fs4g_to_mg(int16_t lsb);
float_t lis2dlc12_from_fs8g_to_mg(int16_t lsb);
float_t lis2dlc12_from_fs16g_to_mg(int16_t lsb);

float_t lis2dlc12_from_lsb_to_celsius(int16_t lsb);

typedef struct
{
  lis2dlc12_fifo_src_t       fifo_src;
  lis2dlc12_status_dup_t     status_dup;
  lis2dlc12_wake_up_src_t    wake_up_src;
  lis2dlc12_tap_src_t        tap_src;
  lis2dlc12_6d_src_t         _6d_src;
  lis2dlc12_func_ck_gate_t   func_ck_gate;
  lis2dlc12_func_src_t       func_src;
} lis2dlc12_all_sources_t;
int32_t lis2dlc12_all_sources_get(const stmdev_ctx_t *ctx,
                                 lis2dlc12_all_sources_t *val);

int32_t lis2dlc12_block_data_update_set(const stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lis2dlc12_block_data_update_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  LIS2DLC12_2g = 0,
  LIS2DLC12_16g = 1,
  LIS2DLC12_4g = 2,
  LIS2DLC12_8g = 3,
} lis2dlc12_fs_t;
int32_t lis2dlc12_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                   lis2dlc12_fs_t val);
int32_t lis2dlc12_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                   lis2dlc12_fs_t *val);

typedef enum
{
  LIS2DLC12_XL_ODR_OFF         = 0x00,
  LIS2DLC12_XL_ODR_1Hz_LP      = 0x08,
  LIS2DLC12_XL_ODR_12Hz5_LP    = 0x09,
  LIS2DLC12_XL_ODR_25Hz_LP     = 0x0A,
  LIS2DLC12_XL_ODR_50Hz_LP     = 0x0B,
  LIS2DLC12_XL_ODR_100Hz_LP    = 0x0C,
  LIS2DLC12_XL_ODR_200Hz_LP    = 0x0D,
  LIS2DLC12_XL_ODR_400Hz_LP    = 0x0E,
  LIS2DLC12_XL_ODR_800Hz_LP    = 0x0F,
  LIS2DLC12_XL_ODR_12Hz5_HR    = 0x01,
  LIS2DLC12_XL_ODR_25Hz_HR     = 0x02,
  LIS2DLC12_XL_ODR_50Hz_HR     = 0x03,
  LIS2DLC12_XL_ODR_100Hz_HR    = 0x04,
  LIS2DLC12_XL_ODR_200Hz_HR    = 0x05,
  LIS2DLC12_XL_ODR_400Hz_HR    = 0x06,
  LIS2DLC12_XL_ODR_800Hz_HR    = 0x07,
  LIS2DLC12_XL_ODR_1k6Hz_HF    = 0x15,
  LIS2DLC12_XL_ODR_3k2Hz_HF    = 0x16,
  LIS2DLC12_XL_ODR_6k4Hz_HF    = 0x17,
} lis2dlc12_odr_t;
int32_t lis2dlc12_xl_data_rate_set(const stmdev_ctx_t *ctx,
                                  lis2dlc12_odr_t val);
int32_t lis2dlc12_xl_data_rate_get(const stmdev_ctx_t *ctx,
                                  lis2dlc12_odr_t *val);

int32_t lis2dlc12_status_reg_get(const stmdev_ctx_t *ctx,
                                lis2dlc12_status_t *val);

int32_t lis2dlc12_xl_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dlc12_acceleration_module_raw_get(const stmdev_ctx_t *ctx,
                                             uint8_t *buff);

int32_t lis2dlc12_temperature_raw_get(const stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t lis2dlc12_acceleration_raw_get(const stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t lis2dlc12_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2dlc12_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_auto_increment_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_reset_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_reset_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_boot_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_boot_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS2DLC12_XL_ST_DISABLE     = 0,
  LIS2DLC12_XL_ST_POSITIVE    = 1,
  LIS2DLC12_XL_ST_NEGATIVE    = 2,
} lis2dlc12_st_t;
int32_t lis2dlc12_xl_self_test_set(const stmdev_ctx_t *ctx,
                                  lis2dlc12_st_t val);
int32_t lis2dlc12_xl_self_test_get(const stmdev_ctx_t *ctx,
                                  lis2dlc12_st_t *val);

typedef enum
{
  LIS2DLC12_DRDY_LATCHED   = 0,
  LIS2DLC12_DRDY_PULSED    = 1,
} lis2dlc12_drdy_pulsed_t;
int32_t lis2dlc12_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                     lis2dlc12_drdy_pulsed_t val);
int32_t lis2dlc12_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                     lis2dlc12_drdy_pulsed_t *val);

typedef enum
{
  LIS2DLC12_HP_INTERNAL_ONLY  = 0,
  LIS2DLC12_HP_ON_OUTPUTS     = 1,
} lis2dlc12_fds_slope_t;
int32_t lis2dlc12_xl_hp_path_set(const stmdev_ctx_t *ctx,
                                lis2dlc12_fds_slope_t val);
int32_t lis2dlc12_xl_hp_path_get(const stmdev_ctx_t *ctx,
                                lis2dlc12_fds_slope_t *val);

typedef enum
{
  LIS2DLC12_SPI_4_WIRE   = 0,
  LIS2DLC12_SPI_3_WIRE   = 1,
} lis2dlc12_sim_t;
int32_t lis2dlc12_spi_mode_set(const stmdev_ctx_t *ctx, lis2dlc12_sim_t val);
int32_t lis2dlc12_spi_mode_get(const stmdev_ctx_t *ctx, lis2dlc12_sim_t *val);

typedef enum
{
  LIS2DLC12_I2C_ENABLE   = 0,
  LIS2DLC12_I2C_DISABLE  = 1,
} lis2dlc12_i2c_disable_t;
int32_t lis2dlc12_i2c_interface_set(const stmdev_ctx_t *ctx,
                                   lis2dlc12_i2c_disable_t val);
int32_t lis2dlc12_i2c_interface_get(const stmdev_ctx_t *ctx,
                                   lis2dlc12_i2c_disable_t *val);

typedef enum
{
  LIS2DLC12_PULL_UP_CONNECTED     = 0,
  LIS2DLC12_PULL_UP_DISCONNECTED  = 1,
} lis2dlc12_if_cs_pu_dis_t;
int32_t lis2dlc12_cs_mode_set(const stmdev_ctx_t *ctx,
                             lis2dlc12_if_cs_pu_dis_t val);
int32_t lis2dlc12_cs_mode_get(const stmdev_ctx_t *ctx,
                             lis2dlc12_if_cs_pu_dis_t *val);

typedef enum
{
  LIS2DLC12_PUSH_PULL   = 0,
  LIS2DLC12_OPEN_DRAIN  = 1,
} lis2dlc12_pp_od_t;
int32_t lis2dlc12_pin_mode_set(const stmdev_ctx_t *ctx,
                              lis2dlc12_pp_od_t val);
int32_t lis2dlc12_pin_mode_get(const stmdev_ctx_t *ctx,
                              lis2dlc12_pp_od_t *val);

typedef enum
{
  LIS2DLC12_ACTIVE_HIGH  = 0,
  LIS2DLC12_ACTIVE_LOW   = 1,
} lis2dlc12_h_lactive_t;
int32_t lis2dlc12_pin_polarity_set(const stmdev_ctx_t *ctx,
                                  lis2dlc12_h_lactive_t val);
int32_t lis2dlc12_pin_polarity_get(const stmdev_ctx_t *ctx,
                                  lis2dlc12_h_lactive_t *val);

typedef enum
{
  LIS2DLC12_INT_PULSED   = 0,
  LIS2DLC12_INT_LATCHED  = 1,
} lis2dlc12_lir_t;
int32_t lis2dlc12_int_notification_set(const stmdev_ctx_t *ctx,
                                      lis2dlc12_lir_t val);
int32_t lis2dlc12_int_notification_get(const stmdev_ctx_t *ctx,
                                      lis2dlc12_lir_t *val);

typedef struct
{
  uint8_t int1_drdy               : 1;
  uint8_t int1_fth                : 1;
  uint8_t int1_6d                 : 1;
  uint8_t int1_tap                : 1;
  uint8_t int1_ff                 : 1;
  uint8_t int1_wu                 : 1;
  uint8_t int1_s_tap              : 1;
  uint8_t int1_master_drdy        : 1;
  uint8_t int1_fss7               : 1;
} lis2dlc12_pin_int1_route_t;
int32_t lis2dlc12_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                    lis2dlc12_pin_int1_route_t val);
int32_t lis2dlc12_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                    lis2dlc12_pin_int1_route_t *val);

typedef struct
{
  uint8_t int2_boot               : 1;
  uint8_t int2_tilt               : 1;
  uint8_t int2_fth                : 1;
  uint8_t int2_drdy               : 1;
} lis2dlc12_pin_int2_route_t;
int32_t lis2dlc12_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                    lis2dlc12_pin_int2_route_t val);
int32_t lis2dlc12_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                    lis2dlc12_pin_int2_route_t *val);

int32_t lis2dlc12_all_on_int1_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_all_on_int1_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_wkup_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_wkup_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_wkup_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_wkup_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_sleep_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_sleep_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_act_sleep_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_act_sleep_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_tap_detection_on_z_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dlc12_tap_detection_on_z_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dlc12_tap_detection_on_y_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dlc12_tap_detection_on_y_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dlc12_tap_detection_on_x_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dlc12_tap_detection_on_x_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dlc12_tap_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_tap_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_tap_shock_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_tap_shock_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_tap_quiet_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_tap_quiet_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_tap_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_tap_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS2DLC12_ONLY_SINGLE  = 0,
  LIS2DLC12_ONLY_DOUBLE  = 1,
} lis2dlc12_single_double_tap_t;
int32_t lis2dlc12_tap_mode_set(const stmdev_ctx_t *ctx,
                              lis2dlc12_single_double_tap_t val);
int32_t lis2dlc12_tap_mode_get(const stmdev_ctx_t *ctx,
                              lis2dlc12_single_double_tap_t *val);

int32_t lis2dlc12_tap_src_get(const stmdev_ctx_t *ctx,
                             lis2dlc12_tap_src_t *val);

typedef enum
{
  LIS2DLC12_DEG_80   = 0,
  LIS2DLC12_DEG_70   = 1,
  LIS2DLC12_DEG_60   = 2,
  LIS2DLC12_DEG_50   = 3,
} lis2dlc12_6d_ths_t;
int32_t lis2dlc12_6d_threshold_set(const stmdev_ctx_t *ctx,
                                  lis2dlc12_6d_ths_t val);
int32_t lis2dlc12_6d_threshold_get(const stmdev_ctx_t *ctx,
                                  lis2dlc12_6d_ths_t *val);

int32_t lis2dlc12_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_6d_src_get(const stmdev_ctx_t *ctx,
                            lis2dlc12_6d_src_t *val);

int32_t lis2dlc12_ff_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_ff_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_ff_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_ff_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_fifo_xl_module_batch_set(const stmdev_ctx_t *ctx,
                                          uint8_t val);
int32_t lis2dlc12_fifo_xl_module_batch_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val);

typedef enum
{
  LIS2DLC12_BYPASS_MODE            = 0,
  LIS2DLC12_FIFO_MODE              = 1,
  LIS2DLC12_STREAM_TO_FIFO_MODE    = 3,
  LIS2DLC12_BYPASS_TO_STREAM_MODE  = 4,
  LIS2DLC12_STREAM_MODE            = 6,
} lis2dlc12_fmode_t;
int32_t lis2dlc12_fifo_mode_set(const stmdev_ctx_t *ctx,
                               lis2dlc12_fmode_t val);
int32_t lis2dlc12_fifo_mode_get(const stmdev_ctx_t *ctx,
                               lis2dlc12_fmode_t *val);

int32_t lis2dlc12_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_fifo_full_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_fifo_data_level_get(const stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t lis2dlc12_fifo_src_get(const stmdev_ctx_t *ctx,
                              lis2dlc12_fifo_src_t *val);

int32_t lis2dlc12_tilt_data_ready_flag_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val);

int32_t lis2dlc12_tilt_sens_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_tilt_sens_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dlc12_module_sens_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dlc12_module_sens_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*__LIS2DLC12_DRIVER__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

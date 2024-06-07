/**
  ******************************************************************************
  * @file    asm330lhm_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          asm330lhm_reg.c driver.
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
#ifndef ASM330LHM_REGS_H
#define ASM330LHM_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup ASM330LHM
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

/** @defgroup ASM330LHM Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define ASM330LHM_I2C_ADD_L                    0xD5U
#define ASM330LHM_I2C_ADD_H                    0xD7U

/** Device Identification (Who am I) **/
#define ASM330LHM_ID                           0x6BU

/**
  * @}
  *
  */

#define ASM330LHM_PIN_CTRL                     0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_pin_ctrl_t;

#define ASM330LHM_FIFO_CTRL1                   0x07U
typedef struct
{
  uint8_t wtm                      : 8;
} asm330lhm_fifo_ctrl1_t;

#define ASM330LHM_FIFO_CTRL2                   0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm                      : 1;
  uint8_t not_used_01              : 3;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_02              : 2;
  uint8_t stop_on_wtm              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_wtm              : 1;
  uint8_t not_used_02              : 2;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_01              : 3;
  uint8_t wtm                      : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_fifo_ctrl2_t;

#define ASM330LHM_FIFO_CTRL3                   0x09U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdr_xl                   : 4;
  uint8_t bdr_gy                   : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bdr_gy                   : 4;
  uint8_t bdr_xl                   : 4;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_fifo_ctrl3_t;

#define ASM330LHM_FIFO_CTRL4                   0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode                : 3;
  uint8_t not_used_01              : 1;
  uint8_t odr_t_batch              : 2;
  uint8_t dec_ts_batch             : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dec_ts_batch             : 2;
  uint8_t odr_t_batch              : 2;
  uint8_t not_used_01              : 1;
  uint8_t fifo_mode                : 3;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_fifo_ctrl4_t;

#define ASM330LHM_COUNTER_BDR_REG1             0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t cnt_bdr_th               : 2;
  uint8_t not_used_01              : 3;
  uint8_t trig_counter_bdr         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t dataready_pulsed         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dataready_pulsed         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t trig_counter_bdr         : 1;
  uint8_t not_used_01              : 3;
  uint8_t cnt_bdr_th               : 2;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_counter_bdr_reg1_t;

#define ASM330LHM_COUNTER_BDR_REG2             0x0CU
typedef struct
{
  uint8_t cnt_bdr_th               : 8;
} asm330lhm_counter_bdr_reg2_t;

#define ASM330LHM_INT1_CTRL                    0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t den_drdy_flag            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy_flag            : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_int1_ctrl_t;

#define ASM330LHM_INT2_CTRL                    0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_int2_ctrl_t;

#define ASM330LHM_WHO_AM_I                     0x0FU
#define ASM330LHM_CTRL1_XL                     0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_xl                   : 4;
  uint8_t fs_xl                    : 2;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl1_xl_t;

#define ASM330LHM_CTRL2_G                      0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t odr_g                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g                    : 4;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl2_g_t;

#define ASM330LHM_CTRL3_C                      0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t h_lactive                : 1;
  uint8_t pp_od                    : 1;
  uint8_t sim                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t not_used_01              : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl3_c_t;

#define ASM330LHM_CTRL4_C                      0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_03              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t not_used_02              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t i2c_disable              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl4_c_t;

#define ASM330LHM_CTRL5_C                      0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t rounding                 : 2;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t rounding                 : 2;
  uint8_t not_used_01              : 1;
  uint8_t st_g                     : 2;
  uint8_t st_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl5_c_t;

#define ASM330LHM_CTRL6_C                      0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ftype                    : 3;
  uint8_t usr_off_w                : 1;
  uint8_t not_used_01              : 1;
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */
  uint8_t not_used_01              : 1;
  uint8_t usr_off_w                : 1;
  uint8_t ftype                    : 3;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl6_c_t;

#define ASM330LHM_CTRL7_G                      0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t not_used_02              : 2;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t not_used_03              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 1;
  uint8_t hp_en_g                  : 1;
  uint8_t hpm_g                    : 2;
  uint8_t not_used_02              : 2;
  uint8_t usr_off_on_out           : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl7_g_t;

#define ASM330LHM_CTRL8_XL                     0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t low_pass_on_6d           : 1;
  uint8_t not_used_01              : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t hpcf_xl                  : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpcf_xl                  : 3;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t not_used_01              : 1;
  uint8_t low_pass_on_6d           : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl8_xl_t;

#define ASM330LHM_CTRL9_XL                     0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t device_conf              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_x                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_z                    : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_lh                   : 1;
  uint8_t device_conf              : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl9_xl_t;

#define ASM330LHM_CTRL10_C                     0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ctrl10_c_t;

#define ASM330LHM_ALL_INT_SRC                  0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                    : 1;
  uint8_t wu_ia                    : 1;
  uint8_t not_used_01              : 2;
  uint8_t d6d_ia                   : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_02              : 1;
  uint8_t timestamp_endcount       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount       : 1;
  uint8_t not_used_02              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t not_used_01              : 2;
  uint8_t wu_ia                    : 1;
  uint8_t ff_ia                    : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_all_int_src_t;

#define ASM330LHM_WAKE_UP_SRC                  0x1BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t wu_ia                    : 1;
  uint8_t x_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t z_wu                     : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_wake_up_src_t;

#define ASM330LHM_D6D_SRC                      0x1DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy                 : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t zh                       : 1;
  uint8_t zl                       : 1;
  uint8_t yh                       : 1;
  uint8_t yl                       : 1;
  uint8_t xh                       : 1;
  uint8_t xl                       : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_d6d_src_t;

#define ASM330LHM_STATUS_REG                   0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t tda                      : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_status_reg_t;

#define ASM330LHM_OUT_TEMP_L                   0x20U
#define ASM330LHM_OUT_TEMP_H                   0x21U
#define ASM330LHM_OUTX_L_G                     0x22U
#define ASM330LHM_OUTX_H_G                     0x23U
#define ASM330LHM_OUTY_L_G                     0x24U
#define ASM330LHM_OUTY_H_G                     0x25U
#define ASM330LHM_OUTZ_L_G                     0x26U
#define ASM330LHM_OUTZ_H_G                     0x27U
#define ASM330LHM_OUTX_L_A                     0x28U
#define ASM330LHM_OUTX_H_A                     0x29U
#define ASM330LHM_OUTY_L_A                     0x2AU
#define ASM330LHM_OUTY_H_A                     0x2BU
#define ASM330LHM_OUTZ_L_A                     0x2CU
#define ASM330LHM_OUTZ_H_A                     0x2DU
#define ASM330LHM_FIFO_STATUS1                 0x3AU
typedef struct
{
  uint8_t diff_fifo                : 8;
} asm330lhm_fifo_status1_t;

#define ASM330LHM_FIFO_STATUS2                 0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t fifo_ovr_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia              : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_ovr_latched         : 1;
  uint8_t not_used_01              : 1;
  uint8_t diff_fifo                : 2;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_fifo_status2_t;

#define ASM330LHM_TIMESTAMP0                   0x40U
#define ASM330LHM_TIMESTAMP1                   0x41U
#define ASM330LHM_TIMESTAMP2                   0x42U
#define ASM330LHM_TIMESTAMP3                   0x43U
#define ASM330LHM_INT_CFG0                     0x56U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                      : 1;
  uint8_t not_used_01              : 3;
  uint8_t slope_fds                : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t slope_fds                : 1;
  uint8_t not_used_01              : 3;
  uint8_t lir                      : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_int_cfg0_t;

#define ASM330LHM_INT_CFG1                     0x58U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable        : 1;
  uint8_t inact_en                 : 2;
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_int_cfg1_t;

#define ASM330LHM_THS_6D                       0x59U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t d4d_en                   : 1;
  uint8_t sixd_ths                 : 2;
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_ths_6d_t;

#define ASM330LHM_WAKE_UP_THS                  0x5BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                   : 6;
  uint8_t usr_off_on_wu            : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t usr_off_on_wu            : 1;
  uint8_t wk_ths                   : 6;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_wake_up_ths_t;

#define ASM330LHM_WAKE_UP_DUR                  0x5CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                : 4;
  uint8_t wake_ths_w               : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 1;
  uint8_t wake_dur                 : 2;
  uint8_t wake_ths_w               : 1;
  uint8_t sleep_dur                : 4;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_wake_up_dur_t;

#define ASM330LHM_FREE_FALL                    0x5DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 5;
  uint8_t ff_ths                   : 3;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_free_fall_t;

#define ASM330LHM_MD1_CFG                      0x5EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t int1_6d                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t not_used_03              : 1;
  uint8_t int1_sleep_change        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_sleep_change        : 1;
  uint8_t not_used_03              : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_ff                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_6d                  : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_md1_cfg_t;

#define ASM330LHM_MD2_CFG                      0x5FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_timestamp           : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_6d                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t not_used_03              : 1;
  uint8_t int2_sleep_change        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_change        : 1;
  uint8_t not_used_03              : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_ff                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_6d                  : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_timestamp           : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_md2_cfg_t;

#define ASM330LHM_INTERNAL_FREQ_FINE           0x63U
typedef struct
{
  uint8_t freq_fine                : 8;
} asm330lhm_internal_freq_fine_t;

#define ASM330LHM_X_OFS_USR                    0x73U
#define ASM330LHM_Y_OFS_USR                    0x74U
#define ASM330LHM_Z_OFS_USR                    0x75U
#define ASM330LHM_FIFO_DATA_OUT_TAG            0x78U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tag_parity               : 1;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_sensor               : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tag_sensor               : 5;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_parity               : 1;
#endif /* DRV_BYTE_ORDER */
} asm330lhm_fifo_data_out_tag_t;

#define ASM330LHM_FIFO_DATA_OUT_X_L            0x79U
#define ASM330LHM_FIFO_DATA_OUT_X_H            0x7AU
#define ASM330LHM_FIFO_DATA_OUT_Y_L            0x7BU
#define ASM330LHM_FIFO_DATA_OUT_Y_H            0x7CU
#define ASM330LHM_FIFO_DATA_OUT_Z_L            0x7DU
#define ASM330LHM_FIFO_DATA_OUT_Z_H            0x7EU

/**
  * @defgroup ASM330LHM_Register_Union
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
  asm330lhm_pin_ctrl_t                      pin_ctrl;
  asm330lhm_fifo_ctrl1_t                    fifo_ctrl1;
  asm330lhm_fifo_ctrl2_t                    fifo_ctrl2;
  asm330lhm_fifo_ctrl3_t                    fifo_ctrl3;
  asm330lhm_fifo_ctrl4_t                    fifo_ctrl4;
  asm330lhm_counter_bdr_reg1_t              counter_bdr_reg1;
  asm330lhm_counter_bdr_reg2_t              counter_bdr_reg2;
  asm330lhm_int1_ctrl_t                     int1_ctrl;
  asm330lhm_int2_ctrl_t                     int2_ctrl;
  asm330lhm_ctrl1_xl_t                      ctrl1_xl;
  asm330lhm_ctrl2_g_t                       ctrl2_g;
  asm330lhm_ctrl3_c_t                       ctrl3_c;
  asm330lhm_ctrl4_c_t                       ctrl4_c;
  asm330lhm_ctrl5_c_t                       ctrl5_c;
  asm330lhm_ctrl6_c_t                       ctrl6_c;
  asm330lhm_ctrl7_g_t                       ctrl7_g;
  asm330lhm_ctrl8_xl_t                      ctrl8_xl;
  asm330lhm_ctrl9_xl_t                      ctrl9_xl;
  asm330lhm_ctrl10_c_t                      ctrl10_c;
  asm330lhm_all_int_src_t                   all_int_src;
  asm330lhm_wake_up_src_t                   wake_up_src;
  asm330lhm_d6d_src_t                       d6d_src;
  asm330lhm_status_reg_t                    status_reg;
  asm330lhm_fifo_status1_t                  fifo_status1;
  asm330lhm_fifo_status2_t                  fifo_status2;
  asm330lhm_int_cfg0_t                      int_cfg0;
  asm330lhm_int_cfg1_t                      int_cfg1;
  asm330lhm_ths_6d_t                        ths_6d;
  asm330lhm_wake_up_ths_t                   wake_up_ths;
  asm330lhm_wake_up_dur_t                   wake_up_dur;
  asm330lhm_free_fall_t                     free_fall;
  asm330lhm_md1_cfg_t                       md1_cfg;
  asm330lhm_md2_cfg_t                       md2_cfg;
  asm330lhm_internal_freq_fine_t            internal_freq_fine;
  asm330lhm_fifo_data_out_tag_t             fifo_data_out_tag;
  bitwise_t                                 bitwise;
  uint8_t                                   byte;
} asm330lhm_reg_t;

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

int32_t asm330lhm_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);
int32_t asm330lhm_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                            uint8_t *data,
                            uint16_t len);

float_t asm330lhm_from_fs2g_to_mg(int16_t lsb);
float_t asm330lhm_from_fs4g_to_mg(int16_t lsb);
float_t asm330lhm_from_fs8g_to_mg(int16_t lsb);
float_t asm330lhm_from_fs16g_to_mg(int16_t lsb);

float_t asm330lhm_from_fs125dps_to_mdps(int16_t lsb);
float_t asm330lhm_from_fs250dps_to_mdps(int16_t lsb);
float_t asm330lhm_from_fs500dps_to_mdps(int16_t lsb);
float_t asm330lhm_from_fs1000dps_to_mdps(int16_t lsb);
float_t asm330lhm_from_fs2000dps_to_mdps(int16_t lsb);
float_t asm330lhm_from_fs4000dps_to_mdps(int16_t lsb);

float_t asm330lhm_from_lsb_to_celsius(int16_t lsb);

float_t asm330lhm_from_lsb_to_nsec(int32_t lsb);

typedef enum
{
  ASM330LHM_2g   = 0,
  ASM330LHM_16g  = 1, /* if XL_FS_MODE = '1' -> ASM330LHM_2g */
  ASM330LHM_4g   = 2,
  ASM330LHM_8g   = 3,
} asm330lhm_fs_xl_t;
int32_t asm330lhm_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_fs_xl_t val);
int32_t asm330lhm_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_fs_xl_t *val);

typedef enum
{
  ASM330LHM_XL_ODR_OFF    = 0,
  ASM330LHM_XL_ODR_12Hz5  = 1,
  ASM330LHM_XL_ODR_26Hz   = 2,
  ASM330LHM_XL_ODR_52Hz   = 3,
  ASM330LHM_XL_ODR_104Hz  = 4,
  ASM330LHM_XL_ODR_208Hz  = 5,
  ASM330LHM_XL_ODR_417Hz  = 6,
  ASM330LHM_XL_ODR_833Hz  = 7,
  ASM330LHM_XL_ODR_1667Hz = 8,
  ASM330LHM_XL_ODR_3333Hz = 9,
  ASM330LHM_XL_ODR_6667Hz = 10,
} asm330lhm_odr_xl_t;
int32_t asm330lhm_xl_data_rate_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_odr_xl_t val);
int32_t asm330lhm_xl_data_rate_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_odr_xl_t *val);

typedef enum
{
  ASM330LHM_125dps = 1,
  ASM330LHM_250dps = 0,
  ASM330LHM_500dps = 2,
  ASM330LHM_1000dps = 4,
  ASM330LHM_2000dps = 6,
} asm330lhm_fs_g_t;
int32_t asm330lhm_gy_full_scale_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_fs_g_t val);
int32_t asm330lhm_gy_full_scale_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_fs_g_t *val);

typedef enum
{
  ASM330LHM_GY_ODR_OFF    = 0,
  ASM330LHM_GY_ODR_12Hz5  = 1,
  ASM330LHM_GY_ODR_26Hz   = 2,
  ASM330LHM_GY_ODR_52Hz   = 3,
  ASM330LHM_GY_ODR_104Hz  = 4,
  ASM330LHM_GY_ODR_208Hz  = 5,
  ASM330LHM_GY_ODR_417Hz  = 6,
  ASM330LHM_GY_ODR_833Hz  = 7,
  ASM330LHM_GY_ODR_1667Hz = 8,
  ASM330LHM_GY_ODR_3333Hz = 9,
  ASM330LHM_GY_ODR_6667Hz = 10,
} asm330lhm_odr_g_t;
int32_t asm330lhm_gy_data_rate_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_odr_g_t val);
int32_t asm330lhm_gy_data_rate_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_odr_g_t *val);

int32_t asm330lhm_block_data_update_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t asm330lhm_block_data_update_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  ASM330LHM_LSb_1mg  = 0,
  ASM330LHM_LSb_16mg = 1,
} asm330lhm_usr_off_w_t;
int32_t asm330lhm_xl_offset_weight_set(const stmdev_ctx_t *ctx,
                                       asm330lhm_usr_off_w_t val);
int32_t asm330lhm_xl_offset_weight_get(const stmdev_ctx_t *ctx,
                                       asm330lhm_usr_off_w_t *val);

typedef struct
{
  asm330lhm_all_int_src_t       all_int_src;
  asm330lhm_wake_up_src_t       wake_up_src;
  asm330lhm_d6d_src_t           d6d_src;
  asm330lhm_status_reg_t        status_reg;
} asm330lhm_all_sources_t;
int32_t asm330lhm_all_sources_get(const stmdev_ctx_t *ctx,
                                  asm330lhm_all_sources_t *val);

int32_t asm330lhm_status_reg_get(const stmdev_ctx_t *ctx,
                                 asm330lhm_status_reg_t *val);

int32_t asm330lhm_xl_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t asm330lhm_gy_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t asm330lhm_temp_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t asm330lhm_xl_usr_offset_x_set(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);
int32_t asm330lhm_xl_usr_offset_x_get(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);

int32_t asm330lhm_xl_usr_offset_y_set(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);
int32_t asm330lhm_xl_usr_offset_y_get(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);

int32_t asm330lhm_xl_usr_offset_z_set(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);
int32_t asm330lhm_xl_usr_offset_z_get(const stmdev_ctx_t *ctx,
                                      uint8_t *buff);

int32_t asm330lhm_xl_usr_offset_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_xl_usr_offset_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_timestamp_rst(const stmdev_ctx_t *ctx);

int32_t asm330lhm_timestamp_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_timestamp_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_timestamp_raw_get(const stmdev_ctx_t *ctx, uint32_t *val);

typedef enum
{
  ASM330LHM_NO_ROUND      = 0,
  ASM330LHM_ROUND_XL      = 1,
  ASM330LHM_ROUND_GY      = 2,
  ASM330LHM_ROUND_GY_XL   = 3,
} asm330lhm_rounding_t;
int32_t asm330lhm_rounding_mode_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_rounding_t val);
int32_t asm330lhm_rounding_mode_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_rounding_t *val);

int32_t asm330lhm_temperature_raw_get(const stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t asm330lhm_angular_rate_raw_get(const stmdev_ctx_t *ctx,
                                       int16_t *val);

int32_t asm330lhm_acceleration_raw_get(const stmdev_ctx_t *ctx,
                                       int16_t *val);

int32_t asm330lhm_fifo_out_raw_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_device_conf_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_device_conf_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_odr_cal_reg_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_odr_cal_reg_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_DRDY_LATCHED = 0,
  ASM330LHM_DRDY_PULSED  = 1,
} asm330lhm_dataready_pulsed_t;
int32_t asm330lhm_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                      asm330lhm_dataready_pulsed_t val);
int32_t asm330lhm_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                      asm330lhm_dataready_pulsed_t *val);

int32_t asm330lhm_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff);

int32_t asm330lhm_reset_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_reset_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_auto_increment_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_boot_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_boot_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_XL_ST_DISABLE  = 0,
  ASM330LHM_XL_ST_POSITIVE = 1,
  ASM330LHM_XL_ST_NEGATIVE = 2,
} asm330lhm_st_xl_t;
int32_t asm330lhm_xl_self_test_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_st_xl_t val);
int32_t asm330lhm_xl_self_test_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_st_xl_t *val);

typedef enum
{
  ASM330LHM_GY_ST_DISABLE  = 0,
  ASM330LHM_GY_ST_POSITIVE = 1,
  ASM330LHM_GY_ST_NEGATIVE = 3,
} asm330lhm_st_g_t;
int32_t asm330lhm_gy_self_test_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_st_g_t val);
int32_t asm330lhm_gy_self_test_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_st_g_t *val);

int32_t asm330lhm_xl_filter_lp2_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_xl_filter_lp2_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_gy_filter_lp1_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_gy_filter_lp1_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_filter_settling_mask_set(const stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t asm330lhm_filter_settling_mask_get(const stmdev_ctx_t *ctx,
                                           uint8_t *val);

typedef enum
{
  ASM330LHM_ULTRA_LIGHT  = 0,
  ASM330LHM_VERY_LIGHT   = 1,
  ASM330LHM_LIGHT        = 2,
  ASM330LHM_MEDIUM       = 3,
  ASM330LHM_STRONG       = 4,
  ASM330LHM_VERY_STRONG  = 5,
  ASM330LHM_AGGRESSIVE   = 6,
  ASM330LHM_XTREME       = 7,
} asm330lhm_ftype_t;
int32_t asm330lhm_gy_lp1_bandwidth_set(const stmdev_ctx_t *ctx,
                                       asm330lhm_ftype_t val);
int32_t asm330lhm_gy_lp1_bandwidth_get(const stmdev_ctx_t *ctx,
                                       asm330lhm_ftype_t *val);

int32_t asm330lhm_xl_lp2_on_6d_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_xl_lp2_on_6d_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_HP_PATH_DISABLE_ON_OUT    = 0x00,
  ASM330LHM_SLOPE_ODR_DIV_4           = 0x10,
  ASM330LHM_HP_ODR_DIV_10             = 0x11,
  ASM330LHM_HP_ODR_DIV_20             = 0x12,
  ASM330LHM_HP_ODR_DIV_45             = 0x13,
  ASM330LHM_HP_ODR_DIV_100            = 0x14,
  ASM330LHM_HP_ODR_DIV_200            = 0x15,
  ASM330LHM_HP_ODR_DIV_400            = 0x16,
  ASM330LHM_HP_ODR_DIV_800            = 0x17,
  ASM330LHM_HP_REF_MD_ODR_DIV_10      = 0x31,
  ASM330LHM_HP_REF_MD_ODR_DIV_20      = 0x32,
  ASM330LHM_HP_REF_MD_ODR_DIV_45      = 0x33,
  ASM330LHM_HP_REF_MD_ODR_DIV_100     = 0x34,
  ASM330LHM_HP_REF_MD_ODR_DIV_200     = 0x35,
  ASM330LHM_HP_REF_MD_ODR_DIV_400     = 0x36,
  ASM330LHM_HP_REF_MD_ODR_DIV_800     = 0x37,
  ASM330LHM_LP_ODR_DIV_10             = 0x01,
  ASM330LHM_LP_ODR_DIV_20             = 0x02,
  ASM330LHM_LP_ODR_DIV_45             = 0x03,
  ASM330LHM_LP_ODR_DIV_100            = 0x04,
  ASM330LHM_LP_ODR_DIV_200            = 0x05,
  ASM330LHM_LP_ODR_DIV_400            = 0x06,
  ASM330LHM_LP_ODR_DIV_800            = 0x07,
} asm330lhm_hp_slope_xl_en_t;
int32_t asm330lhm_xl_hp_path_on_out_set(const stmdev_ctx_t *ctx,
                                        asm330lhm_hp_slope_xl_en_t val);
int32_t asm330lhm_xl_hp_path_on_out_get(const stmdev_ctx_t *ctx,
                                        asm330lhm_hp_slope_xl_en_t *val);

int32_t asm330lhm_xl_fast_settling_set(const stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t asm330lhm_xl_fast_settling_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  ASM330LHM_USE_SLOPE = 0,
  ASM330LHM_USE_HPF   = 1,
} asm330lhm_slope_fds_t;
int32_t asm330lhm_xl_hp_path_internal_set(const stmdev_ctx_t *ctx,
                                          asm330lhm_slope_fds_t val);
int32_t asm330lhm_xl_hp_path_internal_get(const stmdev_ctx_t *ctx,
                                          asm330lhm_slope_fds_t *val);

typedef enum
{
  ASM330LHM_HP_FILTER_NONE     = 0x00,
  ASM330LHM_HP_FILTER_16mHz    = 0x80,
  ASM330LHM_HP_FILTER_65mHz    = 0x81,
  ASM330LHM_HP_FILTER_260mHz   = 0x82,
  ASM330LHM_HP_FILTER_1Hz04    = 0x83,
} asm330lhm_hpm_g_t;
int32_t asm330lhm_gy_hp_path_internal_set(const stmdev_ctx_t *ctx,
                                          asm330lhm_hpm_g_t val);
int32_t asm330lhm_gy_hp_path_internal_get(const stmdev_ctx_t *ctx,
                                          asm330lhm_hpm_g_t *val);

typedef enum
{
  ASM330LHM_PULL_UP_DISC       = 0,
  ASM330LHM_PULL_UP_CONNECT    = 1,
} asm330lhm_sdo_pu_en_t;
int32_t asm330lhm_sdo_sa0_mode_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_sdo_pu_en_t val);
int32_t asm330lhm_sdo_sa0_mode_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_sdo_pu_en_t *val);

typedef enum
{
  ASM330LHM_SPI_4_WIRE = 0,
  ASM330LHM_SPI_3_WIRE = 1,
} asm330lhm_sim_t;
int32_t asm330lhm_spi_mode_set(const stmdev_ctx_t *ctx,
                               asm330lhm_sim_t val);
int32_t asm330lhm_spi_mode_get(const stmdev_ctx_t *ctx,
                               asm330lhm_sim_t *val);

typedef enum
{
  ASM330LHM_I2C_ENABLE  = 0,
  ASM330LHM_I2C_DISABLE = 1,
} asm330lhm_i2c_disable_t;
int32_t asm330lhm_i2c_interface_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_i2c_disable_t val);
int32_t asm330lhm_i2c_interface_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_i2c_disable_t *val);

typedef struct
{
  asm330lhm_int1_ctrl_t          int1_ctrl;
  asm330lhm_md1_cfg_t            md1_cfg;
} asm330lhm_pin_int1_route_t;
int32_t asm330lhm_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                     asm330lhm_pin_int1_route_t *val);
int32_t asm330lhm_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                     asm330lhm_pin_int1_route_t *val);

typedef struct
{
  asm330lhm_int2_ctrl_t          int2_ctrl;
  asm330lhm_md2_cfg_t            md2_cfg;
} asm330lhm_pin_int2_route_t;
int32_t asm330lhm_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                     asm330lhm_pin_int2_route_t *val);
int32_t asm330lhm_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                     asm330lhm_pin_int2_route_t *val);

typedef enum
{
  ASM330LHM_PUSH_PULL   = 0,
  ASM330LHM_OPEN_DRAIN  = 1,
} asm330lhm_pp_od_t;
int32_t asm330lhm_pin_mode_set(const stmdev_ctx_t *ctx,
                               asm330lhm_pp_od_t val);
int32_t asm330lhm_pin_mode_get(const stmdev_ctx_t *ctx,
                               asm330lhm_pp_od_t *val);

typedef enum
{
  ASM330LHM_ACTIVE_HIGH = 0,
  ASM330LHM_ACTIVE_LOW  = 1,
} asm330lhm_h_lactive_t;
int32_t asm330lhm_pin_polarity_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_h_lactive_t val);
int32_t asm330lhm_pin_polarity_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_h_lactive_t *val);

int32_t asm330lhm_all_on_int1_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_all_on_int1_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_ALL_INT_PULSED            = 0,
  ASM330LHM_ALL_INT_LATCHED           = 3,
} asm330lhm_lir_t;
int32_t asm330lhm_int_notification_set(const stmdev_ctx_t *ctx,
                                       asm330lhm_lir_t val);
int32_t asm330lhm_int_notification_get(const stmdev_ctx_t *ctx,
                                       asm330lhm_lir_t *val);

typedef enum
{
  ASM330LHM_LSb_FS_DIV_64       = 0,
  ASM330LHM_LSb_FS_DIV_256      = 1,
} asm330lhm_wake_ths_w_t;
int32_t asm330lhm_wkup_ths_weight_set(const stmdev_ctx_t *ctx,
                                      asm330lhm_wake_ths_w_t val);
int32_t asm330lhm_wkup_ths_weight_get(const stmdev_ctx_t *ctx,
                                      asm330lhm_wake_ths_w_t *val);

int32_t asm330lhm_wkup_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_wkup_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_xl_usr_offset_on_wkup_set(const stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t asm330lhm_xl_usr_offset_on_wkup_get(const stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t asm330lhm_wkup_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_wkup_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_gy_sleep_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_gy_sleep_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_DRIVE_SLEEP_CHG_EVENT = 0,
  ASM330LHM_DRIVE_SLEEP_STATUS    = 1,
} asm330lhm_sleep_status_on_int_t;
int32_t asm330lhm_act_pin_notification_set(const stmdev_ctx_t *ctx,
                                           asm330lhm_sleep_status_on_int_t val);
int32_t asm330lhm_act_pin_notification_get(const stmdev_ctx_t *ctx,
                                           asm330lhm_sleep_status_on_int_t *val);

typedef enum
{
  ASM330LHM_XL_AND_GY_NOT_AFFECTED      = 0,
  ASM330LHM_XL_12Hz5_GY_NOT_AFFECTED    = 1,
  ASM330LHM_XL_12Hz5_GY_SLEEP           = 2,
  ASM330LHM_XL_12Hz5_GY_PD              = 3,
} asm330lhm_inact_en_t;
int32_t asm330lhm_act_mode_set(const stmdev_ctx_t *ctx,
                               asm330lhm_inact_en_t val);
int32_t asm330lhm_act_mode_get(const stmdev_ctx_t *ctx,
                               asm330lhm_inact_en_t *val);

int32_t asm330lhm_act_sleep_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_act_sleep_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_DEG_80  = 0,
  ASM330LHM_DEG_70  = 1,
  ASM330LHM_DEG_60  = 2,
  ASM330LHM_DEG_50  = 3,
} asm330lhm_sixd_ths_t;
int32_t asm330lhm_6d_threshold_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_sixd_ths_t val);
int32_t asm330lhm_6d_threshold_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_sixd_ths_t *val);

int32_t asm330lhm_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_FF_TSH_156mg = 0,
  ASM330LHM_FF_TSH_219mg = 1,
  ASM330LHM_FF_TSH_250mg = 2,
  ASM330LHM_FF_TSH_312mg = 3,
  ASM330LHM_FF_TSH_344mg = 4,
  ASM330LHM_FF_TSH_406mg = 5,
  ASM330LHM_FF_TSH_469mg = 6,
  ASM330LHM_FF_TSH_500mg = 7,
} asm330lhm_ff_ths_t;
int32_t asm330lhm_ff_threshold_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_ff_ths_t val);
int32_t asm330lhm_ff_threshold_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_ff_ths_t *val);

int32_t asm330lhm_ff_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_ff_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_fifo_watermark_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t asm330lhm_fifo_watermark_get(const stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t asm330lhm_fifo_virtual_sens_odr_chg_set(const stmdev_ctx_t *ctx,
                                                uint8_t val);
int32_t asm330lhm_fifo_virtual_sens_odr_chg_get(const stmdev_ctx_t *ctx,
                                                uint8_t *val);

int32_t asm330lhm_fifo_stop_on_wtm_set(const stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t asm330lhm_fifo_stop_on_wtm_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  ASM330LHM_XL_NOT_BATCHED        =  0,
  ASM330LHM_XL_BATCHED_AT_12Hz5   =  1,
  ASM330LHM_XL_BATCHED_AT_26Hz    =  2,
  ASM330LHM_XL_BATCHED_AT_52Hz    =  3,
  ASM330LHM_XL_BATCHED_AT_104Hz   =  4,
  ASM330LHM_XL_BATCHED_AT_208Hz   =  5,
  ASM330LHM_XL_BATCHED_AT_417Hz   =  6,
  ASM330LHM_XL_BATCHED_AT_833Hz   =  7,
  ASM330LHM_XL_BATCHED_AT_1667Hz  =  8,
  ASM330LHM_XL_BATCHED_AT_3333Hz  =  9,
  ASM330LHM_XL_BATCHED_AT_6667Hz  = 10,
  ASM330LHM_XL_BATCHED_AT_6Hz5    = 11,
} asm330lhm_bdr_xl_t;
int32_t asm330lhm_fifo_xl_batch_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_bdr_xl_t val);
int32_t asm330lhm_fifo_xl_batch_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_bdr_xl_t *val);

typedef enum
{
  ASM330LHM_GY_NOT_BATCHED         = 0,
  ASM330LHM_GY_BATCHED_AT_12Hz5    = 1,
  ASM330LHM_GY_BATCHED_AT_26Hz     = 2,
  ASM330LHM_GY_BATCHED_AT_52Hz     = 3,
  ASM330LHM_GY_BATCHED_AT_104Hz    = 4,
  ASM330LHM_GY_BATCHED_AT_208Hz    = 5,
  ASM330LHM_GY_BATCHED_AT_417Hz    = 6,
  ASM330LHM_GY_BATCHED_AT_833Hz    = 7,
  ASM330LHM_GY_BATCHED_AT_1667Hz   = 8,
  ASM330LHM_GY_BATCHED_AT_3333Hz   = 9,
  ASM330LHM_GY_BATCHED_AT_6667Hz   = 10,
  ASM330LHM_GY_BATCHED_AT_6Hz5     = 11,
} asm330lhm_bdr_gy_t;
int32_t asm330lhm_fifo_gy_batch_set(const stmdev_ctx_t *ctx,
                                    asm330lhm_bdr_gy_t val);
int32_t asm330lhm_fifo_gy_batch_get(const stmdev_ctx_t *ctx,
                                    asm330lhm_bdr_gy_t *val);

typedef enum
{
  ASM330LHM_BYPASS_MODE             = 0,
  ASM330LHM_FIFO_MODE               = 1,
  ASM330LHM_STREAM_TO_FIFO_MODE     = 3,
  ASM330LHM_BYPASS_TO_STREAM_MODE   = 4,
  ASM330LHM_STREAM_MODE             = 6,
  ASM330LHM_BYPASS_TO_FIFO_MODE     = 7,
} asm330lhm_fifo_mode_t;
int32_t asm330lhm_fifo_mode_set(const stmdev_ctx_t *ctx,
                                asm330lhm_fifo_mode_t val);
int32_t asm330lhm_fifo_mode_get(const stmdev_ctx_t *ctx,
                                asm330lhm_fifo_mode_t *val);

typedef enum
{
  ASM330LHM_TEMP_NOT_BATCHED        = 0,
  ASM330LHM_TEMP_BATCHED_AT_52Hz    = 1,
  ASM330LHM_TEMP_BATCHED_AT_12Hz5   = 2,
  ASM330LHM_TEMP_BATCHED_AT_1Hz6    = 3,
} asm330lhm_odr_t_batch_t;
int32_t asm330lhm_fifo_temp_batch_set(const stmdev_ctx_t *ctx,
                                      asm330lhm_odr_t_batch_t val);
int32_t asm330lhm_fifo_temp_batch_get(const stmdev_ctx_t *ctx,
                                      asm330lhm_odr_t_batch_t *val);

typedef enum
{
  ASM330LHM_NO_DECIMATION = 0,
  ASM330LHM_DEC_1         = 1,
  ASM330LHM_DEC_8         = 2,
  ASM330LHM_DEC_32        = 3,
} asm330lhm_odr_ts_batch_t;
int32_t asm330lhm_fifo_timestamp_decimation_set(const stmdev_ctx_t *ctx,
                                                asm330lhm_odr_ts_batch_t val);
int32_t asm330lhm_fifo_timestamp_decimation_get(const stmdev_ctx_t *ctx,
                                                asm330lhm_odr_ts_batch_t *val);

typedef enum
{
  ASM330LHM_XL_BATCH_EVENT   = 0,
  ASM330LHM_GYRO_BATCH_EVENT = 1,
} asm330lhm_trig_counter_bdr_t;
int32_t asm330lhm_fifo_cnt_event_batch_set(const stmdev_ctx_t *ctx,
                                           asm330lhm_trig_counter_bdr_t val);
int32_t asm330lhm_fifo_cnt_event_batch_get(const stmdev_ctx_t *ctx,
                                           asm330lhm_trig_counter_bdr_t *val);

int32_t asm330lhm_rst_batch_counter_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t asm330lhm_rst_batch_counter_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t asm330lhm_batch_counter_threshold_set(const stmdev_ctx_t *ctx,
                                              uint16_t val);
int32_t asm330lhm_batch_counter_threshold_get(const stmdev_ctx_t *ctx,
                                              uint16_t *val);

int32_t asm330lhm_fifo_data_level_get(const stmdev_ctx_t *ctx,
                                      uint16_t *val);

int32_t asm330lhm_fifo_status_get(const stmdev_ctx_t *ctx,
                                  asm330lhm_fifo_status2_t *val);

int32_t asm330lhm_fifo_full_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t asm330lhm_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  ASM330LHM_GYRO_NC_TAG    = 1,
  ASM330LHM_XL_NC_TAG,
  ASM330LHM_TEMPERATURE_TAG,
  ASM330LHM_TIMESTAMP_TAG,
  ASM330LHM_CFG_CHANGE_TAG,
} asm330lhm_fifo_tag_t;
int32_t asm330lhm_fifo_sensor_tag_get(const stmdev_ctx_t *ctx,
                                      asm330lhm_fifo_tag_t *val);

typedef enum
{
  ASM330LHM_DEN_DISABLE    = 0,
  ASM330LHM_LEVEL_FIFO     = 6,
  ASM330LHM_LEVEL_LETCHED  = 3,
  ASM330LHM_LEVEL_TRIGGER  = 2,
  ASM330LHM_EDGE_TRIGGER   = 4,
} asm330lhm_den_mode_t;
int32_t asm330lhm_den_mode_set(const stmdev_ctx_t *ctx,
                               asm330lhm_den_mode_t val);
int32_t asm330lhm_den_mode_get(const stmdev_ctx_t *ctx,
                               asm330lhm_den_mode_t *val);

typedef enum
{
  ASM330LHM_DEN_ACT_LOW  = 0,
  ASM330LHM_DEN_ACT_HIGH = 1,
} asm330lhm_den_lh_t;
int32_t asm330lhm_den_polarity_set(const stmdev_ctx_t *ctx,
                                   asm330lhm_den_lh_t val);
int32_t asm330lhm_den_polarity_get(const stmdev_ctx_t *ctx,
                                   asm330lhm_den_lh_t *val);

typedef enum
{
  ASM330LHM_STAMP_IN_GY_DATA     = 0,
  ASM330LHM_STAMP_IN_XL_DATA     = 1,
  ASM330LHM_STAMP_IN_GY_XL_DATA  = 2,
} asm330lhm_den_xl_g_t;
int32_t asm330lhm_den_enable_set(const stmdev_ctx_t *ctx,
                                 asm330lhm_den_xl_g_t val);
int32_t asm330lhm_den_enable_get(const stmdev_ctx_t *ctx,
                                 asm330lhm_den_xl_g_t *val);

int32_t asm330lhm_den_mark_axis_x_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_den_mark_axis_x_get(const stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t asm330lhm_den_mark_axis_y_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_den_mark_axis_y_get(const stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t asm330lhm_den_mark_axis_z_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330lhm_den_mark_axis_z_get(const stmdev_ctx_t *ctx,
                                      uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* ASM330LHM_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

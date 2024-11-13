/*
 ******************************************************************************
 * @file    lps28dfw5_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LPS28DFW5 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "lps28dfw5_reg.h"

/**
  * @defgroup    LPS28DFW5
  * @brief       This file provides a set of functions needed to drive the
  *              lps28dfw5 nano pressure sensor.
  * @{
  *
  */

/**
  * @defgroup    Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lps28dfw5_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                                  uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lps28dfw5_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                                   uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Private_functions
  * @brief     Section collect all the utility functions needed by APIs.
  * @{
  *
  */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

/**
  * @}
  *
  */

/**
  * @defgroup    Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

static uint8_t pag;
static int32_t dgain;
static uint8_t Delta_P0;
static float knl2;
static float knl3;

float_t lps28dfw5_from_fs1260_to_hPa(int32_t lsb)
{
  float_t hpa = (float_t)lsb /  691200.0f; /* 2700.0f * 256 */
  float_t p0 = 760.0f;

  return (hpa + (knl2 * Delta_P0 + knl3 * powf(Delta_P0, 2))
          - knl2 * powf((hpa - p0), 2)
          - knl3 * powf((hpa - p0), 3));
}

float_t lps28dfw5_from_fs5560_to_hPa(int32_t lsb)
{
  float_t hpa = (float_t)lsb /  345600.0f; /* 1350.0f * 256 */
  float_t p0 = 760.0f;

  return (hpa + (knl2 * Delta_P0 + knl3 * powf(Delta_P0, 2))
          - knl2 * powf((hpa - p0), 2)
          - knl3 * powf((hpa - p0), 3));
}

float_t lps28dfw5_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 100.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    Basic functions
  * @brief       This section groups all the functions concerning device basic
  *              configuration.
  * @{
  *
  */

/**
  * @brief  Device "Who am I".[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   ID values.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_id_get(const stmdev_ctx_t *ctx, lps28dfw5_id_t *val)
{
  uint8_t reg;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_WHO_AM_I, &reg, 1);
  val->whoami = reg;

  return ret;
}

/**
  * @brief  Device ID FS.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   FS ID values: 4BAR or 5BAR
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_id_fs_get(stmdev_ctx_t *ctx, lps28dfw5_id_fs_val_t *val)
{
  lps28dfw5_id_fs_t reg;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_ID_FS, (uint8_t *)&reg, 1);
  if (ret == 0)
  {
    switch (reg.id_fs)
    {
      default:
      case 0:
        *val = LPS28DFW5_FS_4BAR;
        break;
      case 1:
        *val = LPS28DFW5_FS_5BAR;
        break;
    }
  }

  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_bus_mode_set(const stmdev_ctx_t *ctx, lps28dfw5_bus_mode_t *val)
{
  lps28dfw5_i3c_if_ctrl_t i3c_if_ctrl;
  lps28dfw5_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  if (ret == 0)
  {
    if_ctrl.int_en_i3c = ((uint8_t)val->interface & 0x04U) >> 2;
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  }

  if (ret != 0)
  {
    return ret;
  }

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_I3C_IF_CTRL,
                           (uint8_t *)&i3c_if_ctrl, 1);
  if (ret == 0)
  {
    i3c_if_ctrl.asf_on = (uint8_t)val->filter & 0x01U;
    i3c_if_ctrl.I3C_Bus_Avb_Sel = (uint8_t)val->bus_avb_time & 0x03U;
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_I3C_IF_CTRL,
                              (uint8_t *)&i3c_if_ctrl, 1);
  }

  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_bus_mode_get(const stmdev_ctx_t *ctx, lps28dfw5_bus_mode_t *val)
{
  lps28dfw5_i3c_if_ctrl_t i3c_if_ctrl;
  lps28dfw5_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_I3C_IF_CTRL, (uint8_t *)&i3c_if_ctrl, 1);
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);

    switch (if_ctrl.int_en_i3c << 2)
    {
      case LPS28DFW5_SEL_BY_HW:
        val->interface = LPS28DFW5_SEL_BY_HW;
        break;
      case LPS28DFW5_INT_PIN_ON_I3C:
        val->interface = LPS28DFW5_INT_PIN_ON_I3C;
        break;
      default:
        val->interface = LPS28DFW5_SEL_BY_HW;
        break;
    }

    switch (i3c_if_ctrl.asf_on)
    {
      case LPS28DFW5_AUTO:
        val->filter = LPS28DFW5_AUTO;
        break;
      case LPS28DFW5_ALWAYS_ON:
        val->filter = LPS28DFW5_ALWAYS_ON;
        break;
      default:
        val->filter = LPS28DFW5_AUTO;
        break;
    }

    switch (i3c_if_ctrl.I3C_Bus_Avb_Sel)
    {
      case LPS28DFW5_BUS_AVB_TIME_50us:
        val->bus_avb_time = LPS28DFW5_BUS_AVB_TIME_50us;
        break;
      case LPS28DFW5_BUS_AVB_TIME_2us:
        val->bus_avb_time = LPS28DFW5_BUS_AVB_TIME_2us;
        break;
      case LPS28DFW5_BUS_AVB_TIME_1ms:
        val->bus_avb_time = LPS28DFW5_BUS_AVB_TIME_1ms;
        break;
      case LPS28DFW5_BUS_AVB_TIME_25ms:
        val->bus_avb_time = LPS28DFW5_BUS_AVB_TIME_25ms;
        break;
      default:
        val->bus_avb_time = LPS28DFW5_BUS_AVB_TIME_50us;
        break;
    }
  }

  return ret;
}

/**
  * @brief  Configures the bus operating mode.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_init_set(const stmdev_ctx_t *ctx, lps28dfw5_init_t val)
{
  lps28dfw5_ctrl_reg2_t ctrl_reg2;
  lps28dfw5_ctrl_reg3_t ctrl_reg3;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG2, reg, 2);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&ctrl_reg2, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg3, &reg[1]);

    switch (val)
    {
      case LPS28DFW5_BOOT:
        ctrl_reg2.boot = PROPERTY_ENABLE;
        ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG2,
                                  (uint8_t *)&ctrl_reg2, 1);
        break;
      case LPS28DFW5_RESET:
        ctrl_reg2.swreset = PROPERTY_ENABLE;
        ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG2,
                                  (uint8_t *)&ctrl_reg2, 1);
        break;
      case LPS28DFW5_DRV_RDY:
        ctrl_reg2.bdu = PROPERTY_ENABLE;
        ctrl_reg3.if_add_inc = PROPERTY_ENABLE;
        bytecpy(&reg[0], (uint8_t *)&ctrl_reg2);
        bytecpy(&reg[1], (uint8_t *)&ctrl_reg3);
        ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG2, reg, 2);
        break;
      default:
        ctrl_reg2.swreset = PROPERTY_ENABLE;
        ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG2,
                                  (uint8_t *)&ctrl_reg2, 1);
        break;
    }
  }
  return ret;
}

/**
  * @brief  Get the status of the device.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the status of the device.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_status_get(const stmdev_ctx_t *ctx, lps28dfw5_stat_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  lps28dfw5_int_source_t int_source;
  lps28dfw5_ctrl_reg2_t ctrl_reg2;
  lps28dfw5_status_t status;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG2,
                           (uint8_t *)&ctrl_reg2, 1);
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INT_SOURCE, (uint8_t *)&int_source, 1);
  }
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_STATUS, (uint8_t *)&status, 1);
  }
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                             (uint8_t *)&interrupt_cfg, 1);
  }
  val->sw_reset  = ctrl_reg2.swreset;
  val->boot      = int_source.boot_on;
  val->drdy_pres = status.p_da;
  val->drdy_temp = status.t_da;
  val->ovr_pres  = status.p_or;
  val->ovr_temp  = status.t_or;
  val->end_meas  = ~ctrl_reg2.oneshot;
  val->ref_done = ~interrupt_cfg.autozero;

  return ret;
}

/**
  * @brief  Electrical pin configuration.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the electrical settings for the configurable pins.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_pin_conf_set(const stmdev_ctx_t *ctx, lps28dfw5_pin_conf_t *val)
{
  lps28dfw5_ctrl_reg3_t ctrl_reg3;
  lps28dfw5_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);

  if (ret == 0)
  {
    if_ctrl.int_pd_dis = ~val->int_pull_down;
    if_ctrl.sda_pu_en = val->sda_pull_up;
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  }
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }
  if (ret == 0)
  {
    ctrl_reg3.pp_od = ~val->int_push_pull;
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }

  return ret;
}

/**
  * @brief  Electrical pin configuration.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the electrical settings for the configurable pins.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_pin_conf_get(const stmdev_ctx_t *ctx, lps28dfw5_pin_conf_t *val)
{
  lps28dfw5_ctrl_reg3_t ctrl_reg3;
  lps28dfw5_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }

  val->int_pull_down = ~if_ctrl.int_pd_dis;
  val->int_push_pull  = ~ctrl_reg3.pp_od;
  val->sda_pull_up  = if_ctrl.sda_pu_en;

  return ret;
}

/**
  * @brief  Get the status of all the interrupt sources.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the status of all the interrupt sources.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_all_sources_get(const stmdev_ctx_t *ctx,
                                  lps28dfw5_all_sources_t *val)
{
  lps28dfw5_fifo_status2_t fifo_status2;
  lps28dfw5_int_source_t int_source;
  lps28dfw5_status_t status;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_STATUS, (uint8_t *)&status, 1);
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INT_SOURCE,
                             (uint8_t *)&int_source, 1);
  }
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_FIFO_STATUS2,
                             (uint8_t *)&fifo_status2, 1);
  }

  val->drdy_pres        = status.p_da;
  val->drdy_temp        = status.t_da;
  val->over_pres        = int_source.ph;
  val->under_pres       = int_source.pl;
  val->thrsld_pres      = int_source.ia;
  val->fifo_full        = fifo_status2.fifo_full_ia;
  val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
  val->fifo_th          = fifo_status2.fifo_wtm_ia;

  return ret;
}


/**
  * @brief  Sensor conversion parameters selection.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   set the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_mode_set(const stmdev_ctx_t *ctx, lps28dfw5_md_t *val)
{
  lps28dfw5_ctrl_reg1_t ctrl_reg1;
  lps28dfw5_ctrl_reg2_t ctrl_reg2;
  uint8_t rev = 0;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG1, reg, 2);

  if (ret == 0)
  {
    bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

    ctrl_reg1.odr = (uint8_t)val->odr;
    ctrl_reg1.avg = (uint8_t)val->avg;
    ctrl_reg2.en_lpfp = (uint8_t)val->lpf & 0x01U;
    ctrl_reg2.lfpf_cfg = ((uint8_t)val->lpf & 0x02U) >> 1;
    ctrl_reg2.fs_mode = (uint8_t)val->fs;

    lps28dfw5_read_reg(ctx, 0x70, &rev, 1);
    switch (rev & 0x3)
    {
      case 0x1: /* bit1 == 0, bit0 == 1 */
        Delta_P0 = 0;
        knl2 = (val->fs == LPS28DFW5_1260hPa) ? -0.000001044f : -0.000001638f;
        knl3 = (val->fs == LPS28DFW5_1260hPa) ? -0.000000000773f : -0.000000000991f;
        break;

      case 0x3: /* bit1 == 1, bit0 == 1 */
      default:
        Delta_P0 = 240;
        if (val->fs == LPS28DFW5_5560hPa)
        {
          uint8_t reg[5] = {0U, 0U, 0U, 0U, 0U}; /* 0x4A - 0x4E */
          float gain;

          /* calculate pag and dgain */
          lps28dfw5_read_reg(ctx, 0x4A, reg, 5);

          pag = (reg[4] >> 3) & 0x1F; /* pag = 0x4E[7:3] */
          dgain = ((reg[3] << 17) & 0x20000) | /* dgain[17] = 0x4D[0]*/
                  ((reg[2] << 16) & 0x10000) | /* dgain[16] = 0x4C[0]*/
                  ((reg[1] <<  8) & 0x0FF00) | /* dgain[15:8] = 0x4B[7:0]*/
                  ((reg[0] <<  0) & 0x000FF);  /* dgain[ 7:0] = 0x4A[7:0]*/

          gain = (12.0f + pag) * dgain * 0.000000004293f * 1350.0f;
          knl2 = -0.0000004321f * gain + 0.000003764f;
          knl3 = -0.0000000004234f * gain + 0.000000003720f;
        }
        else
        {
          knl2 = -0.000001521f;
          knl3 = -0.00000000007694f;
        }

        break;
    }

    bytecpy(&reg[0], (uint8_t *)&ctrl_reg1);
    bytecpy(&reg[1], (uint8_t *)&ctrl_reg2);
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG1, reg, 2);
  }

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   get the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_mode_get(const stmdev_ctx_t *ctx, lps28dfw5_md_t *val)
{
  lps28dfw5_ctrl_reg1_t ctrl_reg1;
  lps28dfw5_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG1, reg, 2);

  if (ret == 0)
  {
    bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

    switch (ctrl_reg2.fs_mode)
    {
      case LPS28DFW5_1260hPa:
        val->fs = LPS28DFW5_1260hPa;
        break;
      case LPS28DFW5_5560hPa:
        val->fs = LPS28DFW5_5560hPa;
        break;
      default:
        val->fs = LPS28DFW5_1260hPa;
        break;
    }

    switch (ctrl_reg1.odr)
    {
      case LPS28DFW5_ONE_SHOT:
        val->odr = LPS28DFW5_ONE_SHOT;
        break;
      case LPS28DFW5_1Hz:
        val->odr = LPS28DFW5_1Hz;
        break;
      case LPS28DFW5_4Hz:
        val->odr = LPS28DFW5_4Hz;
        break;
      case LPS28DFW5_10Hz:
        val->odr = LPS28DFW5_10Hz;
        break;
      case LPS28DFW5_25Hz:
        val->odr = LPS28DFW5_25Hz;
        break;
      case LPS28DFW5_50Hz:
        val->odr = LPS28DFW5_50Hz;
        break;
      case LPS28DFW5_75Hz:
        val->odr = LPS28DFW5_75Hz;
        break;
      case LPS28DFW5_100Hz:
        val->odr = LPS28DFW5_100Hz;
        break;
      case LPS28DFW5_200Hz:
        val->odr = LPS28DFW5_200Hz;
        break;
      default:
        val->odr = LPS28DFW5_ONE_SHOT;
        break;
    }

    switch (ctrl_reg1.avg)
    {
      case LPS28DFW5_4_AVG:
        val->avg = LPS28DFW5_4_AVG;
        break;
      case LPS28DFW5_8_AVG:
        val->avg = LPS28DFW5_8_AVG;
        break;
      case LPS28DFW5_16_AVG:
        val->avg = LPS28DFW5_16_AVG;
        break;
      case LPS28DFW5_32_AVG:
        val->avg = LPS28DFW5_32_AVG;
        break;
      case LPS28DFW5_64_AVG:
        val->avg = LPS28DFW5_64_AVG;
        break;
      case LPS28DFW5_128_AVG:
        val->avg = LPS28DFW5_128_AVG;
        break;
      case LPS28DFW5_256_AVG:
        val->avg = LPS28DFW5_256_AVG;
        break;
      case LPS28DFW5_512_AVG:
        val->avg = LPS28DFW5_512_AVG;
        break;
      default:
        val->avg = LPS28DFW5_4_AVG;
        break;
    }

    switch ((ctrl_reg2.lfpf_cfg << 2) | ctrl_reg2.en_lpfp)
    {
      case LPS28DFW5_LPF_DISABLE:
        val->lpf = LPS28DFW5_LPF_DISABLE;
        break;
      case LPS28DFW5_LPF_ODR_DIV_4:
        val->lpf = LPS28DFW5_LPF_ODR_DIV_4;
        break;
      case LPS28DFW5_LPF_ODR_DIV_9:
        val->lpf = LPS28DFW5_LPF_ODR_DIV_9;
        break;
      default:
        val->lpf = LPS28DFW5_LPF_DISABLE;
        break;
    }

  }
  return ret;
}

/**
  * @brief  Software trigger for One-Shot.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  md    the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_trigger_sw(const stmdev_ctx_t *ctx, lps28dfw5_md_t *md)
{
  lps28dfw5_ctrl_reg2_t ctrl_reg2;
  int32_t ret = 0;

  if (md->odr == LPS28DFW5_ONE_SHOT)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
    ctrl_reg2.oneshot = PROPERTY_ENABLE;
    if (ret == 0)
    {
      ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
    }
  }
  return ret;
}

/**
  * @brief  Retrieve sensor data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  md    the sensor conversion parameters.(ptr)
  * @param  data  data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_data_get(const stmdev_ctx_t *ctx, lps28dfw5_md_t *md,
                           lps28dfw5_data_t *data)
{
  uint8_t buff[5];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_PRESS_OUT_XL, buff, 5);

  /* pressure conversion */
  data->pressure.raw = (int32_t)buff[2];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[1];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[0];
  data->pressure.raw = data->pressure.raw * 256;

  switch (md->fs)
  {
    case LPS28DFW5_1260hPa:
      data->pressure.hpa = lps28dfw5_from_fs1260_to_hPa(data->pressure.raw);
      break;
    case LPS28DFW5_5560hPa:
      data->pressure.hpa = lps28dfw5_from_fs5560_to_hPa(data->pressure.raw);
      break;
    default:
      data->pressure.hpa = 0.0f;
      break;
  }

  /* temperature conversion */
  data->heat.raw = (int16_t)buff[4];
  data->heat.raw = (data->heat.raw * 256) + (int16_t) buff[3];
  data->heat.deg_c = lps28dfw5_from_lsb_to_celsius(data->heat.raw);

  return ret;
}

/**
  * @brief  Pressure output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_pressure_raw_get(const stmdev_ctx_t *ctx, uint32_t *buff)
{
  int32_t ret;
  uint8_t reg[3];

  ret =  lps28dfw5_read_reg(ctx, LPS28DFW5_PRESS_OUT_XL, reg, 3);
  *buff = reg[2];
  *buff = (*buff * 256U) + reg[1];
  *buff = (*buff * 256U) + reg[0];
  *buff *= 256U;

  return ret;
}

/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *buff)
{
  int32_t ret;
  uint8_t reg[2];

  ret =  lps28dfw5_read_reg(ctx, LPS28DFW5_TEMP_OUT_L, reg, 2);
  *buff = (int16_t)reg[1];
  *buff = (*buff * 256) + (int16_t)reg[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     FIFO functions
  * @brief        This section groups all the functions concerning the
  *               management of FIFO.
  * @{
  *
  */

/**
  * @brief  FIFO operation mode selection.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   set the FIFO operation mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_fifo_mode_set(const stmdev_ctx_t *ctx, lps28dfw5_fifo_md_t *val)
{
  lps28dfw5_fifo_ctrl_t fifo_ctrl;
  lps28dfw5_fifo_wtm_t fifo_wtm;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_FIFO_CTRL, reg, 2);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&fifo_ctrl, &reg[0]);
    bytecpy((uint8_t *)&fifo_wtm, &reg[1]);

    fifo_ctrl.f_mode = (uint8_t)val->operation & 0x03U;
    fifo_ctrl.trig_modes = ((uint8_t)val->operation & 0x04U) >> 2;

    if (val->watermark != 0x00U)
    {
      fifo_ctrl.stop_on_wtm = PROPERTY_ENABLE;
    }
    else
    {
      fifo_ctrl.stop_on_wtm = PROPERTY_DISABLE;
    }

    fifo_wtm.wtm = val->watermark;

    bytecpy(&reg[0], (uint8_t *)&fifo_ctrl);
    bytecpy(&reg[1], (uint8_t *)&fifo_wtm);

    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_FIFO_CTRL, reg, 2);
  }
  return ret;
}

/**
  * @brief  FIFO operation mode selection.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   get the FIFO operation mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_fifo_mode_get(const stmdev_ctx_t *ctx, lps28dfw5_fifo_md_t *val)
{
  lps28dfw5_fifo_ctrl_t fifo_ctrl;
  lps28dfw5_fifo_wtm_t fifo_wtm;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_FIFO_CTRL, reg, 2);

  bytecpy((uint8_t *)&fifo_ctrl, &reg[0]);
  bytecpy((uint8_t *)&fifo_wtm, &reg[1]);

  switch ((fifo_ctrl.trig_modes << 2) | fifo_ctrl.f_mode)
  {
    case LPS28DFW5_BYPASS:
      val->operation = LPS28DFW5_BYPASS;
      break;
    case LPS28DFW5_FIFO:
      val->operation = LPS28DFW5_FIFO;
      break;
    case LPS28DFW5_STREAM:
      val->operation = LPS28DFW5_STREAM;
      break;
    case LPS28DFW5_STREAM_TO_FIFO:
      val->operation = LPS28DFW5_STREAM_TO_FIFO;
      break;
    case LPS28DFW5_BYPASS_TO_STREAM:
      val->operation = LPS28DFW5_BYPASS_TO_STREAM;
      break;
    case LPS28DFW5_BYPASS_TO_FIFO:
      val->operation = LPS28DFW5_BYPASS_TO_FIFO;
      break;
    default:
      val->operation = LPS28DFW5_BYPASS;
      break;
  }

  val->watermark = fifo_wtm.wtm;

  return ret;
}

/**
  * @brief  Get the number of samples stored in FIFO.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  md    the sensor conversion parameters.(ptr)
  * @param  val   number of samples stored in FIFO.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_fifo_level_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lps28dfw5_fifo_status1_t fifo_status1;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_FIFO_STATUS1,
                           (uint8_t *)&fifo_status1, 1);

  *val = fifo_status1.fss;

  return ret;
}

/**
  * @brief  Software trigger for One-Shot.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  md    the sensor conversion parameters.(ptr)
  * @param  fmd   get the FIFO operation mode.(ptr)
  * @param  samp  number of samples stored in FIFO.(ptr)
  * @param  data  data retrived from FIFO.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_fifo_data_get(const stmdev_ctx_t *ctx, uint8_t samp,
                                lps28dfw5_md_t *md, lps28dfw5_fifo_data_t *data)
{
  uint8_t fifo_data[3];
  uint8_t i;
  int32_t ret = 0;

  for (i = 0U; i < samp; i++)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_FIFO_DATA_OUT_PRESS_XL, fifo_data, 3);
    data[i].raw = (int32_t)fifo_data[2];
    data[i].raw = (data[i].raw * 256) + (int32_t)fifo_data[1];
    data[i].raw = (data[i].raw * 256) + (int32_t)fifo_data[0];
    data[i].raw = (data[i].raw * 256);

    switch (md->fs)
    {
      case LPS28DFW5_1260hPa:
        data[i].hpa = lps28dfw5_from_fs1260_to_hPa(data[i].raw);
        break;
      case LPS28DFW5_5560hPa:
        data[i].hpa = lps28dfw5_from_fs5560_to_hPa(data[i].raw);
        break;
      default:
        data[i].hpa = 0.0f;
        break;
    }
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     Interrupt signals
  * @brief        This section groups all the functions concerning
  *               the management of interrupt signals.
  * @{
  *
  */

/**
  * @brief  Interrupt pins hardware signal configuration.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the pins hardware signal settings.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_interrupt_mode_set(const stmdev_ctx_t *ctx,
                                     lps28dfw5_int_mode_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  lps28dfw5_ctrl_reg3_t ctrl_reg3;
  lps28dfw5_ctrl_reg4_t ctrl_reg4;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG3, reg, 2);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&ctrl_reg3, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg4, &reg[1]);

    ctrl_reg3.int_h_l = val->active_low;
    ctrl_reg4.drdy_pls = ~val->drdy_latched;

    bytecpy(&reg[0], (uint8_t *)&ctrl_reg3);
    bytecpy(&reg[1], (uint8_t *)&ctrl_reg4);

    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG3, reg, 2);
  }
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                             (uint8_t *)&interrupt_cfg, 1);
  }
  if (ret == 0)
  {
    interrupt_cfg.lir = val->int_latched ;
    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                              (uint8_t *)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt pins hardware signal configuration.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the pins hardware signal settings.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_interrupt_mode_get(const stmdev_ctx_t *ctx,
                                     lps28dfw5_int_mode_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  lps28dfw5_ctrl_reg3_t ctrl_reg3;
  lps28dfw5_ctrl_reg4_t ctrl_reg4;
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG3, reg, 2);
  if (ret == 0)
  {
    ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                             (uint8_t *)&interrupt_cfg, 1);
  }

  bytecpy((uint8_t *)&ctrl_reg3, &reg[0]);
  bytecpy((uint8_t *)&ctrl_reg4, &reg[1]);

  val->active_low = ctrl_reg3.int_h_l;
  val->drdy_latched = ~ctrl_reg4.drdy_pls;
  val->int_latched = interrupt_cfg.lir;

  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the signals to route on int1 pin.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_pin_int_route_set(const stmdev_ctx_t *ctx,
                                    lps28dfw5_pin_int_route_t *val)
{
  lps28dfw5_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  if (ret == 0)
  {
    ctrl_reg4.drdy = val->drdy_pres;
    ctrl_reg4.int_f_wtm = val->fifo_th;
    ctrl_reg4.int_f_ovr = val->fifo_ovr;
    ctrl_reg4.int_f_full = val->fifo_full;

    if ((val->fifo_th != 0U) || (val->fifo_ovr != 0U) || (val->fifo_full != 0U))
    {
      ctrl_reg4.int_en = PROPERTY_ENABLE;
    }
    else
    {
      ctrl_reg4.int_en = PROPERTY_DISABLE;
    }

    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the signals that are routed on int1 pin.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_pin_int_route_get(const stmdev_ctx_t *ctx,
                                    lps28dfw5_pin_int_route_t *val)
{
  lps28dfw5_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  val->drdy_pres =  ctrl_reg4.drdy;
  val->fifo_th = ctrl_reg4.int_f_wtm;
  val->fifo_ovr = ctrl_reg4.int_f_ovr;
  val->fifo_full = ctrl_reg4.int_f_full;

  return ret;

}

/**
  * @}
  *
  */

/**
  * @defgroup     Interrupt on threshold functions
  * @brief        This section groups all the functions concerning
  *               the wake up functionality.
  * @{
  *
  */

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_int_on_threshold_mode_set(const stmdev_ctx_t *ctx,
                                            lps28dfw5_int_th_md_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  lps28dfw5_ths_p_l_t ths_p_l;
  lps28dfw5_ths_p_h_t ths_p_h;
  uint8_t reg[3];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG, reg, 3);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&interrupt_cfg, &reg[0]);
    bytecpy((uint8_t *)&ths_p_l, &reg[1]);
    bytecpy((uint8_t *)&ths_p_h, &reg[2]);

    interrupt_cfg.phe = val->over_th;
    interrupt_cfg.ple = val->under_th;
    ths_p_h.ths = (uint8_t)(val->threshold / 256U);
    ths_p_l.ths = (uint8_t)(val->threshold - (ths_p_h.ths * 256U));

    bytecpy(&reg[0], (uint8_t *)&interrupt_cfg);
    bytecpy(&reg[1], (uint8_t *)&ths_p_l);
    bytecpy(&reg[2], (uint8_t *)&ths_p_h);

    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_INTERRUPT_CFG, reg, 3);
  }
  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_int_on_threshold_mode_get(const stmdev_ctx_t *ctx,
                                            lps28dfw5_int_th_md_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  lps28dfw5_ths_p_l_t ths_p_l;
  lps28dfw5_ths_p_h_t ths_p_h;
  uint8_t reg[3];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG, reg, 3);

  bytecpy((uint8_t *)&interrupt_cfg, &reg[0]);
  bytecpy((uint8_t *)&ths_p_l, &reg[1]);
  bytecpy((uint8_t *)&ths_p_h, &reg[2]);

  val->over_th = interrupt_cfg.phe;
  val->under_th = interrupt_cfg.ple;
  val->threshold = ths_p_h.ths;
  val->threshold = (val->threshold * 256U)  + ths_p_l.ths;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     Reference value of pressure
  * @brief        This section groups all the functions concerning
  *               the wake up functionality.
  * @{
  *
  */

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_reference_mode_set(const stmdev_ctx_t *ctx, lps28dfw5_ref_md_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);
  if (ret == 0)
  {

    interrupt_cfg.autozero = val->get_ref;
    interrupt_cfg.autorefp = (uint8_t)val->apply_ref & 0x01U;

    interrupt_cfg.reset_az  = ((uint8_t)val->apply_ref & 0x02U) >> 1;
    interrupt_cfg.reset_arp = ((uint8_t)val->apply_ref & 0x02U) >> 1;

    ret = lps28dfw5_write_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                              (uint8_t *)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_reference_mode_get(const stmdev_ctx_t *ctx, lps28dfw5_ref_md_t *val)
{
  lps28dfw5_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);

  switch ((interrupt_cfg.reset_az << 1) |
          interrupt_cfg.autorefp)
  {
    case LPS28DFW5_OUT_AND_INTERRUPT:
      val->apply_ref = LPS28DFW5_OUT_AND_INTERRUPT;
      break;
    case LPS28DFW5_ONLY_INTERRUPT:
      val->apply_ref = LPS28DFW5_ONLY_INTERRUPT;
      break;
    default:
      val->apply_ref = LPS28DFW5_RST_REFS;
      break;
  }
  val->get_ref = interrupt_cfg.autozero;

  return ret;
}

/**
  * @brief  Reference Pressure LSB data .[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_refp_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_REF_P_L, reg, 2);

  *val = (int16_t)reg[1];
  *val = *val * 256 + (int16_t)reg[0];

  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_opc_set(const stmdev_ctx_t *ctx, int16_t val)
{
  uint8_t reg[2];
  int32_t ret;

  reg[1] = (uint8_t)(((uint16_t)val & 0xFF00U) / 256U);
  reg[0] = (uint8_t)((uint16_t)val & 0x00FFU);

  ret = lps28dfw5_write_reg(ctx, LPS28DFW5_RPDS_L, reg, 2);

  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps28dfw5_opc_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t reg[2];
  int32_t ret;

  ret = lps28dfw5_read_reg(ctx, LPS28DFW5_RPDS_L, reg, 2);

  *val = (int16_t)reg[1];
  *val = *val * 256 + (int16_t)reg[0];

  return ret;
}


/**
  * @}
  *
  */

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

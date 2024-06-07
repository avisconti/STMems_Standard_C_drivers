/*
 ******************************************************************************
 * @file    steng1ax_reg.c
 * @author  Sensors Software Solution Team
 * @brief   STENG1AX driver file
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

#include "steng1ax_reg.h"

/**
  * @defgroup    STENG1AX
  * @brief       This file provides a set of functions needed to drive the
  *              steng1ax sensor.
  * @{
  *
  */

/**
  * @defgroup    STENG1AX_Interfaces_Functions
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
int32_t __weak steng1ax_read_reg(const stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                                   uint16_t len)
{
  int32_t ret;

  if (ctx == NULL) return -1;

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
int32_t __weak steng1ax_write_reg(const stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                                    uint16_t len)
{
  int32_t ret;

  if (ctx == NULL) return -1;

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    STENG1AX_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t steng1ax_from_fs2g_to_mg(int16_t lsb)
{
  return (float_t)lsb * 0.061f;
}

float_t steng1ax_from_fs4g_to_mg(int16_t lsb)
{
  return (float_t)lsb * 0.122f;
}

float_t steng1ax_from_fs8g_to_mg(int16_t lsb)
{
  return (float_t)lsb * 0.244f;
}

float_t steng1ax_from_fs16g_to_mg(int16_t lsb)
{
  return (float_t)lsb * 0.488f;
}

float_t steng1ax_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 355.5f) + 25.0f;
}

/**
  * @}
  *
  */

/**
  * @defgroup Common
  * @brief    Common
  * @{/
  *
  */
float_t steng1ax_from_lsb_to_mv(int16_t lsb)
{
  return ((float_t)lsb) / 1311.0f;
}

/**
  * @brief  Device ID.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Device ID.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_device_id_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_WHO_AM_I, val, 1);

  return ret;
}

/**
  * @brief  Enter Power Down state
  *
  * @param  ctx      read / write interface definitions
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_power_up(const stmdev_ctx_t *ctx)
{
  steng1ax_en_device_config_t dev_cfg = {0};
  int32_t ret;

  dev_cfg.en_dev_conf = PROPERTY_ENABLE;
  ret = steng1ax_write_reg(ctx, STENG1AX_EN_DEVICE_CONFIG, (uint8_t *)&dev_cfg, 1);

  if (ctx->mdelay == NULL) {
    ctx->mdelay(25);
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
int32_t steng1ax_init_set(const stmdev_ctx_t *ctx, steng1ax_init_t val)
{
  steng1ax_ctrl1_t ctrl1;
  steng1ax_ctrl4_t ctrl4;
  int32_t ret = 0;

  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t*)&ctrl1, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL4, (uint8_t*)&ctrl4, 1);
  switch (val) {
    case STENG1AX_BOOT:
      ctrl4.boot = PROPERTY_ENABLE;
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL4, (uint8_t*)&ctrl4, 1);
      break;
    case STENG1AX_SENSOR_ONLY_ON:
      /* no embedded funcs are used */
      ctrl4.emb_func_en = PROPERTY_DISABLE;
      ctrl1.if_add_inc = PROPERTY_ENABLE;
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL4, (uint8_t*)&ctrl4, 1);
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL1, (uint8_t*)&ctrl1, 1);
      break;
    case STENG1AX_SENSOR_EMB_FUNC_ON:
      /* complete configuration is used */
      ctrl4.emb_func_en = PROPERTY_ENABLE;
      ctrl1.if_add_inc = PROPERTY_ENABLE;
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL4, (uint8_t*)&ctrl4, 1);
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL1, (uint8_t*)&ctrl1, 1);
      break;
    case STENG1AX_RESET:
    default:
      ctrl1.sw_reset = PROPERTY_ENABLE;
      ret += steng1ax_write_reg(ctx, STENG1AX_CTRL1, (uint8_t*)&ctrl1, 1);
      break;
  }
  return ret;
}

/**
  * @brief  Get all status flags of the device.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the status flags of the device (boot, reset, drdy) .(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_all_status_get(const stmdev_ctx_t *ctx, steng1ax_status_t *val)
{
  steng1ax_status_register_t status_register;
  steng1ax_ctrl1_t ctrl1;
  steng1ax_ctrl4_t ctrl4;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_STATUS, (uint8_t*)&status_register, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t*)&ctrl1, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL4, (uint8_t*)&ctrl4, 1);

  val->sw_reset = ctrl1.sw_reset;
  val->boot     = ctrl4.boot;
  val->drdy     = status_register.drdy;

  return ret;
}

/**
  * @brief  Get drdy status flag of the device.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   the drdy status flag of the device .(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_drdy_status_get(const stmdev_ctx_t *ctx, steng1ax_status_t *val)
{
  steng1ax_status_register_t status_register;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_STATUS, (uint8_t*)&status_register, 1);
  val->drdy     = status_register.drdy;

  return ret;
}

/**
  * @brief  Enables pulsed data-ready mode (~75 us).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      DRDY_LATCHED, DRDY_PULSED,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_data_ready_mode_set(const stmdev_ctx_t *ctx, steng1ax_data_ready_mode_t val)
{
  steng1ax_ctrl1_t ctrl1;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);

  if (ret == 0)
  {
    ctrl1.drdy_pulsed = ((uint8_t)val & 0x1U);
    ret = steng1ax_write_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}

/**
  * @brief  Enables pulsed data-ready mode (~75 us).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      DRDY_LATCHED, DRDY_PULSED,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_data_ready_mode_get(const stmdev_ctx_t *ctx, steng1ax_data_ready_mode_t *val)
{
  steng1ax_ctrl1_t ctrl1;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);

  switch ((ctrl1.drdy_pulsed))
  {
    case STENG1AX_DRDY_LATCHED:
      *val = STENG1AX_DRDY_LATCHED;
      break;

    case STENG1AX_DRDY_PULSED:
      *val = STENG1AX_DRDY_PULSED;
      break;

    default:
      *val = STENG1AX_DRDY_LATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Sensor mode.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   set the sensor ODR and bandwith.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_mode_set(const stmdev_ctx_t *ctx, steng1ax_md_t *val)
{
  steng1ax_ctrl3_t ctrl3;
  steng1ax_ctrl5_t ctrl5;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL5, (uint8_t*)&ctrl5, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL3, (uint8_t*)&ctrl3, 1);
  if (ret != 0) { return ret; }

  /* set odr and bandwidth */
  switch (val->odr) {
    case STENG1AX_800Hz:
      ctrl3.lpf0_en = PROPERTY_ENABLE;
      ctrl5.odr = 0xb;
      ctrl5.bw = (uint8_t)val->bw;
      break;
    case STENG1AX_3200Hz:
      ctrl3.lpf0_en = PROPERTY_DISABLE;
      ctrl5.odr = 0xb;
      ctrl5.bw = (uint8_t)val->bw;
      break;
    case STENG1AX_OFF:
    default:
      ctrl5.odr = 0x0;
      break;
  }

  ret = steng1ax_write_reg(ctx, STENG1AX_CTRL5, (uint8_t*)&ctrl5, 1);
  ret += steng1ax_write_reg(ctx, STENG1AX_CTRL3, (uint8_t*)&ctrl3, 1);

  return ret;
}

/**
  * @brief  Sensor mode.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   get the sensor ODR and bandwidth.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_mode_get(const stmdev_ctx_t *ctx, steng1ax_md_t *val)
{
  steng1ax_ctrl3_t ctrl3;
  steng1ax_ctrl5_t ctrl5;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL5, (uint8_t*)&ctrl5, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL3, (uint8_t*)&ctrl3, 1);

  switch (ctrl5.odr) {
    case 0xb:
      val->odr = (ctrl3.lpf0_en == PROPERTY_ENABLE) ? STENG1AX_800Hz : STENG1AX_3200Hz;
      break;
    case 0x0:
    default:
      val->odr = STENG1AX_OFF;
      break;
  }

  switch (ctrl5.bw) {
    case STENG1AX_ODR_div_2:
      val->bw = STENG1AX_ODR_div_2;
      break;
    case STENG1AX_ODR_div_4:
      val->bw = STENG1AX_ODR_div_4;
      break;
    case STENG1AX_ODR_div_8:
      val->bw = STENG1AX_ODR_div_8;
      break;
    case STENG1AX_ODR_div_16:
      val->bw = STENG1AX_ODR_div_16;
      break;
    default:
      val->bw = STENG1AX_ODR_div_2;
      break;
  }

  return ret;
}

int32_t steng1ax_all_sources_get(const stmdev_ctx_t *ctx, steng1ax_all_sources_t *val)
{
  steng1ax_status_register_t status;
  steng1ax_fifo_status1_t fifo;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_STATUS, (uint8_t*)&status, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_STATUS1, (uint8_t*)&fifo, 1);

  val->drdy = status.drdy;
  val->fifo_ovr = fifo.fifo_ovr_ia;
  val->fifo_th = fifo.fifo_wtm_ia;

  return ret;
}

/**
  * @brief  AH_ENG data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  data  data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ah_eng_data_get(const stmdev_ctx_t *ctx, steng1ax_ah_eng_data_t *data)
{
  uint8_t buff[2];
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_OUT_AH_ENG_L, buff, 2);

  data->raw = (int16_t)buff[1U];
  data->raw = (data->raw * 256) + (int16_t) buff[0];

  data->mv = steng1ax_from_lsb_to_mv(data->raw);
  return ret;
}

/**
  * @brief  Configures I3C bus.[set]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   configuration params
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_i3c_configure_set(const stmdev_ctx_t *ctx, steng1ax_i3c_cfg_t *val)
{
  steng1ax_i3c_if_ctrl_t i3c_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_I3C_IF_CTRL, (uint8_t *)&i3c_cfg, 1);

  if (ret == 0)
  {
    i3c_cfg.bus_act_sel = (uint8_t)val->bus_act_sel;
    i3c_cfg.dis_drstdaa = val->drstdaa_en;
    i3c_cfg.asf_on = val->asf_on;
    ret = steng1ax_write_reg(ctx, STENG1AX_I3C_IF_CTRL, (uint8_t *)&i3c_cfg, 1);
  }

  return ret;
}

/**
  * @brief  Configures I3C bus.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  val   configuration params
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */int32_t steng1ax_i3c_configure_get(const stmdev_ctx_t *ctx, steng1ax_i3c_cfg_t *val)
{
  steng1ax_i3c_if_ctrl_t i3c_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_I3C_IF_CTRL, (uint8_t *)&i3c_cfg, 1);

  val->drstdaa_en = i3c_cfg.dis_drstdaa;
  val->asf_on = i3c_cfg.asf_on;

  switch (val->bus_act_sel) {
    case STENG1AX_I3C_BUS_AVAIL_TIME_20US:
     val->bus_act_sel = STENG1AX_I3C_BUS_AVAIL_TIME_20US;
     break;

    case STENG1AX_I3C_BUS_AVAIL_TIME_50US:
     val->bus_act_sel = STENG1AX_I3C_BUS_AVAIL_TIME_50US;
     break;

    case STENG1AX_I3C_BUS_AVAIL_TIME_1MS:
     val->bus_act_sel = STENG1AX_I3C_BUS_AVAIL_TIME_1MS;
     break;

    case STENG1AX_I3C_BUS_AVAIL_TIME_25MS:
    default:
     val->bus_act_sel = STENG1AX_I3C_BUS_AVAIL_TIME_25MS;
     break;
  }

 return ret;
}

/**
  * @brief  Change memory bank.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      MAIN_MEM_BANK, EMBED_FUNC_MEM_BANK, SENSOR_HUB_MEM_BANK, STRED_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_mem_bank_set(const stmdev_ctx_t *ctx, steng1ax_mem_bank_t val)
{
  steng1ax_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  if (ret == 0)
  {
    func_cfg_access.emb_func_reg_access = ((uint8_t)val & 0x1U);
    ret = steng1ax_write_reg(ctx, STENG1AX_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  }

  return ret;
}

/**
  * @brief  Change memory bank.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      MAIN_MEM_BANK, EMBED_FUNC_MEM_BANK, SENSOR_HUB_MEM_BANK, STRED_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_mem_bank_get(const stmdev_ctx_t *ctx, steng1ax_mem_bank_t *val)
{
  steng1ax_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  switch ((func_cfg_access.emb_func_reg_access))
  {
    case STENG1AX_MAIN_MEM_BANK:
      *val = STENG1AX_MAIN_MEM_BANK;
      break;

    case STENG1AX_EMBED_FUNC_MEM_BANK:
      *val = STENG1AX_EMBED_FUNC_MEM_BANK;
      break;

    default:
      *val = STENG1AX_MAIN_MEM_BANK;
      break;
  }
  return ret;
}

/**
  * @brief  Write buffer in a page.
  *
  * @param  ctx      read / write interface definitions
  * @param  address  Address of page register to be written (page number in 8-bit
  *                  msb, register address in 8-bit lsb).
  * @param  buf      Pointer to data buffer.
  * @param  len      Buffer len.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ln_pg_write(const stmdev_ctx_t *ctx, uint16_t address, uint8_t *buf, uint8_t len)
{
  steng1ax_page_address_t  page_address;
  steng1ax_page_sel_t page_sel;
  steng1ax_page_rw_t page_rw;
  uint8_t msb;
  uint8_t lsb;
  int32_t ret;
  uint8_t i ;

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
    page_rw.page_read = PROPERTY_DISABLE;
    page_rw.page_write = PROPERTY_ENABLE;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);

    ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
    page_sel.page_sel = msb;
    page_sel.not_used0 = 1; // Default value
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);

    page_address.page_addr = lsb;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_ADDRESS, (uint8_t *)&page_address, 1);

    for (i = 0; ((i < len) && (ret == 0)); i++)
    {
      ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_VALUE, &buf[i], 1);
      lsb++;

      /* Check if page wrap */
      if (((lsb & 0xFFU) == 0x00U) && (ret == 0))
      {
        msb++;
        ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
        page_sel.page_sel = msb;
        page_sel.not_used0 = 1; // Default value
        ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
      }
    }

    page_sel.page_sel = 0;
    page_sel.not_used0 = 1;// Default value
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);

    ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
    page_rw.page_read = PROPERTY_DISABLE;
    page_rw.page_write = PROPERTY_DISABLE;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Read buffer in a page.
  *
  * @param  ctx      read / write interface definitions
  * @param  address  Address of page register to be read (page number in 8-bit
  *                  msb, register address in 8-bit lsb).
  * @param  buf      Pointer to data buffer.
  * @param  len      Buffer len.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ln_pg_read(const stmdev_ctx_t *ctx, uint16_t address, uint8_t *buf, uint8_t len)
{
  steng1ax_page_address_t  page_address;
  steng1ax_page_sel_t page_sel;
  steng1ax_page_rw_t page_rw;
  uint8_t msb;
  uint8_t lsb;
  int32_t ret;
  uint8_t i ;

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
    page_rw.page_read = PROPERTY_ENABLE;
    page_rw.page_write = PROPERTY_DISABLE;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);

    ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
    page_sel.page_sel = msb;
    page_sel.not_used0 = 1; // Default value
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);

    page_address.page_addr = lsb;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_ADDRESS, (uint8_t *)&page_address, 1);

    for (i = 0; ((i < len) && (ret == 0)); i++)
    {
      ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_VALUE, &buf[i], 1);
      lsb++;

      /* Check if page wrap */
      if (((lsb & 0xFFU) == 0x00U) && (ret == 0))
      {
        msb++;
        ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
        page_sel.page_sel = msb;
        page_sel.not_used0 = 1; // Default value
        ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);
      }
    }

    page_sel.page_sel = 0;
    page_sel.not_used0 = 1;// Default value
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_SEL, (uint8_t *)&page_sel, 1);

    ret += steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
    page_rw.page_read = PROPERTY_DISABLE;
    page_rw.page_write = PROPERTY_DISABLE;
    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup Interrupt PINs
  * @brief    Interrupt PINs
  * @{/
  *
  */

/**
  * @brief       External Clock Enable/Disable on INT pin.[set]
  *
  * @param  ctx  read / write interface definitions
  * @param  val  0: disable ext_clk - 1: enable ext_clk
  * @retval      interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ext_clk_en_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  steng1ax_ext_clk_cfg_t clk;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&clk, 1);
  clk.ext_clk_en = val;
  ret += steng1ax_write_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&clk, 1);

  return ret;
}

/**
  * @brief       External Clock Enable/Disable on INT pin.[get]
  *
  * @param  ctx  read / write interface definitions
  * @param  val  0: disable ext_clk - 1: enable ext_clk
  * @retval      interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ext_clk_en_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_ext_clk_cfg_t clk;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&clk, 1);
  *val = clk.ext_clk_en;

  return ret;
}

/**
  * @brief       Electrical pin configuration.[set]
  *
  * @param  ctx  read / write interface definitions
  * @param  val  the electrical settings for the configurable pins.(ptr)
  * @retval      interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_pin_conf_set(const stmdev_ctx_t *ctx, steng1ax_pin_conf_t *val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  if (ret == 0)
  {
    pin_ctrl.cs_pu_dis = ~val->cs_pull_up;
    pin_ctrl.sda_pu_en = val->sda_pull_up;
    pin_ctrl.sdo_pu_en = val->sdo_pull_up;
    pin_ctrl.pp_od = ~val->int_push_pull;

    ret = steng1ax_write_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}

/**
  * @brief       Electrical pin configuration.[get]
  *
  * @param  ctx  read / write interface definitions
  * @param  val  the electrical settings for the configurable pins.(ptr)
  * @retval      interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_pin_conf_get(const stmdev_ctx_t *ctx, steng1ax_pin_conf_t *val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  val->cs_pull_up = ~pin_ctrl.cs_pu_dis;
  val->sda_pull_up = pin_ctrl.sda_pu_en;
  val->sdo_pull_up = pin_ctrl.sdo_pu_en;
  val->int_push_pull = ~pin_ctrl.pp_od;

  return ret;
}

/**
  * @brief  Interrupt activation level.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      ACTIVE_HIGH, ACTIVE_LOW,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_int_pin_polarity_set(const stmdev_ctx_t *ctx, steng1ax_int_pin_polarity_t val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  if (ret == 0)
  {
    pin_ctrl.h_lactive = (uint8_t)val;
    ret = steng1ax_write_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt activation level.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      ACTIVE_HIGH, ACTIVE_LOW,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_int_pin_polarity_get(const stmdev_ctx_t *ctx, steng1ax_int_pin_polarity_t *val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  switch ((pin_ctrl.h_lactive))
  {
    case STENG1AX_ACTIVE_HIGH:
      *val = STENG1AX_ACTIVE_HIGH;
      break;

    case STENG1AX_ACTIVE_LOW:
      *val = STENG1AX_ACTIVE_LOW;
      break;

    default:
      *val = STENG1AX_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  SPI mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      SPI_4_WIRE, SPI_3_WIRE,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_spi_mode_set(const stmdev_ctx_t *ctx, steng1ax_spi_mode val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  if (ret == 0)
  {
    pin_ctrl.sim = (uint8_t)val;
    ret = steng1ax_write_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}

/**
  * @brief  SPI mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      SPI_4_WIRE, SPI_3_WIRE,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_spi_mode_get(const stmdev_ctx_t *ctx, steng1ax_spi_mode *val)
{
  steng1ax_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);

  switch ((pin_ctrl.sim))
  {
    case STENG1AX_SPI_4_WIRE:
      *val = STENG1AX_SPI_4_WIRE;
      break;

    case STENG1AX_SPI_3_WIRE:
      *val = STENG1AX_SPI_3_WIRE;
      break;

    default:
      *val = STENG1AX_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  routes interrupt signals on INT pin.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      routes interrupt signals on INT pin.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_pin_int_route_set(const stmdev_ctx_t *ctx, steng1ax_pin_int_route_t *val)
{
  steng1ax_ctrl1_t ctrl1;
  steng1ax_ctrl2_t ctrl2;
  steng1ax_md1_cfg_t md1_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);
  ctrl1.int1_pin_en = PROPERTY_ENABLE;
  ret += steng1ax_write_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0) { return ret; }

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL2, (uint8_t *)&ctrl2, 1);

  if (ret == 0)
  {
    ctrl2.int_drdy = val->drdy;
    ctrl2.int_fifo_ovr = val->fifo_ovr;
    ctrl2.int_fifo_th = val->fifo_th;
    ctrl2.int_fifo_full = val->fifo_full;
    ctrl2.int_boot = val->boot;

    ret = steng1ax_write_reg(ctx, STENG1AX_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  ret += steng1ax_read_reg(ctx, STENG1AX_MD1_CFG, (uint8_t *)&md1_cfg, 1);

  if (ret == 0)
  {
    md1_cfg.int_emb_func = val->emb_function;
    md1_cfg.int_timestamp = val->timestamp;

    ret = steng1ax_write_reg(ctx, STENG1AX_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  }

  return ret;
}

/**
  * @brief  routes interrupt signals on INT pin.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get interrupt signals routing on INT pin.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_pin_int_route_get(const stmdev_ctx_t *ctx, steng1ax_pin_int_route_t *val)
{
  steng1ax_ctrl1_t ctrl1;
  steng1ax_ctrl2_t ctrl2;
  steng1ax_md1_cfg_t md1_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_MD1_CFG, (uint8_t *)&md1_cfg, 1);

  if (ret == 0)
  {
    val->drdy = ctrl2.int_drdy;
    val->fifo_ovr = ctrl2.int_fifo_ovr;
    val->fifo_th = ctrl2.int_fifo_th;
    val->fifo_full = ctrl2.int_fifo_full;
    val->boot = ctrl2.int_boot;
    val->emb_function = md1_cfg.int_emb_func;
    val->timestamp = md1_cfg.int_timestamp;
  }

  return ret;
}

/**
  * @brief  routes embedded func interrupt signals on INT pin.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      routes embedded func interrupt signals on INT pin.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_emb_pin_int_route_set(const stmdev_ctx_t *ctx,
                                       steng1ax_emb_pin_int_route_t *val)
{
  steng1ax_emb_func_int_t emb_func_int;
  steng1ax_md1_cfg_t md1_cfg;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);
  ret += steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_INT, (uint8_t *)&emb_func_int, 1);

  if (ret == 0)
  {
    emb_func_int.int_fsm_lc = val->fsm_lc;
    ret = steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_INT, (uint8_t *)&emb_func_int, 1);
  }
  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  ret += steng1ax_read_reg(ctx, STENG1AX_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  if (ret == 0)
  {
    md1_cfg.int_emb_func = 1;
    ret = steng1ax_write_reg(ctx, STENG1AX_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  }

  return ret;
}

/**
  * @brief  routes embedded func interrupt signals on INT pin.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      routes embedded func interrupt signals on INT pin.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_emb_pin_int_route_get(const stmdev_ctx_t *ctx,
                                       steng1ax_emb_pin_int_route_t *val)
{
  steng1ax_emb_func_int_t emb_func_int;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);
  ret += steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_INT, (uint8_t *)&emb_func_int, 1);

  val->fsm_lc = emb_func_int.int_fsm_lc;

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Embedded Interrupt configuration mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      INT_PULSED, INT_LATCHED
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_embedded_int_config_set(const stmdev_ctx_t *ctx, steng1ax_embedded_int_config_t val)
{
  steng1ax_page_rw_t page_rw;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);
  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);

    switch (val)
    {
      case STENG1AX_EMBEDDED_INT_LEVEL:
        page_rw.emb_func_lir = 0;
        break;

      case STENG1AX_EMBEDDED_INT_LATCHED:
      default:
        page_rw.emb_func_lir = 1;
        break;
    }

    ret += steng1ax_write_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Interrupt configuration mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      INT_DISABLED, INT_PULSED, INT_LATCHED
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_embedded_int_config_get(const stmdev_ctx_t *ctx, steng1ax_embedded_int_config_t *val)
{
  steng1ax_page_rw_t page_rw;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);
  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_PAGE_RW, (uint8_t *)&page_rw, 1);

    if (page_rw.emb_func_lir == 0U) {
      *val = STENG1AX_EMBEDDED_INT_LEVEL;
   } else {
      *val = STENG1AX_EMBEDDED_INT_LATCHED;
    }
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup FIFO
  * @brief    FIFO
  * @{/
  *
  */

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      BYPASS_MODE, FIFO_MODE, STREAM_TO_FIFO_MODE, BYPASS_TO_STREAM_MODE, STREAM_MODE, BYPASS_TO_FIFO_MODE,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_fifo_mode_set(const stmdev_ctx_t *ctx, steng1ax_fifo_mode_t val)
{
  steng1ax_ctrl4_t ctrl4;
  steng1ax_fifo_ctrl_t fifo_ctrl;
  steng1ax_fifo_wtm_t fifo_wtm;
  steng1ax_fifo_batch_dec_t fifo_batch;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL4, (uint8_t *)&ctrl4, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_BATCH_DEC, (uint8_t *)&fifo_batch, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_WTM, (uint8_t *)&fifo_wtm, 1);

  if (ret == 0)
  {
    /* set FIFO mode */
    if (val.operation != STENG1AX_FIFO_OFF)
    {
      ctrl4.fifo_en = 1;
      fifo_ctrl.fifo_mode = ((uint8_t)val.operation & 0x7U);
    }
    else {
      ctrl4.fifo_en = 0;
    }

    /* set batching info */
    if (val.batch.dec_ts != STENG1AX_DEC_TS_OFF)
    {
      fifo_batch.dec_ts_batch = (uint8_t)val.batch.dec_ts;
      fifo_batch.bdr = (uint8_t)val.batch.bdr;
    }

    fifo_ctrl.cfg_chg_en = val.cfg_change_in_fifo;

    /* set watermark */
    if (val.watermark > 0U) {
      fifo_ctrl.stop_on_fth = 1;
      fifo_wtm.fth = val.watermark;
    }

    ret += steng1ax_write_reg(ctx, STENG1AX_FIFO_BATCH_DEC, (uint8_t *)&fifo_batch, 1);
    ret += steng1ax_write_reg(ctx, STENG1AX_FIFO_WTM, (uint8_t *)&fifo_wtm, 1);
    ret += steng1ax_write_reg(ctx, STENG1AX_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
    ret += steng1ax_write_reg(ctx, STENG1AX_CTRL4, (uint8_t *)&ctrl4, 1);
  }

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      BYPASS_MODE, FIFO_MODE, STREAM_TO_FIFO_MODE, BYPASS_TO_STREAM_MODE, STREAM_MODE, BYPASS_TO_FIFO_MODE,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_fifo_mode_get(const stmdev_ctx_t *ctx, steng1ax_fifo_mode_t *val)
{
  steng1ax_ctrl4_t ctrl4;
  steng1ax_fifo_ctrl_t fifo_ctrl;
  steng1ax_fifo_wtm_t fifo_wtm;
  steng1ax_fifo_batch_dec_t fifo_batch;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_CTRL4, (uint8_t *)&ctrl4, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_BATCH_DEC, (uint8_t *)&fifo_batch, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_FIFO_WTM, (uint8_t *)&fifo_wtm, 1);

  if (ret == 0)
  {
    /* get FIFO mode */
    if (ctrl4.fifo_en == 0U) {
      val->operation = STENG1AX_FIFO_OFF;
    }
    else {
      val->operation = (enum steng1ax_operation)fifo_ctrl.fifo_mode;
    }
    val->cfg_change_in_fifo = fifo_ctrl.cfg_chg_en;

    /* get batching info */
    val->batch.dec_ts = (enum steng1ax_dec_ts)fifo_batch.dec_ts_batch;
    val->batch.bdr = (enum steng1ax_bdr)fifo_batch.bdr;

    /* get watermark */
    val->watermark = fifo_wtm.fth;
  }

  return ret;
}

/**
  * @brief  Number of unread sensor data (TAG + 6 bytes) stored in FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Number of unread sensor data (TAG + 6 bytes) stored in FIFO.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_fifo_data_level_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FIFO_STATUS2, &buff, 1);

  *val = buff;

  return ret;
}

int32_t steng1ax_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_fifo_status1_t fifo_status1;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FIFO_STATUS1, (uint8_t *)&fifo_status1, 1);

  *val = fifo_status1.fifo_wtm_ia;

  return ret;
}

int32_t steng1ax_fifo_sensor_tag_get(const stmdev_ctx_t *ctx, steng1ax_fifo_sensor_tag_t *val)
{
  steng1ax_fifo_data_out_tag_t fifo_tag;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FIFO_DATA_OUT_TAG, (uint8_t *)&fifo_tag, 1);

  *val = (steng1ax_fifo_sensor_tag_t) fifo_tag.tag_sensor;

  return ret;
}

int32_t steng1ax_fifo_out_raw_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FIFO_DATA_OUT_X_L, buff, 6);

  return ret;
}

int32_t steng1ax_fifo_data_get(const stmdev_ctx_t *ctx, steng1ax_md_t *md,
                               steng1ax_fifo_data_t *data)
{
  steng1ax_fifo_data_out_tag_t fifo_tag;
  uint8_t fifo_raw[6];
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_FIFO_DATA_OUT_TAG, (uint8_t *)&fifo_tag, 1);
  data->tag = fifo_tag.tag_sensor;

  switch (fifo_tag.tag_sensor) {
    case STENG1AX_AH_ENG_DATA_TAG:
      ret = steng1ax_fifo_out_raw_get(ctx, fifo_raw);

      /* A FIFO word consists of 16-bits AH_ENG data  */
      data->ah_eng.raw = (int16_t)fifo_raw[0] + (int16_t)fifo_raw[1] * 256;
      data->ah_eng.mv = steng1ax_from_lsb_to_mv(data->ah_eng.raw);
      break;
     case STENG1AX_TIMESTAMP_CFG_CHG_TAG:
       ret = steng1ax_fifo_out_raw_get(ctx, fifo_raw);

       data->cfg_chg.cfg_change = fifo_raw[0] >> 7;
       data->cfg_chg.odr = (fifo_raw[0] >> 3) & 0xFU;
       data->cfg_chg.bw = (fifo_raw[0] >> 1) & 0x3U;
       data->cfg_chg.lp_hp = fifo_raw[0] & 0x1U;
       data->cfg_chg.qvar_en = fifo_raw[1] >> 7;
       data->cfg_chg.fs = (fifo_raw[1] >> 5) & 0x3U;
       data->cfg_chg.dec_ts = (fifo_raw[1] >> 3) & 0x3U;
       data->cfg_chg.odr_xl_batch = fifo_raw[1] & 0x7U;

       data->cfg_chg.timestamp = fifo_raw[5];
       data->cfg_chg.timestamp = (data->cfg_chg.timestamp * 256U) +  fifo_raw[4];
       data->cfg_chg.timestamp = (data->cfg_chg.timestamp * 256U) +  fifo_raw[3];
       data->cfg_chg.timestamp = (data->cfg_chg.timestamp * 256U) +  fifo_raw[2];
       break;

     case STENG1AX_FIFO_EMPTY:
     default:
       break;
  }

  return ret;
}

/**
  * @brief  Enables AH_ENG chain.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Enables and configures AH_ENG chain.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ah_eng_mode_set(const stmdev_ctx_t *ctx, steng1ax_ah_eng_mode_t val)
{
  steng1ax_ah_eng_cfg1_t cfg1;
  steng1ax_ah_eng_cfg2_t cfg2;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_AH_ENG_CFG1, (uint8_t *)&cfg1, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_AH_ENG_CFG2, (uint8_t *)&cfg2, 1);

  if (ret == 0)
  {
    cfg1.ah_eng_zin_dis_ah2_eng1 = val.ah_eng_zin_dis_1;
    cfg1.ah_eng_zin_dis_ah2_eng2 = val.ah_eng_zin_dis_2;
    cfg2.ah_eng_en = val.ah_eng_en;
    cfg2.ah_eng_gain = (uint8_t)val.ah_eng_gain;
    cfg2.ah_eng_mode = (uint8_t)val.ah_eng_mode;
    cfg2.ah_eng_zin = (uint8_t)val.ah_eng_zin;

    ret = steng1ax_write_reg(ctx, STENG1AX_AH_ENG_CFG1, (uint8_t *)&cfg1, 1);
    ret += steng1ax_write_reg(ctx, STENG1AX_AH_ENG_CFG2, (uint8_t *)&cfg2, 1);
  }

  return ret;
}

/**
  * @brief  Enables AH_ENG chain.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Enables and configures AH_ENG chain.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ah_eng_mode_get(const stmdev_ctx_t *ctx, steng1ax_ah_eng_mode_t *val)
{
  steng1ax_ah_eng_cfg1_t cfg1;
  steng1ax_ah_eng_cfg2_t cfg2;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_AH_ENG_CFG1, (uint8_t *)&cfg1, 1);
  ret += steng1ax_read_reg(ctx, STENG1AX_AH_ENG_CFG2, (uint8_t *)&cfg2, 1);

  switch (cfg2.ah_eng_gain)
  {
    case STENG1AX_GAIN_2:
      val->ah_eng_gain = STENG1AX_GAIN_2;
      break;

    case STENG1AX_GAIN_4:
      val->ah_eng_gain = STENG1AX_GAIN_4;
      break;

    case STENG1AX_GAIN_8:
      val->ah_eng_gain = STENG1AX_GAIN_8;
      break;

    case STENG1AX_GAIN_16:
    default:
      val->ah_eng_gain = STENG1AX_GAIN_16;
      break;
  }

  switch (cfg2.ah_eng_zin)
  {
    case STENG1AX_100MOhm:
      val->ah_eng_zin = STENG1AX_100MOhm;
      break;

    case STENG1AX_200MOhm:
      val->ah_eng_zin = STENG1AX_200MOhm;
      break;

    case STENG1AX_500MOhm:
      val->ah_eng_zin = STENG1AX_500MOhm;
      break;

    case STENG1AX_1GOhm:
    default:
      val->ah_eng_zin = STENG1AX_1GOhm;
      break;
  }

  switch (cfg2.ah_eng_mode)
  {
    case STENG1AX_MODE_DIFFERENTIAL:
      val->ah_eng_mode = STENG1AX_MODE_DIFFERENTIAL;
      break;

    case STENG1AX_MODE_SINGLE_ENDED_I1_FLOAT:
      val->ah_eng_mode = STENG1AX_MODE_SINGLE_ENDED_I1_FLOAT;
      break;

    case STENG1AX_MODE_SINGLE_ENDED_I2_FLOAT:
      val->ah_eng_mode = STENG1AX_MODE_SINGLE_ENDED_I2_FLOAT;
      break;

    case STENG1AX_MODE_RESET:
    default:
      val->ah_eng_mode = STENG1AX_MODE_RESET;
      break;
  }

  val->ah_eng_zin_dis_1 = cfg1.ah_eng_zin_dis_ah2_eng1;
  val->ah_eng_zin_dis_2 = cfg1.ah_eng_zin_dis_ah2_eng2;
  val->ah_eng_en = cfg2.ah_eng_en;

  return ret;
}

/**
  * @brief  Reset AH_ENG chain
  *
  * @param  ctx      read / write interface definitions
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t steng1ax_ah_eng_reset(const stmdev_ctx_t *ctx)
{
  steng1ax_ah_eng_cfg3_t cfg3 = {0};
  int32_t ret;

  cfg3.ah_eng_active = 0U;
  ret = steng1ax_write_reg(ctx, STENG1AX_AH_ENG_CFG3, (uint8_t *)&cfg3, 1);

  ctx->mdelay(10);

  cfg3.ah_eng_active = 1U;
  ret = steng1ax_write_reg(ctx, STENG1AX_AH_ENG_CFG3, (uint8_t *)&cfg3, 1);

  return ret;
}

/**
  * @defgroup   steng1ax_Timestamp
  * @brief      This section groups all the functions that manage the
  *             timestamp generation.
  * @{
  *
  */

/**
  * @brief  Enables timestamp counter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of timestamp_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_timestamp_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  steng1ax_interrupt_cfg_t int_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_INTERRUPT_CFG, (uint8_t *)&int_cfg, 1);

  if (ret == 0)
  {
    int_cfg.timestamp_en = (uint8_t)val;
    ret = steng1ax_write_reg(ctx, STENG1AX_INTERRUPT_CFG, (uint8_t *)&int_cfg, 1);
  }

  return ret;
}

/**
  * @brief  Enables timestamp counter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of timestamp_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_timestamp_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_interrupt_cfg_t int_cfg;
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_INTERRUPT_CFG, (uint8_t *)&int_cfg, 1);
  *val = int_cfg.timestamp_en;

  return ret;
}

/**
  * @brief  Timestamp first data output register (r).
  *         The value is expressed as a 32-bit word and the bit resolution
  *         is 10 us.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_timestamp_raw_get(const stmdev_ctx_t *ctx, uint32_t *val)
{
  uint8_t buff[4];
  int32_t ret;

  ret = steng1ax_read_reg(ctx, STENG1AX_TIMESTAMP0, buff, 4);
  *val = buff[3];
  *val = (*val * 256U) +  buff[2];
  *val = (*val * 256U) +  buff[1];
  *val = (*val * 256U) +  buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   STENG1AX_finite_state_machine
  * @brief      This section groups all the functions that manage the
  *             state_machine.
  * @{
  *
  */

/**
  * @brief  Interrupt status bit for FSM long counter timeout interrupt
  *         event.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of is_fsm_lc in reg EMB_FUNC_STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_long_cnt_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                              uint8_t *val)
{
  steng1ax_emb_func_status_t emb_func_status;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_STATUS,
                              (uint8_t *)&emb_func_status, 1);

    *val = emb_func_status.is_fsm_lc;
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_en in reg EMB_FUNC_EN_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_emb_fsm_en_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret;

  steng1ax_emb_func_en_b_t emb_func_en_b;
  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_B,
                              (uint8_t *)&emb_func_en_b, 1);

    emb_func_en_b.fsm_en = (uint8_t)val;

    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_EN_B,
                               (uint8_t *)&emb_func_en_b, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fsm_en in reg EMB_FUNC_EN_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_emb_fsm_en_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;

  steng1ax_emb_func_en_b_t emb_func_en_b;
  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_B,
                              (uint8_t *)&emb_func_en_b, 1);

    *val = emb_func_en_b.fsm_en;

    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_EN_B,
                                (uint8_t *)&emb_func_en_b, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_enable_set(const stmdev_ctx_t *ctx,
                                  steng1ax_emb_fsm_enable_t *val)
{
  steng1ax_emb_func_en_b_t emb_func_en_b;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);
  ret += steng1ax_write_reg(ctx, STENG1AX_FSM_ENABLE, (uint8_t *)&val->fsm_enable, 1);
  if (ret != 0) { return ret; }

  ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);

  emb_func_en_b.fsm_en = val->fsm_enable.fsm1_en | val->fsm_enable.fsm2_en |
                         val->fsm_enable.fsm3_en | val->fsm_enable.fsm4_en |
                         val->fsm_enable.fsm5_en | val->fsm_enable.fsm6_en |
                         val->fsm_enable.fsm7_en | val->fsm_enable.fsm8_en;

  ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_enable_get(const stmdev_ctx_t *ctx,
                                  steng1ax_emb_fsm_enable_t *val)
{
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_FSM_ENABLE,
                              (uint8_t *)&val->fsm_enable, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_long_cnt_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    buff[1] = (uint8_t)(val / 256U);
    buff[0] = (uint8_t)(val - (buff[1] * 256U));
    ret = steng1ax_write_reg(ctx, STENG1AX_FSM_LONG_COUNTER_L, buff, 2);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_long_cnt_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_FSM_LONG_COUNTER_L, buff, 2);
    *val = buff[1];
    *val = (*val * 256U) + buff[0];
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register FSM_STATUS_MAINPAGE
  *
  */
int32_t steng1ax_fsm_status_get(const stmdev_ctx_t *ctx,
                                  steng1ax_fsm_status_mainpage_t *val)
{
  return steng1ax_read_reg(ctx, STENG1AX_FSM_STATUS_MAINPAGE,
                             (uint8_t *) val, 1);
}

/**
  * @brief  FSM output registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_OUTS1 to FSM_OUTS16
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_out_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_FSM_OUTS1, val, 8);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_data_rate_set(const stmdev_ctx_t *ctx,
                                   steng1ax_fsm_val_odr_t val)
{
  steng1ax_fsm_odr_t fsm_odr_reg;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_FSM_ODR,
                              (uint8_t *)&fsm_odr_reg, 1);

    fsm_odr_reg.fsm_odr = (uint8_t)val;
    ret += steng1ax_write_reg(ctx, STENG1AX_FSM_ODR,
                                (uint8_t *)&fsm_odr_reg, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_data_rate_get(const stmdev_ctx_t *ctx,
                                   steng1ax_fsm_val_odr_t *val)
{
  steng1ax_fsm_odr_t fsm_odr_reg;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_FSM_ODR,
                              (uint8_t *)&fsm_odr_reg, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  switch (fsm_odr_reg.fsm_odr)
  {
    case STENG1AX_ODR_FSM_12Hz5:
      *val = STENG1AX_ODR_FSM_12Hz5;
      break;

    case STENG1AX_ODR_FSM_25Hz:
      *val = STENG1AX_ODR_FSM_25Hz;
      break;

    case STENG1AX_ODR_FSM_50Hz:
      *val = STENG1AX_ODR_FSM_50Hz;
      break;

    case STENG1AX_ODR_FSM_100Hz:
      *val = STENG1AX_ODR_FSM_100Hz;
      break;

    case STENG1AX_ODR_FSM_200Hz:
      *val = STENG1AX_ODR_FSM_200Hz;
      break;

    case STENG1AX_ODR_FSM_400Hz:
      *val = STENG1AX_ODR_FSM_400Hz;
      break;

    case STENG1AX_ODR_FSM_800Hz:
      *val = STENG1AX_ODR_FSM_800Hz;
      break;

    default:
      *val = STENG1AX_ODR_FSM_12Hz5;
      break;
  }

  return ret;
}

/**
  * @brief  FSM initialization request.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_init in reg FSM_INIT
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_init_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  steng1ax_emb_func_init_b_t emb_func_init_b;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_INIT_B,
                              (uint8_t *)&emb_func_init_b, 1);

    emb_func_init_b.fsm_init = (uint8_t)val;

    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_INIT_B,
                               (uint8_t *)&emb_func_init_b, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM initialization request.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_init in reg FSM_INIT
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_init_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_emb_func_init_b_t emb_func_init_b;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_INIT_B,
                              (uint8_t *)&emb_func_init_b, 1);

    *val = emb_func_init_b.fsm_init;
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM FIFO en bit.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the value of fsm_fifo_en in reg STENG1AX_EMB_FUNC_FIFO_EN
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_fifo_en_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  steng1ax_emb_func_fifo_en_t emb_fifo;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
    emb_fifo.fsm_fifo_en = (uint8_t)val;
    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM FIFO en bit.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read the value of fsm_fifo_en in reg STENG1AX_EMB_FUNC_FIFO_EN
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_fifo_en_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_emb_func_fifo_en_t emb_fifo;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
  *val = emb_fifo.fsm_fifo_en;

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value, the FSM
  *         generates an interrupt.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_long_cnt_int_value_set(const stmdev_ctx_t *ctx,
                                          uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = steng1ax_ln_pg_write(ctx, STENG1AX_FSM_LC_TIMEOUT_L, buff, 2);

  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value, the FSM generates
  *         an interrupt.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_long_cnt_int_value_get(const stmdev_ctx_t *ctx,
                                        uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = steng1ax_ln_pg_read(ctx, STENG1AX_FSM_LC_TIMEOUT_L, buff, 2);
  *val = buff[1];
  *val = (*val * 256U) + buff[0];

  return ret;
}

/**
  * @brief  FSM number of programs register.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_number_of_programs_set(const stmdev_ctx_t *ctx,
                                            uint8_t *buff)
{
  int32_t ret;

  ret = steng1ax_ln_pg_write(ctx, STENG1AX_FSM_PROGRAMS, buff, 2);

  return ret;
}

/**
  * @brief  FSM number of programs register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_number_of_programs_get(const stmdev_ctx_t *ctx,
                                            uint8_t *buff)
{
  int32_t ret;

  ret = steng1ax_ln_pg_read(ctx, STENG1AX_FSM_PROGRAMS, buff, 2);

  return ret;
}

/**
  * @brief  FSM start address register (r/w). First available address is
  *         0x033C.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_start_address_set(const stmdev_ctx_t *ctx,
                                       uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = steng1ax_ln_pg_write(ctx, STENG1AX_FSM_START_ADD_L, buff, 2);

  return ret;
}

/**
  * @brief  FSM start address register (r/w). First available address
  *         is 0x033C.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_fsm_start_address_get(const stmdev_ctx_t *ctx,
                                       uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = steng1ax_ln_pg_read(ctx, STENG1AX_FSM_START_ADD_L, buff, 2);
  *val = buff[1];
  *val = (*val * 256U) +  buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @addtogroup  Machine Learning Core
  * @brief   This section group all the functions concerning the
  *          usage of Machine Learning Core
  * @{
  *
  */

/**
  * @brief  Enable Machine Learning Core.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mlc_en in
  *                  reg EMB_FUNC_EN_B and mlc_before_fsm_en
  *                  in EMB_FUNC_INIT_A
  *
  */
int32_t steng1ax_mlc_set(const stmdev_ctx_t *ctx, steng1ax_mlc_mode_t val)
{
  steng1ax_emb_func_en_a_t emb_en_a;
  steng1ax_emb_func_en_b_t emb_en_b;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
    ret += steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);

    switch(val)
    {
      case STENG1AX_MLC_OFF:
        emb_en_a.mlc_before_fsm_en = 0;
        emb_en_b.mlc_en = 0;
        break;
      case STENG1AX_MLC_ON:
        emb_en_a.mlc_before_fsm_en = 0;
        emb_en_b.mlc_en = 1;
        break;
      case STENG1AX_MLC_ON_BEFORE_FSM:
        emb_en_a.mlc_before_fsm_en = 1;
        emb_en_b.mlc_en = 0;
        break;
      default:
        break;
    }

    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Enable Machine Learning Core.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of mlc_en in
  *                  reg EMB_FUNC_EN_B and mlc_before_fsm_en
  *                  in EMB_FUNC_INIT_A
  *
  */
int32_t steng1ax_mlc_get(const stmdev_ctx_t *ctx, steng1ax_mlc_mode_t *val)
{
  steng1ax_emb_func_en_a_t emb_en_a;
  steng1ax_emb_func_en_b_t emb_en_b;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
    ret += steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);

    if (emb_en_a.mlc_before_fsm_en == 0U && emb_en_b.mlc_en == 0U)
    {
      *val = STENG1AX_MLC_OFF;
    }
    else if (emb_en_a.mlc_before_fsm_en == 0U && emb_en_b.mlc_en == 1U)
    {
      *val = STENG1AX_MLC_ON;
    }
    else if (emb_en_a.mlc_before_fsm_en == 1U)
    {
      *val = STENG1AX_MLC_ON_BEFORE_FSM;
    }
    else
    {
      /* Do nothing */
    }
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Machine Learning Core status register[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register MLC_STATUS_MAINPAGE
  *
  */
int32_t steng1ax_mlc_status_get(const stmdev_ctx_t *ctx,
                                  steng1ax_mlc_status_mainpage_t *val)
{
  return steng1ax_read_reg(ctx, STENG1AX_MLC_STATUS_MAINPAGE,
                             (uint8_t *) val, 1);
}

/**
  * @brief  prgsens_out: [get] Output value of all MLCx decision trees.
  *
  * @param  ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t steng1ax_mlc_out_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_MLC1_SRC, buff, 4);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  Machine Learning Core data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of mlc_odr in
  *                  reg EMB_FUNC_ODR_CFG_C
  *
  */
int32_t steng1ax_mlc_data_rate_set(const stmdev_ctx_t *ctx,
                                     steng1ax_mlc_odr_val_t val)
{
  steng1ax_mlc_odr_t reg;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_MLC_ODR, (uint8_t *)&reg, 1);
    reg.mlc_odr = (uint8_t)val;
    ret += steng1ax_write_reg(ctx, STENG1AX_MLC_ODR, (uint8_t *)&reg, 1);
  }

  if (ret == 0)
  {
    ret = steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);
  }

  return ret;
}

/**
  * @brief  Machine Learning Core data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mlc_odr in
  *                  reg EMB_FUNC_ODR_CFG_C
  *
  */
int32_t steng1ax_mlc_data_rate_get(const stmdev_ctx_t *ctx,
                                     steng1ax_mlc_odr_val_t *val)
{
  steng1ax_mlc_odr_t reg;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_MLC_ODR, (uint8_t *)&reg, 1);

    switch (reg.mlc_odr)
    {
      case STENG1AX_ODR_PRGS_12Hz5:
        *val = STENG1AX_ODR_PRGS_12Hz5;
        break;

      case STENG1AX_ODR_PRGS_25Hz:
        *val = STENG1AX_ODR_PRGS_25Hz;
        break;

      case STENG1AX_ODR_PRGS_50Hz:
        *val = STENG1AX_ODR_PRGS_50Hz;
        break;

      case STENG1AX_ODR_PRGS_100Hz:
        *val = STENG1AX_ODR_PRGS_100Hz;
        break;

      case STENG1AX_ODR_PRGS_200Hz:
        *val = STENG1AX_ODR_PRGS_200Hz;
        break;

      default:
        *val = STENG1AX_ODR_PRGS_12Hz5;
        break;
    }
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  MLC FIFO en bit.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the value of mlc_fifo_en in reg STENG1AX_EMB_FUNC_FIFO_EN
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_mlc_fifo_en_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  steng1ax_emb_func_fifo_en_t emb_fifo;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  if (ret == 0)
  {
    ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
    emb_fifo.mlc_fifo_en = (uint8_t)val;
    ret += steng1ax_write_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
  }

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @brief  MLC FIFO en bit.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read the value of mlc_fifo_en in reg STENG1AX_EMB_FUNC_FIFO_EN
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t steng1ax_mlc_fifo_en_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  steng1ax_emb_func_fifo_en_t emb_fifo;
  int32_t ret;

  ret = steng1ax_mem_bank_set(ctx, STENG1AX_EMBED_FUNC_MEM_BANK);

  ret = steng1ax_read_reg(ctx, STENG1AX_EMB_FUNC_FIFO_EN, (uint8_t *)&emb_fifo, 1);
  *val = emb_fifo.mlc_fifo_en;

  ret += steng1ax_mem_bank_set(ctx, STENG1AX_MAIN_MEM_BANK);

  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

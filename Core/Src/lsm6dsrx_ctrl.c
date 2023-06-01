/*
 ******************************************************************************
 * @file    lsmd6dsrx_ctrl.c (modified from lsm6dsrx_read_data_polling.c)
 * @author  Sensors Software Solution Team, modified by Etgel Lu
 * @brief   This file show how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsrx_ctrl.h"

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 100              // ms
#define SPI_CS_TO_SCK_DELAY 16000 // clk -> 200ns

/* Private variables ---------------------------------------------------------*/
// static int16_t data_raw_acceleration[3];
// static int16_t data_raw_angular_rate[3];
// static int16_t data_raw_temperature;
static uint8_t whoamI, rst;
extern SPI_HandleTypeDef hspi2;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

/**
 * @brief initialize lsm6dsrx driver over SPI for polling mode
 *
 * @param dev_ctx
 */
void lsm6dsrx_read_data_polling_init(stmdev_ctx_t *dev_ctx)
{
  /* Initialize mems driver interface */
  dev_ctx->write_reg = platform_write;
  dev_ctx->read_reg = platform_read;
  dev_ctx->handle = &hspi2;
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsrx_device_id_get(dev_ctx, &whoamI);

  printf("Read LSM6DSRX ID: 0x%x\n", whoamI);

  if (whoamI != LSM6DSRX_ID)
  {
    printf("LSM6DSRX ID error\n");
    while (1)
      ;
  }

  /* Restore default configuration */
  lsm6dsrx_reset_set(dev_ctx, PROPERTY_ENABLE);

  do
  {
    lsm6dsrx_reset_get(dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsrx_i3c_disable_set(dev_ctx, LSM6DSRX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsrx_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsrx_xl_data_rate_set(dev_ctx, LSM6DSRX_XL_ODR_1666Hz);
  lsm6dsrx_gy_data_rate_set(dev_ctx, LSM6DSRX_GY_ODR_1666Hz);
  /* Set full scale */
  lsm6dsrx_xl_full_scale_set(dev_ctx, LSM6DSRX_2g);
  lsm6dsrx_gy_full_scale_set(dev_ctx, LSM6DSRX_2000dps);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dsrx_xl_hp_path_on_out_set(dev_ctx, LSM6DSRX_LP_ODR_DIV_100);
  lsm6dsrx_xl_filter_lp2_set(dev_ctx, PROPERTY_ENABLE);
}

/**
 * @brief read the sensor data in polling mode
 *
 * @param dev_ctx
 * @return int
 */

int lsm6dsrx_read_data_polling(stmdev_ctx_t *dev_ctx, int16_t *data_raw_acceleration, int16_t *data_raw_angular_rate)
{
  uint8_t reg;
  /* Read output only if new xl value is available */
  lsm6dsrx_xl_flag_data_ready_get(dev_ctx, &reg);
  if (reg)
  {
    /* Read acceleration field data */
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6dsrx_acceleration_raw_get(dev_ctx, data_raw_acceleration);
    // lsm6dsrx_from_fs2g_to_mg => return ((float_t)lsb * 0.061f);
    // printf("Acceleration raw data: %d, %d, %d\n",
    //        data_raw_acceleration[0],
    //        data_raw_acceleration[1],
    //        data_raw_acceleration[2]);
  }

  lsm6dsrx_gy_flag_data_ready_get(dev_ctx, &reg);
  if (reg)
  {
    /* Read angular rate field data */
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6dsrx_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
    // lsm6dsrx_from_fs2000dps_to_mdps => return ((float_t)lsb * 70.0f);
    // printf("Angular rate raw data: %d, %d, %d\n",
    //        data_raw_angular_rate[0],
    //        data_raw_angular_rate[1],
    //        data_raw_angular_rate[2]);
  }

  // lsm6dsrx_temp_flag_data_ready_get(dev_ctx, &reg);
  // if (reg)
  // {
  //   /* Read temperature data */
  //   memset(&data_raw_temperature, 0x00, sizeof(int16_t));
  //   lsm6dsrx_temperature_raw_get(dev_ctx, &data_raw_temperature);
  //   // lsm6dsrx_from_lsb_to_celsius => return (((float_t)lsb / 256.0f) + 25.0f);
  //   printf("Temperature [degC]:%d\r\n", data_raw_temperature);
  // }
  return 0;
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(LSM6DSRX_CTRL_CS_GPIO_PORT, LSM6DSRX_CTRL_CS_PIN, GPIO_PIN_RESET);
  //   for(int i = 0; i < SPI_CS_TO_SCK_DELAY; i++);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(LSM6DSRX_CTRL_CS_GPIO_PORT, LSM6DSRX_CTRL_CS_PIN, GPIO_PIN_SET);
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
  reg |= 0x80;
  HAL_GPIO_WritePin(LSM6DSRX_CTRL_CS_GPIO_PORT, LSM6DSRX_CTRL_CS_PIN, GPIO_PIN_RESET);
  //   for(int i = 0; i < SPI_CS_TO_SCK_DELAY; i++);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(LSM6DSRX_CTRL_CS_GPIO_PORT, LSM6DSRX_CTRL_CS_PIN, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

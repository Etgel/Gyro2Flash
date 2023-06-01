/**
  ******************************************************************************
  * @file    lsm6dsrx_ctrl.h
  * @brief   This file contains all the function prototypes for
  *          the lsm6dsrx_ctrl.c driver.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSRX_CTRL_H__
#define __LSM6DSRX_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6dsrx_reg.h"
#include "main.h"

#define LSM6DSRX_CTRL_CS_GPIO_PORT GPIOB
#define LSM6DSRX_CTRL_CS_PIN       GPIO_PIN_12
#define LSM6DSRX_DATA_SIZE         6 * sizeof(int16_t)

#define LSM6DSRX_READY_MAX_COUNT   200

void lsm6dsrx_read_data_polling_init(stmdev_ctx_t *dev_ctx);
int lsm6dsrx_read_data_polling(stmdev_ctx_t *dev_ctx, int16_t *data_raw_acceleration, int16_t *data_raw_angular_rate);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSRX_CTRL_H__ */

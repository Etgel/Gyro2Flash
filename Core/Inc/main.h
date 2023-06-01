/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define WORK_MODE_BUTTON_GPIO_GROUP GPIOB
#define WORK_MODE_BUTTON_GPIO_PIN   GPIO_PIN_1

#define RESERVE_ADDRESS         0x000000
#define MAX_BLOCK_COUNT         50
#define RESERVE_DATA_SIZE       MAX_BLOCK_COUNT * 6
#define BLOCK_SIZE              0x1000
#define MAX_FLASH_SIZE          0x1000000

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
enum GYRO2FLASH_WORK_STATE
{
  GYRO2FLASH_IDLE = 0,
  GYRO2FLASH_WORKING,
  GYRO2FLASH_OVERFLOW,
  GYRO2FLASH_CMD_ERASE_ALL,
  // GYRO2FLASH_CMD_ERASE_RESERVE,
  GYRO2FLASH_CMD_READ_RESERVE,
  GYRO2FLASH_CMD_READ_ALL,
  GYRO2FLASH_CMD_READ_SILCE,
  GYRO2FLASH_CMD_WAIT_FOR_INDEX,
};

struct command_t
{
  const char *pCmd;
  const char *pContent;
  int state;
};

/**
 * ARRAY size of a static declared array
 */
#ifndef ARRAY_SIZE
#   define ARRAY_SIZE(a)  ((sizeof(a) / sizeof(a[0])))
#endif

void back_to_idle(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

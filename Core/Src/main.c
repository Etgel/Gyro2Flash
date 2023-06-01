/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "lsm6dsrx_ctrl.h"
#include "at25sl128_ctrl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const struct command_t commands[] = {
  {
    .pCmd = "erase all",
    .pContent = "erasing all the memory... it may take a while.\r\n",
    .state = GYRO2FLASH_CMD_ERASE_ALL,
  },
  // {
  //   .pCmd = "erase reserve",
  //   .pContent = "erasing reserve area... it is not recommanded.\r\n",
  //   .state = GYRO2FLASH_CMD_ERASE_RESERVE,
  // },
  {
    .pCmd = "read reserve",
    .pContent = "start reading reserve area...\r\n",
    .state = GYRO2FLASH_CMD_READ_RESERVE,
  },
  {
    .pCmd = "read all",
    .pContent = "start reading all the memory...\r\n",
    .state = GYRO2FLASH_CMD_READ_ALL,
  },
  {
    .pCmd = "read",
    .pContent = "",
    .state = GYRO2FLASH_CMD_READ_SILCE,
  },
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
stmdev_ctx_t lsm6dsrx_ctx;
int gyro2flash_state = GYRO2FLASH_IDLE;
extern uint8_t uart_rx_flag;
extern uint8_t uart_rx_buffer[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t  reserve_data[RESERVE_DATA_SIZE] = {0};
  uint8_t  total_block_count = 0;
  uint32_t write_address = RESERVE_DATA_SIZE;
  uint32_t write_start_address = write_address;
  uint32_t write_block_count = 0;
  uint8_t  command_flag = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_GPIO_WritePin(AT25SL128_CTRL_CS_GPIO_PORT, AT25SL128_CTRL_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LSM6DSRX_CTRL_CS_GPIO_PORT, LSM6DSRX_CTRL_CS_PIN, GPIO_PIN_SET);
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lsm6dsrx_read_data_polling_init(&lsm6dsrx_ctx);
  at25sl128_init();
  back_to_idle();
  // UART_RX_IT();
  // read reserve data & set start address & block count
  at25sl128_read_data(RESERVE_ADDRESS, reserve_data, RESERVE_DATA_SIZE);
  for (int i = 0; i < MAX_BLOCK_COUNT; i++)
  {
    if (reserve_data[6 * i] == 0xff) {
      total_block_count = i;
      if (i == 0)
      {
        write_start_address = BLOCK_SIZE;
      }
      else
      {
        write_address = ((reserve_data[6 * i + 2] << 8) | reserve_data[6 * i + 1]) * BLOCK_SIZE
                      + ((reserve_data[6 * i + 5] << 16) | (reserve_data[6 * i + 4] << 8) | reserve_data[6 * i + 3]) * LSM6DSRX_DATA_SIZE;
        write_start_address = (uint32_t)((write_address + BLOCK_SIZE - 1)/ BLOCK_SIZE);
        write_address = write_start_address * BLOCK_SIZE;
      }
      break;
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (gyro2flash_state == GYRO2FLASH_IDLE || gyro2flash_state == GYRO2FLASH_OVERFLOW)
    {
      if (uart_rx_flag == 1)
      {
        command_flag = 0;
        for (int i = 0; i < ARRAY_SIZE(commands); i++)
        {
          if (strncmp((char *)uart_rx_buffer, commands[i].pCmd, strlen(commands[i].pCmd)) == 0)
          {
            gyro2flash_state = commands[i].state;
            printf("%s", commands[i].pContent);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            command_flag = 1;
            break;
          }
        }
        if (command_flag == 0) {
          printf("Invalid command, please input again.\r\n");
          uart_rx_flag = 0;
          UART_RX_IT();
        }
      }
      else
      {
        if (HAL_GPIO_ReadPin(WORK_MODE_BUTTON_GPIO_GROUP, WORK_MODE_BUTTON_GPIO_PIN) == GPIO_PIN_RESET)
        {
          do {
            HAL_Delay(20);
          } while (HAL_GPIO_ReadPin(WORK_MODE_BUTTON_GPIO_GROUP, WORK_MODE_BUTTON_GPIO_PIN) == GPIO_PIN_RESET);
          if (total_block_count >= MAX_BLOCK_COUNT)
          {
            printf("Already test 50 times, please read out and earse data before testing.\r\n");
            gyro2flash_state = GYRO2FLASH_OVERFLOW;
            uart_rx_flag = 0;
            UART_RX_IT();
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
          }
          else
          {
            write_start_address = (uint32_t)((write_address + BLOCK_SIZE - 1)/ BLOCK_SIZE);
            write_address = write_start_address * BLOCK_SIZE;
            // printf("start testing block 0x%x\r\n", write_start_address);
            // printf("write address: 0x%x\r\n", write_address);
            gyro2flash_state = GYRO2FLASH_WORKING;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
          }
        }
      }
    }
    // else if (gyro2flash_state == GYRO2FLASH_CMD_ERASE_RESERVE)
    // {
    //   at25sl128_erase_4k(RESERVE_ADDRESS);
    //   back_to_idle();
    // }
    else if (gyro2flash_state == GYRO2FLASH_CMD_ERASE_ALL)
    {
      at25sl128_erase_all();
      total_block_count = 0;
      write_address = RESERVE_DATA_SIZE;
      printf("all the memory has been erased.\r\n");
      back_to_idle();
    }
    else if (gyro2flash_state == GYRO2FLASH_CMD_READ_RESERVE)
    {
      at25sl128_read_data(RESERVE_ADDRESS, reserve_data, RESERVE_DATA_SIZE);
      for (int i = 0; i < MAX_BLOCK_COUNT; i++)
      {
        if (reserve_data[6 * i] == 0xff)
          break;
        printf("index %d with start address: 0x%x, size: %d\n",
               reserve_data[6 * i],
               ((reserve_data[6 * i + 2] << 8) | reserve_data[6 * i + 1]) * BLOCK_SIZE,
               ((reserve_data[6 * i + 5] << 16) | (reserve_data[6 * i + 4] << 8) | reserve_data[6 * i + 3]) * LSM6DSRX_DATA_SIZE);
        HAL_Delay(1);
      }
      printf("reserve data has been read out.\r\n");
      back_to_idle();
    }
    else if (gyro2flash_state == GYRO2FLASH_CMD_READ_ALL)
    {
      for (int index = 0; index < total_block_count; index++) {
        uint32_t start_addr = ((reserve_data[6 * index + 2] << 8) | reserve_data[6 * index + 1]) * BLOCK_SIZE;
        uint32_t end_addr = start_addr + ((reserve_data[6 * index + 5] << 16) | (reserve_data[6 * index + 4] << 8) | reserve_data[6 * index + 3]) * LSM6DSRX_DATA_SIZE;
        printf("reading slice index: %d\n, start address from: 0x%x, size: %d",
                index,
                ((reserve_data[6 * index + 2] << 8) | reserve_data[6 * index + 1]) * BLOCK_SIZE,
                ((reserve_data[6 * index + 5] << 16) | (reserve_data[6 * index + 4] << 8) | reserve_data[6 * index + 3]) * LSM6DSRX_DATA_SIZE);
        for (int addr = start_addr; addr < end_addr; addr += LSM6DSRX_DATA_SIZE)
        {
          int16_t data[LSM6DSRX_DATA_SIZE / 2] = {0};
          at25sl128_read_data(addr, (uint8_t *)data, LSM6DSRX_DATA_SIZE);
          printf("%d, %d, %d, %d, %d, %d\n",
                  data[0], data[1], data[2], data[3], data[4], data[5]);
          HAL_Delay(1);
        }
      }
      printf("all the data has been read out.\r\n");
      back_to_idle();
    }
    else if (gyro2flash_state == GYRO2FLASH_CMD_READ_SILCE)
    {
      if (total_block_count == 0) {
        printf("memory is empty\n");
        back_to_idle();
      }
      else
      {
        printf("please input the index of the slice:\n");
        HAL_Delay(10);
        uart_rx_flag = 0;
        UART_RX_IT();
        gyro2flash_state = GYRO2FLASH_CMD_WAIT_FOR_INDEX;
      }
    }
    else if (gyro2flash_state == GYRO2FLASH_CMD_WAIT_FOR_INDEX)
    {
      if (uart_rx_flag == 1)
      {
        uint8_t index = (uint8_t)atoi(uart_rx_buffer) - 1;
        if (index > total_block_count)
        {
          printf("index out of range, total slice count: %d, please reselect: \n", total_block_count);
          uart_rx_flag = 0;
          UART_RX_IT();
        }
        else
        {
          uint32_t start_addr = ((reserve_data[6 * index + 2] << 8) | reserve_data[6 * index + 1]) * BLOCK_SIZE;
          uint32_t end_addr = start_addr + ((reserve_data[6 * index + 5] << 16) | (reserve_data[6 * index + 4] << 8) | reserve_data[6 * index + 3]) * LSM6DSRX_DATA_SIZE;
          printf("reading slice index: %d, start address from: 0x%x, size: %d\n",
                  index + 1, start_addr, end_addr - start_addr);
          for (int addr = start_addr; addr < end_addr; addr += LSM6DSRX_DATA_SIZE)
          {
            int16_t data[LSM6DSRX_DATA_SIZE / 2] = {0};
            at25sl128_read_data(addr, (uint8_t *)data, LSM6DSRX_DATA_SIZE);
            printf("%d, %d, %d, %d, %d, %d\n",
                    data[0], data[1], data[2], data[3], data[4], data[5]);
            HAL_Delay(1);
          }
          printf("slice has been read out.\r\n");
          back_to_idle();
        }
      }
    }
    else if (gyro2flash_state == GYRO2FLASH_WORKING)
    {
      if (HAL_GPIO_ReadPin(WORK_MODE_BUTTON_GPIO_GROUP, WORK_MODE_BUTTON_GPIO_PIN) == GPIO_PIN_RESET)
      {
        do {
          HAL_Delay(20);
        } while (HAL_GPIO_ReadPin(WORK_MODE_BUTTON_GPIO_GROUP, WORK_MODE_BUTTON_GPIO_PIN) == GPIO_PIN_RESET);
        back_to_idle();
        uint8_t tmp[6] = {0};
        tmp[0] = total_block_count + 1;
        tmp[1] = write_start_address & 0xff;
        tmp[2] = (write_start_address >> 8) & 0xff;
        tmp[3] = write_block_count & 0xff;
        tmp[4] = (write_block_count >> 8) & 0xff;
        at25sl128_write_data(6 * total_block_count, tmp, 6);
        // printf("last write_address: 0x%x\n", write_address);
        total_block_count++;
        write_block_count = 0;
      }
      if (write_address >= MAX_FLASH_SIZE - LSM6DSRX_DATA_SIZE)
      {
        printf("flash is full, please read data and clear flash\n");
        gyro2flash_state = GYRO2FLASH_OVERFLOW;
        uart_rx_flag = 0;
        UART_RX_IT();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      }
      int16_t gyro_data[LSM6DSRX_DATA_SIZE / 2] = {0};
      lsm6dsrx_read_data_polling(&lsm6dsrx_ctx, &gyro_data[0], &gyro_data[3]);
      at25sl128_write_data(write_address, (uint8_t *)gyro_data, LSM6DSRX_DATA_SIZE);
      write_address += LSM6DSRX_DATA_SIZE;
      write_block_count++;
      HAL_Delay(1);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void back_to_idle(void)
{
  uart_rx_flag = 0;
  UART_RX_IT();
  gyro2flash_state = GYRO2FLASH_IDLE;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

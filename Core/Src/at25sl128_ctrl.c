/**
 * @file at25sl128_ctrl.c
 * @author etgel lu (lucifer_lu@gcoreinc.com)
 * @brief
 * @version 0.1
 * @date 2023-05-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <string.h>
#include <stdio.h>
#include "at25sl128_ctrl.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI[3] = {0};

/**
 * @brief at25sl128 init
 *
 * @param none
 */
void at25sl128_init(void)
{
  HAL_Delay(10);
  at25sl128_read_id(whoamI);
  printf("Read AT25SL128 ID: 0x%x, 0x%x, 0x%x\n", whoamI[0], whoamI[1], whoamI[2]);
  if (whoamI[0] != 0x1F)
  {
    printf("at25sl128 ID error\n");
    while (1)
      ;
  }

  // check status registers
  // at25sl128_write_enable(AT25SL128_WRITE_ENABLE);
  // uint8_t read_status[3] = {0};
  // uint8_t cmd = AT25SL128_OPCODE_READ_STATUS;
  // at25sl128_read_command(&cmd, 1, read_status, 3);
  // printf("at25sl128 read status: 0x%x, 0x%x, 0x%x\n", read_status[0], read_status[1], read_status[2]);

  // //write & read test
  // uint8_t write_buff[] = "Test AT25SL128";
  // uint8_t read_buff[256] = {0};
  // at25sl128_erase_4k(0x000000);
  // at25sl128_write_data(0x000000, write_buff, sizeof(write_buff) - 1);
  // HAL_Delay(10);
  // at25sl128_read_data(0x000000, read_buff, sizeof(read_buff));
  // memset(write_buff, 0, sizeof(write_buff));
  // memcpy(write_buff, read_buff, sizeof(write_buff));
  // write_buff[sizeof(write_buff) - 1] = '\0';
  // printf("at25sl128 read data: %s\n", write_buff);
}

/**
 * @brief at25sl128_read_id
 *
 * @param buff who am i
 */
void at25sl128_read_id(uint8_t *buff)
{
  uint8_t cmd = AT25SL128_OPCODE_MANUFACTURER_ID;
  at25sl128_read_command(&cmd, 1, buff, 3);
}

/**
 * @brief at25sl128_write_enable
 *
 * @param enable
 */
void at25sl128_write_enable(uint8_t enable)
{
  uint8_t cmd = enable == AT25SL128_WRITE_DISABLE ? AT25SL128_OPCODE_WRITE_DISABLE : AT25SL128_OPCODE_WRITE_ENABLE;
  at25sl128_send_command(&cmd, 1, NULL, 0);
}

/**
 * @brief at25sl128_wait_busy
 * 
 */
void at25sl128_wait_busy(void)
{
  uint8_t status = 0;
  uint8_t cmd = AT25SL128_OPCODE_WAIT_BUSY;
  do
  {
    at25sl128_read_command(&cmd, 1, &status, 1);
  } while (status & 0x01);
}

/**
 * @brief at25sl128_erase_chip
 * 
 */
void at25sl128_erase_4k(uint32_t addr)
{
  at25sl128_write_enable(AT25SL128_WRITE_ENABLE);
  uint8_t cmd[4] = {0};
  cmd[0] = AT25SL128_OPCODE_ERASE_4K;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;
  at25sl128_send_command(cmd, 4, NULL, 0);
  at25sl128_wait_busy();
}

void at25sl128_erase_all(void)
{
  at25sl128_write_enable(AT25SL128_WRITE_ENABLE);
  uint8_t cmd[4] = {0};
  cmd[0] = AT25SL128_OPCODE_ERASE_ALL;
  at25sl128_send_command(cmd, 1, NULL, 0);
  at25sl128_wait_busy();
}

/**
 * @brief at25sl128_write_data
 * 
 * @param addr
 * @param data
 * @param data_size
 */
void at25sl128_write_data(uint32_t addr, uint8_t *data, uint16_t data_size)
{
  at25sl128_write_enable(AT25SL128_WRITE_ENABLE);
  uint8_t cmd[4] = {0};
  cmd[0] = AT25SL128_OPCODE_WRITE;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;
  at25sl128_send_command(cmd, 4, data, data_size);
  at25sl128_wait_busy();
}

/**
 * @brief at25sl128_read_data
 * 
 * @param addr
 * @param data
 * @param data_size
 */
void at25sl128_read_data(uint32_t addr, uint8_t *data, uint16_t data_size)
{
  uint8_t cmd[4] = {0};
  cmd[0] = AT25SL128_OPCODE_READ;  // 03
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;
  at25sl128_read_command(cmd, 4, data, data_size);
  // uint8_t cmd[5] = {0};
  // cmd[0] = AT25SL128_OPCODE_READ; // 0b
  // cmd[1] = (addr >> 16) & 0xFF;
  // cmd[2] = (addr >> 8) & 0xFF;
  // cmd[3] = addr & 0xFF;
  // cmd[4] = 0x00;
  // at25sl128_read_command(cmd, 5, data, data_size);
}

/**
 * @brief at25sl128_send_command
 *
 * @param command
 * @param cmd_size
 * @param data
 * @param data_size
 */
void at25sl128_send_command(uint8_t *command, uint16_t cmd_size, uint8_t *data, uint16_t data_size)
{
  HAL_GPIO_WritePin(AT25SL128_CTRL_CS_GPIO_PORT, AT25SL128_CTRL_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&AT25SL128_CTRL_SPI_HANDLE, command, cmd_size, HAL_MAX_DELAY);
  if (data != NULL && data_size > 0)
  {
    HAL_SPI_Transmit(&AT25SL128_CTRL_SPI_HANDLE, data, data_size, HAL_MAX_DELAY);
  }
  HAL_GPIO_WritePin(AT25SL128_CTRL_CS_GPIO_PORT, AT25SL128_CTRL_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief at25sl128_read_command
 *
 * @param command
 * @param cmd_size
 * @param data
 * @param data_size
 */
void at25sl128_read_command(uint8_t *command, uint16_t cmd_size, uint8_t *data, uint16_t data_size)
{
  HAL_GPIO_WritePin(AT25SL128_CTRL_CS_GPIO_PORT, AT25SL128_CTRL_CS_PIN, GPIO_PIN_RESET);
  // for(int i = 0; i < 10000; i++)
  HAL_SPI_Transmit(&AT25SL128_CTRL_SPI_HANDLE, command, cmd_size, HAL_MAX_DELAY);
  if (data != NULL && data_size > 0)
  {
    HAL_SPI_Receive(&AT25SL128_CTRL_SPI_HANDLE, data, data_size, HAL_MAX_DELAY);
  }
  HAL_GPIO_WritePin(AT25SL128_CTRL_CS_GPIO_PORT, AT25SL128_CTRL_CS_PIN, GPIO_PIN_SET);
}
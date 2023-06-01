/**
 * @file at25sl128_ctrl.h
 * @author etgel lu (lucifer_lu@gcoreinc.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT25SL128_CTRL_H__
#define __AT25SL128_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern SPI_HandleTypeDef hspi1;
#define AT25SL128_CTRL_SPI_HANDLE          hspi1
#define AT25SL128_CTRL_CS_GPIO_PORT        GPIOA
#define AT25SL128_CTRL_CS_PIN              GPIO_PIN_15

/** Device Identification (Who am I) **/
#define AT25SL128_ID                       0x1F
#define AT25SL128_OPCODE_MANUFACTURER_ID   0x9F
#define AT25SL128_OPCODE_WRITE_ENABLE      0x06
#define AT25SL128_OPCODE_WRITE_DISABLE     0x04
#define AT25SL128_OPCODE_ERASE_4K          0x20
#define AT25SL128_OPCODE_ERASE_ALL         0xC7
#define AT25SL128_OPCODE_WRITE             0x02
#define AT25SL128_OPCODE_WAIT_BUSY         0x05
#define AT25SL128_OPCODE_READ              0x03
#define AT25SL128_OPCODE_READ_STATUS       0x05

enum AT25SL128_WRITE_FLAG
{
  AT25SL128_WRITE_DISABLE = 0,
  AT25SL128_WRITE_ENABLE = 1,
};

void at25sl128_init(void);
void at25sl128_read_id(uint8_t *buff);
void at25sl128_write_enable(uint8_t enable);
void at25sl128_wait_busy(void);
void at25sl128_erase_4k(uint32_t addr);
void at25sl128_erase_all(void);
void at25sl128_write_data(uint32_t addr, uint8_t *data, uint16_t size);
void at25sl128_read_data(uint32_t addr, uint8_t *data, uint16_t size);
void at25sl128_send_command(uint8_t *command, uint16_t cmd_size, uint8_t *data, uint16_t data_size);
void at25sl128_read_command(uint8_t *command, uint16_t cmd_size, uint8_t *data, uint16_t data_size);

#ifdef __cplusplus
}
#endif

#endif // __AT25SL128_CTRL_H__

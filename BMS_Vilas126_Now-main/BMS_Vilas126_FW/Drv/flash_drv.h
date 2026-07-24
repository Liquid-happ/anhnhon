#ifndef DRV_FLASH_DRV_H_
#define DRV_FLASH_DRV_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "types.h"

uint8_t Flash_Read_U8(uint32_t address);
uint16_t Flash_Read_U16(uint32_t address);
uint32_t Flash_Read_U32(uint32_t address);
void Flash_Read_Buffer(uint32_t address, uint8_t *buffer, uint32_t length);

HAL_StatusTypeDef Flash_Erase_App(uint32_t start_addr, uint32_t size);
HAL_StatusTypeDef Flash_WriteBuffer(uint32_t address, uint8_t *data, uint32_t length);
HAL_StatusTypeDef Flash_WritePage(uint32_t address, uint8_t *data);

#endif /* DRV_FLASH_DRV_H_ */

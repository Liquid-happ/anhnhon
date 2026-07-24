#ifndef _RD_FLASH_H_
#define _RD_FLASH_H_


#include "main.h"
#include <string.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "rd_control.h"
#include "rd_sha256.h"



extern IWDG_HandleTypeDef hiwdg;
#define OTA_INFO_MAGIC     0xA5A55A5AUL
#define APP_STATE_EMPTY    0xFFFFFFFFUL
#define APP_STATE_VALID    0x12345678UL
#define APP_STATE_PENDING  0x87654321UL
#define APP_STATE_INVALID  0x00000000UL

#define ACTIVE_APP_NONE    0UL
#define ACTIVE_APP_A       1UL
#define ACTIVE_APP_B       2UL

#define FLASH_BASE_ADDR        0x08000000UL

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE        0x800U
#endif

#define BOOT_ADDR              0x08000000UL
#define BOOT_SIZE              (64UL * 1024UL)

#define APP_A_ADDR             0x08010000UL
#define APP_B_ADDR             0x08047800UL
#define APP_SLOT_SIZE          (222UL * 1024UL)

#define OTA_INFO_ADDR          0x0800F000UL
#define OTA_INFO_SIZE          (4UL * 1024UL)

#define FLASH_END_ADDR         0x0807F000UL



typedef struct
{
    uint32_t magic;

    uint32_t app_a_version;
    uint32_t app_a_size;
    uint8_t   app_a_crc[32]; 

    uint32_t app_b_version;
    uint32_t app_b_size;
    uint8_t   app_b_crc[32]; 

    uint32_t meta_crc;

} OTA_Info_t;

#define RAM_START_ADDR         0x20000000UL
#define RAM_END_ADDR           0x20010000UL   // STM32F103VET6 RAM 64KB

extern OTA_Info_t ota_info;

void rd_flash_init(void);
HAL_StatusTypeDef Flash_Erase_App(uint32_t start_addr, uint32_t size);
HAL_StatusTypeDef Flash_WriteBuffer(uint32_t address, uint8_t *data, uint32_t length);
void rd_copy_app_b_to_a(void);
void rd_run_whiletrue_boot(void);


uint32_t rd_crc_flash_infor(const OTA_Info_t *info);
uint32_t rd_crc32_flash(uint32_t start_addr, uint32_t size);

HAL_StatusTypeDef Flash_WritePage(uint32_t address, uint8_t *data);

void CalculateSHA256(uint32_t flash_addr, uint32_t size, uint8_t hash[32]);



#endif


#include "boot.h"
#include "flash_drv.h"
#include "sha256.h"
#include "feat_cfg.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

#if RD_DEBUG
extern UART_HandleTypeDef huart4;
#define uart_debug_id huart4
static uint8_t temp_debug[256];
#endif

extern IWDG_HandleTypeDef hiwdg;

OTA_Info_t ota_info;
uint8_t app_a_valid = 0;
uint8_t sha256_a[32];
uint8_t sha256_b[32];
uint32_t time_get_ota_init = 0;

// Ota_data is defined externally, e.g. in App/ota.c
extern volatile Ota_data_t Ota_data;

uint32_t boot_crc_flash_infor(const OTA_Info_t *info)
{
    const uint8_t *data = (const uint8_t *)info;
    uint32_t crc = 0;

    // Calculate CRC from start of struct until before meta_crc
    for (uint32_t i = 0; i < offsetof(OTA_Info_t, meta_crc); i++)
    {
        crc += data[i];
    }

    return crc;
}

uint8_t boot_is_valid_app(uint32_t app_addr)
{
    uint32_t stack_addr;
    uint32_t reset_addr;

    stack_addr = *(volatile uint32_t *)app_addr;
    reset_addr = *(volatile uint32_t *)(app_addr + 4);

    // Empty Flash is typically 0xFFFFFFFF
    if ((stack_addr == 0xFFFFFFFFUL) || (reset_addr == 0xFFFFFFFFUL))
    {
        return 0;
    }

    // Stack Pointer must be in RAM STM32F103VET6 (64KB RAM)
    if ((stack_addr < RAM_START_ADDR) || (stack_addr > RAM_END_ADDR))
    {
        return 0;
    }

    // Reset_Handler is Thumb address, bit 0 can be 1. Mask it before check.
    reset_addr &= 0xFFFFFFFEUL;

    // Reset_Handler must be in the corresponding app slot
    if ((reset_addr < app_addr) || (reset_addr >= (app_addr + APP_SLOT_SIZE)))
    {
        return 0;
    }

    return 1;
}

typedef void (*pFunction)(void);

void boot_jump_to_app(uint32_t app_addr)
{
    uint32_t app_stack = *(volatile uint32_t *)app_addr;
    uint32_t app_reset = *(volatile uint32_t *)(app_addr + 4);

    if (boot_is_valid_app(app_addr) == 0)
    {
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Invalid app\r\n", 13, 100);
#endif
        return;
    }

#if RD_DEBUG
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Jumping to app\r\n", 16, 100);
#endif
    HAL_Delay(100);

    HAL_RCC_DeInit();
    HAL_DeInit();
#if RD_DEBUG
    HAL_UART_DeInit(&uart_debug_id);
#endif
    HAL_Delay(100);

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    SCB->VTOR = app_addr;
    __set_MSP(app_stack);

    __enable_irq();

    pFunction app_entry = (pFunction)app_reset;
    app_entry();
}

void boot_init(void)
{
    uint32_t crc_calc = 0;
    Flash_Read_Buffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));

    crc_calc = boot_crc_flash_infor(&ota_info);
    app_a_valid = boot_is_valid_app(APP_A_ADDR);
    if (app_a_valid)
    {
        boot_calculate_sha256(APP_A_ADDR, ota_info.app_a_size, sha256_a);
    }

#if (RD_DEBUG && IS_BOOTLOADER)
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Checking OTA info...\r\n", 22, 100);
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"App A valid: ", 13, 100);
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)(app_a_valid ? "Yes\r\n" : "No\r\n"), 5, 100);
#endif

    if ((ota_info.magic != OTA_INFO_MAGIC) || (ota_info.meta_crc != crc_calc))
    {
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Invalid OTA info, scan app\r\n", 28, 100);
#endif
        ota_info.magic = OTA_INFO_MAGIC;
        ota_info.app_a_version = 0;
        ota_info.app_a_size    = 0;
        memset(ota_info.app_a_crc, 0, 32);

        ota_info.app_b_version = 0;
        ota_info.app_b_size    = 0;
        memset(ota_info.app_b_crc, 0, 32);

        ota_info.meta_crc = boot_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));
    }
    else
    {
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"OTA info loaded successfully\r\n", 30, 100);
#endif
    }

#if RD_DEBUG
    sprintf((char*)temp_debug, "App A version: %u, size: %u\r\n", (unsigned int)ota_info.app_a_version, (unsigned int)ota_info.app_a_size);
    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
    sprintf((char*)temp_debug, "App B version: %u, size: %u\r\n", (unsigned int)ota_info.app_b_version, (unsigned int)ota_info.app_b_size);
    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#if (IS_BOOTLOADER == 0)
    sprintf((char*)temp_debug, "App A Running\r\n");
    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
#endif
}

void boot_process(void)
{
    uint32_t temp_time_now = HAL_GetTick();
    uint8_t need_update = 0;

    if (ota_info.app_b_size > 0 && ota_info.app_b_size <= APP_SLOT_SIZE)
    {
        boot_calculate_sha256(APP_B_ADDR, ota_info.app_b_size, sha256_b);

        // Confirm firmware in Slot B is intact
        if (memcmp(ota_info.app_b_crc, sha256_b, 32) == 0)
        {
            // Valid firmware B exists. Check if it's a new version or if A is invalid/damaged.
            if ((ota_info.app_b_version != ota_info.app_a_version) ||
                (memcmp(ota_info.app_a_crc, sha256_a, 32) != 0) ||
                (!app_a_valid))
            {
                need_update = 1;
            }
        }
    }

    if (need_update)
    {
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"start_copy...\r\n", 15, 100);
#endif

        boot_copy_app_b_to_a();

        // If copy succeeds, MCU should have reset. If it gets here, copy failed.
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Copy failed or rejected!\r\n", 26, 100);
#endif

        // Prevent bootloop by invalidating junk OTA info
        ota_info.app_b_size = 0;
        memset(ota_info.app_b_crc, 0, 32);
        ota_info.meta_crc = boot_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));
    }

    if (!app_a_valid)
    {
        // No valid App A, wait here for OTA command
    }
    else
    {
        // Valid App A, wait 2s to check for UART OTA command
        if (((temp_time_now - time_get_ota_init) > 2000) && Ota_data.start_ota == 0)
        {
            time_get_ota_init = temp_time_now;
#if RD_DEBUG
            HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Running...\r\n", 12, 100);
#endif
            HAL_Delay(100);
            HAL_IWDG_Refresh(&hiwdg);
            boot_jump_to_app(APP_A_ADDR);
        }
    }
}

#define CRC32_INIT_VALUE   0xFFFFFFFFUL
#define CRC32_POLY         0xEDB88320UL

static uint32_t boot_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ CRC32_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint32_t boot_crc32_flash(uint32_t start_addr, uint32_t size)
{
    uint32_t crc = CRC32_INIT_VALUE;

    if (size == 0 || start_addr < FLASH_BASE_ADDR || (start_addr + size) > FLASH_END_ADDR)
    {
        return 0;
    }

    for (uint32_t i = 0; i < size; i++)
    {
        uint8_t data = *(volatile uint8_t *)(start_addr + i);
        crc = boot_crc32_update(crc, &data, 1);
        if ((i % 1024) == 0)
        {
            HAL_IWDG_Refresh(&hiwdg);
        }
    }

    return ~crc;
}

void boot_copy_app_b_to_a(void)
{
    boot_calculate_sha256(APP_B_ADDR, ota_info.app_b_size, sha256_b);
    if (memcmp(ota_info.app_b_crc, sha256_b, 32) == 0)
    {
        uint8_t buffer[256];
        uint32_t bytes_copied = 0;
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Start_eraseA", 12, 100);
#endif
        for (uint32_t i = 0; i < APP_SLOT_SIZE; i += FLASH_PAGE_SIZE)
        {
            HAL_IWDG_Refresh(&hiwdg);
            Flash_Erase_App(APP_A_ADDR + i, FLASH_PAGE_SIZE);
            HAL_IWDG_Refresh(&hiwdg);
        }
#if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"eraseA_done", 11, 100);
#endif

        while (bytes_copied < ota_info.app_b_size)
        {
            uint32_t chunk_size = (ota_info.app_b_size - bytes_copied) > sizeof(buffer) ? sizeof(buffer) : (ota_info.app_b_size - bytes_copied);

            Flash_Read_Buffer(APP_B_ADDR + bytes_copied, buffer, chunk_size);
            HAL_IWDG_Refresh(&hiwdg);
            Flash_WriteBuffer(APP_A_ADDR + bytes_copied, buffer, chunk_size);
            HAL_IWDG_Refresh(&hiwdg);
            bytes_copied += chunk_size;
        }

        ota_info.app_a_version = ota_info.app_b_version;
        ota_info.app_a_size    = ota_info.app_b_size;
        memcpy(ota_info.app_a_crc, ota_info.app_b_crc, 32);

        ota_info.meta_crc = boot_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(100);
        NVIC_SystemReset();
    }
}

void boot_calculate_sha256(uint32_t flash_addr, uint32_t size, uint8_t hash[32])
{
    SHA256_CTX ctx;

    sha256_init(&ctx);

    while (size)
    {
        uint32_t chunk = (size > 1024) ? 1024 : size;

        sha256_update(&ctx, (const BYTE *)flash_addr, chunk);
        flash_addr += chunk;
        size -= chunk;
        HAL_IWDG_Refresh(&hiwdg);
    }

    sha256_final(&ctx, hash);
}

#include "flash_drv.h"

extern IWDG_HandleTypeDef hiwdg;

uint8_t Flash_Read_U8(uint32_t address)
{
    return *(volatile uint8_t *)address;
}

uint16_t Flash_Read_U16(uint32_t address)
{
    return *(volatile uint16_t *)address;
}

uint32_t Flash_Read_U32(uint32_t address)
{
    return *(volatile uint32_t *)address;
}

void Flash_Read_Buffer(uint32_t address, uint8_t *buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        buffer[i] = *(volatile uint8_t *)(address + i);
    }
}

HAL_StatusTypeDef Flash_Erase_App(uint32_t start_addr, uint32_t size)
{
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError;

    uint32_t pages = size / FLASH_PAGE_SIZE;
    HAL_IWDG_Refresh(&hiwdg);
    if (size % FLASH_PAGE_SIZE)
        pages++;
    HAL_IWDG_Refresh(&hiwdg);
    if (start_addr % FLASH_PAGE_SIZE != 0)
        return HAL_ERROR;

    HAL_FLASH_Unlock();
    HAL_IWDG_Refresh(&hiwdg);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                           FLASH_FLAG_PGERR |
                           FLASH_FLAG_WRPERR);

    EraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInit.PageAddress = start_addr;
    EraseInit.NbPages     = pages;

    if (HAL_FLASHEx_Erase(&EraseInit, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    HAL_IWDG_Refresh(&hiwdg);
    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef Flash_WriteBuffer(uint32_t address, uint8_t *data, uint32_t length)
{
    if (address < FLASH_BASE_ADDR || address >= FLASH_END_ADDR)
        return HAL_ERROR;

    if ((address + length) > FLASH_END_ADDR)
        return HAL_ERROR;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < length; i += 2)
    {
        uint16_t halfword;

        if ((i + 1) < length)
        {
            halfword = data[i] | ((uint16_t)data[i + 1] << 8);
        }
        else
        {
            halfword = data[i] | 0xFF00;
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }

        // Verification check
        uint16_t read_back = *(volatile uint16_t *)(address + i);
        if (read_back != halfword)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef Flash_WritePage(uint32_t address, uint8_t *data)
{
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError;

    if (address % FLASH_PAGE_SIZE != 0)
        return HAL_ERROR;

    HAL_FLASH_Unlock();

    EraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInit.PageAddress = address;
    EraseInit.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInit, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 2)
    {
        uint16_t halfword = data[i] | (data[i + 1] << 8);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}

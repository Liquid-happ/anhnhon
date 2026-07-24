#include "xmodem.h"
#include "rb.h"
#include "boot.h"
#include "ota.h"
#include "flash_drv.h"
#include "feat_cfg.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

#if RD_DEBUG
extern UART_HandleTypeDef huart4;
#define uart_debug_id huart4
static uint8_t temp_debug[256];
#endif

extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;
extern ringbuffer_t vrts_ringbuffer_Data;
extern volatile Ota_data_t Ota_data;

Xmodem_state_t xmodem_state = RD_get_infor;

uint8_t RD_INFOR[5] = {0x49, 0x4e, 0x46, 0x4f, 0x0d};
uint8_t RD_IMAG1[5] = {0x49, 0x4d, 0x41, 0x47, 0x0d};

uint8_t X_STX = 0x02;
uint8_t X_ACK = 0x06;
uint8_t X_NAK = 0x15;
uint8_t X_EOF[1] = {0x04};

static void rd_xmodem_clear_ringbuffer(void)
{
    __disable_irq();
    vrts_ringbuffer_Data.tail  = vrts_ringbuffer_Data.head;
    vrts_ringbuffer_Data.count = 0;
    __enable_irq();
}

static uint8_t rd_crc32_equal_volatile(const uint8_t *a, volatile const uint8_t *b, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (a[i] != b[i])
        {
            return 0;
        }
    }
    return 1;
}

void xmodem_send_data(uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&huart1, data, length, 100);
}

static uint8_t Check_sum_received(uint8_t *data)
{
    uint8_t check_sum_rd = 0;
    for (int i = 0; i < BLOCK_SIZE; i++)
    {
        check_sum_rd = check_sum_rd + data[i + 3];
    }
    return check_sum_rd & 0xff;
}

static uint32_t last_rx_tick = 0;
static size_t last_buffer_count = 0;
uint8_t have_err_ota = 0;

void ota_run_received_ota(void)
{
    uint32_t temp_tick = 0;
    uint32_t timeout_ota_check = 0;
    temp_tick = HAL_GetTick();

    if (Ota_data.TimeoutCounter > temp_tick)
    {
        Ota_data.TimeoutCounter = temp_tick;
    }
    else
    {
        timeout_ota_check = temp_tick - Ota_data.TimeoutCounter;
    }

    if (timeout_ota_check > 5000)
    {
        Ota_data.start_ota = 0;
        Ota_data.TimeoutCounter = temp_tick;
        xmodem_state = RD_get_infor;
#if RD_DEBUG
        sprintf((char*)temp_debug, "OTA update failed -- Time out\r\n");
        HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
    }

    switch (xmodem_state)
    {
        case RD_get_infor:
            if (Ota_data.ok_start_ota)
            {
                Ota_data.ok_start_ota = 0;
                ota_send_data_esp(NULL, OP_START_OTA, 0);
#if RD_DEBUG
                sprintf((char*)temp_debug, "send start OTA\r\n");
                HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
            }
            if (vrts_ringbuffer_Data.count >= 5)
            {
                uint8_t infor[5];
                for (int i = 0; i < 5; i++)
                {
                    ring_pop_tail(&vrts_ringbuffer_Data, &infor[i]);
                }
                if (memcmp(infor, RD_INFOR, 5) == 0)
                {
                    xmodem_send_data((uint8_t *)RIIM_type, strlen(RIIM_type));
                    xmodem_state = RD_get_infor_wait;
                    have_err_ota = 0;
                    Ota_data.TimeoutCounter = temp_tick;
                }
                else
                {
                    have_err_ota = 1;
                }
            }
            break;

        case RD_get_infor_wait:
            if (vrts_ringbuffer_Data.count >= 5)
            {
                uint8_t imag[5];
                for (int i = 0; i < 5; i++)
                {
                    ring_pop_tail(&vrts_ringbuffer_Data, &imag[i]);
                }
                if (memcmp(imag, RD_IMAG1, 5) == 0)
                {
                    xmodem_send_data((uint8_t *)&X_NAK, 1);
                    xmodem_state = RD_imag;
                    Ota_data.TimeoutCounter = temp_tick;
                }
                else
                {
                    have_err_ota = 1;
                }
            }
            break;

        case RD_imag:
#if RD_DEBUG
            // sprintf((char*)temp_debug, "debug count %d \r\n", vrts_ringbuffer_Data.count);
            // HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
            if (vrts_ringbuffer_Data.count > 0 && vrts_ringbuffer_Data.count < (FLASH_PAGE_SIZE + 4))
            {
                uint8_t *tail_ptr = (uint8_t *)vrts_ringbuffer_Data.tail;
                uint8_t first_byte = *tail_ptr;
                if (vrts_ringbuffer_Data.count != last_buffer_count)
                {
                    last_buffer_count = vrts_ringbuffer_Data.count;
                    last_rx_tick = temp_tick;
                }
                else if (temp_tick - last_rx_tick > 1500)
                {
                    rd_xmodem_clear_ringbuffer();
                    if (first_byte == 0x01 || last_buffer_count > 1)
                    {
                        xmodem_send_data((uint8_t *)&X_NAK, 1);
                    }
                    last_buffer_count = 0;
                }
            }
            if (vrts_ringbuffer_Data.count >= FLASH_PAGE_SIZE + 4)
            {
                static uint8_t data[FLASH_PAGE_SIZE + 4];
                data_flash_in_t *data_flash_in = (data_flash_in_t *)data;
                HAL_IWDG_Refresh(&hiwdg);
                for (int i = 0; i < FLASH_PAGE_SIZE + 4; i++)
                {
                    ring_pop_tail(&vrts_ringbuffer_Data, &data[i]);
                }
                HAL_IWDG_Refresh(&hiwdg);
                if (data_flash_in->start == 0x01)
                {
                    uint8_t check_block_number = data_flash_in->number_of_block;
                    check_block_number = ~check_block_number;

                    if (check_block_number == data_flash_in->check_number_of_block)
                    {
                        if (Check_sum_received(data) == data_flash_in->check_sum_block)
                        {
                            uint8_t id_end_block = 0;
                            id_end_block = (Ota_data.size % BLOCK_SIZE == 0) ? (Ota_data.size / BLOCK_SIZE) : (Ota_data.size / BLOCK_SIZE) + 1;
                            if (data_flash_in->number_of_block == 0 || data_flash_in->number_of_block > id_end_block)
                            {
                                xmodem_send_data((uint8_t *)&X_NAK, 1);
                                rd_xmodem_clear_ringbuffer();
#if RD_DEBUG
                                HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"CRITICAL: Block number out of bounds!\r\n", 39, 100);
#endif
                                break;
                            }
                            Flash_WritePage(APP_B_ADDR + (data_flash_in->number_of_block - 1) * BLOCK_SIZE, data_flash_in->data);
                            Ota_data.TimeoutCounter = temp_tick;
                            xmodem_send_data((uint8_t *)&X_ACK, 1);
#if RD_DEBUG
                            sprintf((char*)temp_debug, "Block %d received\r\n", data_flash_in->number_of_block);
                            HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                            HAL_IWDG_Refresh(&hiwdg);
                            if (Ota_data.size % BLOCK_SIZE == 0)
                            {
                                id_end_block = (Ota_data.size / BLOCK_SIZE);
                            }
                            else
                            {
                                id_end_block = (Ota_data.size / BLOCK_SIZE) + 1;
                            }
                            if (data_flash_in->number_of_block == id_end_block)
                            {
#if RD_DEBUG
                                sprintf((char*)temp_debug, "full %d received\r\n", data_flash_in->number_of_block);
                                HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                                xmodem_state = RD_end;
                            }
                        }
                        else
                        {
                            xmodem_send_data((uint8_t *)&X_NAK, 1);
                            rd_xmodem_clear_ringbuffer();
#if RD_DEBUG
                            sprintf((char*)temp_debug, "Block %d checksum error\r\n", data_flash_in->number_of_block);
                            HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                        }
                    }
                    else
                    {
                        xmodem_send_data((uint8_t *)&X_NAK, 1);
                        rd_xmodem_clear_ringbuffer();
#if RD_DEBUG
                        sprintf((char*)temp_debug, "Block %d number error\r\n", data_flash_in->number_of_block);
                        HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                    }
                }
                else
                {
                    xmodem_send_data((uint8_t *)&X_NAK, 1);
                    rd_xmodem_clear_ringbuffer();
#if RD_DEBUG
                    sprintf((char*)temp_debug, "Block %d start byte error\r\n", data_flash_in->number_of_block);
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                }
            }
            break;

        case RD_imag_wait:
            break;
        case RD_xmodem_send:
            break;
        case RD_xmodem_send_wait:
            break;

        case RD_end:
            if (vrts_ringbuffer_Data.count >= 1)
            {
                uint8_t end_byte;
                ring_pop_tail(&vrts_ringbuffer_Data, &end_byte);
                if (end_byte == X_EOF[0])
                {
                    xmodem_send_data((uint8_t *)&X_ACK, 1);
#if RD_DEBUG
                    sprintf((char*)temp_debug, "End byte received\r\n");
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                    Ota_data.TimeoutCounter = temp_tick;
                    xmodem_state = RD_end_wait;
                }
                else
                {
                    xmodem_send_data((uint8_t *)&X_NAK, 1);
#if RD_DEBUG
                    sprintf((char*)temp_debug, "End byte error\r\n");
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                    HAL_Delay(100);
                    NVIC_SystemReset();
                }
            }
            break;

        case RD_end_wait:
            HAL_Delay(100);
            if (have_err_ota == 0)
            {
                ota_info.app_b_version = Ota_data.version;
                ota_info.app_b_size    = Ota_data.size;
                boot_calculate_sha256(APP_B_ADDR, Ota_data.size, ota_info.app_b_crc);
                if (rd_crc32_equal_volatile(ota_info.app_b_crc, Ota_data.crc, 32))
                {
#if RD_DEBUG
                    sprintf((char*)temp_debug, "CRC check passed\r\n");
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                    ota_info.meta_crc = boot_crc_flash_infor(&ota_info);
                    Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
                    Flash_WriteBuffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));

#if RD_DEBUG
                    sprintf((char*)temp_debug, "OTA update complete\r\n");
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif

                    xmodem_send_data((uint8_t *)RIIM_code_ok, strlen(RIIM_code_ok));
                    HAL_Delay(300);
                    NVIC_SystemReset();
                }
                else
                {
                    xmodem_send_data((uint8_t *)"ERROR_SHA256\r\n", 14);
#if RD_DEBUG
                    sprintf((char*)temp_debug, "CRC check failed\r\n");
                    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                    Ota_data.start_ota = 0;
                    rd_xmodem_clear_ringbuffer();
                    HAL_Delay(200);
                    NVIC_SystemReset();
                }
            }
            else
            {
                xmodem_send_data((uint8_t *)"ERROR_OTA\r\n", 11);
#if RD_DEBUG
                sprintf((char*)temp_debug, "OTA update failed\r\n");
                HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
#endif
                Ota_data.start_ota = 0;
                rd_xmodem_clear_ringbuffer();
                HAL_Delay(100);
                NVIC_SystemReset();
            }
            break;
    }
}

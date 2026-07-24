#include "can_drv.h"

#if IS_BOOTLOADER == 0

#include "main.h"

extern CAN_HandleTypeDef hcan;

uint8_t Can_Drv_Transmit(uint32_t std_id, uint32_t ext_id, uint8_t is_ext, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t tickstart = HAL_GetTick();

    // Wait for free mailbox
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        if ((HAL_GetTick() - tickstart) > 3)
        {
            return HAL_TIMEOUT;
        }
    }

    if (is_ext)
    {
        tx_header.IDE = CAN_ID_EXT;
        tx_header.ExtId = ext_id;
    }
    else
    {
        tx_header.IDE = CAN_ID_STD;
        tx_header.StdId = std_id;
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = (len > 8) ? 8 : len;

    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

#endif /* IS_BOOTLOADER == 0 */

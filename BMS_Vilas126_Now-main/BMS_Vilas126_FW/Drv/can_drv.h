#ifndef DRV_CAN_DRV_H_
#define DRV_CAN_DRV_H_

#include "Cfg/feat_cfg.h"

#if IS_BOOTLOADER == 0

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern CAN_HandleTypeDef hcan;

/**
 * @brief Sends a standard or extended CAN frame.
 * @param std_id Standard ID (set to 0 if using ext_id)
 * @param ext_id Extended ID (set to 0 if using std_id)
 * @param is_ext 1 if extended frame, 0 if standard frame
 * @param data Pointer to transmit data buffer
 * @param len Data length (0 to 8)
 * @return HAL status or custom error code (0 for success, non-zero for error)
 */
uint8_t Can_Drv_Transmit(uint32_t std_id, uint32_t ext_id, uint8_t is_ext, uint8_t *data, uint8_t len);

#endif /* IS_BOOTLOADER == 0 */
#endif /* DRV_CAN_DRV_H_ */

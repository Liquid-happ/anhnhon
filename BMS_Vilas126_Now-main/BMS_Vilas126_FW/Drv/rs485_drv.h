#ifndef DRV_RS485_DRV_H_
#define DRV_RS485_DRV_H_

#include "Cfg/feat_cfg.h"

#if IS_BOOTLOADER == 0

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define RS485_PORT_UART4   0
#define RS485_PORT_UART5   1

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/**
 * @brief Initializes RS485 transceiver GPIO controls and interrupts.
 */
void RS485_Drv_Init(void);

/**
 * @brief Sets the RS485 transceiver to Transmit mode.
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 */
void RS485_Drv_Set_Tx_Mode(uint8_t port);

/**
 * @brief Sets the RS485 transceiver to Receive mode.
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 */
void RS485_Drv_Set_Rx_Mode(uint8_t port);

/**
 * @brief Transmits a buffer over RS485.
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 * @param data Data buffer to transmit.
 * @param len Length of the data.
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef RS485_Drv_Transmit(uint8_t port, const uint8_t *data, uint16_t len);

/**
 * @brief Starts UART interrupt receive for a single byte.
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 * @param rx_byte_ptr Pointer to the byte variable where received byte will be stored.
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef RS485_Drv_Start_Rx_IT(uint8_t port, uint8_t *rx_byte_ptr);

/**
 * @brief Dynamically changes the baudrate of a port.
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 * @param baudrate Target baudrate (e.g. 9600, 115200).
 * @param rx_byte_ptr Pointer to the byte variable to re-start the interrupt reception.
 */
void RS485_Drv_Set_Baudrate(uint8_t port, uint32_t baudrate, uint8_t *rx_byte_ptr);

#endif /* IS_BOOTLOADER == 0 */
#endif /* DRV_RS485_DRV_H_ */

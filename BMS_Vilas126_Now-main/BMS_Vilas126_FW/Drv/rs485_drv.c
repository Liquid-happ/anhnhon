#include "Drv/rs485_drv.h"

#if IS_BOOTLOADER == 0

#include "main.h"

void RS485_Drv_Init(void) {
    RS485_Drv_Set_Rx_Mode(RS485_PORT_UART4);
    RS485_Drv_Set_Rx_Mode(RS485_PORT_UART5);
}

void RS485_Drv_Set_Tx_Mode(uint8_t port) {
    if (port == RS485_PORT_UART4) {
        HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_SET);
    } else if (port == RS485_PORT_UART5) {
        HAL_GPIO_WritePin(GPIOD, RS485_DE_RE1_Pin, GPIO_PIN_SET);
    }
}

void RS485_Drv_Set_Rx_Mode(uint8_t port) {
    if (port == RS485_PORT_UART4) {
        HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);
    } else if (port == RS485_PORT_UART5) {
        HAL_GPIO_WritePin(GPIOD, RS485_DE_RE1_Pin, GPIO_PIN_RESET);
    }
}

HAL_StatusTypeDef RS485_Drv_Transmit(uint8_t port, const uint8_t *data, uint16_t len) {
    UART_HandleTypeDef *huart = (port == RS485_PORT_UART4) ? &huart4 : &huart5;
    
    RS485_Drv_Set_Tx_Mode(port);
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)data, len, 1000);
    
    // Wait for Transmission Complete flag
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
        // Option to add timeout or break if link goes down
    }
    
    RS485_Drv_Set_Rx_Mode(port);
    return status;
}

HAL_StatusTypeDef RS485_Drv_Start_Rx_IT(uint8_t port, uint8_t *rx_byte_ptr) {
    UART_HandleTypeDef *huart = (port == RS485_PORT_UART4) ? &huart4 : &huart5;
    return HAL_UART_Receive_IT(huart, rx_byte_ptr, 1);
}

void RS485_Drv_Set_Baudrate(uint8_t port, uint32_t baudrate, uint8_t *rx_byte_ptr) {
    UART_HandleTypeDef *huart = (port == RS485_PORT_UART4) ? &huart4 : &huart5;
    if (huart->Init.BaudRate != baudrate) {
        HAL_UART_DeInit(huart);
        huart->Init.BaudRate = baudrate;
        if (HAL_UART_Init(huart) != HAL_OK) {
            // Error Handler hook if needed
        }
        if (rx_byte_ptr != NULL) {
            HAL_UART_Receive_IT(huart, rx_byte_ptr, 1);
        }
    }
}

#endif /* IS_BOOTLOADER == 0 */

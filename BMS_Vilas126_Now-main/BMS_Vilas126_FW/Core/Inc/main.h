/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void BQ769x2_Init(void);
HAL_StatusTypeDef CommandSubcommands(uint16_t command);
void Update_SOC_SOH_FromBQ(void);
void BQ769x2_ReadAllVoltages(void);
int16_t BQ769x2_ReadCurrent(void);
void BQ769x2_ClearLatchedAlerts(void);
void BQ769x2_PrepareFetOn(void);
void BMS_Clear_All_Buffers(void);
void Check_Sleep_Button(void);
void Check_Reset_Button(void);
uint8_t BMS_Flash_Init(void);
uint8_t BMS_Save_Data_To_Flash(void);
uint8_t BMS_Load_Data_From_Flash(void);
uint16_t BQ769x2_ReadAlarmStatusReg(void);
uint16_t BQ769x2_ReadBatteryStatus(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_STATE_Pin GPIO_PIN_13
#define BUTTON_STATE_GPIO_Port GPIOC
#define POWER_HOLD_Pin GPIO_PIN_14
#define POWER_HOLD_GPIO_Port GPIOC
#define RST_SHUT_Pin GPIO_PIN_15
#define RST_SHUT_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOC
#define POW_EN_Pin GPIO_PIN_1
#define POW_EN_GPIO_Port GPIOC
#define DDSG_Pin GPIO_PIN_3
#define DDSG_GPIO_Port GPIOC
#define DCHG_Pin GPIO_PIN_0
#define DCHG_GPIO_Port GPIOA
#define DFETOFF_Pin GPIO_PIN_1
#define DFETOFF_GPIO_Port GPIOA
#define CFETOFF_Pin GPIO_PIN_2
#define CFETOFF_GPIO_Port GPIOA
#define ALERT_Pin GPIO_PIN_3
#define ALERT_GPIO_Port GPIOA
#define SRN_Pin GPIO_PIN_4
#define SRN_GPIO_Port GPIOA
#define SRP_Pin GPIO_PIN_5
#define SRP_GPIO_Port GPIOA
#define APTOMAT_SWITCH_Pin GPIO_PIN_6
#define APTOMAT_SWITCH_GPIO_Port GPIOA
#define NTC_FET_Pin GPIO_PIN_7
#define NTC_FET_GPIO_Port GPIOA
#define NTC_1_Pin GPIO_PIN_4
#define NTC_1_GPIO_Port GPIOC
#define NTC_2_Pin GPIO_PIN_5
#define NTC_2_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define DRY_CONTACT_LOWBAT_Pin GPIO_PIN_1
#define DRY_CONTACT_LOWBAT_GPIO_Port GPIOB
#define DRY_CONTACT_FAULT_Pin GPIO_PIN_2
#define DRY_CONTACT_FAULT_GPIO_Port GPIOB
#define RST_SWITCH_Pin GPIO_PIN_7
#define RST_SWITCH_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_8
#define K1_GPIO_Port GPIOE
#define K2_Pin GPIO_PIN_9
#define K2_GPIO_Port GPIOE
#define K3_Pin GPIO_PIN_10
#define K3_GPIO_Port GPIOE
#define K4_Pin GPIO_PIN_11
#define K4_GPIO_Port GPIOE
#define K5_Pin GPIO_PIN_12
#define K5_GPIO_Port GPIOE
#define K6_Pin GPIO_PIN_13
#define K6_GPIO_Port GPIOE
#define RTC_INT_A_Pin GPIO_PIN_15
#define RTC_INT_A_GPIO_Port GPIOE
#define RTC_INT_A_EXTI_IRQn EXTI15_10_IRQn
#define RTC_SCL_Pin GPIO_PIN_10
#define RTC_SCL_GPIO_Port GPIOB
#define RTC_SDA_Pin GPIO_PIN_11
#define RTC_SDA_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LCD_TX_Pin GPIO_PIN_8
#define LCD_TX_GPIO_Port GPIOD
#define LCD_RX_Pin GPIO_PIN_9
#define LCD_RX_GPIO_Port GPIOD
#define TEST_LED_Pin GPIO_PIN_11
#define TEST_LED_GPIO_Port GPIOD
#define LED9_Pin GPIO_PIN_12
#define LED9_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_13
#define LED6_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_14
#define LED7_GPIO_Port GPIOD
#define LED8_Pin GPIO_PIN_15
#define LED8_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_6
#define LED5_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define RXD_ESP_Pin GPIO_PIN_9
#define RXD_ESP_GPIO_Port GPIOA
#define TXD_ESP_Pin GPIO_PIN_10
#define TXD_ESP_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RS485_DE_RE_Pin GPIO_PIN_15
#define RS485_DE_RE_GPIO_Port GPIOA
#define RS485_DI_Pin GPIO_PIN_10
#define RS485_DI_GPIO_Port GPIOC
#define RS485_RO_Pin GPIO_PIN_11
#define RS485_RO_GPIO_Port GPIOC
#define RS485_DI1_Pin GPIO_PIN_12
#define RS485_DI1_GPIO_Port GPIOC
#define RS485_DE_RE1_Pin GPIO_PIN_0
#define RS485_DE_RE1_GPIO_Port GPIOD
#define RS485_RO1_Pin GPIO_PIN_2
#define RS485_RO1_GPIO_Port GPIOD
#define RS232_TX_Pin GPIO_PIN_5
#define RS232_TX_GPIO_Port GPIOD
#define RS232_RX_Pin GPIO_PIN_6
#define RS232_RX_GPIO_Port GPIOD
#define UP_IN_Pin GPIO_PIN_3
#define UP_IN_GPIO_Port GPIOB
#define DN_OP_Pin GPIO_PIN_4
#define DN_OP_GPIO_Port GPIOB
#define EN_PRECHARGE_Pin GPIO_PIN_5
#define EN_PRECHARGE_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

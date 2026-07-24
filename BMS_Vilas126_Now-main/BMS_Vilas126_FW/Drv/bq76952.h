#ifndef DRV_BQ76952_H_
#define DRV_BQ76952_H_

#include "feat_cfg.h"
#include "stm32f1xx_hal.h"

#if IS_BOOTLOADER == 0

#define DEV_ADDR            0x10  // 7-bit address is 0x08, 8-bit with write bit is 0x10
#define CRC_Mode            1     // 0: disabled, 1: enabled
#define MAX_BUFFER_SIZE     64

#define R   0 // Read
#define W   1 // Write
#define W2  2 // Write 2 bytes

// External global variables referenced by BQ769x2 driver functions (mapped to g_bms/state as needed)
extern uint16_t CellVoltage[16];
extern float CellVoltage_Float[16];
extern float CellVoltage_V[16];
extern int16_t PackCurrent;
extern int16_t TS1Temp, TS2Temp, TS3Temp, IntTemp;
extern uint16_t AlarmStatusReg;
extern uint16_t BatteryStatusReg;
extern uint16_t FET_Status;
extern uint8_t CHG, PCHG, DSG, PDSG, DCHG_pin, DDSG_pin, ALRT_pin;
extern uint16_t SafetyStatusA, SafetyStatusB, SafetyStatusC;
extern uint32_t PFStatusA, PFStatusB, PFStatusC, PFStatusD;
extern uint8_t RX_data[2];
extern uint8_t RX_32Byte[32];
extern uint8_t RX_2Byte[2];

// Core driver function declarations
HAL_StatusTypeDef Bq_Init(void);
void Bq_Unlock(void);
HAL_StatusTypeDef Bq_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);
HAL_StatusTypeDef Bq_Subcmd(uint16_t command);
HAL_StatusTypeDef Bq_Subcommands(uint16_t command, uint16_t data, uint8_t type);
HAL_StatusTypeDef Bq_DirectCommands(uint8_t command, uint16_t data, uint8_t type, HAL_StatusTypeDef *ptr_status);
HAL_StatusTypeDef Bq_ReadVoltages(void);
HAL_StatusTypeDef Bq_ReadCurrent(void);
HAL_StatusTypeDef Bq_ReadTemps(void);
HAL_StatusTypeDef Bq_ReadStatus(void);
void Bq_SetFet(uint8_t chg, uint8_t dsg);
void Bq_ReadPassQ(void);

// Low-level helper declarations
void delayUS(uint32_t us);
void Safe_Delay_ms(uint32_t ms);
void Emergency_Delay(uint32_t ms);
unsigned char Checksum(unsigned char *ptr, unsigned char len);
unsigned char CRC8(unsigned char *ptr, unsigned char len);
void I2C_Bus_Recovery(void);
void I2C2_Bus_Recovery(void);
HAL_StatusTypeDef I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
HAL_StatusTypeDef I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
HAL_StatusTypeDef I2C_WriteReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
HAL_StatusTypeDef I2C_ReadReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

#endif /* IS_BOOTLOADER == 0 */

#endif /* DRV_BQ76952_H_ */

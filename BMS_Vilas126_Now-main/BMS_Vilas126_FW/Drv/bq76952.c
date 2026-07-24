#include "bq76952.h"
#include "bq_reg.h"
#include <string.h>
#include <math.h>

#if IS_BOOTLOADER == 0

// Extern HAL Handles
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;

// Global BQ state variables
uint16_t CellVoltage[16] = {0};
float CellVoltage_Float[16] = {0.0f};
float CellVoltage_V[16] = {0.0f};
int16_t PackCurrent = 0;
int16_t TS1Temp = 0, TS2Temp = 0, TS3Temp = 0, IntTemp = 0;
uint16_t AlarmStatusReg = 0;
uint16_t BatteryStatusReg = 0;
uint16_t FET_Status = 0;
uint8_t CHG = 0, PCHG = 0, DSG = 0, PDSG = 0, DCHG_pin = 0, DDSG_pin = 0, ALRT_pin = 0;
uint8_t RSVD_pin = 0;
uint16_t SafetyStatusA = 0, SafetyStatusB = 0, SafetyStatusC = 0;
uint32_t PFStatusA = 0, PFStatusB = 0, PFStatusC = 0, PFStatusD = 0;
uint8_t RX_data[2] = {0};
uint8_t RX_32Byte[32] = {0};
uint8_t RX_2Byte[2] = {0};

// Static arrays for averaging (port from main.c static variables)
static uint16_t cell_buffer[16][AVERAGE_SAMPLES];
static uint16_t stack_buffer[AVERAGE_SAMPLES];
static uint16_t pack_buffer[AVERAGE_SAMPLES];
static uint16_t ld_buffer[AVERAGE_SAMPLES];
static int16_t current_buffer[AVERAGE_SAMPLES_CURRENT];

static uint8_t first_sample_after_reset = 1;
static uint8_t avg_index = 0;
static uint8_t avg_index_current = 0;

// Variables used in BQ769x2 functions from main.c
uint8_t value_SafetyAlertA = 0;
uint8_t value_SafetyAlertB = 0;
uint8_t value_SafetyAlertC = 0;
uint8_t value_SafetyStatusA = 0;
uint8_t value_SafetyStatusB = 0;
uint8_t value_SafetyStatusC = 0;
uint8_t value_PFStatusA = 0;
uint8_t value_PFStatusB = 0;
uint8_t value_PFStatusC = 0;
uint8_t value_PFStatusD = 0;
uint8_t ProtectionsTriggered = 0;
uint8_t PFErrorsTriggered = 0;
uint8_t SFET = 0, SLEEPCHG = 0, HOST_FET_EN = 0, FET_CTRL_EN = 0, PDSG_EN = 0, FET_INIT_OFF = 0, RSVD_01 = 0, RSVD_00 = 0;
uint8_t value_FETOptions = 0;
uint16_t battery_status_value = 0;
uint16_t battery_status_sleep = 0;
uint8_t SLEEP = 0;
uint16_t battery_status = 0;
uint32_t Stack_Voltage = 0;
uint32_t Pack_Voltage = 0;
uint32_t LD_Voltage = 0;
int16_t Pack_Current = 0;
float sum_voltage = 0.0f;
int32_t AccumulatedCharge_Int = 0;
uint32_t AccumulatedCharge_Frac = 0;
uint32_t AccumulatedCharge_Time = 0;
float passed_charge_mAh = 0.0f;

// Helper function definitions
void delayUS(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void Safe_Delay_ms(uint32_t ms) {
    HAL_IWDG_Refresh(&hiwdg);
    for (uint32_t i = 0; i < ms; i++) {
        delayUS(1000);
        HAL_IWDG_Refresh(&hiwdg);
    }
}

void Emergency_Delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 6000; j++) {
            __NOP();
        }
        HAL_IWDG_Refresh(&hiwdg);
    }
}

static void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    for (uint8_t copyIndex = 0; copyIndex < count; copyIndex++) {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len) {
    unsigned char checksum = 0;
    for (unsigned char i = 0; i < len; i++) {
        checksum += ptr[i];
    }
    checksum = 0xff & ~checksum;
    return checksum;
}

unsigned char CRC8(unsigned char *ptr, unsigned char len) {
    unsigned char crc = 0;
    while (len-- != 0) {
        for (unsigned char i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= 0x107;
            } else {
                crc *= 2;
            }
            if ((*ptr & (0x80 >> i)) != 0) {
                crc ^= 0x107;
            }
        }
        ptr++;
    }
    return crc;
}

void I2C_Bus_Recovery(void) {
    __HAL_RCC_I2C1_FORCE_RESET();
    delayUS(10);
    __HAL_RCC_I2C1_RELEASE_RESET();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        delayUS(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        delayUS(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) break;
    }
    // MX_I2C1_Init is generated in main.c, we assume it gets called or we extern it
    extern void MX_I2C1_Init(void);
    MX_I2C1_Init();
}

void I2C2_Bus_Recovery(void) {
    __HAL_RCC_I2C2_FORCE_RESET();
    delayUS(10);
    __HAL_RCC_I2C2_RELEASE_RESET();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c2);
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        delayUS(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        delayUS(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) break;
    }
    extern void MX_I2C2_Init(void);
    MX_I2C2_Init();
}

HAL_StatusTypeDef I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    uint8_t TX_Buffer[MAX_BUFFER_SIZE] = {0};
#if CRC_Mode
    {
        uint8_t crc_count = count * 2;
        uint8_t crc1stByteBuffer[3] = {0x10, reg_addr, reg_data[0]};
        TX_Buffer[0] = reg_data[0];
        TX_Buffer[1] = CRC8(crc1stByteBuffer, 3);
        unsigned int j = 2;
        for (unsigned int i = 1; i < count; i++) {
            TX_Buffer[j] = reg_data[i];
            j++;
            uint8_t temp_crc_buffer[1] = {reg_data[i]};
            TX_Buffer[j] = CRC8(temp_crc_buffer, 1);
            j++;
        }
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count, 100);
        return status;
    }
#else
    return HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 10);
#endif
}

HAL_StatusTypeDef I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    HAL_StatusTypeDef status = HAL_OK;
    unsigned int RX_CRC_Fail = 0;
    uint8_t RX_Buffer[MAX_BUFFER_SIZE] = {0};
#if CRC_Mode
    {
        uint8_t crc_count = count * 2;
        uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
        unsigned char CRCc = 0;
        uint8_t temp_crc_buffer[3];
        status = HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 100);
        if (status != HAL_OK) {
            return status;
        }
        uint8_t crc1stByteBuffer[4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
        CRCc = CRC8(crc1stByteBuffer, 4);
        if (CRCc != ReceiveBuffer[1]) {
            RX_CRC_Fail++;
            return HAL_ERROR;
        }
        RX_Buffer[0] = ReceiveBuffer[0];
        unsigned int j = 2;
        for (unsigned int i = 1; i < count; i++) {
            RX_Buffer[i] = ReceiveBuffer[j];
            temp_crc_buffer[0] = ReceiveBuffer[j];
            j++;
            CRCc = CRC8(temp_crc_buffer, 1);
            if (CRCc != ReceiveBuffer[j]) {
                RX_CRC_Fail++;
                return HAL_ERROR;
            }
            j++;
        }
        CopyArray(RX_Buffer, reg_data, count);
        return HAL_OK;
    }
#else
    status = HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 10);
    return status;
#endif
}

HAL_StatusTypeDef I2C_ReadReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    HAL_StatusTypeDef status;
    for (uint8_t retry = 0; retry < 3; retry++) {
        status = I2C_ReadReg(reg_addr, reg_data, count);
        if (status == HAL_OK) {
            if (reg_addr == 0x40 && count >= 3) {
                if (reg_data[0] == 0x00 && reg_data[1] == 0xFF && reg_data[2] == 0xFF) {
                    status = HAL_BUSY;
                    Safe_Delay_ms(2);
                    continue;
                }
            }
            return HAL_OK;
        }
        I2C_Bus_Recovery();
        Safe_Delay_ms(5);
    }
    return status;
}

HAL_StatusTypeDef I2C_WriteReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    HAL_StatusTypeDef status;
    for (uint8_t retry = 0; retry < 3; retry++) {
        status = I2C_WriteReg(reg_addr, reg_data, count);
        if (status == HAL_OK) return HAL_OK;
        I2C_Bus_Recovery();
        Safe_Delay_ms(5);
    }
    return status;
}

// BQ interface implementation
HAL_StatusTypeDef Bq_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen) {
    uint8_t TX_RegData[3] = {0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};
    HAL_StatusTypeDef status = HAL_ERROR;

    TX_RegData[0] = (uint8_t)reg_addr;
    TX_RegData[1] = (uint8_t)(reg_addr >> 8);

    if (datalen == 1) {
        TX_RegData[2] = (uint8_t)reg_data;
        status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 3);
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05; // datalen + 4
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
    } else if (datalen == 2) {
        TX_RegData[2] = (uint8_t)reg_data;
        status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 3);
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05;
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
        if (status == HAL_OK) {
            TX_RegData[0] = (uint8_t)(reg_addr + 1);
            TX_RegData[1] = (uint8_t)((reg_addr + 1) >> 8);
            TX_RegData[2] = (uint8_t)(reg_data >> 8);
            status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 3);
        }
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05;
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
    } else if (datalen == 4) {
        // 4-byte write
        uint8_t temp_TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        temp_TX_RegData[0] = (uint8_t)reg_addr;
        temp_TX_RegData[1] = (uint8_t)(reg_addr >> 8);
        temp_TX_RegData[2] = (uint8_t)reg_data;
        temp_TX_RegData[3] = (uint8_t)(reg_data >> 8);
        temp_TX_RegData[4] = (uint8_t)(reg_data >> 16);
        temp_TX_RegData[5] = (uint8_t)(reg_data >> 24);
        status = I2C_WriteReg_WithRetry(0x3E, temp_TX_RegData, 6);
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(temp_TX_RegData, 6);
            TX_Buffer[1] = 0x08; // datalen + 4
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
    }
    return status;
}

HAL_StatusTypeDef DM_Read8(uint16_t reg_addr, uint8_t *value) {
    uint8_t tx[2] = { (uint8_t)reg_addr, (uint8_t)(reg_addr >> 8) };
    HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(0x3E, tx, 2);
    if (status != HAL_OK) return status;
    Safe_Delay_ms(2);
    uint8_t temp_data = 0;
    status = I2C_ReadReg_WithRetry(0x40, &temp_data, 1);
    if (status == HAL_OK) {
        *value = temp_data;
    }
    return status;
}

HAL_StatusTypeDef Bq_Subcmd(uint16_t command) {
    uint8_t TX_Reg[2] = { (uint8_t)command, (uint8_t)(command >> 8) };
    return I2C_WriteReg_WithRetry(0x3E, TX_Reg, 2);
}

HAL_StatusTypeDef Bq_Subcommands(uint16_t command, uint16_t data, uint8_t type) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t TX_Reg[4] = {0};
    uint8_t TX_Buffer[2] = {0};

    TX_Reg[0] = (uint8_t)command;
    TX_Reg[1] = (uint8_t)(command >> 8);

    if (type == R) {
        status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 2);
        if (status != HAL_OK) return status;
        Safe_Delay_ms(4);
        uint8_t cmd_check[2] = {0};
        for (int i = 0; i < 5; i++) {
            status = I2C_ReadReg_WithRetry(0x3E, cmd_check, 2);
            if (status == HAL_OK && cmd_check[0] == TX_Reg[0] && cmd_check[1] == TX_Reg[1]) {
                break;
            }
            Safe_Delay_ms(2);
        }
        return I2C_ReadReg_WithRetry(0x40, RX_32Byte, 32);
    } else if (type == W) {
        TX_Reg[2] = (uint8_t)data;
        status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 3);
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(TX_Reg, 3);
            TX_Buffer[1] = 0x05;
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
    } else if (type == W2) {
        TX_Reg[2] = (uint8_t)data;
        TX_Reg[3] = (uint8_t)(data >> 8);
        status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 4);
        if (status == HAL_OK) {
            TX_Buffer[0] = Checksum(TX_Reg, 4);
            TX_Buffer[1] = 0x06;
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        }
    }
    return status;
}

HAL_StatusTypeDef Bq_DirectCommands(uint8_t command, uint16_t data, uint8_t type, HAL_StatusTypeDef *ptr_status) {
    HAL_StatusTypeDef status = HAL_ERROR;
    if (type == R) {
        status = I2C_ReadReg_WithRetry(command, RX_data, 2);
        if (ptr_status) *ptr_status = status;
        return status;
    } else if (type == W) {
        uint8_t TX_data[2] = { (uint8_t)data, (uint8_t)(data >> 8) };
        status = I2C_WriteReg_WithRetry(command, TX_data, 2);
        if (ptr_status) *ptr_status = status;
        return status;
    }
    return HAL_ERROR;
}

void Bq_Unlock(void) {
    Bq_Subcmd(0x1011); Safe_Delay_ms(2);
    Bq_Subcmd(0x2001); Safe_Delay_ms(2);
    Bq_Subcmd(0x4C4F); Safe_Delay_ms(2);
    Bq_Subcmd(0x4E47); Safe_Delay_ms(2);

    Bq_Subcmd(0x0414); Safe_Delay_ms(2);
    Bq_Subcmd(0x3672); Safe_Delay_ms(2);
    Bq_Subcmd(0xFFFF); Safe_Delay_ms(2);
    Bq_Subcmd(0xFFFF); Safe_Delay_ms(2);
}

static uint32_t Average_U16(const uint16_t buf[AVERAGE_SAMPLES]) {
    uint32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES; i++) sum += buf[i];
    return sum / AVERAGE_SAMPLES;
}

static int32_t Average_I16(const int16_t buf[AVERAGE_SAMPLES_CURRENT]) {
    int32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES_CURRENT; i++) sum += buf[i];
    return sum / AVERAGE_SAMPLES_CURRENT;
}

uint16_t Bq_ReadVoltage(uint8_t command, HAL_StatusTypeDef *ptr_status) {
    HAL_StatusTypeDef status = I2C_ReadReg_WithRetry(command, RX_data, 2);
    if (ptr_status) *ptr_status = status;
    if (status == HAL_OK) {
        if (command >= Cell1Voltage && command <= Cell16Voltage) {
            return (RX_data[1] * 256 + RX_data[0]);
        } else {
            return 10 * (RX_data[1] * 256 + RX_data[0]);
        }
    }
    return 0;
}

HAL_StatusTypeDef Bq_ReadVoltages(void) {
    HAL_StatusTypeDef status = HAL_OK;
    int cellvoltageholder = Cell1Voltage;
    for (int i = 0; i < 16; i++) {
        uint16_t raw = Bq_ReadVoltage(cellvoltageholder, &status);
        if (status == HAL_OK) {
            if (raw > 6500) {
                raw = 0;
                for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                    cell_buffer[i][j] = 0;
                }
            }
            if (first_sample_after_reset) {
                for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                    cell_buffer[i][j] = raw;
                }
            } else {
                cell_buffer[i][avg_index] = raw;
            }
            CellVoltage[i] = (uint16_t)Average_U16(cell_buffer[i]);
            CellVoltage_Float[i] = (float)CellVoltage[i];
            CellVoltage_V[i] = CellVoltage_Float[i] / 1000.0f;
        }
        cellvoltageholder += 2;
    }

    uint16_t raw_stack = Bq_ReadVoltage(StackVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                stack_buffer[j] = raw_stack;
            }
        } else {
            stack_buffer[avg_index] = raw_stack;
        }
        Stack_Voltage = (uint16_t)Average_U16(stack_buffer);
    }

    uint16_t raw_pack = Bq_ReadVoltage(PACKPinVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                pack_buffer[j] = raw_pack;
            }
        } else {
            pack_buffer[avg_index] = raw_pack;
        }
        Pack_Voltage = (uint16_t)Average_U16(pack_buffer);
    }

    uint16_t raw_ld = Bq_ReadVoltage(LDPinVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                ld_buffer[j] = raw_ld;
            }
        } else {
            ld_buffer[avg_index] = raw_ld;
        }
        LD_Voltage = (uint16_t)Average_U16(ld_buffer);
    }

    avg_index = (avg_index + 1) % AVERAGE_SAMPLES;
    sum_voltage = 0;
    for (int i = 0; i < 16; i++) {
        sum_voltage += CellVoltage[i];
    }
    return status;
}

HAL_StatusTypeDef Bq_ReadCurrent(void) {
    HAL_StatusTypeDef status;
    Bq_DirectCommands(CC2Current, 0x00, R, &status);
    if (status == HAL_OK) {
        int16_t raw = (int16_t)((RX_data[1] << 8) | RX_data[0]);
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES_CURRENT; j++) {
                current_buffer[j] = raw;
            }
        } else {
            current_buffer[avg_index_current] = raw;
        }
        avg_index_current = (avg_index_current + 1) % AVERAGE_SAMPLES_CURRENT;
        Pack_Current = (int16_t)Average_I16(current_buffer);
        PackCurrent = Pack_Current;
    }
    return status;
}

static float Bq_ReadTemperature(uint8_t command) {
    HAL_StatusTypeDef status;
    Bq_DirectCommands(command, 0x00, R, &status);
    if (status == HAL_OK) {
        return (0.1f * (float)(RX_data[1] * 256 + RX_data[0])) - 273.15f;
    }
    return 25.0f;
}

HAL_StatusTypeDef Bq_ReadTemps(void) {
    TS1Temp = (int16_t)Bq_ReadTemperature(TS1Temperature);
    TS2Temp = (int16_t)Bq_ReadTemperature(TS2Temperature);
    TS3Temp = (int16_t)Bq_ReadTemperature(TS3Temperature);
    IntTemp = (int16_t)Bq_ReadTemperature(InternalTemperature);
    first_sample_after_reset = 0; // Temp read is usually done after voltage
    return HAL_OK;
}

HAL_StatusTypeDef Bq_ReadStatus(void) {
    HAL_StatusTypeDef status;
    Bq_DirectCommands(FETStatus, 0x00, R, &status);
    if (status == HAL_OK) {
        FET_Status = (RX_data[1] * 256 + RX_data[0]);
        CHG        = (RX_data[0] & 0x01) >> 0;
        PCHG       = (RX_data[0] & 0x02) >> 1;
        DSG        = (RX_data[0] & 0x04) >> 2;
        PDSG       = (RX_data[0] & 0x08) >> 3;
        DCHG_pin   = (RX_data[0] & 0x10) >> 4;
        DDSG_pin   = (RX_data[0] & 0x20) >> 5;
        ALRT_pin   = (RX_data[0] & 0x40) >> 6;
        RSVD_pin   = (RX_data[0] & 0x80) >> 7;
    }

    Bq_DirectCommands(SafetyStatusA, 0x00, R, &status);
    if (status == HAL_OK) SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
    Bq_DirectCommands(SafetyStatusB, 0x00, R, &status);
    if (status == HAL_OK) SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);
    Bq_DirectCommands(SafetyStatusC, 0x00, R, &status);
    if (status == HAL_OK) SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);

    Bq_DirectCommands(PFStatusA, 0x00, R, &status);
    if (status == HAL_OK) PFStatusA = (RX_data[1] * 256 + RX_data[0]);
    Bq_DirectCommands(PFStatusB, 0x00, R, &status);
    if (status == HAL_OK) PFStatusB = (RX_data[1] * 256 + RX_data[0]);
    Bq_DirectCommands(PFStatusC, 0x00, R, &status);
    if (status == HAL_OK) PFStatusC = (RX_data[1] * 256 + RX_data[0]);
    Bq_DirectCommands(PFStatusD, 0x00, R, &status);
    if (status == HAL_OK) PFStatusD = (RX_data[1] * 256 + RX_data[0]);

    Bq_DirectCommands(BatteryStatus, 0x00, R, &status);
    if (status == HAL_OK) {
        battery_status_value = ((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0];
        battery_status = battery_status_value;
    }

    Bq_DirectCommands(AlarmStatus, 0x00, R, &status);
    if (status == HAL_OK) {
        AlarmStatusReg = (RX_data[1] * 256 + RX_data[0]);
    }
    return status;
}

void Bq_SetFet(uint8_t chg, uint8_t dsg) {
    if (chg && dsg) {
        Bq_Subcmd(ALL_FETS_ON);
    } else if (!chg && !dsg) {
        Bq_Subcmd(ALL_FETS_OFF);
    }
}

void Bq_ReadPassQ(void) {
    memset(RX_32Byte, 0, 32);
    HAL_StatusTypeDef status = Bq_Subcommands(DASTATUS6, 0x00, R);
    if (status == HAL_OK) {
        AccumulatedCharge_Int  = ((int32_t)RX_32Byte[3]<<24) | ((int32_t)RX_32Byte[2]<<16) | ((int32_t)RX_32Byte[1]<<8) | (int32_t)RX_32Byte[0];
        if (AccumulatedCharge_Int == -32767 || (uint32_t)AccumulatedCharge_Int == 0xFFFFFFFF) return;
        AccumulatedCharge_Frac = ((uint32_t)RX_32Byte[7]<<24) | ((uint32_t)RX_32Byte[6]<<16) | ((uint32_t)RX_32Byte[5]<<8) | (uint32_t)RX_32Byte[4];
        AccumulatedCharge_Time = ((uint32_t)RX_32Byte[11]<<24) | ((uint32_t)RX_32Byte[10]<<16) | ((uint32_t)RX_32Byte[9]<<8) | (uint32_t)RX_32Byte[8];
        double total_userAh = (double)AccumulatedCharge_Int + (double)AccumulatedCharge_Frac / 4294967296.0f;
        passed_charge_mAh = (float)(total_userAh * 9.95f);
    }
}

HAL_StatusTypeDef Bq_Init(void) {
    Bq_Unlock();
    HAL_StatusTypeDef status = Bq_Subcmd(SET_CFGUPDATE);
    if (status != HAL_OK) {
        return status;
    }
    // Calibration settings
    Bq_SetRegister(Cell1Gain, 0, 2);
    Bq_SetRegister(Cell2Gain, 0, 2);
    Bq_SetRegister(Cell3Gain, 0, 2);
    Bq_SetRegister(Cell4Gain, 0, 2);
    Bq_SetRegister(Cell5Gain, 0, 2);
    Bq_SetRegister(Cell6Gain, 0, 2);
    Bq_SetRegister(Cell7Gain, 0, 2);
    Bq_SetRegister(Cell8Gain, 0, 2);
    Bq_SetRegister(Cell9Gain, 0, 2);
    Bq_SetRegister(Cell10Gain, 0, 2);
    Bq_SetRegister(Cell11Gain, 0, 2);
    Bq_SetRegister(Cell12Gain, 0, 2);
    Bq_SetRegister(Cell13Gain, 0, 2);
    Bq_SetRegister(Cell14Gain, 0, 2);
    Bq_SetRegister(Cell15Gain, 0, 2);
    Bq_SetRegister(Cell16Gain, 0, 2);
    Bq_SetRegister(PackGain, 0, 2);
    Bq_SetRegister(TOSGain, 0, 2);
    Bq_SetRegister(LDGain, 0, 2);
    Bq_SetRegister(ADCGain, 0, 2);
    Bq_SetRegister(CCGain, 0x42958937, 4);
    Bq_SetRegister(CapacityGain, 0x4BAA2384, 4);
    Bq_SetRegister(VcellOffset, 0, 2);
    Bq_SetRegister(VdivOffset, 0, 2);
    Bq_SetRegister(CoulombCounterOffsetSamples, 64, 2);
    Bq_SetRegister(BoardOffset, 18, 2);
    Bq_SetRegister(InternalTempOffset, 0, 1);
    Bq_SetRegister(CFETOFFTempOffset, 0, 1);
    Bq_SetRegister(DFETOFFTempOffset, 0, 1);
    Bq_SetRegister(ALERTTempOffset, 0, 1);
    Bq_SetRegister(TS1TempOffset, 0, 1);
    Bq_SetRegister(TS2TempOffset, 0, 1);
    Bq_SetRegister(TS3TempOffset, 0, 1);
    Bq_SetRegister(HDQTempOffset, 0, 1);
    Bq_SetRegister(DCHGTempOffset, 0, 1);
    Bq_SetRegister(DDSGTempOffset, 0, 1);
    Bq_SetRegister(IntGain, 25390, 2);
    Bq_SetRegister(Intbaseoffset, 3032, 2);
    Bq_SetRegister(IntMaximumAD, 16383, 2);
    Bq_SetRegister(IntMaximumTemp, 6379, 2);
    Bq_SetRegister(T18kCoeffa1, (uint16_t)-15524, 2);
    Bq_SetRegister(T18kCoeffa2, 26423, 2);
    Bq_SetRegister(T18kCoeffa3, (uint16_t)-22664, 2);
    Bq_SetRegister(T18kCoeffa4, 28834, 2);
    Bq_SetRegister(T18kCoeffa5, 672, 2);
    Bq_SetRegister(T18kCoeffb1, (uint16_t)-371, 2);
    Bq_SetRegister(T18kCoeffb2, 708, 2);
    Bq_SetRegister(T18kCoeffb3, (uint16_t)-3498, 2);
    Bq_SetRegister(T18kCoeffb4, 5051, 2);
    Bq_SetRegister(T18kAdc0, 11703, 2);
    Bq_SetRegister(T180kCoeffa1, (uint16_t)-17513, 2);
    Bq_SetRegister(T180kCoeffa2, 25759, 2);
    Bq_SetRegister(T180kCoeffa3, (uint16_t)-23593, 2);
    Bq_SetRegister(T180kCoeffa4, 32175, 2);
    Bq_SetRegister(T180kCoeffa5, 2090, 2);
    Bq_SetRegister(T180kCoeffb1, (uint16_t)-2055, 2);
    Bq_SetRegister(T180kCoeffb2, 2955, 2);
    Bq_SetRegister(T180kCoeffb3, (uint16_t)-3427, 2);
    Bq_SetRegister(T180kCoeffb4, 4385, 2);
    Bq_SetRegister(T180kAdc0, 17246, 2);
    Bq_SetRegister(CustomCoeffa1, 0x00, 2);
    Bq_SetRegister(CustomCoeffa2, 0x00, 2);
    Bq_SetRegister(CustomCoeffa3, 0x00, 2);
    Bq_SetRegister(CustomCoeffa4, 0x00, 2);
    Bq_SetRegister(CustomCoeffa5, 0x00, 2);
    Bq_SetRegister(CustomCoeffb1, 0x00, 2);
    Bq_SetRegister(CustomCoeffb2, 0x00, 2);
    Bq_SetRegister(CustomCoeffb3, 0x00, 2);
    Bq_SetRegister(CustomCoeffb4, 0x00, 2);
    Bq_SetRegister(CustomRc0, 0x00, 2);
    Bq_SetRegister(CustomAdc0, 0x00, 2);
    Bq_SetRegister(CoulombCounterDeadband, 9, 1);
    Bq_SetRegister(CUVThresholdOverride, 0xFFFF, 2);
    Bq_SetRegister(COVThresholdOverride, 0xFFFF, 2);

    Bq_SetRegister(PowerConfig, 0x2CB0, 2);
    Bq_SetRegister(REG12Config, 0x00, 1);
    Bq_SetRegister(REG0Config, 0x00, 1);
    Bq_SetRegister(HWDRegulatorOptions, 0x00, 1);
    Bq_SetRegister(CommType, 0x12, 1);
    Bq_SetRegister(I2CAddress, 0, 1);
    Bq_SetRegister(SPIConfiguration, 0x20, 1);
    Bq_SetRegister(CommIdleTime, 0, 1);
    Bq_SetRegister(DFETOFFPinConfig, 0x46, 1);
    Bq_SetRegister(CFETOFFPinConfig, 0x02, 1);
    Bq_SetRegister(ALERTPinConfig, 0x82, 1);
    Bq_SetRegister(TS1Config, 0x07, 1);
    Bq_SetRegister(TS2Config, 0x07, 1);
    Bq_SetRegister(TS3Config, 0x07, 1);
    Bq_SetRegister(HDQPinConfig, 0x07, 1);
    Bq_SetRegister(DCHGPinConfig, 0x01, 1);
    Bq_SetRegister(DDSGPinConfig, 0x01, 1);
    Bq_SetRegister(DAConfiguration, 0x06, 1);
    Bq_SetRegister(VCellMode, 0xFFFF, 2);
    Bq_SetRegister(CC3Samples, 80, 1);
    Bq_SetRegister(ProtectionConfiguration, 0x0602, 2);
    Bq_SetRegister(EnabledProtectionsA, 0xFC, 1);
    Bq_SetRegister(EnabledProtectionsB, 0xF7, 1);
    Bq_SetRegister(EnabledProtectionsC, 0x76, 1);
    Bq_SetRegister(CHGFETProtectionsA, 0x98, 1);
    Bq_SetRegister(CHGFETProtectionsB, 0xD5, 1);
    Bq_SetRegister(CHGFETProtectionsC, 0x56, 1);
    Bq_SetRegister(DSGFETProtectionsA, 0xE4, 1);
    Bq_SetRegister(DSGFETProtectionsB, 0xE6, 1);
    Bq_SetRegister(DSGFETProtectionsC, 0x62, 1);
    Bq_SetRegister(BodyDiodeThreshold, 2000, 2);
    Bq_SetRegister(DefaultAlarmMask, 0xF800, 2);
    Bq_SetRegister(SFAlertMaskA, 0xFC, 1);
    Bq_SetRegister(SFAlertMaskB, 0xF7, 1);
    Bq_SetRegister(SFAlertMaskC, 0x74, 1);
    Bq_SetRegister(PFAlertMaskA, 0x5F, 1);
    Bq_SetRegister(PFAlertMaskB, 0x9F, 1);
    Bq_SetRegister(PFAlertMaskC, 0x00, 1);
    Bq_SetRegister(PFAlertMaskD, 0x00, 1);
    Bq_SetRegister(EnabledPFA, 0x5F, 1);
    Bq_SetRegister(EnabledPFB, 0x90, 1);
    Bq_SetRegister(EnabledPFC, 0x07, 1);
    Bq_SetRegister(EnabledPFD, 0x01, 1);
    Bq_SetRegister(FETOptions, 0x1D, 1);
    Bq_SetRegister(ChgPumpControl, 0x05, 1);
    Bq_SetRegister(PrechargeStartVoltage, 2750, 2);
    Bq_SetRegister(PrechargeStopVoltage, 2900, 2);
    Bq_SetRegister(PredischargeTimeout, 250, 1);
    Bq_SetRegister(PredischargeStopDelta, 20, 1);
    Bq_SetRegister(DsgCurrentThreshold, 10, 2);
    Bq_SetRegister(ChgCurrentThreshold, 5, 2);
    Bq_SetRegister(CheckTime, 5, 1);
    Bq_SetRegister(Cell1Interconnect, 0, 2);
    Bq_SetRegister(Cell2Interconnect, 0, 2);
    Bq_SetRegister(Cell3Interconnect, 0, 2);
    Bq_SetRegister(Cell4Interconnect, 0, 2);
    Bq_SetRegister(Cell5Interconnect, 0, 2);
    Bq_SetRegister(Cell6Interconnect, 0, 2);
    Bq_SetRegister(Cell7Interconnect, 0, 2);
    Bq_SetRegister(Cell8Interconnect, 0, 2);
    Bq_SetRegister(Cell9Interconnect, 0, 2);
    Bq_SetRegister(Cell10Interconnect, 0, 2);
    Bq_SetRegister(Cell11Interconnect, 0, 2);
    Bq_SetRegister(Cell12Interconnect, 0, 2);
    Bq_SetRegister(Cell13Interconnect, 0, 2);
    Bq_SetRegister(Cell14Interconnect, 0, 2);
    Bq_SetRegister(Cell15Interconnect, 0, 2);
    Bq_SetRegister(Cell16Interconnect, 0, 2);
    Bq_SetRegister(MfgStatusInit, 0x00D0, 2);
    Bq_SetRegister(BalancingConfiguration, 0x03, 1);
    Bq_SetRegister(MinCellTemp, (uint8_t)-20, 1);
    Bq_SetRegister(MaxCellTemp, 60, 1);
    Bq_SetRegister(MaxInternalTemp, 70, 1);
    Bq_SetRegister(CellBalanceInterval, 3, 1);
    Bq_SetRegister(CellBalanceMaxCells, 12, 1);
    Bq_SetRegister(CellBalanceMinCellVCharge, 3400, 2);
    Bq_SetRegister(CellBalanceMinDeltaCharge, 30, 1);
    Bq_SetRegister(CellBalanceStopDeltaCharge, 20, 1);
    Bq_SetRegister(CellBalanceMinCellVRelax, 3400, 2);
    Bq_SetRegister(CellBalanceMinDeltaRelax, 30, 1);
    Bq_SetRegister(CellBalanceStopDeltaRelax, 20, 1);
    Bq_SetRegister(ShutdownCellVoltage, 0, 2);
    Bq_SetRegister(ShutdownStackVoltage, 4320, 2);
    Bq_SetRegister(LowVShutdownDelay, 5, 1);
    Bq_SetRegister(ShutdownTemperature, 80, 1);
    Bq_SetRegister(ShutdownTemperatureDelay, 5, 1);
    Bq_SetRegister(FETOffDelay, 0, 1);
    Bq_SetRegister(ShutdownCommandDelay, 0, 1);
    Bq_SetRegister(AutoShutdownTime, 0, 1);
    Bq_SetRegister(RAMFailShutdownTime, 5, 1);
    Bq_SetRegister(SleepCurrent, 10, 2);
    Bq_SetRegister(VoltageTime, 5, 1);
    Bq_SetRegister(WakeComparatorCurrent, 200, 2);
    Bq_SetRegister(SleepHysteresisTime, 10, 1);
    Bq_SetRegister(SleepChargerVoltageThreshold, 6000, 2);
    Bq_SetRegister(SleepChargerPACKTOSDelta, 200, 2);
    Bq_SetRegister(ConfigRAMSignature, 0, 2);
    Bq_SetRegister(CUVThreshold, 54, 1);
    Bq_SetRegister(CUVDelay, 301, 2);
    Bq_SetRegister(CUVRecoveryHysteresis, 4, 1);
    Bq_SetRegister(COVThreshold, 72, 1);
    Bq_SetRegister(COVDelay, 301, 2);
    Bq_SetRegister(COVRecoveryHysteresis, 5, 1);
    Bq_SetRegister(COVLLatchLimit, 3, 1);
    Bq_SetRegister(COVLCounterDecDelay, 10, 1);
    Bq_SetRegister(COVLRecoveryTime, 15, 1);
    Bq_SetRegister(OCCThreshold, 7, 1);
    Bq_SetRegister(OCCDelay, 127, 1);
    Bq_SetRegister(OCCRecoveryThreshold, (uint16_t)-1000, 2);
    Bq_SetRegister(OCCPACKTOSDelta, 200, 2);
    Bq_SetRegister(OCD1Threshold, 7, 1);
    Bq_SetRegister(OCD1Delay, 127, 1);
    Bq_SetRegister(OCD2Threshold, 8, 1);
    Bq_SetRegister(OCD2Delay, 127, 1);
    Bq_SetRegister(SCDThreshold, 6, 1);
    Bq_SetRegister(SCDDelay, 31, 1);
    Bq_SetRegister(SCDRecoveryTime, 5, 1);
    Bq_SetRegister(OCD3Threshold, (uint16_t)-2000, 2);
    Bq_SetRegister(OCD3Delay, 2, 1);
    Bq_SetRegister(OCDRecoveryThreshold, 1000, 2);
    Bq_SetRegister(OCDLLatchLimit, 10, 1);
    Bq_SetRegister(OCDLCounterDecDelay, 10, 1);
    Bq_SetRegister(OCDLRecoveryTime, 60, 1);
    Bq_SetRegister(OCDLRecoveryThreshold, 1000, 2);
    Bq_SetRegister(SCDLLatchLimit, 5, 1);
    Bq_SetRegister(SCDLCounterDecDelay, 10, 1);
    Bq_SetRegister(SCDLRecoveryTime, 15, 1);
    Bq_SetRegister(SCDLRecoveryThreshold, 1000, 2);
    Bq_SetRegister(OTCThreshold, 65, 1);
    Bq_SetRegister(OTCDelay, 2, 1);
    Bq_SetRegister(OTCRecovery, 55, 1);
    Bq_SetRegister(OTDThreshold, 70, 1);
    Bq_SetRegister(OTDDelay, 2, 1);
    Bq_SetRegister(OTDRecovery, 60, 1);
    Bq_SetRegister(OTFThreshold, 115, 1);
    Bq_SetRegister(OTFDelay, 2, 1);
    Bq_SetRegister(OTFRecovery, 85, 1);
    Bq_SetRegister(OTINTThreshold, 75, 1);
    Bq_SetRegister(OTINTDelay, 2, 1);
    Bq_SetRegister(OTINTRecovery, 65, 1);
    Bq_SetRegister(UTCThreshold, 0, 1);
    Bq_SetRegister(UTCDelay, 2, 1);
    Bq_SetRegister(UTCRecovery, 5, 1);
    Bq_SetRegister(UTDThreshold, (uint8_t)-20, 1);
    Bq_SetRegister(UTDDelay, 2, 1);
    Bq_SetRegister(UTDRecovery, (uint8_t)-15, 1);
    Bq_SetRegister(UTINTThreshold, (uint8_t)-20, 1);
    Bq_SetRegister(UTINTDelay, 2, 1);
    Bq_SetRegister(UTINTRecovery, (uint8_t)-15, 1);
    Bq_SetRegister(ProtectionsRecoveryTime, 5, 1);
    Bq_SetRegister(HWDDelay, 60, 2);
    Bq_SetRegister(LoadDetectActiveTime, 5, 1);
    Bq_SetRegister(LoadDetectRetryDelay, 50, 1);
    Bq_SetRegister(LoadDetectTimeout, 1, 2);
    Bq_SetRegister(PTOChargeThreshold, 250, 2);
    Bq_SetRegister(PTODelay, 0, 2);
    Bq_SetRegister(PTOReset, 2, 2);
    Bq_SetRegister(CUDEPThreshold, 2250, 2);
    Bq_SetRegister(CUDEPDelay, 2, 1);
    Bq_SetRegister(SUVThreshold, 2550, 2);
    Bq_SetRegister(SUVDelay, 5, 1);
    Bq_SetRegister(SOVThreshold, 3800, 2);
    Bq_SetRegister(SOVDelay, 5, 1);
    Bq_SetRegister(TOSSThreshold, 500, 2);
    Bq_SetRegister(TOSSDelay, 5, 1);
    Bq_SetRegister(SOCCThreshold, 18000, 2);
    Bq_SetRegister(SOCCDelay, 5, 1);
    Bq_SetRegister(SOCDThreshold, (uint16_t)-20000, 2);
    Bq_SetRegister(SOCDDelay, 5, 1);
    Bq_SetRegister(SOTThreshold, 80, 1);
    Bq_SetRegister(SOTDelay, 5, 1);
    Bq_SetRegister(SOTFThreshold, 125, 1);
    Bq_SetRegister(SOTFDelay, 5, 1);
    Bq_SetRegister(VIMRCheckVoltage, 3500, 2);
    Bq_SetRegister(VIMRMaxRelaxCurrent, 10, 2);
    Bq_SetRegister(VIMRThreshold, 500, 2);
    Bq_SetRegister(VIMRDelay, 5, 1);
    Bq_SetRegister(VIMRRelaxMinDuration, 100, 2);
    Bq_SetRegister(VIMACheckVoltage, 3700, 2);
    Bq_SetRegister(VIMAMinActiveCurrent, 50, 2);
    Bq_SetRegister(VIMAThreshold, 200, 2);
    Bq_SetRegister(VIMADelay, 5, 1);
    Bq_SetRegister(CFETFOFFThreshold, 20, 2);
    Bq_SetRegister(CFETFOFFDelay, 5, 1);
    Bq_SetRegister(DFETFOFFThreshold, (uint16_t)-20, 2);
    Bq_SetRegister(DFETFOFFDelay, 5, 1);
    Bq_SetRegister(VSSFFailThreshold, 100, 2);
    Bq_SetRegister(VSSFDelay, 5, 1);
    Bq_SetRegister(PF2LVLDelay, 5, 1);
    Bq_SetRegister(LFOFDelay, 5, 1);
    Bq_SetRegister(HWMXDelay, 5, 1);
    Bq_SetRegister(SecuritySettings, 0x01, 1);
    Bq_SetRegister(UnsealKeyStep1, 0x1011, 2);
    Bq_SetRegister(UnsealKeyStep2, 0x2001, 2);
    Bq_SetRegister(FullAccessKeyStep1, 0x4C4F, 2);
    Bq_SetRegister(FullAccessKeyStep2, 0x4E47, 2);

    status = Bq_Subcmd(EXIT_CFGUPDATE);
    return status;
}

#endif /* IS_BOOTLOADER == 0 */

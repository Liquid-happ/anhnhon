#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <stdint.h>
#include <stddef.h>

#define OTA_INFO_MAGIC     0xA5A55A5AUL
#define APP_STATE_EMPTY    0xFFFFFFFFUL
#define APP_STATE_VALID    0x12345678UL
#define APP_STATE_PENDING  0x87654321UL
#define APP_STATE_INVALID  0x00000000UL

#define ACTIVE_APP_NONE    0UL
#define ACTIVE_APP_A       1UL
#define ACTIVE_APP_B       2UL

#define FLASH_BASE_ADDR        0x08000000UL

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE        0x800U
#endif

#define BOOT_ADDR              0x08000000UL
#define BOOT_SIZE              (64UL * 1024UL)

#define APP_A_ADDR             0x08010000UL
#define APP_B_ADDR             0x08047800UL
#define APP_SLOT_SIZE          (222UL * 1024UL)

#define OTA_INFO_ADDR          0x0800F000UL
#define OTA_INFO_SIZE          (4UL * 1024UL)

#define FLASH_END_ADDR         0x0807F000UL

#define RAM_START_ADDR         0x20000000UL
#define RAM_END_ADDR           0x20010000UL   // STM32F103VET6 RAM 64KB

typedef struct
{
    uint32_t magic;
    uint32_t app_a_version;
    uint32_t app_a_size;
    uint8_t   app_a_crc[32]; 
    uint32_t app_b_version;
    uint32_t app_b_size;
    uint8_t   app_b_crc[32]; 
    uint32_t meta_crc;
} OTA_Info_t;

typedef struct Ota_data_t{
    uint8_t start_ota;
    uint8_t ok_start_ota;
    uint32_t version;
    uint32_t size;
    uint8_t crc[32];  // sha256 crc is 32 bytes
    uint32_t TimeoutCounter;
} Ota_data_t;

// BMS Measurement data
typedef struct {
    uint16_t cell_mv[16];
    uint32_t pack_mv;
    uint32_t stack_mv;
    uint32_t ld_mv;
    int32_t pack_ma;
    int16_t temp_c[6];
    uint16_t cell_min_mv;
    uint16_t cell_max_mv;
    uint16_t cell_diff_mv;
} Bms_Meas_t;

// BMS Fault indicators
typedef struct {
    uint8_t uv;
    uint8_t ov;
    uint8_t occ;
    uint8_t ocd;
    uint8_t scd;
    uint8_t otc;
    uint8_t otd;
    uint8_t utc;
    uint8_t utd;
    uint8_t pf;
    uint8_t cell_diff;
    uint8_t low_batt;
    uint8_t cell_fail;
} Bms_Fault_t;

#endif /* COMMON_TYPES_H_ */

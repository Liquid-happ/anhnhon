#ifndef DRIVER_W25QXX_H
#define DRIVER_W25QXX_H

#include "feat_cfg.h"

#if IS_BOOTLOADER == 0

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

#ifndef W25QXX_WRITE_STATUS_TIMEOUT_MS
    #define W25QXX_WRITE_STATUS_TIMEOUT_MS      			(1000U)        
#endif

#ifndef W25QXX_ERASE_CHIP_TIMEOUT_MS
    #define W25QXX_ERASE_CHIP_TIMEOUT_MS        			(100U * 1000U)        
#endif

#ifndef W25QXX_ERASE_SECURITY_TIMEOUT_MS
    #define W25QXX_ERASE_SECURITY_TIMEOUT_MS        	(100U)      
#endif

#ifndef W25QXX_PROGRAM_SECURITY_TIMEOUT_MS
    #define W25QXX_PROGRAM_SECURITY_TIMEOUT_MS        (3U)      
#endif

#ifndef W25QXX_PAGE_PROGRAM_TIMEOUT_MS
    #define W25QXX_PAGE_PROGRAM_TIMEOUT_MS        		(3U)      
#endif

#ifndef W25QXX_ERASE_4K_TIMEOUT_MS
    #define W25QXX_ERASE_4K_TIMEOUT_MS        				(400U)      
#endif

#ifndef W25QXX_ERASE_32K_TIMEOUT_MS
    #define W25QXX_ERASE_32K_TIMEOUT_MS        				(1600U)      
#endif

#ifndef W25QXX_ERASE_64K_TIMEOUT_MS
    #define W25QXX_ERASE_64K_TIMEOUT_MS        				(2000U)      
#endif

typedef enum
{
    W25Q10  = 0XEF10U,        
    W25Q20  = 0XEF11U,        
    W25Q40  = 0XEF12U,        
    W25Q80  = 0XEF13U,        
    W25Q16  = 0XEF14U,        
    W25Q32  = 0XEF15U,        
    W25Q64  = 0XEF16U,        
    W25Q128 = 0XEF17U,        
    W25Q256 = 0XEF18U,        
    W25Q512 = 0XEF19U,        
    W25Q01  = 0XEF20U,        
    W25Q02  = 0XEF21U,        
} w25qxx_type_t;

typedef enum
{
    W25QXX_INTERFACE_SPI  = 0x00,        
    W25QXX_INTERFACE_QSPI = 0x01,        
} w25qxx_interface_t;

typedef enum
{
    W25QXX_BOOL_FALSE = 0x00,        
    W25QXX_BOOL_TRUE  = 0x01,        
} w25qxx_bool_t;

typedef enum
{
    W25QXX_ADDRESS_MODE_3_BYTE = 0x00,        
    W25QXX_ADDRESS_MODE_4_BYTE = 0x01,        
} w25qxx_address_mode_t;

typedef enum
{
    W25QXX_QSPI_READ_DUMMY_2_33MHZ = 0x00,        
    W25QXX_QSPI_READ_DUMMY_4_55MHZ = 0x01,        
    W25QXX_QSPI_READ_DUMMY_6_80MHZ = 0x02,        
    W25QXX_QSPI_READ_DUMMY_8_80MHZ = 0x03,        
} w25qxx_qspi_read_dummy_t;

typedef enum
{
    W25QXX_QSPI_READ_WRAP_LENGTH_8_BYTE  = 0x00,        
    W25QXX_QSPI_READ_WRAP_LENGTH_16_BYTE = 0x01,        
    W25QXX_QSPI_READ_WRAP_LENGTH_32_BYTE = 0x02,        
    W25QXX_QSPI_READ_WRAP_LENGTH_64_BYTE = 0x03,        
} w25qxx_qspi_read_wrap_length_t;

typedef enum
{
    W25QXX_SECURITY_REGISTER_1 = 0x10,        
    W25QXX_SECURITY_REGISTER_2 = 0x20,        
    W25QXX_SECURITY_REGISTER_3 = 0x30,        
} w25qxx_security_register_t;

typedef enum
{
    W25QXX_BURST_WRAP_NONE    = 0x10,        
    W25QXX_BURST_WRAP_8_BYTE  = 0x00,        
    W25QXX_BURST_WRAP_16_BYTE = 0x20,        
    W25QXX_BURST_WRAP_32_BYTE = 0x40,        
    W25QXX_BURST_WRAP_64_BYTE = 0x60,        
} w25qxx_burst_wrap_t;

typedef enum
{
    W25QXX_STATUS1_STATUS_REGISTER_PROTECT_0             = (1 << 7),        
    W25QXX_STATUS1_SECTOR_PROTECT_OR_TOP_BOTTOM_PROTECT  = (1 << 6),        
    W25QXX_STATUS1_TOP_BOTTOM_PROTECT_OR_BLOCK_PROTECT_3 = (1 << 5),        
    W25QXX_STATUS1_BLOCK_PROTECT_2                       = (1 << 4),        
    W25QXX_STATUS1_BLOCK_PROTECT_1                       = (1 << 3),        
    W25QXX_STATUS1_BLOCK_PROTECT_0                       = (1 << 2),        
    W25QXX_STATUS1_WRITE_ENABLE_LATCH                    = (1 << 1),        
    W25QXX_STATUS1_ERASE_WRITE_PROGRESS                  = (1 << 0),        
} w25qxx_status1_t;

typedef enum
{
    W25QXX_STATUS2_SUSPEND_STATUS                = (1 << 7),        
    W25QXX_STATUS2_COMPLEMENT_PROTECT            = (1 << 6),        
    W25QXX_STATUS2_SECURITY_REGISTER_3_LOCK_BITS = (1 << 5),        
    W25QXX_STATUS2_SECURITY_REGISTER_2_LOCK_BITS = (1 << 4),        
    W25QXX_STATUS2_SECURITY_REGISTER_1_LOCK_BITS = (1 << 3),        
    W25QXX_STATUS2_QUAD_ENABLE                   = (1 << 1),        
    W25QXX_STATUS2_STATUS_REGISTER_PROTECT_1     = (1 << 0),        
} w25qxx_status2_t;

typedef enum
{
    W25QXX_STATUS3_HOLD_RESET_FUNCTION                   = (1 << 7),        
    W25QXX_STATUS3_OUTPUT_DRIVER_STRENGTH_100_PERCENTAGE = (0 << 5),        
    W25QXX_STATUS3_OUTPUT_DRIVER_STRENGTH_75_PERCENTAGE  = (1 << 5),        
    W25QXX_STATUS3_OUTPUT_DRIVER_STRENGTH_50_PERCENTAGE  = (2 << 5),        
    W25QXX_STATUS3_OUTPUT_DRIVER_STRENGTH_25_PERCENTAGE  = (3 << 5),        
    W25QXX_STATUS3_WRITE_PROTECT_SELECTION               = (1 << 2),        
    W25QXX_STATUS3_POWER_UP_ADDRESS_MODE                 = (1 << 1),        
    W25QXX_STATUS3_CURRENT_ADDRESS_MODE                  = (1 << 0),        
} w25qxx_status3_t;

typedef struct w25qxx_handle_s
{
    uint8_t (*spi_qspi_init)(void);                                                                    
    uint8_t (*spi_qspi_deinit)(void);                                                                  
    uint8_t (*spi_qspi_write_read)(uint8_t instruction, uint8_t instruction_line,
                                   uint32_t address, uint8_t address_line, uint8_t address_len,
                                   uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                   uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                   uint8_t *out_buf, uint32_t out_len, uint8_t data_line);             
    void (*delay_ms)(uint32_t ms);                                                                     
    void (*delay_us)(uint32_t us);                                                                     
    void (*debug_print)(const char *const fmt, ...);                                                   
    uint8_t inited;                                                                                    
    uint16_t type;                                                                                     
    uint8_t address_mode;                                                                              
    uint8_t param;                                                                                     
    uint8_t dummy;                                                                                     
    uint8_t dual_quad_spi_enable;                                                                      
    uint8_t spi_qspi;                                                                                  
    uint8_t buf[256 + 6];                                                                              
    uint8_t buf_4k[4096 + 1];                                                                          
} w25qxx_handle_t;

typedef struct w25qxx_info_s
{
    char chip_name[32];                
    char manufacturer_name[32];        
    char interface[16];                
    float supply_voltage_min_v;        
    float supply_voltage_max_v;        
    float max_current_ma;              
    float temperature_min;             
    float temperature_max;             
    uint32_t driver_version;           
} w25qxx_info_t;

#define DRIVER_W25QXX_LINK_INIT(HANDLE, STRUCTURE)                memset(HANDLE, 0, sizeof(STRUCTURE))

#define DRIVER_W25QXX_LINK_SPI_QSPI_INIT(HANDLE, FUC)             (HANDLE)->spi_qspi_init = FUC

#define DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(HANDLE, FUC)           (HANDLE)->spi_qspi_deinit = FUC

#define DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(HANDLE, FUC)       (HANDLE)->spi_qspi_write_read = FUC

#define DRIVER_W25QXX_LINK_DELAY_MS(HANDLE, FUC)                  (HANDLE)->delay_ms = FUC

#define DRIVER_W25QXX_LINK_DELAY_US(HANDLE, FUC)                  (HANDLE)->delay_us = FUC

#define DRIVER_W25QXX_LINK_DEBUG_PRINT(HANDLE, FUC)               (HANDLE)->debug_print = FUC

uint8_t w25qxx_info(w25qxx_info_t *info);

uint8_t w25qxx_set_dual_quad_spi(w25qxx_handle_t *handle, w25qxx_bool_t enable);

uint8_t w25qxx_get_dual_quad_spi(w25qxx_handle_t *handle, w25qxx_bool_t *enable);

uint8_t w25qxx_set_type(w25qxx_handle_t *handle, w25qxx_type_t type);

uint8_t w25qxx_get_type(w25qxx_handle_t *handle, w25qxx_type_t *type);

uint8_t w25qxx_set_interface(w25qxx_handle_t *handle, w25qxx_interface_t interface);

uint8_t w25qxx_get_interface(w25qxx_handle_t *handle, w25qxx_interface_t *interface);

uint8_t w25qxx_set_address_mode(w25qxx_handle_t *handle, w25qxx_address_mode_t mode);

uint8_t w25qxx_get_address_mode(w25qxx_handle_t *handle, w25qxx_address_mode_t *mode);

uint8_t w25qxx_init(w25qxx_handle_t *handle);

uint8_t w25qxx_deinit(w25qxx_handle_t *handle);

uint8_t w25qxx_read(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_write(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_only_spi_read(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_fast_read(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_page_program(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint16_t len);

uint8_t w25qxx_sector_erase_4k(w25qxx_handle_t *handle, uint32_t addr);

uint8_t w25qxx_block_erase_32k(w25qxx_handle_t *handle, uint32_t addr);

uint8_t w25qxx_block_erase_64k(w25qxx_handle_t *handle, uint32_t addr);

uint8_t w25qxx_chip_erase(w25qxx_handle_t *handle);

uint8_t w25qxx_power_down(w25qxx_handle_t *handle);

uint8_t w25qxx_release_power_down(w25qxx_handle_t *handle);

uint8_t w25qxx_get_manufacturer_device_id(w25qxx_handle_t *handle, uint8_t *manufacturer, uint8_t *device_id);

uint8_t w25qxx_fast_read_dual_output(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_fast_read_quad_output(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_fast_read_dual_io(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_fast_read_quad_io(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_word_read_quad_io(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_octal_word_read_quad_io(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint32_t len);

uint8_t w25qxx_page_program_quad_input(w25qxx_handle_t *handle, uint32_t addr, uint8_t *data, uint16_t len);

uint8_t w25qxx_enable_write(w25qxx_handle_t *handle);

uint8_t w25qxx_enable_volatile_sr_write(w25qxx_handle_t *handle);

uint8_t w25qxx_disable_write(w25qxx_handle_t *handle);

uint8_t w25qxx_get_status1(w25qxx_handle_t *handle, uint8_t *status);

uint8_t w25qxx_get_status2(w25qxx_handle_t *handle, uint8_t *status);

uint8_t w25qxx_get_status3(w25qxx_handle_t *handle, uint8_t *status);

uint8_t w25qxx_set_status1(w25qxx_handle_t *handle, uint8_t status);

uint8_t w25qxx_set_status2(w25qxx_handle_t *handle, uint8_t status);

uint8_t w25qxx_set_status3(w25qxx_handle_t *handle, uint8_t status);

uint8_t w25qxx_erase_program_suspend(w25qxx_handle_t *handle);

uint8_t w25qxx_erase_program_resume(w25qxx_handle_t *handle);

uint8_t w25qxx_get_manufacturer_device_id_dual_io(w25qxx_handle_t *handle, uint8_t *manufacturer, uint8_t *device_id);

uint8_t w25qxx_get_manufacturer_device_id_quad_io(w25qxx_handle_t *handle, uint8_t *manufacturer, uint8_t *device_id);

uint8_t w25qxx_get_jedec_id(w25qxx_handle_t *handle, uint8_t *manufacturer, uint8_t device_id[2]);

uint8_t w25qxx_global_block_lock(w25qxx_handle_t *handle);

uint8_t w25qxx_global_block_unlock(w25qxx_handle_t *handle);

uint8_t w25qxx_set_read_parameters(w25qxx_handle_t *handle, w25qxx_qspi_read_dummy_t dummy, w25qxx_qspi_read_wrap_length_t length);

uint8_t w25qxx_enter_qspi_mode(w25qxx_handle_t *handle);

uint8_t w25qxx_exit_qspi_mode(w25qxx_handle_t *handle);

uint8_t w25qxx_enable_reset(w25qxx_handle_t *handle);

uint8_t w25qxx_reset_device(w25qxx_handle_t *handle);

uint8_t w25qxx_get_unique_id(w25qxx_handle_t *handle, uint8_t id[8]);

uint8_t w25qxx_get_sfdp(w25qxx_handle_t *handle, uint8_t sfdp[256]);

uint8_t w25qxx_erase_security_register(w25qxx_handle_t *handle, w25qxx_security_register_t num);

uint8_t w25qxx_program_security_register(w25qxx_handle_t *handle, w25qxx_security_register_t num, uint8_t data[256]);

uint8_t w25qxx_read_security_register(w25qxx_handle_t *handle, w25qxx_security_register_t num, uint8_t data[256]);

uint8_t w25qxx_individual_block_lock(w25qxx_handle_t *handle, uint32_t addr);

uint8_t w25qxx_individual_block_unlock(w25qxx_handle_t *handle, uint32_t addr);

uint8_t w25qxx_read_block_lock(w25qxx_handle_t *handle, uint32_t addr, uint8_t *value);

uint8_t w25qxx_set_burst_with_wrap(w25qxx_handle_t *handle, w25qxx_burst_wrap_t wrap);

uint8_t w25qxx_write_read_reg(w25qxx_handle_t *handle, uint8_t instruction, uint8_t instruction_line,
                              uint32_t address, uint8_t address_line, uint8_t address_len,
                              uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                              uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                              uint8_t *out_buf, uint32_t out_len, uint8_t data_line);

#ifdef __cplusplus
}
#endif

#endif /* IS_BOOTLOADER == 0 */

#endif

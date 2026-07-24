#ifndef APP_BOOT_H_
#define APP_BOOT_H_

#include <stdint.h>
#include "types.h"
#include "err.h"

extern OTA_Info_t ota_info;
extern uint8_t app_a_valid;
extern uint8_t sha256_a[32];
extern uint8_t sha256_b[32];
extern uint32_t time_get_ota_init;

void boot_init(void);
uint8_t boot_is_valid_app(uint32_t app_addr);
void boot_jump_to_app(uint32_t app_addr);
void boot_process(void);
void boot_copy_app_b_to_a(void);
uint32_t boot_crc_flash_infor(const OTA_Info_t *info);
uint32_t boot_crc32_flash(uint32_t start_addr, uint32_t size);
void boot_calculate_sha256(uint32_t flash_addr, uint32_t size, uint8_t hash[32]);

#endif /* APP_BOOT_H_ */

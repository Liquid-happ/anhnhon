#include "rd_flash.h"

#include <stdio.h>

OTA_Info_t ota_info;


uint8_t Flash_Read_U8(uint32_t address)
{
    return *(volatile uint8_t *)address;
}

uint16_t Flash_Read_U16(uint32_t address)
{
    return *(volatile uint16_t *)address;
}

uint32_t Flash_Read_U32(uint32_t address)
{
    return *(volatile uint32_t *)address;
}

void Flash_Read_Buffer(uint32_t address, uint8_t *buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        buffer[i] = *(volatile uint8_t *)(address + i);
    }
}

HAL_StatusTypeDef Flash_Erase_App(uint32_t start_addr,
                                  uint32_t size)
{
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError;

    uint32_t pages = size / FLASH_PAGE_SIZE;
    HAL_IWDG_Refresh(&hiwdg);
    if(size % FLASH_PAGE_SIZE)
        pages++;
    HAL_IWDG_Refresh(&hiwdg);
    if(start_addr % FLASH_PAGE_SIZE != 0)
        return HAL_ERROR;

    HAL_FLASH_Unlock();
    HAL_IWDG_Refresh(&hiwdg);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                           FLASH_FLAG_PGERR |
                           FLASH_FLAG_WRPERR);

    EraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInit.PageAddress = start_addr;
    EraseInit.NbPages     = pages;

    if(HAL_FLASHEx_Erase(&EraseInit,
                         &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    HAL_IWDG_Refresh(&hiwdg);
    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef Flash_WriteBuffer(uint32_t address, uint8_t *data, uint32_t length)
{
    if (address < FLASH_BASE_ADDR || address >= FLASH_END_ADDR)
        return HAL_ERROR;

    if ((address + length) > FLASH_END_ADDR)
        return HAL_ERROR;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < length; i += 2)
    {
        uint16_t halfword;

        if ((i + 1) < length)
        {
            halfword = data[i] | ((uint16_t)data[i + 1] << 8);
        }
        else
        {
            halfword = data[i] | 0xFF00;
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              address + i,
                              halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }

        // Verify lại dữ liệu vừa ghi
        uint16_t read_back = *(volatile uint16_t *)(address + i);

        if (read_back != halfword)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}


uint32_t rd_crc_flash_infor(const OTA_Info_t *info)
{
    const uint8_t *data = (const uint8_t *)info;
    uint32_t crc = 0;

    /*
     * Tính từ đầu struct đến trước trường meta_crc.
     * Yêu cầu: meta_crc nằm cuối struct OTA_Info_t.
     */
    for (uint32_t i = 0; i < offsetof(OTA_Info_t, meta_crc); i++)
    {
        crc += data[i];
    }

    return crc;
}


uint8_t rd_flash_is_valid_app(uint32_t app_addr)
{
    uint32_t stack_addr;
    uint32_t reset_addr;

    stack_addr = *(volatile uint32_t *)app_addr;
    reset_addr = *(volatile uint32_t *)(app_addr + 4);

    /*
     * Flash trống sau erase thường là 0xFFFFFFFF
     */
    if ((stack_addr == 0xFFFFFFFFUL) || (reset_addr == 0xFFFFFFFFUL))
    {
        return 0;
    }

    /*
     * Stack Pointer phải nằm trong RAM STM32F103C8T6
     */
    if ((stack_addr < RAM_START_ADDR) || (stack_addr > RAM_END_ADDR))
    {
        return 0;
    }

    /*
     * Reset_Handler là địa chỉ Thumb, bit 0 có thể bằng 1.
     * Mask bit 0 trước khi so sánh.
     */
    reset_addr &= 0xFFFFFFFEUL;

    /*
     * Reset_Handler phải nằm trong vùng app tương ứng
     */
    if ((reset_addr < app_addr) || (reset_addr >= (app_addr + APP_SLOT_SIZE)))
    {
        return 0;
    }

    return 1;
}


typedef void (*pFunction)(void);

void Bootloader_JumpToApp(uint32_t app_addr)
{
    uint32_t app_stack = *(volatile uint32_t *)app_addr;
    uint32_t app_reset = *(volatile uint32_t *)(app_addr + 4);

    if (rd_flash_is_valid_app(app_addr) == 0)
    {
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id,(uint8_t *)"Invalid app\r\n",  13,  100);
        #endif
        return;
    }
    #if RD_DEBUG
    HAL_UART_Transmit(&uart_debug_id,  (uint8_t *)"Jumping to app\r\n", 16, 100);
    #endif
    HAL_Delay(100);

    HAL_RCC_DeInit();
    HAL_DeInit();
    #if RD_DEBUG
    HAL_UART_DeInit(&uart_debug_id);
    #endif
    HAL_Delay(100);

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    SCB->VTOR = app_addr;
    __set_MSP(app_stack);

    __enable_irq();

    pFunction app_entry = (pFunction)app_reset;
    app_entry();
}

uint8_t app_a_valid = 0;
uint8_t sha256_a[32];
uint8_t sha256_b[32];
void rd_flash_init(void)
{
    uint32_t crc_calc = 0;
    Flash_Read_Buffer(OTA_INFO_ADDR, (uint8_t *)&ota_info,sizeof(OTA_Info_t));

    crc_calc = rd_crc_flash_infor(&ota_info);
    app_a_valid = rd_flash_is_valid_app(APP_A_ADDR);
    if(app_a_valid){
        CalculateSHA256(APP_A_ADDR,ota_info.app_a_size,sha256_a);
    }
    #if (RD_DEBUG && IS_BOOTLOADER)
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Checking OTA info...\r\n",  24, 100);
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"App A valid: ", 13, 100);
    HAL_UART_Transmit(&uart_debug_id, (uint8_t *)(app_a_valid ? "Yes\r\n" : "No\r\n"), 4, 100); 
                    
    #endif
    /*
     * Trường hợp OTA_INFO chưa có hoặc bị lỗi
     */
    if ((ota_info.magic != OTA_INFO_MAGIC) ||(ota_info.meta_crc != crc_calc))
    {
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id,(uint8_t *)"Invalid OTA info, scan app\r\n", 28, 100);
        #endif
        ota_info.magic = OTA_INFO_MAGIC;

        ota_info.app_a_version = 0;
        ota_info.app_a_size    = 0;
        memset(ota_info.app_a_crc, 0, 32); // Set app_a_crc to 0

        ota_info.app_b_version = 0;
        ota_info.app_b_size    = 0;
        memset(ota_info.app_b_crc, 0, 32); // Set app_b_crc to 0

        ota_info.meta_crc = rd_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR,  (uint8_t *)&ota_info, sizeof(OTA_Info_t));
    }
    else
    {
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"OTA info loaded successfully\r\n", 30, 100);
        #endif
    }

    #if RD_DEBUG
    sprintf((char*)temp_debug, "App A version: %u, size: %u\r\n", ota_info.app_a_version, ota_info.app_a_size);
    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
    sprintf((char*)temp_debug, "App B version: %u, size: %u\r\n", ota_info.app_b_version, ota_info.app_b_size);
    HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
		#if  (IS_BOOTLOADER == 0)
			sprintf((char*)temp_debug, "App A Running\r\n");
			HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
		#endif
    #endif

}

uint32_t time_get_ota_init = 0;

//void rd_run_whiletrue_boot(void){
//    uint32_t temp_time_now = HAL_GetTick();
//    if((ota_info.app_b_version != ota_info.app_a_version) || (memcmp(ota_info.app_a_crc, sha256_a, 32) != 0)){ // kiem tra version, neu app b moi hon thi copy sang app a
//        // tinh crc app b, neu dung thi copy sang app a
//        // ham copy sang a
//        // cap nhat ota_info de a = b,
//        // NVIC_SystemReset();
//        #if RD_DEBUG
//        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"start_copy...\r\n", 16, 100);
//        #endif
//        rd_copy_app_b_to_a();
//        #if RD_DEBUG
//        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"New app copied to slot A, restarting...\r\n", 43, 100);
//        #endif
//    }
//    else{
//        // khong ton tai b hoac b khong moi hon a
//    }

//    if (!app_a_valid)
//    {
//        // neu k co A -> chay chuong trinh cho OTA vao B
//        
//    }
//    else{
//        // neu co A -> chờ 1s xem có tác vụ OTA không, sau 1s mà không có thì nhảy vào chạy chương trình a
//        if(((temp_time_now - time_get_ota_init) > 50000) && Ota_data.start_ota == 0){
//            time_get_ota_init = temp_time_now;
//            #if RD_DEBUG
//            HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Running...\r\n", 12, 100);
//            #endif
//            HAL_Delay(100);
//            HAL_IWDG_Refresh(&hiwdg);
//            Bootloader_JumpToApp(APP_A_ADDR);
//        }
//    }
//}
void rd_run_whiletrue_boot(void){
    uint32_t temp_time_now = HAL_GetTick();
    uint8_t need_update = 0;

    /* TƯ DUY MỚI: Chỉ cập nhật khi Slot B thực sự có dữ liệu hợp lệ */
    if (ota_info.app_b_size > 0 && ota_info.app_b_size <= APP_SLOT_SIZE) {
        CalculateSHA256(APP_B_ADDR, ota_info.app_b_size, sha256_b);
        
        // Xác nhận firmware trong Slot B còn nguyên vẹn
        if (memcmp(ota_info.app_b_crc, sha256_b, 32) == 0) {
            
            // Có firmware B xịn. Giờ xem có khác version A hoặc A đang hỏng không?
            if ((ota_info.app_b_version != ota_info.app_a_version) || 
                (memcmp(ota_info.app_a_crc, sha256_a, 32) != 0) || 
                (!app_a_valid)) {
                need_update = 1;
            }
        }
    }

    if (need_update) {
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"start_copy...\r\n", 16, 100);
        #endif
        
        rd_copy_app_b_to_a(); 
        
        // NẾU COPY THÀNH CÔNG, MCU ĐÃ RESET BÊN TRONG rd_copy_app_b_to_a()
        // Nếu chạy đến dòng này, nghĩa là quá trình copy đã bị lỗi hoặc bị từ chối
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Copy failed or rejected!\r\n", 26, 100);
        #endif
        
        // Chống bootloop bằng cách vô hiệu hóa OTA info rác
        ota_info.app_b_size = 0;
        memset(ota_info.app_b_crc, 0, 32);
        ota_info.meta_crc = rd_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR, (uint8_t *)&ota_info, sizeof(OTA_Info_t));
    }

    if (!app_a_valid) {
        // Không có App A, treo ở đây chờ lệnh OTA
        // (Bạn nên thêm một hàm nháy LED hoặc cảnh báo ở đây)
    }
    else {
        // Có App A, chờ 2s xem có lệnh OTA qua UART không
        if(((temp_time_now - time_get_ota_init) > 2000) && Ota_data.start_ota == 0){
            time_get_ota_init = temp_time_now;
            #if RD_DEBUG
            HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Running...\r\n", 12, 100);
            #endif
            HAL_Delay(100);
            HAL_IWDG_Refresh(&hiwdg);
            Bootloader_JumpToApp(APP_A_ADDR);
        }
    }
}

#define CRC32_INIT_VALUE   0xFFFFFFFFUL
#define CRC32_POLY         0xEDB88320UL

uint32_t rd_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ CRC32_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint32_t rd_crc32_flash(uint32_t start_addr, uint32_t size)
{
    uint32_t crc = CRC32_INIT_VALUE;

    if (size == 0)
    {
        return 0;
    }

    if (start_addr < FLASH_BASE_ADDR)
    {
        return 0;
    }

    if ((start_addr + size) > FLASH_END_ADDR)
    {
        return 0;
    }

    for (uint32_t i = 0; i < size; i++)
    {
        uint8_t data;
        data = *(volatile uint8_t *)(start_addr + i);
        crc = rd_crc32_update(crc, &data, 1);
        if ((i % 1024) == 0) HAL_IWDG_Refresh(&hiwdg);
    }

    return ~crc;
}




void rd_copy_app_b_to_a(void){
    // ham copy app b sang a
    CalculateSHA256(APP_B_ADDR,ota_info.app_b_size,sha256_b);
    if(memcmp(ota_info.app_b_crc, sha256_b, 32) == 0){
        // crc dung, copy sang a
        uint8_t buffer[256];
        uint32_t bytes_copied = 0;
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"Start_eraseA", 12, 100);
        #endif
        for(uint32_t i = 0; i < APP_SLOT_SIZE; i += FLASH_PAGE_SIZE){
            HAL_IWDG_Refresh(&hiwdg);
            Flash_Erase_App(APP_A_ADDR + i, FLASH_PAGE_SIZE);
            HAL_IWDG_Refresh(&hiwdg);
        }
        #if RD_DEBUG
        HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"eraseA_done", 12, 100);
        #endif

        while (bytes_copied < ota_info.app_b_size)
        {
            uint32_t chunk_size = (ota_info.app_b_size - bytes_copied) > sizeof(buffer) ? sizeof(buffer) : (ota_info.app_b_size - bytes_copied);

            Flash_Read_Buffer(APP_B_ADDR + bytes_copied, buffer, chunk_size);
            HAL_IWDG_Refresh(&hiwdg);
            Flash_WriteBuffer(APP_A_ADDR + bytes_copied, buffer, chunk_size);
            HAL_IWDG_Refresh(&hiwdg);
            bytes_copied += chunk_size;
        }

        // cap nhat ota_info de app a = app b
        ota_info.app_a_version = ota_info.app_b_version;
        ota_info.app_a_size    = ota_info.app_b_size;
        memcpy(ota_info.app_a_crc, ota_info.app_b_crc, 32);

        ota_info.meta_crc = rd_crc_flash_infor(&ota_info);
        Flash_Erase_App(OTA_INFO_ADDR, OTA_INFO_SIZE);
        Flash_WriteBuffer(OTA_INFO_ADDR,  (uint8_t *)&ota_info, sizeof(OTA_Info_t));
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(100);
        NVIC_SystemReset();
    }
    else{
       // crc sai, khong copy sang a
    }
}


HAL_StatusTypeDef Flash_WritePage(uint32_t address, uint8_t *data)
{
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError;

    if(address % FLASH_PAGE_SIZE != 0)
        return HAL_ERROR;

    HAL_FLASH_Unlock();

    EraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInit.PageAddress = address;
    EraseInit.NbPages     = 1;

    if(HAL_FLASHEx_Erase(&EraseInit, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    for(uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 2)
    {
        uint16_t halfword = data[i] | (data[i + 1] << 8);

        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                             address + i,
                             halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}



void CalculateSHA256(uint32_t flash_addr, uint32_t size, uint8_t hash[32])
{
    SHA256_CTX ctx;

    sha256_init(&ctx);

    while(size)
    {
        uint32_t chunk = (size > 1024) ? 1024 : size;

        sha256_update(&ctx,(const BYTE *)flash_addr,chunk);
        flash_addr += chunk;
        size -= chunk;
				HAL_IWDG_Refresh(&hiwdg);
    }

    sha256_final(&ctx, hash);
}
#include "bms_state.h"
#include "bq_bms_485.h"
#include "main.h"
#include "led_flash.h"
#include "dwin.h"
#include "string.h"
#include <stdlib.h>

BMS_State_t  bms_state           	= BMS_STATE_SLEEP;
BMS_Alarms_t bms_alarms         	= {0};
BuzzerMode_t current_buzzer_mode 	= BUZZER_MODE_OFF;
uint16_t alarms_bitmask         	= 0;
uint8_t bms_sleep_mode          	= 0;
uint8_t bms_is_resetting        	= 0;
uint8_t reset_step              	= 0;
uint8_t buzzer_enabled 					  = 1;

static uint32_t last_update_tick          	= 0;
static uint32_t idle_timer_ms             	= 0;
static uint32_t low_pack_voltage_timer_ms  	= 0;
static uint32_t uv_protection_timer_ms    	= 0;
static uint32_t current_hyst_timer_ms     	= 0;
static uint32_t buzzer_next_toggle        	= 0;
static uint8_t  buzzer_is_on              	= 0;
static uint8_t  remote_reset_step 					= 0;
static uint32_t remote_step_tick 						= 0;

void Update_BMS_State(void)
{
//      Monitor_Precharge_Status();
    uint32_t current_tick_1 = HAL_GetTick();
    uint32_t delta_ms = (last_update_tick == 0) ? 0 : (current_tick_1 - last_update_tick);
    last_update_tick = current_tick_1;
    uint8_t is_permanent_fail = (value_PFStatusA || value_PFStatusB || value_PFStatusC || value_PFStatusD);
    uint8_t is_safety_protection = (value_SafetyStatusA || value_SafetyStatusB || value_SafetyStatusC );
    uint8_t afe_sleep_bit = (battery_status_sleep >> 15) & 0x01;
    int16_t current_ma = Pack_Current;
    uint16_t pack_voltage_mv = Stack_Voltage;
    uint16_t ld_voltage_mv  = LD_Voltage;
    uint8_t raw_current_activity = (current_ma > 50 || current_ma < -50);
    uint8_t low_pack_voltage_cond = (sys_min_cell_v < 3150);
    uint8_t wake_trigger = (pack_voltage_mv > 44800) || raw_current_activity || (ld_voltage_mv > 43200);

    if (raw_current_activity || wake_trigger || is_permanent_fail || is_safety_protection)
    {
        idle_timer_ms = 0;
        low_pack_voltage_timer_ms = 0;
        uv_protection_timer_ms = 0;
        current_hyst_timer_ms = 1000UL;
    }
    else
    {
        if (idle_timer_ms < 86400000UL * 2)
        {
            idle_timer_ms += delta_ms;
        }
    }

    if (low_pack_voltage_cond && !is_permanent_fail && !is_safety_protection)
    {
        if (low_pack_voltage_timer_ms < 3600000UL)
        {
            low_pack_voltage_timer_ms += delta_ms;
        }
    }
    else
    {
        low_pack_voltage_timer_ms = 0;
    }

    if (UV_Fault)
    {
        if (uv_protection_timer_ms < 600000UL)
        {
            uv_protection_timer_ms += delta_ms;
        }
    }
    else
    {
        uv_protection_timer_ms = 0;
    }
        
    if (raw_current_activity)
    {
        current_hyst_timer_ms = 1000UL;
    }
    else if (current_hyst_timer_ms > delta_ms)
    {
        current_hyst_timer_ms -= delta_ms;
    }
    else
    {
        current_hyst_timer_ms = 0;
    }
		uint8_t has_critical_fault = (is_permanent_fail || cell_failure_locked || SCD_Fault || SCDL_Fault || HWDF_Fault);
		uint8_t has_discharge_protection = (UV_Fault || OCD_Fault || OCD_Fault1 || OTD_Fault || UTD_Fault || OTF_Fault || OTINT_Fault || UTINT_Fault);
    if (has_critical_fault) {
        bms_state = BMS_STATE_FAULT;
    }
    else if (Pack_Current < -50 || has_discharge_protection) { 
        bms_state = BMS_STATE_DISCHARGING;
    }
		else if (Pack_Current > 50 || OV_Fault) { 
        bms_state = BMS_STATE_CHARGING;
    }
    else {
        uint8_t can_sleep = (afe_sleep_bit && !has_critical_fault && !is_safety_protection && !wake_trigger && idle_timer_ms >= 86400000UL);
       
        if (can_sleep) {
            bms_state = BMS_STATE_SLEEP;
        } else {
            bms_state = BMS_STATE_STANDBY;
        }
    }
    if (bms_state == BMS_STATE_SLEEP && wake_trigger)
    {
        bms_state = BMS_STATE_STANDBY;
    }
}

void Check_Reset_Button(void)
{
    static uint32_t press_start_time = 0;          
    static uint32_t step_start_tick = 0;
    static uint8_t reset_triggered = 0;
    uint32_t current_tick = HAL_GetTick();
    uint8_t key_pressed = (HAL_GPIO_ReadPin(RST_SWITCH_GPIO_Port, RST_SWITCH_Pin) == GPIO_PIN_RESET);
    if (reset_step == 0)
    {
        if (key_pressed) 
        {
            if (press_start_time == 0) 
            {
                press_start_time = current_tick;
                reset_triggered = 0;
            } 
            else 
            {
                uint32_t duration = current_tick - press_start_time;
                if (duration >= 5000UL && reset_triggered == 0) 
                {
                    reset_triggered = 1;
                    LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_ON);
                    LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_ON);
                    reset_step = 1;             
                    step_start_tick = current_tick;
                    bms_is_resetting = 1;
                }
            }
        } 
        else 
        {
            press_start_time = 0;
            reset_triggered = 0;			
        }
    }
    if (reset_step > 0) {
        bms_is_resetting = 1;
        uint32_t elapsed = current_tick - step_start_tick;
        switch (reset_step) {
            case 1:
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
                CommandSubcommands(ALL_FETS_OFF);
                step_start_tick = current_tick;
                reset_step = 2;
                break;
            case 2:
                if (elapsed >= 50) {
                    BMS_Clear_All_Buffers();
                    last_valid_stack_voltage = 0;
                    last_valid_pack_voltage = 0;
                    last_valid_ld_voltage = 0;
                    last_valid_current = 0;
                    last_valid_soc = 100.0f;
                    first_sample_after_reset = 1;
                    avg_index = 0;
                    avg_index_current = 0;
                    pf_active_latched = 0;
                    pf_ui_already_updated = 0;
                    step_start_tick = current_tick;
                    reset_step = 3;
                }
                break;
            case 3:
                if (elapsed >= 50) { 
                    HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_SET);
                    step_start_tick = current_tick;
                    reset_step = 4;
                }
                break;
            case 4: 
                if (elapsed >= 200) {
                    HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_RESET);
                    step_start_tick = current_tick;
                    reset_step = 5;
                }
                break;
            case 5: 
                if (elapsed >= 100) {
                    CommandSubcommands(BQ769x2_RESET);
                    step_start_tick = current_tick;
                    reset_step = 6;
                }
                break;
            case 6:
                if (elapsed >= 100) {
                    BQ769x2_Init();
                    CommandSubcommands(SLEEP_DISABLE); 
                    step_start_tick = current_tick;
                    reset_step = 7;
                }
                break;
            case 7: 
                if (elapsed >= 500) {
                    first_sample_after_reset = 1;
                    if (Stack_Voltage > 10000)
                    {
                        if (flash_init_success) 
                        {
                            BMS_Sync_Flash_To_State(); 
                        }
                        BQ769x2_ReadAllVoltages();
                        Pack_Current = BQ769x2_ReadCurrent();		
                        Update_SOC_SOH_FromBQ();
                        last_valid_stack_voltage = Stack_Voltage;
                        last_valid_pack_voltage  = Pack_Voltage;
                        last_valid_ld_voltage    = LD_Voltage;
                        last_valid_current       = Pack_Current;
                        last_valid_soc           = SOC;
                        BQ769x2_ClearLatchedAlerts();                   
                        occ_software_lock = 0;  
                        occ_streak_count = 0;   
                        prev_occ_bit = 0;      
                        cell_failure_locked = 0;
                        cell_failure_count = 0;
                        ov_recovery_locked = 0;
                        uv_recovery_locked = 0;
                        protection_blocked = 0;
                        last_lock_state = 0;
												BMS_Init_Address();
                        dwin_need_update = 1;
                        first_sample_after_reset = 0;
                        HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
                        reset_step = 0;
                        bms_is_resetting = 0;
                    }
                    else {
                        if (elapsed > 3000) { 
                            first_sample_after_reset = 0; 
                            reset_step = 0;
                            bms_is_resetting = 0;
                        }
                    }
                }									
                break;
            default:
                reset_step = 0;  
                break;
        }
    }
}

void Update_Buzzer_Logic(void)
{
    if (bms_sleep_mode || !buzzer_enabled) { 
        current_buzzer_mode = BUZZER_MODE_OFF;
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_is_on = 0;
        return; 
    }
		uint8_t has_critical_fault = PFErrorsTriggered || cell_failure_locked;
		uint8_t has_other_protection = (UV_Fault || OCC_Fault || OCD_Fault || OCD_Fault1 || 
                                    SCD_Fault || OTC_Fault || OTD_Fault || OTF_Fault || 
                                    UTC_Fault || UTD_Fault || UTINT_Fault || OTINT_Fault || 
                                    HWDF_Fault || PTO_Fault || OCDL_Fault || SCDL_Fault || OCD3_Fault);
		uint16_t other_alarms = alarms_bitmask & ~((1 << 0) | (1 << 2));
    if (has_critical_fault) {
        current_buzzer_mode = BUZZER_MODE_FAULT;
    }
    else if (has_other_protection) {
        current_buzzer_mode = BUZZER_MODE_PROTECTION;
    }
    else if (other_alarms != 0) {
        current_buzzer_mode = BUZZER_MODE_ALARM;
    }
    else {
        current_buzzer_mode = BUZZER_MODE_OFF;
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_is_on = 0;
    }
}

void Buzzer_Task_1ms(void)
{
    if (current_buzzer_mode == BUZZER_MODE_OFF) return;
    uint32_t now = HAL_GetTick();
    if (now < buzzer_next_toggle) return;
    uint32_t on_time 	= 250;  
    uint32_t off_time = 750;
    switch (current_buzzer_mode) {
        case BUZZER_MODE_FAULT:      off_time = 750;  break;
        case BUZZER_MODE_PROTECTION: off_time = 1750; break;
        case BUZZER_MODE_ALARM:      off_time = 250; break;
        default: return;
    }
    if (buzzer_is_on) 
    {
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_is_on = 0;
        buzzer_next_toggle = now + off_time;
    } else 
    {
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);
        buzzer_is_on = 1;
        buzzer_next_toggle = now + on_time;
    }
}

void Update_Alarms(void)
{
    extern uint8_t alarms_enabled;          
    bms_alarms = (BMS_Alarms_t){0};
		
    if (!alarms_enabled)
    {
        alarms_bitmask = 0;
        return;
    }
		
    uint16_t batt_stat = BQ769x2_ReadBatteryStatus();
    uint8_t is_pf_active = (batt_stat & (1u << 12)) != 0;
		
    if (is_pf_active) {
        bms_state = BMS_STATE_FAULT;
    }
		
    for (int i = 0; i < 16; i++)
    {
        if (CellVoltage[i] >= 3600)
        {
            bms_alarms.OV_Alarm = 1;
        }
				if (CellVoltage[i] <= 2800)
        {
            bms_alarms.UV_Alarm = 1;
        }
    }
		
    if (Stack_Voltage >= 57600)
    {
        bms_alarms.Stack_OV_Alarm = 1;
    }
		
    if (Stack_Voltage <= 44800)
    {
        bms_alarms.Stack_UV_Alarm = 1;
    }

    if (Pack_Current >= 12000 && bms_state == BMS_STATE_CHARGING)  
    {
        bms_alarms.OCC_Alarm = 1;
    }
		
    if (Pack_Current <= -12000 && bms_state == BMS_STATE_DISCHARGING) 
    {
        bms_alarms.OCD1_Alarm = 1;
    }

    float cell_temp = Temperature[0];
    float min_temp  = Temperature[0];
    for (int i = 1; i < 4; i++)
    {
        if (Temperature[i] > cell_temp) cell_temp = Temperature[i];
        if (Temperature[i] < min_temp)  min_temp  = Temperature[i];
    }
    if (ntc_1 > cell_temp) cell_temp = ntc_1;
    if (ntc_1 < min_temp)  min_temp  = ntc_1;
    if (ntc_2 > cell_temp) cell_temp = ntc_2;
    if (ntc_2 < min_temp)  min_temp  = ntc_2;
		
    float env_temp_local = Temperature[6];
		float fet_temp = ntc_temp;
		
    if (cell_temp >= 60 && Pack_Current > 50)
    {
        bms_alarms.OTC_Alarm = 1;
    }
		
    if (cell_temp >= 65 && Pack_Current <= -50)
    {
        bms_alarms.OTD_Alarm = 1;
    }
		
    if (min_temp <= 5 && Pack_Current > 50)
    {
        bms_alarms.UTC_Alarm = 1;
    }
		
    if (min_temp <= -15 && Pack_Current <= -50)
    {
        bms_alarms.UTD_Alarm = 1;
    }
		
    if (env_temp_local >= 65)
    {
        bms_alarms.EOT_Alarm = 1;
    }
		
    if (env_temp_local <= -15)
    {
        bms_alarms.EUT_Alarm = 1;
    }

    if (fet_temp >= 90)
    {
        bms_alarms.OTF_Alarm = 1;
    }

    if (SOC < 5.0f && bms_state != BMS_STATE_CHARGING)
    {
        bms_alarms.Low_Battery_Alarm = 1;
    }
		
    uint16_t max_cell = 0, min_cell = 65535;
    for (int i = 0; i < 16; i++)
    {
        if (CellVoltage[i] > max_cell)
        {
            max_cell = CellVoltage[i];
        }
        if (CellVoltage[i] < min_cell)
        {
            min_cell = CellVoltage[i];
        }
    }
    if (max_cell - min_cell > 1000)
    {
        bms_alarms.Cell_Failure_Alarm = 1;
    }
    alarms_bitmask = 0;
    alarms_bitmask |= (bms_alarms.OV_Alarm            << 0);
    alarms_bitmask |= (bms_alarms.UV_Alarm            << 1);
    alarms_bitmask |= (bms_alarms.Stack_OV_Alarm      << 2);
    alarms_bitmask |= (bms_alarms.Stack_UV_Alarm      << 3);
    alarms_bitmask |= (bms_alarms.OCC_Alarm           << 4);
    alarms_bitmask |= (bms_alarms.OCD1_Alarm          << 5);
    alarms_bitmask |= (bms_alarms.OTC_Alarm           << 6);
    alarms_bitmask |= (bms_alarms.OTD_Alarm           << 7);
    alarms_bitmask |= (bms_alarms.UTC_Alarm           << 8);
    alarms_bitmask |= (bms_alarms.UTD_Alarm           << 9);
    alarms_bitmask |= (bms_alarms.EOT_Alarm           << 10);
    alarms_bitmask |= (bms_alarms.EUT_Alarm           << 11);
    alarms_bitmask |= (bms_alarms.OTF_Alarm           << 12);
    alarms_bitmask |= (bms_alarms.Low_Battery_Alarm   << 13);
    alarms_bitmask |= (bms_alarms.Cell_Failure_Alarm  << 14);
}

void Update_DryContacts(void)
{
    if (bms_state == BMS_STATE_FAULT || ProtectionsTriggered) {
        HAL_GPIO_WritePin(GPIOB, DRY_CONTACT_FAULT_Pin, GPIO_PIN_SET);   
    } else {
        HAL_GPIO_WritePin(GPIOB, DRY_CONTACT_FAULT_Pin, GPIO_PIN_RESET); 
    }
    if (bms_alarms.Low_Battery_Alarm) {  
        HAL_GPIO_WritePin(GPIOB, DRY_CONTACT_LOWBAT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, DRY_CONTACT_LOWBAT_Pin, GPIO_PIN_RESET);
    }
}

uint8_t BMS_IsResetting(void) {
    return bms_is_resetting;
}

BMS_State_t Get_BMS_State(void)
{
    return bms_state;
}

void Trigger_Remote_Reset_Task(uint8_t type) {
		(void)type;
    if (remote_reset_step == 0 && bms_is_resetting == 0) {
//        remote_reset_cmd_type = type;
        remote_reset_step = 1;
        remote_step_tick = HAL_GetTick();
        bms_is_resetting = 1; 
        LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_ON);
        LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_ON);
        LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_ON);
        LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_ON);
        LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_ON);
        LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_ON);
        LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_ON);
        LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_ON);
        LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_ON);
    }
}

void Process_Remote_Reset_Task(void) {
    if (remote_reset_step == 0) return;  
    bms_is_resetting = 1;
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed = current_tick - remote_step_tick;
    switch (remote_reset_step) {
        case 1:
            HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
            CommandSubcommands(ALL_FETS_OFF);
            remote_step_tick = current_tick;
            remote_reset_step = 2;
            break;          
        case 2:
            if (elapsed >= 50) {
                BMS_Clear_All_Buffers();
                last_valid_stack_voltage = 0;
                last_valid_pack_voltage = 0;
                last_valid_ld_voltage = 0;
                last_valid_current = 0;
                last_valid_soc = 100.0f;
                first_sample_after_reset = 1;
                avg_index = 0;
                avg_index_current = 0;
                pf_active_latched = 0;
                pf_ui_already_updated = 0;             
                remote_step_tick = current_tick;
                remote_reset_step = 3;
            }
            break;           
        case 3:
            if (elapsed >= 50) { 
                HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_SET);
                remote_step_tick = current_tick;
                remote_reset_step = 4;
            }
            break;           
        case 4:
            if (elapsed >= 200) {
                HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_RESET);
                remote_step_tick = current_tick;
                remote_reset_step = 5;
            }
            break;            
        case 5:
            if (elapsed >= 100) {
                CommandSubcommands(BQ769x2_RESET);
                remote_step_tick = current_tick;
                remote_reset_step = 6;
            }
            break;           
        case 6:
            if (elapsed >= 100) {
                BQ769x2_Init();
                CommandSubcommands(SLEEP_DISABLE); 
                remote_step_tick = current_tick;
                remote_reset_step = 7;
            }
            break;          
        case 7:
            if (elapsed >= 500) {
                first_sample_after_reset = 1;
                if (Stack_Voltage > 10000) {
                    if (flash_init_success) {
                        BMS_Sync_Flash_To_State(); 
                    }                
                    BQ769x2_ReadAllVoltages();
                    Pack_Current = BQ769x2_ReadCurrent();      
                    Update_SOC_SOH_FromBQ();                 
                    last_valid_stack_voltage = Stack_Voltage;
                    last_valid_pack_voltage  = Pack_Voltage;
                    last_valid_ld_voltage    = LD_Voltage;
                    last_valid_current       = Pack_Current;
                    last_valid_soc           = SOC;            
                    BQ769x2_ClearLatchedAlerts();                              
                    occ_software_lock = 0;  
                    occ_streak_count = 0;   
                    prev_occ_bit = 0;      
                    cell_failure_locked = 0;
                    cell_failure_count = 0;
                    ov_recovery_locked = 0;
                    uv_recovery_locked = 0;
                    protection_blocked = 0;
                    last_lock_state = 0;                    
                    BMS_Init_Address();
                    dwin_need_update = 1;
                    first_sample_after_reset = 0;                    
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
//                    if (remote_reset_cmd_type == 0x03 || remote_reset_cmd_type == 0x01) {
//                        CommandSubcommands(ALL_FETS_ON);
//                    } 
//                    else if (remote_reset_cmd_type == 0x02) {
//												CommandSubcommands(ALL_FETS_ON);
//												Safe_Delay_ms(20);
//												HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_SET);
//												Safe_Delay_ms(10);
//												CommandSubcommands(CHG_PCHG_OFF);												
//												slave_is_in_precharge_mode = 1; 
//												precharge_start_tick = HAL_GetTick();
//										}                                    
                    remote_reset_step = 0;
                    bms_is_resetting = 0;
                }
                else {
                    if (elapsed > 3000) { 
                        first_sample_after_reset = 0; 
                        remote_reset_step = 0;
                        bms_is_resetting = 0;
                    }
                }
            }                                   
            break;           
        default:
            remote_reset_step = 0;
            bms_is_resetting = 0;
            break;
    } 
}

//void Monitor_Precharge_Status(void) {
//    if (!slave_is_in_precharge_mode) return;
//    uint32_t elapsed = HAL_GetTick() - precharge_start_tick;
//    int32_t diff = abs((int32_t)master_analog_data.total_voltage - (int32_t)Stack_Voltage);
//    if (diff < 1000 || elapsed > 18000000) {
//        CommandSubcommands(ALL_FETS_ON);
//        HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);      
//        HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
//        Safe_Delay_ms(50);
//        HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
//        slave_is_in_precharge_mode = 0;
//    }
//}

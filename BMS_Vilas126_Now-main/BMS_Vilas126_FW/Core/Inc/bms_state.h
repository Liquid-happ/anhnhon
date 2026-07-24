#ifndef BMS_STATE_H
#define BMS_STATE_H

#include "../rd_ota/rd_control.h"

#if IS_BOOTLOADER == 0

#include "main.h"

#define AVERAGE_SAMPLES 						64
#define AVERAGE_SAMPLES_CURRENT			16
#define AVERAGE_SAMPLES_ADC 				16
#define MAX_SLAVES 									3
/* --- Enumerations --- */
typedef enum {
    BMS_STATE_SLEEP = 0,
    BMS_STATE_STANDBY,
    BMS_STATE_CHARGING,
    BMS_STATE_DISCHARGING,
    BMS_STATE_FAULT
} BMS_State_t;

typedef enum {
    BUZZER_MODE_OFF = 0,
    BUZZER_MODE_FAULT,      		 // 0.25s on - 0.75s off
    BUZZER_MODE_PROTECTION, 		 // 0.25s on - 1.75s off
    BUZZER_MODE_ALARM       		 // 0.25s on - 2.75s off
} BuzzerMode_t;

/* --- Structures --- */
typedef struct {
    uint8_t OV_Alarm;            // Cell Over Voltage 		: >= 3600mV(warning)
    uint8_t UV_Alarm;            // Cell Under Voltage		: <= 2800mV(warning)
		uint8_t Stack_OV_Alarm;      // Pack									: >= 57.6V
		uint8_t Stack_UV_Alarm;      // Pack									: <= 44.8V
    uint8_t OCC_Alarm;           // Over Current Charge		: >= 120A
    uint8_t OCD1_Alarm;          // Over Current Discharge: >= 120A
    uint8_t OTC_Alarm;           // Over Temp Charge			: >= 60°C 	&& Pack_Current > 50
    uint8_t OTD_Alarm;           // Over Temp Discharge		: >= 65°C 	&& Pack_Current <= -50
    uint8_t UTC_Alarm;           // Under Temp Charge			: <= 5°C 		&& Pack_Current > 50
    uint8_t UTD_Alarm;           // Under Temp Discharge	: <= -15°C	&& Pack_Current <= -50
    uint8_t EOT_Alarm;           // Environment Over Temp	: >= 65°C
    uint8_t EUT_Alarm;           // Environment Under Temp: <= -15°C
    uint8_t OTF_Alarm;           // FET Over Temp					: >= 90°C
    uint8_t Low_Battery_Alarm;   // SOC < 5%
    uint8_t Cell_Failure_Alarm;  // Delta V between cells > 1000mV (1V)
} BMS_Alarms_t;

extern BMS_State_t bms_state;
extern BMS_Alarms_t bms_alarms;
extern BuzzerMode_t current_buzzer_mode;
extern uint16_t alarms_bitmask;
extern uint8_t bms_sleep_mode;
extern uint8_t bms_is_resetting;
extern uint8_t reset_step;

extern uint16_t CellVoltage[16];
extern int16_t  Pack_Current;
extern uint16_t Stack_Voltage;
extern uint16_t Pack_Voltage;
extern uint16_t LD_Voltage;
extern float    SOC;
extern float    ntc_temp;
extern float    ntc_1;
extern float    ntc_2;
extern float    Temperature[9];
extern uint16_t sys_min_cell_v;
extern uint16_t battery_status_sleep;

extern uint8_t  first_sample_after_reset;
extern uint8_t  dwin_need_update;
extern uint8_t  cell_failure_locked;
extern uint8_t  flash_init_success;

/* --- Function Prototypes --- */
void Update_BMS_State(void);
void Check_Reset_Button(void);
void Update_Alarms(void);
void Update_Buzzer_Logic(void);
void Buzzer_Task_1ms(void);
void Update_DryContacts(void);
void Trigger_Remote_Reset_Task(uint8_t type);
void Process_Remote_Reset_Task(void);
void Monitor_Precharge_Status(void);
uint8_t BMS_IsResetting(void);
BMS_State_t Get_BMS_State(void);

extern void BQ769x2_ReadPassQ(void);
extern void Safe_Delay_ms(uint32_t ms);
extern void BMS_Integrated_Power_Management(void);
extern void BMS_Sync_Flash_To_State(void);
extern void BMS_Init_Address(void);
extern IWDG_HandleTypeDef hiwdg;
extern uint16_t last_valid_stack_voltage;
extern uint16_t last_valid_pack_voltage;
extern uint16_t last_valid_ld_voltage;
extern int16_t  last_valid_current;
extern float    last_valid_soc;

extern uint8_t  occ_software_lock;
extern uint8_t  occ_streak_count;
extern uint8_t  prev_occ_bit;
extern uint8_t  cell_failure_count;
extern uint8_t  ov_recovery_locked;
extern uint8_t  uv_recovery_locked;
extern uint8_t  protection_blocked;
extern uint8_t  pf_active_latched;

extern uint8_t  is_master;
extern uint8_t  is_auto_coding;
extern uint8_t  active_packs_count;
extern uint8_t  slave_online_status[MAX_SLAVES];

extern uint16_t    last_soc_x100;
extern uint16_t    last_soh_x100;
extern int16_t     last_current;
extern uint16_t    last_stack_v;
extern uint16_t 	 cell_buffer[16][AVERAGE_SAMPLES];
extern uint16_t 	 pack_buffer[AVERAGE_SAMPLES];
extern uint16_t 	 stack_buffer[AVERAGE_SAMPLES];
extern uint16_t 	 ld_buffer[AVERAGE_SAMPLES];
extern int16_t   	 current_buffer[AVERAGE_SAMPLES_CURRENT];
extern uint16_t 	 adc_buffer[4][AVERAGE_SAMPLES_ADC];
extern uint8_t  	 avg_index;
extern uint8_t 		 avg_index_current;
extern uint8_t 		 pf_ui_already_updated;
extern uint32_t 	 sleep_timer_start;
extern uint32_t 	 mos_ot_lock_tick;
extern uint8_t 		 last_lock_state; 

#endif /* IS_BOOTLOADER == 0 */
#endif /* BMS_STATE_H */

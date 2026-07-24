#ifndef SVC_INV_H_
#define SVC_INV_H_

#include "Cfg/feat_cfg.h"

#if IS_BOOTLOADER == 0

#include <stdint.h>

extern uint8_t full_charge_request;
extern uint8_t force_charge;
extern uint8_t current_pf_status[4];
extern uint8_t bms_device_address;
extern uint8_t is_master;
extern uint16_t system_charge_limit_A;
extern uint16_t system_discharge_limit_A;
extern uint16_t system_charge_limit_A_final;
extern uint16_t system_discharge_limit_A_final;
extern uint16_t system_charge_v_limit_mV;
extern uint16_t system_discharge_v_limit_mV;
extern uint16_t system_charge_v_limit_final;
extern uint16_t system_discharge_v_limit_final;
extern int32_t system_total_current_mA;
extern int32_t system_total_current_01A;
extern uint16_t system_total_voltage_01V;
extern uint16_t system_avg_soc;          
extern uint16_t system_avg_soh;          
extern uint8_t active_packs_count;       
extern int16_t system_max_temp_C;
extern int16_t system_min_temp_C;         
extern uint16_t sys_min_cell_v;       
extern uint16_t sys_max_cell_v;
extern uint8_t sys_max_v_pack_id;
extern uint8_t sys_max_v_cell_id;
extern uint8_t sys_min_v_pack_id;
extern uint8_t sys_min_v_cell_id;
extern uint8_t sys_max_t_pack_id;
extern uint8_t sys_max_t_cell_id;
extern uint8_t sys_min_t_pack_id;
extern uint8_t sys_min_t_cell_id;
extern uint32_t fet_fail_timer;           
extern uint32_t last_inverter_alive_tick; 
extern uint8_t slave_isolated[];         
extern uint8_t PFErrorsTriggered; 
extern uint8_t inverter_comm_fault;
extern uint8_t uv_recovery_locked;
extern uint8_t ov_recovery_locked;
extern uint8_t sys_protect_ov;
extern uint8_t sys_protect_uv;
extern uint8_t sys_protect_occ;
extern uint8_t sys_protect_ocd;
extern uint8_t sys_protect_ot;
extern uint8_t sys_protect_ut;
extern uint8_t sys_alarm_ov;
extern uint8_t sys_alarm_uv;
extern uint8_t sys_alarm_occ;
extern uint8_t sys_alarm_ocd;
extern uint8_t sys_alarm_ot;
extern uint8_t sys_alarm_ut;
extern uint8_t sys_system_error;
extern uint32_t system_total_capacity_mAh;

// Prototypes for CAN inverter service
void Inv_Init(void);
void Inv_Transmit_Status(void);
void Inv_Handle_Rx_Frame(uint32_t std_id, uint32_t ext_id, uint8_t is_ext, uint8_t *data, uint8_t len);

#endif /* IS_BOOTLOADER == 0 */
#endif /* SVC_INV_H_ */

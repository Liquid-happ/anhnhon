#include "led_indication.h"
#include "led_flash.h"
#include "bms_state.h"
#include "bq_bms_485.h"
#include "main.h"

void Update_LED_Indication(void)
{
    if (bms_sleep_mode) 
    {
        LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_OFF);
        return; 
    }
    
    Update_BMS_State();
    Update_Alarms();
    uint8_t has_alarm = (bms_alarms.UV_Alarm  || bms_alarms.Stack_UV_Alarm ||
                         bms_alarms.OCC_Alarm || bms_alarms.OCD1_Alarm || bms_alarms.OTC_Alarm      || bms_alarms.OTD_Alarm ||
                         bms_alarms.UTC_Alarm || bms_alarms.UTD_Alarm  || bms_alarms.EOT_Alarm      || bms_alarms.EUT_Alarm ||
                         bms_alarms.OTF_Alarm || bms_alarms.Low_Battery_Alarm || bms_alarms.Cell_Failure_Alarm);
    uint8_t overcharge_prot      = (OV_Fault || COVL_Fault);  
    uint8_t underdischarge_prot  = (UV_Fault);
    uint8_t has_fault_charge     = (OTC_Fault || UTC_Fault || OTF_Fault || OCC_Fault  || OTINT_Fault || UTINT_Fault);
    uint8_t has_fault_discharge  = (OTD_Fault || UTD_Fault || OTF_Fault || OCD_Fault || OCD_Fault1 || OTINT_Fault  || UTINT_Fault || OCDL_Fault);   
    uint8_t soc_leds_should_off  = 0;
    uint8_t soc_leds_should_on   = 0; 

    // ================== LED9 (ON/OFF) + LED8 (RUN) + LED7 (ALM) ==================
    switch (bms_state)
    {
        case BMS_STATE_SLEEP:
            LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_OFF);
            LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
            LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
            soc_leds_should_off = 1;
            break;

        case BMS_STATE_FAULT:
            LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_OFF);
            LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
            LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_ON);
            soc_leds_should_off = 1;
            break;

        case BMS_STATE_STANDBY:
            LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_ON);
            LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_1);
            if (has_alarm) {
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_3);
            } else {
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
            }
            break;

        case BMS_STATE_CHARGING:
            LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_ON);
            if (has_fault_charge) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_ON);
                soc_leds_should_off = 1;
            }
            else if (overcharge_prot) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_ON);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
                soc_leds_should_on = 1;  
            }
            else if (has_alarm) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_ON);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_3);
            }
            else {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_ON);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
            }
            break;

        case BMS_STATE_DISCHARGING:
            LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_ON);
            if (has_fault_discharge) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_ON);
                soc_leds_should_off = 1;
            }
            else if (underdischarge_prot) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
                soc_leds_should_off = 1;
            }
            else if (has_alarm) {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_3);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_3);
            }
            else {
                LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_3);
                LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
            }
            break;

        default:
            soc_leds_should_off = 1;
            break;
    }

    if (soc_leds_should_off) {
        LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_OFF);
        LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_OFF);
    }
    else if (soc_leds_should_on) {
        LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_ON);
        LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_ON);
        LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_ON);
        LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_ON);
        LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_ON);
        LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_ON);
    }
    else {
        uint8_t soc_level = 0;
        if      (SOC > 83.0f) soc_level = 6;
        else if (SOC > 66.4f) soc_level = 5;
        else if (SOC > 49.8f) soc_level = 4;
        else if (SOC > 33.2f) soc_level = 3;
        else if (SOC > 16.6f) soc_level = 2;
        else                  soc_level = 1;

        if (bms_state == BMS_STATE_CHARGING) {
            LED_SetMode(LED1_GPIO_Port, LED1_Pin, (soc_level == 1) ? FLASH_MODE_2 : FLASH_MODE_OFF);
            if (soc_level > 1) LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_ON);

            LED_SetMode(LED2_GPIO_Port, LED2_Pin, (soc_level == 2) ? FLASH_MODE_2 : FLASH_MODE_OFF);
            if (soc_level > 2) LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_ON);

            LED_SetMode(LED3_GPIO_Port, LED3_Pin, (soc_level == 3) ? FLASH_MODE_2 : FLASH_MODE_OFF);
            if (soc_level > 3) LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_ON);

            LED_SetMode(LED4_GPIO_Port, LED4_Pin, (soc_level == 4) ? FLASH_MODE_2 : FLASH_MODE_OFF);
            if (soc_level > 4) LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_ON);

            LED_SetMode(LED5_GPIO_Port, LED5_Pin, (soc_level == 5) ? FLASH_MODE_2 : FLASH_MODE_OFF);
            if (soc_level > 5) LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_ON);

            LED_SetMode(LED6_GPIO_Port, LED6_Pin, (soc_level == 6) ? FLASH_MODE_2 : FLASH_MODE_OFF);
        }
				
        else {
            if (soc_level >= 1) LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_ON); else LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_OFF);
            if (soc_level >= 2) LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_ON); else LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_OFF);
            if (soc_level >= 3) LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_ON); else LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_OFF);
            if (soc_level >= 4) LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_ON); else LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_OFF);
            if (soc_level >= 5) LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_ON); else LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_OFF);
            if (soc_level >= 6) LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_ON); else LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_OFF);
        }
    }
}

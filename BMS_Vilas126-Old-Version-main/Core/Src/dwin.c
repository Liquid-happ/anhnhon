#include "dwin.h"
#include "main.h"
#include <math.h>
#include "bms_state.h"
#include "bq_bms_485.h"

#define ALARM_ON_ICON    14
#define ALARM_OFF_ICON   15
#define ALARM_ICON(alarm) ((alarm) ? ALARM_ON_ICON : ALARM_OFF_ICON)
#define PROTECT_ON_ICON  18
#define PROTECT_OFF_ICON 19
#define PROTECT_ICON(x)   ((x) ? PROTECT_ON_ICON : PROTECT_OFF_ICON)
#define MAX_SLAVES 			 3

static void DWIN_SetSlaveTouch_Page1 (uint8_t touch_num, uint8_t enable);
static void DWIN_SetSlaveTouch_Page14(uint8_t touch_num, uint8_t enable);
static void DWIN_SetSlaveTouch_Page21(uint8_t touch_num, uint8_t enable);
static void DWIN_SetSlaveTouch_Page24(uint8_t touch_num, uint8_t enable);

extern void BQ769x2_ReadPFStatus(void);
extern uint8_t SUV_PF_Fault;
extern uint8_t SOV_PF_Fault;
extern uint8_t SOCC_PF_Fault;
extern uint8_t SOCD_PF_Fault;
extern uint8_t SOT_PF_Fault;
extern uint8_t SOTF_PF_Fault;
extern uint8_t IRMF_PF_Fault;
extern uint8_t LFOF_PF_Fault;
extern uint8_t VREF_PF_Fault;
extern uint8_t VSSF_PF_Fault;
extern uint8_t CUDEP_PF_Fault;
extern uint8_t HWMX_PF_Fault;
extern uint8_t CFETF_PF_Fault;
extern uint8_t CMDF_PF_Fault;
extern uint8_t DFETF_PF_Fault;
extern uint8_t LVL2_PF_Fault;
extern uint8_t VIMR_PF_Fault;
extern uint8_t VIMA_PF_Fault;
extern uint8_t SCDL_PF_Fault;
extern uint8_t OTPF_PF_Fault;
extern uint8_t DRMF_PF_Fault;

void DWIN_WriteWord(uint16_t VP, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = 0x05;
    frame[3] = 0x82;
    frame[4] = (VP >> 8) & 0xFF;
    frame[5] = VP & 0xFF;
    frame[6] = (value >> 8) & 0xFF;
    frame[7] = value & 0xFF;
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

void DWIN_B_WriteWord(uint16_t VP, int16_t value)
{
    uint8_t frame[8];
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = 0x05;
    frame[3] = 0x82;
    frame[4] = (VP >> 8) & 0xFF;
    frame[5] = VP & 0xFF;
    frame[6] = (value >> 8) & 0xFF;
    frame[7] = value & 0xFF;
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

void DWIN_C_WriteByte(uint16_t VP, uint8_t value)
{
    uint8_t frame[7];
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = 0x04;
    frame[3] = 0x82;
    frame[4] = (VP >> 8) & 0xFF;
    frame[5] = VP & 0xFF;
    frame[6] = value; 
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

void DWIN_Write2Word(uint16_t VP, uint16_t value_h, uint16_t value_l)
{
    uint8_t frame[10];
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = 0x07;       
    frame[3] = 0x82;          
    frame[4] = (VP >> 8) & 0xFF;
    frame[5] = VP & 0xFF;
    frame[6] = (value_h >> 8) & 0xFF;
    frame[7] = value_h & 0xFF;
    frame[8] = (value_l >> 8) & 0xFF;
    frame[9] = value_l & 0xFF;
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

void DWIN_TP_Simulate(uint16_t mode, uint16_t x, uint16_t y)
{
    uint8_t frame[14];
    frame[0]  = 0x5A;
    frame[1]  = 0xA5;
    frame[2]  = 0x0B;          
    frame[3]  = 0x82;         
    frame[4]  = 0x00;         
    frame[5]  = 0xD4;
    frame[6]  = 0x5A;          
    frame[7]  = 0xA5;
    frame[8]  = (mode >> 8) & 0xFF;   
    frame[9]  = mode & 0xFF;
    frame[10] = (x >> 8) & 0xFF;      
    frame[11] = x & 0xFF;
    frame[12] = (y >> 8) & 0xFF;     
    frame[13] = y & 0xFF;
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

void DWIN_WriteFloat(uint16_t VP, float value, uint8_t decimal_places)
{
    uint16_t scale = 1;
    for (uint8_t i = 0; i < decimal_places; i++) scale *= 10;
    uint16_t scaled_value = (int16_t)roundf(value * scale);
    DWIN_WriteWord(VP, scaled_value);
}

void DWIN_B_WriteFloat(uint16_t VP, float value, uint8_t decimal_places)
{
    uint16_t scale = 1;
    for (uint8_t i = 0; i < decimal_places; i++) scale *= 10;
    int16_t scaled_value = (int16_t)roundf(value * scale);
    DWIN_B_WriteWord(VP, scaled_value);
}

void DWIN_SendCellVoltages(float *cell_voltages)
{
		if (cell_voltages == NULL) return;
    uint16_t vp_list[16] = 
		{
        0x2000, 0x2100, 0x2200, 0x2300,
        0x2400, 0x2500, 0x2600, 0x2700,
        0x2800, 0x2900, 0x2110, 0x2120,
        0x2130, 0x2140, 0x2150, 0x2160
    };
    for (uint8_t i = 0; i < 16; i++) 
		{
        DWIN_WriteFloat(vp_list[i], cell_voltages[i], 3);
    }
}

void DWIN_SendPackVoltage(float pack_voltage)
{
    uint16_t pack_vp = 0x1000;
    DWIN_WriteFloat(pack_vp, pack_voltage, 1);
}

void DWIN_SendTemperature(float temperature)
{
    uint16_t temp_vp = 0x1100;
    DWIN_B_WriteWord(temp_vp, (int16_t)roundf(temperature));
}

void DWIN_SendTemperature1(float temperature1)
{
		uint16_t temp_vp = 0x1101;
		DWIN_B_WriteFloat(temp_vp,temperature1,1);	
}

void DWIN_SendTemperature2(float temperature2)
{
		uint16_t temp_vp = 0x1102;
		DWIN_B_WriteFloat(temp_vp,temperature2,1);	
} 

void DWIN_SendTemperature3(float temperature3)
{
		uint16_t temp_vp = 0x1103;
		DWIN_B_WriteFloat(temp_vp,temperature3,1);	
} 

void DWIN_SendTemperature4(float temperature4)
{
		uint16_t temp_vp = 0x1104;
		DWIN_B_WriteFloat(temp_vp,temperature4,1);	
} 

void DWIN_SendTemperature5(float temperature5)
{
		uint16_t temp_vp = 0x1107;
		DWIN_B_WriteFloat(temp_vp,temperature5,1);	
} 

void DWIN_SendTemperature6(float temperature6)
{
		uint16_t temp_vp = 0x1108;
		DWIN_B_WriteFloat(temp_vp,temperature6,1);	
} 

void DWIN_SendTemperatureFET(float temperatureFET)
{
		uint16_t temp_vp = 0x1105;
		DWIN_B_WriteFloat(temp_vp,temperatureFET,1);	
} 

void DWIN_SendTemperatureIC(float temperatureIC)
{
		uint16_t temp_vp = 0x1106;
		DWIN_B_WriteFloat(temp_vp,temperatureIC,1);	
} 

void DWIN_SendCurrent(float currentA)
{
	if (currentA > -0.2f && currentA < 0.2f)
    {
        DWIN_B_WriteWord(0x1200, 0);
        return;
    }
    DWIN_B_WriteFloat(0x1200, currentA, 1);
}

void DWIN_SendSOC(float soc)
{
		uint16_t soc_uint = (uint16_t)roundf(soc);
    DWIN_WriteWord(0x1010, soc_uint);
}

void DWIN_SetSOCIcon(float soc)
{
		uint8_t icon_id;
		if(soc <= 0.0f)
		{
			icon_id = 5;
		}
		else
			{
			if(soc >= 100.0f)
			{
				soc = 100.0f;
			}
			uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);	
			if(level > 5)
			{
				level = 5;
			}
			icon_id = level;
			}
		DWIN_WriteWord(0x0500, icon_id);
}

void DWIN_BSetSOCIcon(float soc)
{
		uint8_t icon_id;
		if(soc <= 0.0f)
		{
		icon_id = 11;
		}
		else
			{
			if(soc >= 100.0f)
			{
				soc = 100.0f;
			}
			uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);
			if(level > 11)
			{
				level = 11;
			}
			icon_id = level + 6;
			}
		DWIN_WriteWord(0x1700, icon_id);
}

void DWIN_SendReCapa(float recapa)
{
		uint16_t recapa_uint = (uint16_t)roundf(recapa);
		DWIN_WriteWord(0x1300, recapa_uint);
}

void DWIN_SendCycle(float cycle)
{
		DWIN_WriteWord(0x1500, (uint16_t) cycle);
}

void DWIN_SendTime(DS1307_TIME *t)
{
		DWIN_WriteWord(0x0000, t ->hour);
		DWIN_WriteWord(0x0100, t ->min);
		DWIN_WriteWord(0x0200, t ->date);
		DWIN_WriteWord(0x0300, t ->month);
		DWIN_WriteWord(0x0400, 2000+ t ->year);
}

void DWIN_SendSOH(float soh)
{
		uint16_t soh_uint = (uint16_t)roundf(soh);
    DWIN_WriteWord(0x1800, soh_uint);
}

void DWIN_SendRemainEnergy (float remainenergy)
{
		DWIN_WriteFloat(0x0600, remainenergy, 1);
}

static uint8_t DWIN_HasAnyAlarm(void)
{
    uint8_t *p = (uint8_t *)&bms_alarms;
    for (uint8_t i = 0; i < sizeof(BMS_Alarms_t); i++) 
		{
        if (p[i]) 
				{
            return 1;   
        }
    }
    return 0;    
}

void DWIN_UpdateAnyAlarm(void)
{
    uint16_t icon_id;
    if (DWIN_HasAnyAlarm())
		{
        icon_id = 13;   
    } else 
		{
        icon_id = 12;   
    }
    DWIN_WriteWord(0x1900, icon_id);   
}

void DWIN_UpdateAlarms(void)
{
    DWIN_WriteWord(0x1901, ALARM_ICON(bms_alarms.OV_Alarm));
    DWIN_WriteWord(0x1902, ALARM_ICON(bms_alarms.UV_Alarm));
    DWIN_WriteWord(0x1903, ALARM_ICON(bms_alarms.Stack_OV_Alarm));
    DWIN_WriteWord(0x1904, ALARM_ICON(bms_alarms.Stack_UV_Alarm));
    DWIN_WriteWord(0x1905, ALARM_ICON(bms_alarms.OCC_Alarm));
    DWIN_WriteWord(0x1906, ALARM_ICON(bms_alarms.OCD1_Alarm));
    DWIN_WriteWord(0x1907, ALARM_ICON(bms_alarms.OTC_Alarm));
    DWIN_WriteWord(0x1908, ALARM_ICON(bms_alarms.OTD_Alarm));
    DWIN_WriteWord(0x1909, ALARM_ICON(bms_alarms.UTC_Alarm));
    DWIN_WriteWord(0x1910, ALARM_ICON(bms_alarms.UTD_Alarm));
    DWIN_WriteWord(0x1911, ALARM_ICON(bms_alarms.EOT_Alarm));
    DWIN_WriteWord(0x1912, ALARM_ICON(bms_alarms.EUT_Alarm));
    DWIN_WriteWord(0x1913, ALARM_ICON(bms_alarms.Low_Battery_Alarm));
		DWIN_WriteWord(0x1914, ALARM_ICON(bms_alarms.OTF_Alarm));
    DWIN_WriteWord(0x1915, ALARM_ICON(bms_alarms.Cell_Failure_Alarm));
}

static uint8_t DWIN_HasAnyProtection(void)
{
    return ProtectionsTriggered;  
}

void DWIN_UpdateAnyProtection(void)
{
    uint16_t icon_id;
    if (DWIN_HasAnyProtection()) 
		{
        icon_id = 17;   
    } 
		else 
		{
        icon_id = 16;   
    }
    DWIN_WriteWord(0x1400, icon_id);
}

void DWIN_UpdateProtections(void)
{
    DWIN_WriteWord(0x1401, PROTECT_ICON(UV_Fault));    
    DWIN_WriteWord(0x1402, PROTECT_ICON(OV_Fault));    
    DWIN_WriteWord(0x1403, PROTECT_ICON(OCC_Fault)); 
    DWIN_WriteWord(0x1404, PROTECT_ICON(OCD_Fault1));      
    DWIN_WriteWord(0x1405, PROTECT_ICON(SCD_Fault)); 
    DWIN_WriteWord(0x1406, PROTECT_ICON(UTC_Fault));  
    DWIN_WriteWord(0x1407, PROTECT_ICON(UTD_Fault));  
    DWIN_WriteWord(0x1408, PROTECT_ICON(UTINT_Fault)); 
		DWIN_WriteWord(0x1409, PROTECT_ICON(OTC_Fault));
    DWIN_WriteWord(0x1410, PROTECT_ICON(OTD_Fault));
    DWIN_WriteWord(0x1411, PROTECT_ICON(OTINT_Fault));
    DWIN_WriteWord(0x1412, PROTECT_ICON(OTF_Fault)); 
		DWIN_WriteWord(0x1413, PROTECT_ICON(COVL_Fault));
	  DWIN_WriteWord(0x1414, PROTECT_ICON(OCDL_Fault)); 
    DWIN_WriteWord(0x1415, PROTECT_ICON(SCDL_Fault)); 
}

void DWIN_BacklightOff(void)
{
    DWIN_C_WriteByte(0x0082, 0);
}

void DWIN_BacklightOn(uint8_t brightness)
{
    if (brightness > 100)
    {
        brightness = 100;
    }
    DWIN_C_WriteByte(0x0082, brightness);
}

void DWIN_TouchDisable(void)
{
    DWIN_Write2Word(0x00FC, 0x55AA, 0x5A5A);
}

void DWIN_TouchEnable(void)
{
    DWIN_Write2Word(0x00FC, 0x0000, 0x0000);
}

void DWIN_TP_Click(uint16_t x, uint16_t y)
{
    DWIN_TP_Simulate(0x0004, x, y);
}

void DWIN_ScreenOff(void)
{
		DWIN_TouchDisable();
		DWIN_BacklightOff();
}

void DWIN_ScreenOn(void)
{  
		DWIN_TouchEnable(); 
		DWIN_TP_Click(766,6);  
		DWIN_BacklightOn(100);
}

void DWIN_SendSlavePackVoltage(void)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
    float slave_pack_v = (float)slave_analog_data[0].total_voltage / 1000.0f;  
		float slave_pack_v1 = (float)slave_analog_data[1].total_voltage / 1000.0f;
		float slave_pack_v2 = (float)slave_analog_data[2].total_voltage / 1000.0f;
    DWIN_WriteFloat(0x4000, slave_pack_v, 1);  
		DWIN_WriteFloat(0x3001, slave_pack_v1, 1);  
		DWIN_WriteFloat(0x3008, slave_pack_v2, 1); 
}

void DWIN_SendSlaveCurrent(void)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
    float slave_current = (float)slave_analog_data[0].current / 100.0f;  
		float slave1_current = (float)slave_analog_data[1].current / 100.0f;
		float slave2_current = (float)slave_analog_data[2].current / 100.0f;
    DWIN_B_WriteFloat(0x4500, slave_current, 1);   
		DWIN_B_WriteFloat(0x3003, slave1_current, 1);
		DWIN_B_WriteFloat(0x3011, slave2_current, 1);
}

uint32_t Get_Slave_Total_Cap_mAh(uint8_t index)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
    if (slave_analog_data[index].user_def_count >= 4) {
        return ((uint32_t)slave_analog_data[index].total_cap_2[0] << 16) | 
               ((uint32_t)slave_analog_data[index].total_cap_2[1] << 8)  | 
               ((uint32_t)slave_analog_data[index].total_cap_2[2]);
    } 
    else {
        return (uint32_t)slave_analog_data[index].total_cap_1 * 100; 
    }
}

uint32_t Get_Slave_Remain_Cap_mAh(uint8_t index)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];    
    if (slave_analog_data[index].user_def_count >= 4) {
        return ((uint32_t)slave_analog_data[index].remain_cap_2[0] << 16) | 
               ((uint32_t)slave_analog_data[index].remain_cap_2[1] << 8)  | 
               ((uint32_t)slave_analog_data[index].remain_cap_2[2]);
    } else {
        return (uint32_t)slave_analog_data[index].remain_cap_1 * 100;
    }
}

void DWIN_SendSlaveSOC(void)
{
		//Slave 0
    float soc_float = 0.0f;
    uint32_t total = Get_Slave_Total_Cap_mAh(0);
    uint32_t remain = Get_Slave_Remain_Cap_mAh(0);   
    if (total > 0) {
        soc_float = (float)remain * 100.0f / (float)total;
    }   
    DWIN_WriteWord(0x4300, (uint16_t)roundf(soc_float));
		//Slave 1
		float soc_float1 = 0.0f;
    uint32_t total1  = Get_Slave_Total_Cap_mAh(1);
    uint32_t remain1 = Get_Slave_Remain_Cap_mAh(1);
    if (total1 > 0) {
        soc_float1 = (float)remain1 * 100.0f / (float)total1;
    }
    DWIN_WriteWord(0x3002, (uint16_t)roundf(soc_float1));  
		//Slave 2
		float soc_float2 = 0.0f;
    uint32_t total2  = Get_Slave_Total_Cap_mAh(2);
    uint32_t remain2 = Get_Slave_Remain_Cap_mAh(2);
    if (total2 > 0) {
        soc_float2 = (float)remain2 * 100.0f / (float)total2;
    }
    DWIN_WriteWord(0x3015, (uint16_t)roundf(soc_float2));
}


void DWIN_SendSlaveSOH(void)
{
		// Slave 0
    uint32_t total = Get_Slave_Total_Cap_mAh(0);
    float soh_float = (float)total * 100.0f / 314000.0f; 
    DWIN_WriteWord(0x4200, (uint16_t)roundf(soh_float));
		// Slave 1
		uint32_t total1 = Get_Slave_Total_Cap_mAh(1);
    float soh_float1 = (float)total1 * 100.0f / 314000.0f;
    DWIN_WriteWord(0x3006, (uint16_t)roundf(soh_float1));
	// Slave 2
		uint32_t total2 = Get_Slave_Total_Cap_mAh(2);
    float soh_float2 = (float)total2 * 100.0f / 314000.0f;
    DWIN_WriteWord(0x3009, (uint16_t)roundf(soh_float2));
}

void DWIN_SendSlaveRemainCapacity(void)
{
		// Slave 0
    uint32_t remain = Get_Slave_Remain_Cap_mAh(0);
    float remain_Ah = (float)remain / 1000.0f; 
    DWIN_WriteWord(0x4100, (uint16_t)roundf(remain_Ah));
		// Slave 1
	  uint32_t remain1 = Get_Slave_Remain_Cap_mAh(1);
    float remain_Ah1 = (float)remain1 / 1000.0f;
    DWIN_WriteWord(0x3004, (uint16_t)roundf(remain_Ah1));
	// Slave 2
	  uint32_t remain2 = Get_Slave_Remain_Cap_mAh(2);
    float remain_Ah2 = (float)remain2 / 1000.0f;
    DWIN_WriteWord(0x3013, (uint16_t)roundf(remain_Ah2));
}

void DWIN_SendSlaveCycleCount(void)
{	
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
    DWIN_WriteWord(0x4600, (uint16_t)slave_analog_data[0].cycle_count);
		DWIN_WriteWord(0x3005, (uint16_t)slave_analog_data[1].cycle_count);
		DWIN_WriteWord(0x3014, (uint16_t)slave_analog_data[2].cycle_count);
}

void DWIN_SetSlaveSOCIcon(void)
{
		// Slave 0
    uint32_t total = Get_Slave_Total_Cap_mAh(0);
    uint32_t remain = Get_Slave_Remain_Cap_mAh(0);
    uint8_t soc = 0;
    if (total > 0) {
        soc = (uint8_t)((float)remain * 100.0f / (float)total);
    } 
    uint8_t icon_id;
    if (soc <= 0) icon_id = 5;
    else {
        if (soc >= 100) soc = 100;
        uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);
        if (level > 5) level = 5;
        icon_id = level;
    }
    DWIN_WriteWord(0x4020, icon_id);
		// Slave 1
    uint32_t total1  = Get_Slave_Total_Cap_mAh(1);
    uint32_t remain1 = Get_Slave_Remain_Cap_mAh(1);
    uint8_t soc1 = 0;
    if (total1 > 0) {
        soc1 = (uint8_t)((float)remain1 * 100.0f / (float)total1);
    }
    uint8_t icon_id1;
    if (soc1 <= 0) icon_id1 = 5;
    else {
        if (soc1 >= 100) soc1 = 100;
        uint8_t level1 = (uint8_t)((100.0f - soc1) / 16.67f);
        if (level1 > 5) level1 = 5;
        icon_id1 = level1;
    }
    DWIN_WriteWord(0x1002, icon_id1); 
		// Slave 2
    uint32_t total2  = Get_Slave_Total_Cap_mAh(2);
    uint32_t remain2 = Get_Slave_Remain_Cap_mAh(2);
    uint8_t soc2 = 0;
    if (total2 > 0) {
        soc2 = (uint8_t)((float)remain2 * 100.0f / (float)total2);
    }
    uint8_t icon_id2;
    if (soc2 <= 0) icon_id2 = 5;
    else {
        if (soc2 >= 100) soc2 = 100;
        uint8_t level2 = (uint8_t)((100.0f - soc2) / 16.67f);
        if (level2 > 5) level2 = 5;
        icon_id2 = level2;
    }
    DWIN_WriteWord(0x1005, icon_id2); 
}

void DWIN_BSetSlaveSOCIcon(void)
{
		// Slave 0
    uint32_t remain = Get_Slave_Remain_Cap_mAh(0);
    uint32_t total  = Get_Slave_Total_Cap_mAh(0);
    float soc = 0.0f;
    if (total > 0) {
        soc = (float)remain * 100.0f / (float)total;
    }
    uint8_t icon_id;
    if(soc <= 0.0f)
    {
        icon_id = 11;
    }
    else
    {
        if(soc >= 100.0f) { soc = 100.0f; }       
        uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);
        if(level > 11) { level = 11; }
        icon_id = level + 6;
    }
    DWIN_WriteWord(0x4400, icon_id);
		// Slave 1
		uint32_t remain1 = Get_Slave_Remain_Cap_mAh(1);
    uint32_t total1  = Get_Slave_Total_Cap_mAh(1);
    float soc1 = 0.0f;
    if (total1 > 0) {
        soc1 = (float)remain1 * 100.0f / (float)total1;
    }
    uint8_t icon_id1;
    if (soc1 <= 0.0f) {
        icon_id1 = 11;
    } else {
        if (soc1 >= 100.0f) soc1 = 100.0f;
        uint8_t level1 = (uint8_t)((100.0f - soc1) / 16.67f);
        if (level1 > 11) level1 = 11;
        icon_id1 = level1 + 6;
    }
    DWIN_WriteWord(0x3007, icon_id1);
		// Slave 2
		uint32_t remain2 = Get_Slave_Remain_Cap_mAh(2);
    uint32_t total2  = Get_Slave_Total_Cap_mAh(2);
    float soc2 = 0.0f;
    if (total2 > 0) {
        soc2 = (float)remain2 * 100.0f / (float)total2;
    }
    uint8_t icon_id2;
    if (soc2 <= 0.0f) {
        icon_id2 = 11;
    } else {
        if (soc2 >= 100.0f) soc2 = 100.0f;
        uint8_t level2 = (uint8_t)((100.0f - soc2) / 16.67f);
        if (level2 > 11) level2 = 11;
        icon_id2 = level2 + 6;
    }
    DWIN_WriteWord(0x3012, icon_id2);
}

void DWIN_SendSlaveTemperature0(void)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
		// Slave 0
    float kelvin = (float)slave_analog_data[0].temperatures[0] / 10.0f;
    float temp_C = kelvin - 273.1f;
    uint16_t temp_uint = (uint16_t)roundf(temp_C);
    DWIN_WriteWord(0x4010, temp_uint);
		// Slave 1
		float kelvin1 = (float)slave_analog_data[1].temperatures[0] / 10.0f;
    float temp_C1 = kelvin1 - 273.1f;
    DWIN_WriteWord(0x1001, (uint16_t)roundf(temp_C1));
		// Slave 2
		float kelvin2 = (float)slave_analog_data[2].temperatures[0] / 10.0f;
    float temp_C2 = kelvin2 - 273.1f;
    DWIN_WriteWord(0x1004, (uint16_t)roundf(temp_C2));
}

void DWIN_SendSlaveCellVoltages(void)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
		// Slave 0
    uint16_t vp_list[16] = 
    {
        0x4111, 0x4112, 0x4113, 0x4114, 0x4115, 0x4116, 0x4117, 0x4118,
        0x4119, 0x4211, 0x4212, 0x4213, 0x4214, 0x4215, 0x4216, 0x4217
    };
    for (uint8_t i = 0; i < 16; i++)
    {
        float cell_V = (float)slave_analog_data[0].cell_voltages[i] / 1000.0f;  
        DWIN_WriteFloat(vp_list[i], cell_V, 3);   
    }
		// Slave 1
		 uint16_t vp_list1[16] = 
    {
				0x2001, 0x2002, 0x2003, 0x2004, 0x2005, 0x2006, 0x2007, 0x2008,
				0x2009, 0x2010, 0x2011, 0x2012, 0x2013, 0x2014, 0x2015, 0x2016
    };
    for (uint8_t i = 0; i < 16; i++)
    {
        float cell_V = (float)slave_analog_data[1].cell_voltages[i] / 1000.0f;
        DWIN_WriteFloat(vp_list1[i], cell_V, 3);
    }
		// Slave 2
		 uint16_t vp_list2[16] = 
    {
				0x2017, 0x2018, 0x2019, 0x2020, 0x2021, 0x2022, 0x2023, 0x2024,
				0x2025, 0x2026, 0x2027, 0x2028, 0x2029, 0x2030, 0x2031, 0x2032
    };
    for (uint8_t i = 0; i < 16; i++)
    {
        float cell_V = (float)slave_analog_data[2].cell_voltages[i] / 1000.0f;
        DWIN_WriteFloat(vp_list2[i], cell_V, 3);
    }
}

void DWIN_SendSlaveRemainEnergy(void)
{
		// Slave 0
    uint32_t remain = Get_Slave_Remain_Cap_mAh(0);
    float slave_remainenergy = ((float)remain / 1000.0f) * 51.2f / 1000.0f;
    DWIN_WriteFloat(0x4030, slave_remainenergy, 1); 
		//Slave 1
		uint32_t remain1 = Get_Slave_Remain_Cap_mAh(1);
    float slave_remainenergy1 = ((float)remain1 / 1000.0f) * 51.2f / 1000.0f;
    DWIN_WriteFloat(0x1003, slave_remainenergy1, 1); 
		//Slave 2
		uint32_t remain2 = Get_Slave_Remain_Cap_mAh(2);
    float slave_remainenergy2 = ((float)remain2 / 1000.0f) * 51.2f / 1000.0f;
    DWIN_WriteFloat(0x1006, slave_remainenergy2, 1);
}

void DWIN_SendSystemTotalVoltage(void)
{
    extern uint16_t system_total_voltage_01V;   
    float voltage_V = (float)system_total_voltage_01V / 1000.0f;   
    DWIN_WriteFloat(0x3000, voltage_V, 1);      
}

void DWIN_SendSystemAvgSOH(void)
{
    extern uint16_t system_avg_soh;
    float soh_float = (float)system_avg_soh;
    uint16_t soh_rounded = (uint16_t)roundf(soh_float);
    DWIN_WriteWord(0x3200, soh_rounded);
}

void DWIN_SendSystemAvgSOC(void)
{
    extern uint16_t system_avg_soc;
    float soc_float = (float)system_avg_soc;
    uint16_t soc_rounded = (uint16_t)roundf(soc_float);
    DWIN_WriteWord(0x3300, soc_rounded);
}

void DWIN_SendSystemAvgCycles(void)
{
    extern uint16_t system_avg_cycles;
    DWIN_WriteWord(0x3600, system_avg_cycles);   
}

void DWIN_SendSystemTotalCurrent(void)
{
    extern int32_t system_total_current_01A;   
    float current_A = (float)system_total_current_01A / 100.0f;  
    DWIN_B_WriteFloat(0x3500, current_A, 1);     
}

void DWIN_SetSystemAvgSOCIcon(void)
{
    extern uint16_t system_avg_soc; 
    uint8_t soc = (uint8_t)system_avg_soc;   
    uint8_t icon_id;
    if (soc <= 0)
    {
        icon_id = 5;
    }
    else
    {
        if (soc >= 100)
        {
            soc = 100;
        }
        uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);
        if (level > 5)
        {
            level = 5;
        }
        icon_id = level;
    }
    DWIN_WriteWord(0x3020, icon_id);
}

void DWIN_BSetSystemAvgSOCIcon(void)
{
    extern uint16_t system_avg_soc;
    uint8_t soc = (uint8_t)system_avg_soc;   
    uint8_t icon_id;
    if (soc <= 0)
    {
        icon_id = 11;
    }
    else
    {
        if (soc >= 100)
        {
            soc = 100;
        }
        uint8_t level = (uint8_t)((100.0f - soc) / 16.67f);
        if (level > 11)
        {
            level = 11;
        }
        icon_id = level + 6;
    }
    DWIN_WriteWord(0x3400, icon_id);
}

void DWIN_SendSystemTemperature(void)
{
    extern float Temperature[];     
    DWIN_B_WriteWord(0x3010, (int16_t)roundf(Temperature[0]));
}

void DWIN_SendSystemReCapa(void)
{
    extern float remain_capa_Ah;            
    extern uint8_t slave_online_status[MAX_SLAVES];    
    float total_recapa_mAh = (remain_capa_Ah * 1000.0f);
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (slave_online_status[i] > 0) {
            total_recapa_mAh += (float)Get_Slave_Remain_Cap_mAh(i);
        }
    }
    uint16_t system_recapa = (uint16_t)roundf(total_recapa_mAh / 1000.0f);   
    DWIN_WriteWord(0x3100, system_recapa);    
}

void DWIN_SendSystemRemainEnergy(void)
{
    extern float remain_capa_Ah;   
    extern uint8_t slave_online_status[MAX_SLAVES];   
    float system_recapa_Ah = remain_capa_Ah;
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (slave_online_status[i] > 0) {
            system_recapa_Ah += (float)Get_Slave_Remain_Cap_mAh(i) / 1000.0f;
        }
    }   
    float system_remainenergy = (system_recapa_Ah * 51.2f) / 1000.0f;   
    DWIN_WriteFloat(0x3030, system_remainenergy, 1);
}

void DWIN_ShowDualPackConnectionStatus(void)
{
    extern uint8_t is_master;
		extern uint8_t active_packs_count;
		extern uint8_t slave_online_status[MAX_SLAVES];
    if (is_master == 1)                    
    {
        uint8_t has_slave = 0;
        for (int i = 0; i < MAX_SLAVES; i++) {
            if (slave_online_status[i] > 0) {
                has_slave = 1;
                break;
            }
        }
        if (active_packs_count >= 2 || has_slave == 1)
       {
            DWIN_WriteWord(0x1812, 22);    
            DWIN_WriteWord(0x1813, 23);
       }
        else
       {
            DWIN_WriteWord(0x1812, 20);   
            DWIN_WriteWord(0x1813, 20);
        }
    }
    else                                   
    {
        DWIN_WriteWord(0x1812, 20);
        DWIN_WriteWord(0x1813, 20);
    }
}

void DWIN_UpdateSpecialProtection(void)
{
    uint8_t has_special_fault = 0;
    if (OCD_Fault || HWDF_Fault || PTO_Fault || OCD3_Fault)
    {
        has_special_fault = 1;
    }
    if (has_special_fault)
    {
        DWIN_WriteWord(0x0114, PROTECT_ON_ICON);
    }
    else
    {
        DWIN_WriteWord(0x0114, PROTECT_OFF_ICON);
    }
}

void DWIN_SendFETStatusDetail(void)
{
    extern uint8_t FET_Status;
    if (FET_Status & 0x01)
        DWIN_WriteWord(0x0105, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0105, PROTECT_OFF_ICON);
    if (FET_Status & 0x02)
        DWIN_WriteWord(0x0106, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0106, PROTECT_OFF_ICON);
    if (FET_Status & 0x04)
        DWIN_WriteWord(0x0107, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0107, PROTECT_OFF_ICON);
    if (FET_Status & 0x08)
        DWIN_WriteWord(0x0108, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0108, PROTECT_OFF_ICON);
    if (FET_Status & 0x10)
        DWIN_WriteWord(0x0109, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0109, PROTECT_OFF_ICON);
    if (FET_Status & 0x20)
        DWIN_WriteWord(0x0110, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0110, PROTECT_OFF_ICON);
    if (FET_Status & 0x40)
        DWIN_WriteWord(0x0111, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0111, PROTECT_OFF_ICON);
    if (FET_Status & 0x80)
        DWIN_WriteWord(0x0112, PROTECT_ON_ICON);
    else
        DWIN_WriteWord(0x0112, PROTECT_OFF_ICON);
		if (HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin) == GPIO_PIN_SET)
        DWIN_WriteWord(0x0113, PROTECT_ON_ICON);  
    else
        DWIN_WriteWord(0x0113, PROTECT_OFF_ICON);
}

//==================PF Fault===============
void DWIN_UpdatePermanentFail(void)
{
    BQ769x2_ReadPFStatus();    
    uint8_t has_pf_fault = 0;
    if (PFErrorsTriggered || 
        value_PFStatusA != 0 || 
        value_PFStatusB != 0 || 
        value_PFStatusC != 0 || 
        value_PFStatusD != 0)
    {
        has_pf_fault = 1;
    }
    if (has_pf_fault)
        DWIN_WriteWord(0x0115, PROTECT_ON_ICON);   
    else
        DWIN_WriteWord(0x0115, PROTECT_OFF_ICON); 
}

void DWIN_UpdatePF_IndividualIcons(void)
{
    BQ769x2_ReadPFStatus();
    DWIN_WriteWord(0x0116, SUV_PF_Fault   ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0117, IRMF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0118, SOV_PF_Fault   ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0119, LFOF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0120, SOCC_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0121, SOCD_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0122, VREF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0123, SOT_PF_Fault   ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0124, VSSF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0125, SOTF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0126, CUDEP_PF_Fault ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0127, HWMX_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0128, CFETF_PF_Fault ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0129, CMDF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0130, DFETF_PF_Fault ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0131, LVL2_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0132, CMDF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);  
    DWIN_WriteWord(0x0133, VIMR_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0134, VIMA_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0135, SCDL_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0136, OTPF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
    DWIN_WriteWord(0x0137, DRMF_PF_Fault  ? PROTECT_ON_ICON : PROTECT_OFF_ICON);
}

void DWIN_Control_SlavePageTouch(void)
{
    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
    extern uint8_t is_master;
    if (is_master != 1)
    {
        DWIN_SetSlaveTouch_Page1(0, 0);
        DWIN_SetSlaveTouch_Page1(1, 0);
        DWIN_SetSlaveTouch_Page14(0, 0);
				DWIN_SetSlaveTouch_Page21(0, 0);
				DWIN_SetSlaveTouch_Page24(0, 0);
        return;
    }
    //Slave [0] 
    uint8_t slave0_enable = (slave_analog_data[0].command_value == 0x03);
    DWIN_SetSlaveTouch_Page1(0, slave0_enable);   
    DWIN_SetSlaveTouch_Page1(1, slave0_enable);  
    // Slave [1] 
    uint8_t slave1_enable = (slave_analog_data[1].command_value == 0x04);
    DWIN_SetSlaveTouch_Page14(0, slave1_enable);
		// Slave [2]
		uint8_t slave2_enable = (slave_analog_data[2].command_value == 0x05);
    DWIN_SetSlaveTouch_Page21(0, slave2_enable);  
		// Slave [3] 
    DWIN_SetSlaveTouch_Page24(0, 0);  		
}

static void DWIN_SetSlaveTouch_Page1(uint8_t touch_num, uint8_t enable)
{
    uint8_t frame[] = {
        0x5A, 0xA5, 0x0B, 0x82, 0x00, 0xB0,
        0x5A, 0xA5,
        0x00, 0x01,           // Page 1
        touch_num,
        0x05,
        0x00, enable
    };
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

static void DWIN_SetSlaveTouch_Page14(uint8_t touch_num, uint8_t enable)
{
    uint8_t frame[] = {
        0x5A, 0xA5, 0x0B, 0x82, 0x00, 0xB0,
        0x5A, 0xA5,
        0x00, 0x0E,           // Page 14
        touch_num,
        0x05,
        0x00, enable
    };
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}
static void DWIN_SetSlaveTouch_Page21(uint8_t touch_num, uint8_t enable)
{
    uint8_t frame[] = {
        0x5A, 0xA5, 0x0B, 0x82, 0x00, 0xB0,
        0x5A, 0xA5,
        0x00, 0x15,           // Page 21 
        touch_num,           
        0x05,
        0x00, enable
    };
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}
static void DWIN_SetSlaveTouch_Page24(uint8_t touch_num, uint8_t enable)
{
    uint8_t frame[] = {
        0x5A, 0xA5, 0x0B, 0x82, 0x00, 0xB0,
        0x5A, 0xA5,
        0x00, 0x18,           
        touch_num,            
        0x05,
        0x00, enable
    };
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10);
}

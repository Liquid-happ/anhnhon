#ifndef NTC_TABLE_H
#define NTC_TABLE_H
#include "main.h"

#define NTC_103AT_3435 0 				 // NTC 10kO with Beta 3435 (theo PDF spec ME103F3435V3B905)
#define NTC_103AT_3950 1 				 // NTC 10kO with Beta 3950 

#define RT_TABLE NTC_103AT_3435  // Default to 3435 based on schematic NTC 10kO assumption

#if RT_TABLE == NTC_103AT_3435
  #define NTC103AT_ARRAY_LEN 136 // Length of NTC lookup table (from -30°C to 105°C)
  #define TEMP_UPPER_LIMIT 105 	 // Upper temperature limit (°C)
  #define TEMP_LOWER_LIMIT -30   // Lower temperature limit (°C)
#elif RT_TABLE == NTC_103AT_3950
  #define NTC103AT_ARRAY_LEN 136 // Length of NTC lookup table
  #define TEMP_UPPER_LIMIT 105   // Upper temperature limit (°C)
  #define TEMP_LOWER_LIMIT -30 	 // Lower temperature limit (°C)
#else
  #error "undefined RT_TABLE!"
#endif

extern const uint32_t NTC103AT[NTC103AT_ARRAY_LEN]; 
extern float debug_voltage;
extern float debug_r_lower;
extern float debug_resistance;
extern float debug_temp;
float NTC_AdcToTemp(uint16_t adc_value);

#endif

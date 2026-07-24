#include "ntc_table.h"

#if IS_BOOTLOADER == 0
#include "main.h"

#if RT_TABLE == NTC_103AT_3435
const uint32_t NTC103AT[NTC103AT_ARRAY_LEN] = { 																											// 103AT beta=3435 lookup table (-30°C to 105°C)
		116509, 110493, 104827, 99488, 94458, 89710, 85233, 81008, 77019, 73252, 69693, 66329, 63149, 		//13         -30 -18
		60140, 57294, 54600, 52049, 49633, 47344, 45174, 43117, 41166, 39315, 37559, 35891, 34307,				//13				 -17 -5
		32803, 31373, 30015, 28723, 27494, 26325, 25212, 24153, 23144, 22184, 21268, 20396, 19564,				//13				 -4   8
		18771, 18015, 17294, 16605, 15948, 15320, 14720, 14148, 13600, 13077, 12577, 12099, 11641,				//13					9		21
		11204, 10785, 10384, 10000, 9632, 9280, 8943, 8620, 8310, 8012, 7728, 7454, 7192,									//13					22  34
		6940, 6699, 6467, 6244, 6030, 5825, 5628, 5438, 5256, 5080, 4912, 4750, 4594,											//13					35  47
		4444, 4300, 4161, 4027, 3898, 3774, 3654, 3540, 3428, 3322, 3219, 3119, 3023,											//13				  48	60
		2931, 2842, 2756, 2673, 2593, 2516, 2441, 2369, 2299, 2232, 2167, 2105, 2044,											//13					61	73
		1985, 1929, 1874, 1821, 1770, 1720, 1673, 1626, 1581, 1538, 1496, 1455, 1416,											//13					74  86
		1378, 1341, 1305, 1270, 1237, 1204, 1173, 1142, 1112, 1084, 1056, 1029, 1003,											//13					87  99
		997, 953, 929, 906, 883, 861																																			//6						100 105
};

float NTC_AdcToTemp(uint16_t adc_value) {
    debug_voltage = (adc_value * 3.3f) / 4095.0f;
    if (debug_voltage >= 3.29f) {
        return (float)TEMP_LOWER_LIMIT;
    }
    if (debug_voltage <= 0.01f) {
        return (float)TEMP_UPPER_LIMIT;
    }
    debug_r_lower = (10000.0f * debug_voltage) / (3.3f - debug_voltage);
    debug_resistance = debug_r_lower;
    if (debug_resistance >= NTC103AT[0]) return (float)TEMP_LOWER_LIMIT;
    if (debug_resistance <= NTC103AT[NTC103AT_ARRAY_LEN-1]) return (float)TEMP_UPPER_LIMIT;
    for (int i = 0; i < NTC103AT_ARRAY_LEN - 1; i++) {
        if (debug_resistance <= NTC103AT[i] && debug_resistance > NTC103AT[i + 1]) {
            float r1 = (float)NTC103AT[i];
            float r2 = (float)NTC103AT[i + 1];
            float t1 = (float)(TEMP_LOWER_LIMIT + i);
            float t2 = (float)(TEMP_LOWER_LIMIT + i + 1);
            debug_temp = t1 + ((debug_resistance - r2) * (t2 - t1)) / (r1 - r2);
            return debug_temp;
        }
    }
    return (float)TEMP_UPPER_LIMIT;
}

#elif RT_TABLE == NTC_103AT_3950

#else
#error "undefined RT_TABLE!"

#endif

#endif

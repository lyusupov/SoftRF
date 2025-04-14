#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_NRF52840)

#include "SenseCAP.h"

#include <Arduino.h>

#define A0                      (2)     // Baterry level dectect
#define A1                      (4)     // VCC voltage dectect
#define A2                      (5)     // Charger insert dectect
#define A3                      (29)    // Light sensor
#define A4                      (31)    // Temperature sensor

#define NTC_ANALOG_INPUT        (A4)
#define LUX_ANALOG_INPUT        (A3)
#define BAT_ANALOG_INPUT        (A0)

#define ADC_MAX                 1023    // 10-bit ADC max value
#define VOLTAGE_REF             3000    // ADC reference voltage in millivolts
#define ADC_SAMPLE_NUM_MAX      16      // Number of samples
#define HEATER_NTC_RP           8250    // Ohm, series resistance to thermistor
#define LIGHT_REF_VCC           2400    // mV

static uint32_t ntc_res2[136] = {
    113347,107565,102116,96978,92132,87559,83242,79166,75316,71677,
    68237,64991,61919,59011,56258,53650,51178,48835,46613,44506,
    42506,40600,38791,37073,35442,33892,32420,31020,29689,28423,
    27219,26076,24988,23951,22963,22021,21123,20267,19450,18670,
    17926,17214,16534,15886,15266,14674,14108,13566,13049,12554,
    12081,11628,11195,10780,10382,10000,9634,9284,8947,8624,
    8315,8018,7734,7461,7199,6948,6707,6475,6253,6039,
    5834,5636,5445,5262,5086,4917,4754,4597,4446,4301,
    4161,4026,3896,3771,3651,3535,3423,3315,3211,3111,
    3014,2922,2834,2748,2666,2586,2509,2435,2364,2294,
    2228,2163,2100,2040,1981,1925,1870,1817,1766,1716,
    1669,1622,1578,1535,1493,1452,1413,1375,1338,1303,
    1268,1234,1202,1170,1139,1110,1081,1053,1026,999,
    974,949,925,902,880,858,
};

static int8_t ntc_temp2[136] = {
    -30,-29,-28,-27,-26,-25,-24,-23,-22,-21,
    -20,-19,-18,-17,-16,-15,-14,-13,-12,-11,
    -10,-9,-8,-7,-6,-5,-4,-3,-2,-1,
    0,1,2,3,4,5,6,7,8,9,
    10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,26,27,28,29,
    30,31,32,33,34,35,36,37,38,39,
    40,41,42,43,44,45,46,47,48,49,
    50,51,52,53,54,55,56,57,58,59,
    60,61,62,63,64,65,66,67,68,69,
    70,71,72,73,74,75,76,77,78,79,
    80,81,82,83,84,85,86,87,88,89,
    90,91,92,93,94,95,96,97,98,99,
    100,101,102,103,104,105,
};

#define BATTERY_POINT 12
static const int Battery_Level_Percent_Table[BATTERY_POINT] = {3200, 3590, 3650, 3700, 3740, 3760, 3795, 3840, 3910, 3980, 4070, 4150};

static int16_t get_heater_temperature(uint16_t vcc_volt, uint16_t ntc_volt)
{
    uint8_t u8i = 0;
    float Vout = 0, Rt = 0, temp = 0;
    int16_t HeaterTempValue = 0;
    Vout = ntc_volt;

    Rt = (HEATER_NTC_RP * vcc_volt) / Vout - HEATER_NTC_RP;

    for (u8i = 0; u8i < 136; u8i++)
    {
        if (Rt >= ntc_res2[u8i])
            break;
    }

    temp = ntc_temp2[u8i - 1] + 1 * (ntc_res2[u8i - 1] - Rt) / (float)(ntc_res2[u8i - 1] - ntc_res2[u8i]);
    HeaterTempValue = (temp * 100 + 5) / 10;

    return HeaterTempValue;
}

static int16_t get_light_lv(uint16_t light_mv)
{
    float Vout = 0;
    uint16_t light_level = 0;

    if (light_mv <= 80)
    {
        light_level = 0;
        return light_level;
    }
    else if (light_mv >= 2480)
    {
        light_level = 100;
        return light_level;
    }
    Vout = light_mv;
    light_level = 100 * (Vout - 80) / LIGHT_REF_VCC;

    return light_level;
}

static uint8_t vol_to_percentage(uint16_t voltage)
{
    if (voltage < Battery_Level_Percent_Table[0])
    {
        return 0;
    }

    if (voltage < Battery_Level_Percent_Table[1])
    {
        return 0 + (20UL * (int)(voltage - Battery_Level_Percent_Table[0])) /
                       (int)(Battery_Level_Percent_Table[1] - Battery_Level_Percent_Table[0]);
    }

    for (uint8_t i = 0; i < BATTERY_POINT; i++)
    {
        if (voltage < Battery_Level_Percent_Table[i])
        {
            return 20 + (8 * (i - 2)) + (8UL * (int)(voltage - Battery_Level_Percent_Table[i - 1])) / (int)(Battery_Level_Percent_Table[i] - Battery_Level_Percent_Table[i - 1]);
        }
    }

    return 100;
}

static void adc_sample(size_t sensor_channel, uint16_t *vcc, uint16_t *sensor)
{
    uint32_t sum_vcc = 0, sum_sensor = 0;
    uint8_t count_vcc = 0, count_sensor = 0;

    for (int i = 0; i < ADC_SAMPLE_NUM_MAX; i++)
    {
        uint16_t raw_vcc = analogReadVDD();
        uint16_t raw_sensor = analogRead(sensor_channel);

        if (raw_vcc)
        {
            sum_vcc += raw_vcc;
            count_vcc++;
        }
        if (raw_sensor)
        {
            sum_sensor += raw_sensor;
            count_sensor++;
        }

        delay(1);
    }

    // Calculate the average and convert to millivolts
    *vcc = (count_vcc > 0) ? ((sum_vcc / count_vcc) * VOLTAGE_REF) / ADC_MAX : 0;
    *sensor = (count_sensor > 0) ? ((sum_sensor / count_sensor) * VOLTAGE_REF) / ADC_MAX : 0;
}

int16_t t1000e_ntc_sample()
{
    int16_t temp = 0;
    uint16_t vcc_mv = 0, ntc_mv = 0;

//    digitalWrite(PIN_SENSE_POWER_EN, HIGH);
    adc_sample(NTC_ANALOG_INPUT, &vcc_mv, &ntc_mv);
    temp = get_heater_temperature(vcc_mv, ntc_mv);
//    digitalWrite(PIN_SENSE_POWER_EN, LOW);

    return temp;
}

int16_t t1000e_lux_sample()
{
    int16_t lux = 0;
    uint16_t vcc_mv = 0, lux_mv = 0;

//    digitalWrite(PIN_SENSE_POWER_EN, HIGH);
    adc_sample(LUX_ANALOG_INPUT, &vcc_mv, &lux_mv);
    lux = get_light_lv(lux_mv);
//    digitalWrite(PIN_SENSE_POWER_EN, LOW);

    return lux;
}

int16_t t1000e_bat_sample()
{
    int16_t bat = 0;
    uint16_t vcc_mv = 0, bat_mv = 0;

//    digitalWrite(PIN_SENSE_POWER_EN, HIGH);
    adc_sample(BAT_ANALOG_INPUT, &vcc_mv, &bat_mv);
    bat = vol_to_percentage(bat_mv * 2);
//    digitalWrite(PIN_SENSE_POWER_EN, LOW);

    return bat;
}
#endif /* ARDUINO_ARCH_NRF52 */

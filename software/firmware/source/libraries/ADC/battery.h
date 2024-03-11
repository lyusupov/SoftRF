#ifndef _BATTERY_H
#define _BATTERY_H

uint16_t read_voltage(void);

#if !defined(ESP_IDF_VERSION_MAJOR) || ESP_IDF_VERSION_MAJOR < 5
#include <driver/adc.h>
#include <esp_adc_cal.h>

void calibrate_voltage(adc1_channel_t, adc_atten_t atten = ADC_ATTEN_DB_11);
#else
void calibrate_voltage(uint8_t, adc_attenuation_t atten = ADC_11db);
#endif /* ESP_IDF_VERSION_MAJOR */

#define DEFAULT_VREF 1100 // tbd: use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES 32  // we do some multisampling to get better values

#endif /* _BATTERY_H */

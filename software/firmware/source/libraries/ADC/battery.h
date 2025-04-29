#ifndef _BATTERY_H
#define _BATTERY_H

#if defined(ESP32)

uint16_t adc1_read_voltage(void);
uint16_t adc2_read_voltage(void);

#define read_voltage adc1_read_voltage

#if !defined(ESP_IDF_VERSION_MAJOR) || ESP_IDF_VERSION_MAJOR < 5
#include <driver/adc.h>
#include <esp_adc_cal.h>

void adc1_calibrate_voltage(adc1_channel_t, adc_atten_t atten = ADC_ATTEN_DB_11);
void adc2_calibrate_voltage(adc2_channel_t, adc_atten_t atten = ADC_ATTEN_DB_11);

#define calibrate_voltage adc1_calibrate_voltage

#else
void calibrate_voltage(uint8_t, adc_attenuation_t atten = ADC_11db);
#endif /* ESP_IDF_VERSION_MAJOR */

#define DEFAULT_VREF 1100 // tbd: use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES 32  // we do some multisampling to get better values

#endif /* ESP32 */
#endif /* _BATTERY_H */

#ifndef _BATTERY_H
#define _BATTERY_H

#if defined(CONFIG_IDF_TARGET_ESP32C6)
/* TBD */
typedef unsigned int adc1_channel_t;
typedef unsigned int adc_atten_t;
#define ADC_ATTEN_DB_11 0
#else
#include <driver/adc.h>
#include <esp_adc_cal.h>
#endif /* CONFIG_IDF_TARGET_ESP32C6 */

#define DEFAULT_VREF 1100 // tbd: use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES 32  // we do some multisampling to get better values

uint16_t read_voltage(void);
void calibrate_voltage(adc1_channel_t, adc_atten_t atten = ADC_ATTEN_DB_11);

#endif

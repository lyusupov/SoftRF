#if defined(ESP32)

#include <esp32-hal-log.h>
#include <soc/adc_channel.h>

#include <Arduino.h>

#include "battery.h"

#if !defined(ESP_IDF_VERSION_MAJOR) || ESP_IDF_VERSION_MAJOR < 5

// Local logging tag
static const char TAG[] = "ADC";

esp_adc_cal_characteristics_t *adc_characs =
    (esp_adc_cal_characteristics_t *)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
static adc1_channel_t adc1_channel = (adc1_channel_t) ADC1_GPIO36_CHANNEL;
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
static adc1_channel_t adc1_channel = (adc1_channel_t) ADC1_GPIO9_CHANNEL;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static adc1_channel_t adc1_channel = (adc1_channel_t) ADC1_GPIO2_CHANNEL;
static adc2_channel_t adc2_channel = (adc2_channel_t) ADC2_GPIO17_CHANNEL;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
static adc1_channel_t adc1_channel = (adc1_channel_t) ADC1_GPIO1_CHANNEL;
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32 */

static const adc_unit_t unit = ADC_UNIT_1;

void adc1_calibrate_voltage(adc1_channel_t channel, adc_atten_t atten) {

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  adc_bits_width_t width = ADC_WIDTH_BIT_12;
#else
  adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

  adc1_channel = channel;

  // configure ADC
  ESP_ERROR_CHECK(adc1_config_width(width));
  ESP_ERROR_CHECK(adc1_config_channel_atten(adc1_channel, atten));

  esp_err_t ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
  if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
  } else if (ret == ESP_ERR_INVALID_VERSION) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else if (ret == ESP_OK) {
    // calibrate ADC
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        unit, atten, width, DEFAULT_VREF, adc_characs);
    // show ADC characterization base
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      ESP_LOGI(TAG,
               "ADC characterization based on Two Point values stored in eFuse");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      ESP_LOGI(TAG,
               "ADC characterization based on reference voltage stored in eFuse");
    } else {
      ESP_LOGI(TAG, "ADC characterization based on default reference voltage");
    }
  } else {
      ESP_LOGE(TAG, "Invalid arg");
  }
}

#if defined(CONFIG_IDF_TARGET_ESP32S3)
void adc2_calibrate_voltage(adc2_channel_t channel, adc_atten_t atten) {

  adc_bits_width_t width = ADC_WIDTH_BIT_12;
  adc2_channel = channel;

  // configure ADC
//  ESP_ERROR_CHECK(adc2_config_width(width));
  ESP_ERROR_CHECK(adc2_config_channel_atten(adc2_channel, atten));

  esp_err_t ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
  if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
  } else if (ret == ESP_ERR_INVALID_VERSION) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else if (ret == ESP_OK) {
    // calibrate ADC
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_2, atten, width, DEFAULT_VREF, adc_characs);
    // show ADC characterization base
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      ESP_LOGI(TAG,
               "ADC characterization based on Two Point values stored in eFuse");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      ESP_LOGI(TAG,
               "ADC characterization based on reference voltage stored in eFuse");
    } else {
      ESP_LOGI(TAG, "ADC characterization based on default reference voltage");
    }
  } else {
      ESP_LOGE(TAG, "Invalid arg");
  }
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

uint16_t adc1_read_voltage() {
  // multisample ADC
  uint32_t adc1_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc1_reading += adc1_get_raw(adc1_channel);
    yield();
  }
  adc1_reading /= NO_OF_SAMPLES;
  // Convert ADC reading to voltage in mV
  uint16_t voltage =
      (uint16_t)esp_adc_cal_raw_to_voltage(adc1_reading, adc_characs);
#ifdef BATT_FACTOR
  voltage *= BATT_FACTOR;
#endif
  ESP_LOGD(TAG, "Raw: %d / Voltage: %dmV", adc1_reading, voltage);
  return voltage;
}

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static int prev_adc2_reading = 0;

uint16_t adc2_read_voltage() {
  adc_bits_width_t width = ADC_WIDTH_BIT_12;

  // multisample ADC
  uint32_t adc2_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    int raw = 0;

    esp_err_t err = adc2_get_raw(adc2_channel, width, &raw);
    if ( err == ESP_OK ) {
      adc2_reading += raw;
      prev_adc2_reading = raw;
    } else if ( err == ESP_ERR_TIMEOUT ) {
      // if WiFi is in use, such error code could be returned.
      adc2_reading += prev_adc2_reading;
      // prev_adc2_reading = 0;
    }

    yield();
  }
  adc2_reading /= NO_OF_SAMPLES;
  // Convert ADC reading to voltage in mV
  uint16_t voltage =
      (uint16_t)esp_adc_cal_raw_to_voltage(adc2_reading, adc_characs);
#ifdef BATT_FACTOR
  voltage *= BATT_FACTOR;
#endif
  ESP_LOGD(TAG, "Raw: %d / Voltage: %dmV", adc2_reading, voltage);
  return voltage;
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#else

static uint8_t adc_pin = 1;

void calibrate_voltage(uint8_t pin, adc_attenuation_t atten) {
  adc_pin = pin;
  analogReadResolution(12);
  analogSetPinAttenuation(adc_pin, atten);
}

uint16_t adc1_read_voltage() {
  // multisample ADC
  uint32_t adc_reading = 0;

  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += analogReadMilliVolts(adc_pin);
    yield();
  }

  adc_reading /= NO_OF_SAMPLES;

  return (uint16_t) adc_reading;
}

#endif /* ESP_IDF_VERSION_MAJOR */
#endif /* ESP32 */

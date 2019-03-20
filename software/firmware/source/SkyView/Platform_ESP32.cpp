/*
 * Platform_ESP32.cpp
 * Copyright (C) 2019 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#if defined(ESP32)

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>

#include "SoCHelper.h"
#include "EPDHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"

#include "SkyView.h"

#include <battery.h>

WebServer server ( 80 );

/*
 * TTGO-T5S. Pin definition

#define BUSY_PIN        4
#define CS_PIN          5
#define RST_PIN         16
#define DC_PIN          17
#define SCK_PIN         18
#define MOSI_PIN        23

P1-1                    21
P1-2                    22

I2S                     27
                        32
                        33

B0                      0?
B1                      38
B2                      37
B3                      39

SD                      2
                        13
                        14
                        15

P2                      0
                        12
                        13
                        RXD
                        TXD
                        34
                        35
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

/*
 * Waveshare E-Paper ESP32 Driver Board

#define SCK_PIN         13
#define MOSI_PIN        14
#define CS_PIN          15
#define BUSY_PIN        25
#define RST_PIN         26
#define DC_PIN          27

B1                      0
LED                     2

RX0, TX0                3,1

P                       0,2,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34,35
 */
// GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(/*CS=15*/ SS, /*DC=*/ 27, /*RST=*/ 26, /*BUSY=*/ 25));

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static void ESP32_setup()
{
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }

  SPI.begin(SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static const int8_t ESP32_dB_to_power_level[21] = {
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) {
    dB = 20;
  }

  if (dB < 0) {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static void ESP32_Battery_setup()
{
  calibrate_voltage(hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                    ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
}

static float ESP32_Battery_voltage()
{
  float voltage = ((float) read_voltage()) * 0.001 ;

  /* T-Beam has voltage divider 100k/100k on board */
  return (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ? 2 * voltage : voltage);
}

static void ESP32_SPI_begin()
{
  SPI.begin(SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_getChipId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_hostname,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_WiFiUDP_stopAll,
  ESP32_Battery_setup,
  ESP32_Battery_voltage
};

#endif /* ESP32 */

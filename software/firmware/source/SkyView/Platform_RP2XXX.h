/*
 * Platform_RP2XXX.h
 * Copyright (C) 2023-2025 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)

#ifndef PLATFORM_RP2XXX_H
#define PLATFORM_RP2XXX_H

#if defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#define Serial_setDebugOutput(x) ({})
#define WIFI_STA_TIMEOUT         20000
extern WebServer server;
#else
#include <Arduino.h>
#define EXCLUDE_WIFI
#define EXCLUDE_BLUETOOTH
#endif /* ARDUINO_RASPBERRY_PI_PICO_W or 2W */

#define EXCLUDE_ETHERNET

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define SOC_ADC_VOLTAGE_DIV   (3.0) // 20K + 10K voltage divider of VSYS

#define SerialOutput          Serial1
#define SerialInput           Serial2

#if !defined(ARDUINO_ARCH_MBED)
#define USBSerial             Serial
#else
#define USBSerial             SerialUSB
#endif /* ARDUINO_ARCH_MBED */

#define EEPROM_commit()       EEPROM.commit()

#define INA219_ADDRESS_ALT    (0x43) // 1000011 (A0=SCL, A1=GND)

#define MAX_FILENAME_LEN      64
#define WAV_FILE_PREFIX       "/Audio/"

#define POWER_SAVING_WIFI_TIMEOUT 600000UL /* 10 minutes */

#define LED_STATE_ON          HIGH  // State when LED is litted

enum RP2xxx_board_id {
  RP2040_RAK11300,
  RP2040_RESERVED1,
  RP2040_RPIPICO,
  RP2040_RPIPICO_W,
  RP2040_WEACT,
  RP2350_RPIPICO_2,
  RP2350_RPIPICO_2W,
};

/* Console I/O */
#define SOC_GPIO_PIN_CONS_RX  (1u)
#define SOC_GPIO_PIN_CONS_TX  (0u)

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX  (5u)
#define SOC_GPIO_PIN_GNSS_TX  (4u)

/* Raspberry Pico W SPI pins mapping */
#define SOC_W_PIN_MOSI        (24u)
#define SOC_W_PIN_SCK         (29u)
#define SOC_W_PIN_SS          (25u)

/* Waveshare Pico SPI pins mapping */
#define SOC_EPD_PIN_MOSI_WS   (11u)
#define SOC_EPD_PIN_MISO_WS   (28u)
#define SOC_EPD_PIN_SCK_WS    (10u)
#define SOC_EPD_PIN_SS_WS     (9u)

/* Waveshare Pico EPD pins mapping */
#define SOC_EPD_PIN_DC_WS     (8u)
#define SOC_EPD_PIN_RST_WS    (12u)
#define SOC_EPD_PIN_BUSY_WS   (13u)

/* Waveshare Pico UPS-B I2C1 */
#define SOC_GPIO_PIN_SDA      (6u)
#define SOC_GPIO_PIN_SCL      (7u)

/* Waveshare Pico keys mapping */
#define SOC_GPIO_PIN_KEY0     (15u)
#define SOC_GPIO_PIN_KEY1     (17u)
#define SOC_GPIO_PIN_KEY2     (2u)

#define SOC_GPIO_PIN_USBH_DP  (20u)  // Pin used as D+ for host, D- = D+ + 1
#define SOC_GPIO_PIN_USBH_DN  (21u)

#define SOC_GPIO_PIN_SMPS     (23u) // Pico
#define SOC_GPIO_PIN_BUTTON   (23u) // WeAct
#define SOC_GPIO_PIN_CYW43_PW (23u) // Pico W
#define SOC_GPIO_PIN_VBUS     (24u) // Pico
#define SOC_GPIO_PIN_LED      (25u) // Pico/WeAct
#define SOC_GPIO_PIN_CYW43_EN (25u) // Pico W
#define SOC_GPIO_PIN_VSYS     (29u) // Pico/Pico W
#define SOC_GPIO_PIN_BATTERY  SOC_GPIO_PIN_VSYS
#define SOC_GPIO_PIN_LED_W    (64u) // Pico W (CYW43 GPIO 0)
#define SOC_GPIO_PIN_SMPS_W   (65u) // Pico W (CYW43 GPIO 1)
#define SOC_GPIO_PIN_VBUS_W   (66u) // Pico W (CYW43 GPIO 2)

//#define EXCLUDE_AUDIO

#define SOC_GPIO_PIN_PWM_OUT  (3u)

/* Waveshare Pico-Audio, PCM5101A I2S DAC */
#define SOC_GPIO_PIN_DATA     (26u)
#define SOC_GPIO_PIN_BCK      (27u)
#define SOC_GPIO_PIN_LRCK     (28u)
#define SOC_GPIO_PIN_MCK      SOC_UNUSED_PIN

/* Waveshare Pico-Audio Rev2.1, CS4344 I2S DAC */
//#define SOC_GPIO_PIN_DATA     (22u)
//#define SOC_GPIO_PIN_BCK      (28u)
//#define SOC_GPIO_PIN_LRCK     (27u)
//#define SOC_GPIO_PIN_MCK      (26u)

#if !defined(ARDUINO_ARCH_MBED)
#define USE_BOOTSEL_BUTTON
#else
#define EXCLUDE_EEPROM
#endif /* ARDUINO_ARCH_MBED */

/* Experimental */
#if !defined(EXCLUDE_AUDIO)
//#define USE_EXT_I2S_DAC
#endif /* EXCLUDE_AUDIO */
#if defined(USE_TINYUSB) // && !defined(USE_EXT_I2S_DAC)
#define USE_USB_HOST
#endif /* USE_TINYUSB */

#endif /* PLATFORM_RP2XXX_H */
#endif /* ARDUINO_ARCH_RP2XXX */

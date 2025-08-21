/*
 * Platform_RK35.h
 * Copyright (C) 2025 Linar Yusupov
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

#if defined(LUCKFOX_LYRA)

#ifndef PLATFORM_RK35_H
#define PLATFORM_RK35_H

#include <raspi/TTYSerial.h>
#include <raspi/raspi.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    9

#define EEPROM_commit()         {}

#define PCM_DEVICE              "default"
#define WAV_FILE_PREFIX         "Audio/"

#define HTTP_SRV_PORT           8081 /* port 8080 can cause conflict with dump1090 */

/* Content of /etc/luckfox.cfg for Lyra board

TP_STATUS=0
SPI0_STATUS=1
SPI0_SPEED=2000000
SPI0_SCLK_RM_IO=7
SPI0_MOSI_RM_IO=6
SPI0_MISO_RM_IO=10
SPI0_CS_RM_IO=8
I2C1_STATUS=1
I2C1_SPEED=400000
I2C1_SDA_RM_IO=27
I2C1_SCL_RM_IO=26
UART1_STATUS=1
UART1_RX_RM_IO=1
UART1_TX_RM_IO=0

*/

/* Waveshare Pico e-Paper 2.7" buttons mapping */
#define SOC_GPIO_PIN_KEY0    2
#define SOC_GPIO_PIN_KEY1    30
#define SOC_GPIO_PIN_KEY2    12

#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET
//#define EXCLUDE_EEPROM

extern TTYSerial SerialInput;

#endif /* PLATFORM_RK35_H */

#endif /* LUCKFOX_LYRA */

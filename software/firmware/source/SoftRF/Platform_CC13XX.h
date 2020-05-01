/*
 * Platform_CC13XX.h
 * Copyright (C) 2019-2020 Linar Yusupov
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
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#ifndef PLATFORM_CC13XX_H
#define PLATFORM_CC13XX_H

#include "IPAddress.h"
#include <WS2812.h>

#if defined(ENERGIA_ARCH_CC13X2)
extern size_t strnlen (const char *string, size_t length);
#endif /* ENERGIA_ARCH_CC13X2 */

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.sendBuffer(LEDs, strip.numPixels())
#define uni_setPixelColor(i, c) ({ uint32_t temp = c;          \
                                   LEDs[i][2] = (temp & 0xff); \
                                   temp = temp >> 8;           \
                                   LEDs[i][1] = (temp & 0xff); \
                                   temp = temp >> 8;           \
                                   LEDs[i][0] = (temp & 0xff); })
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define SerialOutput            Serial
#define swSer                   Serial
#define UATSerial               Serial /* TBD */
#define yield()                 ({ })
#define snprintf_P              snprintf

#define SSD1306_OLED_I2C_ADDR   0x3C

#define SOC_GPIO_PIN_SS         18               // GPIO 11
#define SOC_GPIO_PIN_RST        LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUZZER     GREEN_LED        // GPIO 7

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN
#define SOC_GPIO_PIN_GNSS_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS     SOC_UNUSED_PIN
#define SOC_GPIO_PIN_LED        15 // MOSI (DIO_09)

extern WS2812 strip;
extern uint8_t LEDs[][3];

#if defined(ENERGIA_ARCH_CC13XX)

/*
 *  UART pins
 *
 * Board_UART_TX               GPIO 3
 * Board_UART_RX               GPIO 2
 * BootLoader                  GPIO 1
 */

/*
 * Built-in 128K flash memory of the CC1310F128 (7x7)
 * does fit for either:
 * - RECEIVER & BRIDGE modes, or
 * - NORMAL mode
 * but not both at the same time.
 */
#define ENABLE_OTHER_MODES

#define EXCLUDE_BMP180
#define EXCLUDE_MPL3115A2
#define EXCLUDE_NRF905
#define EXCLUDE_D1090
#define EXCLUDE_EGM96

#elif defined(ENERGIA_ARCH_CC13X2)

/*
 *  UART pins
 *
 * Board_UART_TX               GPIO 13
 * Board_UART_RX               GPIO 12
 * BootLoader                  GPIO 15
 */

#define ENABLE_NORMAL_MODE
#define ENABLE_OTHER_MODES

#define USE_OLED                 //  +5.5 kb
#define USE_NMEA_CFG             //  +3.3 kb
#define EXCLUDE_NRF905
#define EXCLUDE_EGM96

//#define USE_BASICMAC

#else
#error "This hardware platform is not supported!"
#endif /* ENERGIA_ARCH_CC13X0 & ENERGIA_ARCH_CC13X2 */

#endif /* PLATFORM_CC13XX_H */

#endif /* ENERGIA_ARCH_CC13X0 || ENERGIA_ARCH_CC13X2 */

/*
 * Platform_CC13XX.h
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
#if defined(ENERGIA_ARCH_CC13XX)

#ifndef PLATFORM_CC13XX_H
#define PLATFORM_CC13XX_H

#include "IPAddress.h"
#include <WS2812.h>

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

#define swSer                   Serial
#define UATSerial               Serial /* TBD */
#define yield()                 ({ })
#define snprintf_P              snprintf

#define SOC_GPIO_PIN_SS         18               // GPIO 11
#define SOC_GPIO_PIN_RST        LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUZZER     GREEN_LED        // GPIO 7

/* 
 *  UART pins
 *
 * Board_UART_TX               GPIO 3
 * Board_UART_RX               GPIO 2
 * BootLoader                  GPIO 1
 */

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN
#define SOC_GPIO_PIN_GNSS_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS     SOC_UNUSED_PIN

extern WS2812 strip;
extern uint8_t LEDs[][3];

#endif /* PLATFORM_CC13XX_H */

#endif /* ENERGIA_ARCH_CC13XX */

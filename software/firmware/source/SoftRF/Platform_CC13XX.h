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

#include <ti/devices/cc13x0/driverlib/ioc.h>
//#include "Board.h"

#include <uat.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define swSer Serial
#define yield() ({ })

#define SOC_GPIO_PIN_MOSI     IOID_9  // Board_SPI0_MOSI
#define SOC_GPIO_PIN_MISO     IOID_8  // Board_SPI0_MISO
#define SOC_GPIO_PIN_SCK      IOID_10 // Board_SPI0_CLK
#define SOC_GPIO_PIN_SS       IOID_11
#define SOC_GPIO_PIN_RST      IOID_2
#define SOC_GPIO_PIN_DIO0     PIN_UNASSIGNED

#define SOC_GPIO_PIN_SDA      IOID_5  // Board_I2C0_SDA0
#define SOC_GPIO_PIN_SCL      IOID_4  // Board_I2C0_SCL0

/* 
 *  UART pins used by driver
 *
 * Board_UART_TX               IOID_3
 * Board_UART_RX               IOID_2
 * BootLoader                  IOID_13
 */

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

typedef struct Stratux_LPUATRadio_UART_frame {
  byte      magic1;
  byte      magic2;
  byte      magic3;
  byte      magic4;

  uint16_t  msgLen;

  int8_t    rssi;
  uint32_t  timestamp;

  uint8_t   data[LONG_FRAME_BYTES];
} Stratux_frame_t;

#endif /* PLATFORM_CC13XX_H */

#endif /* ENERGIA_ARCH_CC13XX */

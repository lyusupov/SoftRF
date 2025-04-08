/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      TouchDrv_GT9895_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-14
 *
 */
#pragma once


#if   defined(ARDUINO_T_DECK)
// T-Deck GT911
#define TOUCH_SDA 18
#define TOUCH_SCL 8
#define TOUCH_IRQ 16
#define TOUCH_RST -1
#elif defined(ARDUINO_T_AMOLED_191)
// T-Display-AMOLED 1.91 Inch CST816T✅
#define TOUCH_SDA 3
#define TOUCH_SCL 2
#define TOUCH_IRQ 21
#define TOUCH_RST -1
#elif defined(ARDUINO_T_AMOLED_191_QWIIC)
// T-Display-AMOLED 1.91 Inch QWIIC
#define SENSOR_SDA 3
#define SENSOR_SCL 2
#define SENSOR_IRQ 43//TX
#define SENSOR_RST 44//RX
#elif defined(ARDUINO_T_AMOLED_241)
// T4-S3 CST226SE ✅
#define TOUCH_SDA 6
#define TOUCH_SCL 7
#define TOUCH_IRQ 8
#define TOUCH_RST 17
#elif defined(ARDUINO_T_AMOLED_147)
// T-Display-Lite 1.47 Inch CHSC5816 ✅
#define SENSOR_SDA 1
#define SENSOR_SCL 2
#define SENSOR_IRQ 8
#define TOUCH_SDA  1
#define TOUCH_SCL  2
#define TOUCH_RST 14
#define TOUCH_IRQ  13
#elif defined(ARDUINO_T_DISPLAY_S3_PRO)
// T-Display-S3-Pro CST226SE✅
#define SENSOR_SDA 5
#define SENSOR_SCL 6
#define SENSOR_IRQ 21
#define SENSOR_RST 13
#elif defined(ARDUINO_T_DISPLAY_S3)
// T-Display-S3 CST816T✅
#define SENSOR_SDA 18
#define SENSOR_SCL 17
#define SENSOR_IRQ 16
#define SENSOR_RST 21
#elif defined(ARDUINO_T_EPD47_S3)
// T-EPD47 S3 GT911 2 Point touch✅
#define SENSOR_SDA 6
#define SENSOR_SCL 5
#define SENSOR_IRQ 15
#define SENSOR_RST 41
#elif defined(ARDUINO_T_WATCH_S3_U)
// T-Watch-S3-U
#define SENSOR_SDA 2
#define SENSOR_SCL 3
#define SENSOR_IRQ 12
#define SENSOR_RST 16
#elif defined(ARDUINO_T_WATCH_S3)
// T-Watch-S3
#define SENSOR_SDA 10
#define SENSOR_SCL 11
#define PCF8563_IRQ 17
#define BMA423_IRQ 14
#define FT6336_SDA 39
#define FT6336_SCL 40
#define FT6336_IRQ 16
#elif defined(ARDUINO_T_BEAM_S3_SUPREME)
#define OLED_SDA   (17)
#define OLED_SCL   (18)
#define SPI_MOSI   (35)
#define SPI_SCK    (36)
#define SPI_MISO   (37)
#define SPI_CS     (47)
#define IMU_CS     (34)
#define IMU_IRQ    (33)  //INTERRUPT PIN1 & PIN2 ,Use or logic to form a pin
#define RTC_IRQ    (14)
#define RTC_PMU_SDA (42)
#define RTC_PMU_SCL (41)
#elif defined(ARDUINO_BHI260_SENSOR)
#define SPI_MOSI     33
#define SPI_MISO     34
#define SPI_SCK      35
#define BHI260_CS    36
#define BHI260_IRQ   37
#define BHI260_RST   47
#define BHI260_POWER 48
#define SENSOR_SDA   9
#define SENSOR_SCL   8
#define USE_SPI_INTERFACE   1
#define TOUCH_BUTTON  14
#elif defined(ARDUINO_T_ULTIMA)
#define SPI_MOSI     4
#define SPI_MISO     7
#define SPI_SCK      26
#define BHI260_CS    27
#define BHI260_IRQ   5
#define BHI260_RST   -1
#define USE_SPI_INTERFACE   1
#endif






























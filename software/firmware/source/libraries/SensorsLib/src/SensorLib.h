/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      SensorLib.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-05
 */

#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#else
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#endif

#include "SensorLib_Version.h"
#include "DevicesPins.h"

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
#define PLATFORM_HAS_PRINTF
#endif

#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
#define SPIClass SPIClassRP2040
#endif

#ifdef _BV
#undef _BV
#endif
#define _BV(b)                          (1UL << (uint32_t)(b))

#ifndef lowByte
#define lowByte(w)                      ((uint8_t) ((w) & 0xff))
#endif

#ifndef highByte
#define highByte(w)                     ((uint8_t) ((w) >> 8))
#endif

#ifndef bitRead
#define bitRead(value, bit)             (((value) >> (bit)) & 0x01)
#endif

#ifndef bitSet
#define bitSet(value, bit)              ((value) |= (1UL << (bit)))
#endif

#ifndef bitClear
#define bitClear(value, bit)            ((value) &= ~(1UL << (bit)))
#endif

#ifndef bitToggle
#define bitToggle(value, bit)           ((value) ^= (1UL << (bit)))
#endif

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue)  ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

#ifndef isBitSet
#define isBitSet(value, bit)            (((value) & (1UL << (bit))) == (1UL << (bit)))
#endif

#define ATTR_NOT_IMPLEMENTED            __attribute__((error("Not implemented")))


#if !defined(ARDUINO_ARCH_ESP32) && defined(LOG_PORT) && defined(ARDUINO) && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)

#define LOG_FILE_LINE_INFO __FILE__, __LINE__

#ifndef log_e
#define log_e(fmt, ...)                 LOG_PORT.printf("[E][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_e*/

#ifndef log_i
#define log_i(fmt, ...)                 LOG_PORT.printf("[I][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_i*/

#ifndef log_d
#define log_d(fmt, ...)                 LOG_PORT.printf("[D][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_d*/

#elif defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_ZEPHYR)

#define LOG_FILE_LINE_INFO __FILE__, __LINE__

#ifndef log_e
#define log_e(fmt, ...)                 printf("[E][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_e*/

#ifndef log_i
#define log_i(fmt, ...)                 printf("[I][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_i*/

#ifndef log_d
#define log_d(fmt, ...)                 printf("[D][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif  /*log_d*/

#elif defined(ESP_PLATFORM) && !defined(ARDUINO)

#include "esp_log.h"

#define ESP_TAG_LIB                     "SensorLib"
#if defined(__cplusplus) && (__cplusplus >  201703L)
#define log_e(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   ESP_TAG_LIB, format __VA_OPT__(,) __VA_ARGS__)
#define log_w(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    ESP_TAG_LIB, format __VA_OPT__(,) __VA_ARGS__)
#define log_i(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    ESP_TAG_LIB, format __VA_OPT__(,) __VA_ARGS__)
#define log_d(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   ESP_TAG_LIB, format __VA_OPT__(,) __VA_ARGS__)
#define log_v(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, ESP_TAG_LIB, format __VA_OPT__(,) __VA_ARGS__)
#else // !(defined(__cplusplus) && (__cplusplus >  201703L))
#define log_e(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   ESP_TAG_LIB, format, ##__VA_ARGS__)
#define log_w(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    ESP_TAG_LIB, format, ##__VA_ARGS__)
#define log_i(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    ESP_TAG_LIB, format, ##__VA_ARGS__)
#define log_d(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   ESP_TAG_LIB, format, ##__VA_ARGS__)
#define log_v(format, ... )             ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, ESP_TAG_LIB, format, ##__VA_ARGS__)
#endif // !(defined(__cplusplus) && (__cplusplus >  201703L))


#else   /*ESP_PLATFORM*/

#ifndef log_e
#define log_e(...)
#endif

#ifndef log_i
#define log_i(...)
#endif

#ifndef log_d
#define log_d(...)
#endif

#endif /*ARDUINO*/

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

#ifndef INPUT
#define INPUT                 (0x0)
#endif

#ifndef OUTPUT
#define OUTPUT                (0x1)
#endif

#ifndef RISING
#define RISING                (0x01)
#endif

#ifndef FALLING
#define FALLING               (0x02)
#endif

#ifndef LOW
#define LOW                   (0)
#endif

#ifndef HIGH
#define HIGH                  (1)
#endif

#endif


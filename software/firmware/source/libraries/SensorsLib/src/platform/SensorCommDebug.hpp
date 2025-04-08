/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      SensorCommDebug.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-20
 *
 */
#pragma once


// Usage example, define SENSORLIB_DEBUG as the platform's print function
// *Arduino  #define SENSORLIB_DEBUG Serial.printf */
// *Other    #define SENSORLIB_DEBUG printf */

#ifndef ARDUINO
#include <time.h>
#include <stdio.h>
#include <ctype.h>
#else
#include <Arduino.h>
#include <time.h>
#endif

#ifdef SENSORLIB_DEBUG
namespace
{
#ifdef __GNUC__
#define UNUSED __attribute__((unused))
#else
#define UNUSED
#endif


template <typename T>
static void DBG_PLAIN(T last);


template <typename T, typename... Args>
static void DBG_PLAIN(T head, Args... tail);


template <typename... Args>
static void DBG(Args... args);


template <typename T>
static void printParam(T value) UNUSED;

static void printParam(const char *str) UNUSED;

static void printParam(int num) UNUSED;

static void printParam(double num) UNUSED;

static void printParam(float num) UNUSED;

static void printParam(unsigned long num) UNUSED;


template <typename T>
static void DBG_PLAIN(T last)
{
    printParam(last);
    SENSORLIB_DEBUG("\n");
}

template <typename T, typename... Args>
static void DBG_PLAIN(T head, Args... tail)
{
    printParam(head);
    SENSORLIB_DEBUG(" ");
    DBG_PLAIN(tail...);
}

template <typename... Args>
static void DBG(Args... args)
{
    SENSORLIB_DEBUG("[");
    time_t now = time(NULL);
    SENSORLIB_DEBUG("%lu] ", (unsigned long)now);
    DBG_PLAIN(args...);
}


template <typename T>
void printParam(T value)
{
    SENSORLIB_DEBUG("%s", "Unsupported type");
}


void printParam(const char *str)
{
    SENSORLIB_DEBUG("%s", str);
}

void printParam(int num)
{
    SENSORLIB_DEBUG("%d", num);
}


void printParam(double num)
{
    SENSORLIB_DEBUG("%lf", num);
}


void printParam(float num)
{
    SENSORLIB_DEBUG("%f", num);
}


void printParam(unsigned long num)
{
    SENSORLIB_DEBUG("%lu", num);
}


}  // namespace
#else
#define DBG_PLAIN(...)
#define DBG(...)
#endif


void SensorLibDumpBuffer(const uint8_t *buffer, size_t size);


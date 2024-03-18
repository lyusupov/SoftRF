// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _debug_h_
#define _debug_h_

#ifndef CFG_DEBUG

#define debug_snprintf(s,n,f,...)	do { } while (0)
#define debug_printf(f,...)		do { } while (0)
#define debug_str(s)			do { } while (0)
#define debug_led(val)			do { } while (0)
#define debug_verbose_printf(f,...)		do { } while (0)

#ifdef CFG_DEBUG_VERBOSE
#error CFG_DEBUG_VERBOSE requires CFG_DEBUG
#endif

#else

#if defined(__AVR__) || defined(ARDUINO_ARCH_STM32) || \
    defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || \
    defined(ARDUINO_ARCH_RENESAS)
#include <avr/pgmspace.h>
#endif

#if defined(ESP32) || defined(ESP8266)
#include <pgmspace.h>
#endif

// write formatted string to buffer
//int debug_snprintf (char *str, int size, const char *format, ...);

// write formatted string to USART
void debug_printf_pstr (char const *format, ...);
#define debug_printf(format, ...) debug_printf_pstr(PSTR("%F: " format), (u4_t)os_getTime(), 0, ## __VA_ARGS__)

// write nul-terminated string to USART
//void debug_str (const char* str);

// set LED state
void debug_led (int val);

#ifndef CFG_DEBUG_VERBOSE
#define debug_verbose_printf(f,...)		do { } while (0)
#else
#define debug_verbose_printf debug_printf
#endif

#endif

#endif

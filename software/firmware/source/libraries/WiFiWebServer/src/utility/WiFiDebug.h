/****************************************************************************************************************************
  WiFiDebug.h - Dead simple web-server.
  For any WiFi shields, such as WiFiNINA W101, W102, W13x, or custom, such as ESP8266/ESP32-AT, Ethernet, etc

  WiFiWebServer is a library for the ESP32-based WiFi shields to run WebServer
  Forked and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Forked and modified from Arduino WiFiNINA library https://www.arduino.cc/en/Reference/WiFiNINA
  Built by Khoi Hoang https://github.com/khoih-prog/WiFiWebServer
  Licensed under MIT license

  Original author:
  @file       Esp8266WebServer.h
  @author     Ivan Grokhotkov

  Version: 1.10.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      12/02/2020 Initial coding for SAMD21, Nano 33 IoT, etc running WiFiNINA
  ...
  1.6.0   K Hoang      13/02/2022 Add support to new ESP32-S3 and ESP32_C3
  1.6.1   K Hoang      13/02/2022 Fix v1.6.0 issue
  1.6.2   K Hoang      22/02/2022 Add support to megaAVR using Arduino megaAVR core
  1.6.3   K Hoang      02/03/2022 Fix decoding error bug
  1.7.0   K Hoang      05/04/2022 Fix issue with Portenta_H7 core v2.7.2+
  1.8.0   K Hoang      26/04/2022 Add WiFiMulti library support and examples
  1.9.0   K Hoang      12/08/2022 Add support to RASPBERRY_PI_PICO_W using CYW4343 WiFi
  1.9.1   K Hoang      13/08/2022 Add WiFiMulti support to RASPBERRY_PI_PICO_W using CYW4343 WiFi
  1.9.2   K Hoang      16/08/2022 Workaround for RP2040W WiFi.status() bug
  1.9.3   K Hoang      16/08/2022 Better workaround for RP2040W WiFi.status() bug using ping() to local gateway
  1.9.4   K Hoang      06/09/2022 Restore support to ESP32 and ESP8266
  1.9.5   K Hoang      10/09/2022 Restore support to Teensy, etc. Fix bug in examples
  1.10.0  K Hoang      13/11/2022 Add new features, such as CORS. Update code and examples
  1.10.1  K Hoang      24/11/2022 Using new WiFi101_Generic library to send larger data
 *****************************************************************************************************************************/

#pragma once

#ifndef WiFiDebug_H
#define WiFiDebug_H

#if defined(ARDUINO)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif

#include <stdio.h>

#ifdef DEBUG_WIFI_WEBSERVER_PORT
  #define WS_DEBUG_OUTPUT DEBUG_WIFI_WEBSERVER_PORT
#else
  #define WS_DEBUG_OUTPUT Serial
#endif

// Change _WIFI_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _WIFI_LOGLEVEL_
  #define _WIFI_LOGLEVEL_       0
#endif

const char WWS_MARK[]  = "[WIFI] ";
const char WWS_SPACE[] = " ";
const char WWS_LINE[]  = "========================================\n";

#define WWS_PRINT_MARK   WWS_PRINT(WWS_MARK)
#define WWS_PRINT_SP     WWS_PRINT(WWS_SPACE)
#define WWS_PRINT_LINE   WWS_PRINT(WWS_LINE)

#define WWS_PRINT        WS_DEBUG_OUTPUT.print
#define WWS_PRINTLN      WS_DEBUG_OUTPUT.println

///////////////////////////////////////

#define WS_LOGERROR(x)         if(_WIFI_LOGLEVEL_>0) { WWS_PRINT_MARK; WWS_PRINTLN(x); }
#define WS_LOGERROR0(x)        if(_WIFI_LOGLEVEL_>0) { WWS_PRINT(x); }
#define WS_LOGERROR1(x,y)      if(_WIFI_LOGLEVEL_>0) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINTLN(y); }
#define WS_LOGERROR2(x,y,z)    if(_WIFI_LOGLEVEL_>0) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINTLN(z); }
#define WS_LOGERROR3(x,y,z,w)  if(_WIFI_LOGLEVEL_>0) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINTLN(w); }
#define WS_LOGERROR5(x,y,z,w, xx, yy)  if(_WIFI_LOGLEVEL_>0) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINT(w); WWS_PRINT_SP; WWS_PRINT(xx); WWS_PRINT_SP; WWS_PRINTLN(yy);}

///////////////////////////////////////

#define WS_LOGWARN(x)          if(_WIFI_LOGLEVEL_>1) { WWS_PRINT_MARK; WWS_PRINTLN(x); }
#define WS_LOGWARN0(x)         if(_WIFI_LOGLEVEL_>1) { WWS_PRINT(x); }
#define WS_LOGWARN1(x,y)       if(_WIFI_LOGLEVEL_>1) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINTLN(y); }
#define WS_LOGWARN2(x,y,z)     if(_WIFI_LOGLEVEL_>1) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINTLN(z); }
#define WS_LOGWARN3(x,y,z,w)   if(_WIFI_LOGLEVEL_>1) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINTLN(w); }
#define WS_LOGWARN5(x,y,z,w, xx, yy)  if(_WIFI_LOGLEVEL_>1) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINT(w); WWS_PRINT_SP; WWS_PRINT(xx); WWS_PRINT_SP; WWS_PRINTLN(yy);}

///////////////////////////////////////

#define WS_LOGINFO(x)          if(_WIFI_LOGLEVEL_>2) { WWS_PRINT_MARK; WWS_PRINTLN(x); }
#define WS_LOGINFO0(x)         if(_WIFI_LOGLEVEL_>2) { WWS_PRINT(x); }
#define WS_LOGINFO1(x,y)       if(_WIFI_LOGLEVEL_>2) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINTLN(y); }
#define WS_LOGINFO2(x,y,z)     if(_WIFI_LOGLEVEL_>2) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINTLN(z); }
#define WS_LOGINFO3(x,y,z,w)   if(_WIFI_LOGLEVEL_>2) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINTLN(w); }
#define WS_LOGINFO5(x,y,z,w, xx, yy)  if(_WIFI_LOGLEVEL_>2) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINT(w); WWS_PRINT_SP; WWS_PRINT(xx); WWS_PRINT_SP; WWS_PRINTLN(yy);}

///////////////////////////////////////

#define WS_LOGDEBUG(x)         if(_WIFI_LOGLEVEL_>3) { WWS_PRINT_MARK; WWS_PRINTLN(x); }
#define WS_LOGDEBUG0(x)        if(_WIFI_LOGLEVEL_>3) { WWS_PRINT(x); }
#define WS_LOGDEBUG1(x,y)      if(_WIFI_LOGLEVEL_>3) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINTLN(y); }
#define WS_LOGDEBUG2(x,y,z)    if(_WIFI_LOGLEVEL_>3) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINTLN(z); }
#define WS_LOGDEBUG3(x,y,z,w)  if(_WIFI_LOGLEVEL_>3) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINTLN(w); }
#define WS_LOGDEBUG5(x,y,z,w, xx, yy)  if(_WIFI_LOGLEVEL_>3) { WWS_PRINT_MARK; WWS_PRINT(x); WWS_PRINT_SP; WWS_PRINT(y); WWS_PRINT_SP; WWS_PRINT(z); WWS_PRINT_SP; WWS_PRINT(w); WWS_PRINT_SP; WWS_PRINT(xx); WWS_PRINT_SP; WWS_PRINTLN(yy);}

#endif    // WiFiDebug_H

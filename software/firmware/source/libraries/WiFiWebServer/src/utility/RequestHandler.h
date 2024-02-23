/****************************************************************************************************************************
  RequestHandler.h - Dead simple web-server.
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

#ifndef RequestHandler_H
#define RequestHandler_H

#ifndef WFW_UNUSED
  #define WFW_UNUSED(x) (void)(x)
#endif

#include "utility/WiFiDebug.h"

#include <vector>

////////////////////////////////////////

class RequestHandler
{
  public:

    virtual ~RequestHandler() { }

    ////////////////////////////////////////

    virtual bool canHandle(const HTTPMethod& method, const String& uri)
    {
      WFW_UNUSED(method);
      WFW_UNUSED(uri);

      return false;
    }

    ////////////////////////////////////////

    virtual bool canUpload(const String& uri)
    {
      WFW_UNUSED(uri);

      return false;
    }

    ////////////////////////////////////////

    virtual bool handle(WiFiWebServer& server, const HTTPMethod& requestMethod, /*const*/ String& requestUri)
    {
      WFW_UNUSED(server);
      WFW_UNUSED(requestMethod);
      WFW_UNUSED(requestUri);

      return false;
    }

    ////////////////////////////////////////

    virtual void upload(WiFiWebServer& server, const String& requestUri, const HTTPUpload& upload)
    {
      WFW_UNUSED(server);
      WFW_UNUSED(requestUri);
      WFW_UNUSED(upload);
    }

    ////////////////////////////////////////

    RequestHandler* next()
    {
      return _next;
    }

    ////////////////////////////////////////

    void next(RequestHandler* r)
    {
      _next = r;
    }

    ////////////////////////////////////////

  private:

    RequestHandler* _next = nullptr;

    ////////////////////////////////////////

  protected:
    std::vector<String> pathArgs;

    ////////////////////////////////////////

  public:

    ////////////////////////////////////////

    const String& pathArg(unsigned int i)
    {
      if (i < pathArgs.size())
      {
        return pathArgs[i];
      }
      else
      {
        WS_LOGERROR3(F("RequestHandler::pathArg: error i ="), i, F(" > pathArgs.size() ="), pathArgs.size());

        return pathArgs[0];
      }
    }
};

#endif    // RequestHandler_H

/****************************************************************************************************************************
  RequestHandlersImpl.h - Dead simple web-server.
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

#ifndef RequestHandlersImpl_H
#define RequestHandlersImpl_H

#if !(ESP32 || ESP8266)
#include "RequestHandler.h"
#include "mimetable.h"

////////////////////////////////////////
////////////////////////////////////////

class FunctionRequestHandler : public RequestHandler
{
  public:

    ////////////////////////////////////////

    FunctionRequestHandler(WiFiWebServer::THandlerFunction fn, WiFiWebServer::THandlerFunction ufn, const String &uri,
                           const HTTPMethod& method)
      : _fn(fn)
      , _ufn(ufn)
      , _uri(uri)
      , _method(method)
    {
    }

    ////////////////////////////////////////

    bool canHandle(const HTTPMethod& requestMethod, const String& requestUri) override
    {
      if (_method != HTTP_ANY && _method != requestMethod)
        return false;

      if (requestUri == _uri)
        return true;

      if (_uri.endsWith("/*"))
      {
        String _uristart = _uri;
        _uristart.replace("/*", "");

        if (requestUri.startsWith(_uristart))
          return true;
      }

      return false;
    }

    ////////////////////////////////////////

    bool canUpload(const String& requestUri) override
    {
      if (!_ufn || !canHandle(HTTP_POST, requestUri))
        return false;

      return true;
    }

    ////////////////////////////////////////

    bool handle(WiFiWebServer& server, const HTTPMethod& requestMethod, /*const*/ String& requestUri) override
    {
      WFW_UNUSED(server);

      if (!canHandle(requestMethod, requestUri))
        return false;

      _fn();
      return true;
    }

    ////////////////////////////////////////

    void upload(WiFiWebServer& server, const String& requestUri, const HTTPUpload& upload) override
    {
      WFW_UNUSED(server);
      WFW_UNUSED(upload);

      if (canUpload(requestUri))
        _ufn();
    }

    ////////////////////////////////////////

  protected:
    WiFiWebServer::THandlerFunction _fn;
    WiFiWebServer::THandlerFunction _ufn;
    String _uri;
    HTTPMethod _method;
};

////////////////////////////////////////
////////////////////////////////////////

class StaticRequestHandler : public RequestHandler
{
  public:

    ////////////////////////////////////////

    bool canHandle(const HTTPMethod& requestMethod, const String& requestUri) override
    {
      if (requestMethod != HTTP_GET)
        return false;

      if ((_isFile && requestUri != _uri) || !requestUri.startsWith(_uri))
        return false;

      return true;
    }

    ////////////////////////////////////////

#if USE_NEW_WEBSERVER_VERSION

    ////////////////////////////////////////

    static String getContentType(const String& path)
    {
      using namespace mime;
      char buff[sizeof(mimeTable[0].mimeType)];

      // Check all entries but last one for match, return if found
      for (size_t i = 0; i < sizeof(mimeTable) / sizeof(mimeTable[0]) - 1; i++)
      {
        strcpy(buff, mimeTable[i].endsWith);

        if (path.endsWith(buff))
        {
          strcpy(buff, mimeTable[i].mimeType);
          return String(buff);
        }
      }

      // Fall-through and just return default type
      strcpy(buff, mimeTable[sizeof(mimeTable) / sizeof(mimeTable[0]) - 1].mimeType);
      return String(buff);
    }

    ////////////////////////////////////////

#else   // #if USE_NEW_WEBSERVER_VERSION

    ////////////////////////////////////////

    static String getContentType(const String& path)
    {
      if (path.endsWith(".html"))
        return "text/html";
      else if (path.endsWith(".htm"))
        return "text/html";
      else if (path.endsWith(".css"))
        return "text/css";
      else if (path.endsWith(".txt"))
        return "text/plain";
      else if (path.endsWith(".js"))
        return "application/javascript";
      else if (path.endsWith(".png"))
        return "image/png";
      else if (path.endsWith(".gif"))
        return "image/gif";
      else if (path.endsWith(".jpg"))
        return "image/jpeg";
      else if (path.endsWith(".ico"))
        return "image/x-icon";
      else if (path.endsWith(".svg"))
        return "image/svg+xml";
      else if (path.endsWith(".ttf"))
        return "application/x-font-ttf";
      else if (path.endsWith(".otf"))
        return "application/x-font-opentype";
      else if (path.endsWith(".woff"))
        return "application/font-woff";
      else if (path.endsWith(".woff2"))
        return "application/font-woff2";
      else if (path.endsWith(".eot"))
        return "application/vnd.ms-fontobject";
      else if (path.endsWith(".sfnt"))
        return "application/font-sfnt";
      else if (path.endsWith(".xml"))
        return "text/xml";
      else if (path.endsWith(".pdf"))
        return "application/pdf";
      else if (path.endsWith(".zip"))
        return "application/zip";
      else if (path.endsWith(".gz"))
        return "application/x-gzip";
      else if (path.endsWith(".appcache"))
        return "text/cache-manifest";

      return "application/octet-stream";
    }

    ////////////////////////////////////////

#endif    // #if USE_NEW_WEBSERVER_VERSION

  protected:

    String  _uri;
    String  _path;
    String  _cache_header;
    bool    _isFile;
    size_t  _baseUriLength;
};

#else // #if !(ESP32 || ESP8266)
#include "ESP_RequestHandlersImpl.h"
#endif


#endif    // RequestHandlersImpl_H

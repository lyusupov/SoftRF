/****************************************************************************************************************************
  ESP_RequestHandlersImpl.h - Dead simple web-server.
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

#ifndef ESP_RequestHandlersImpl_H
#define ESP_RequestHandlersImpl_H

#include "RequestHandler.h"
#include "utility/esp_detail/mimetable.h"
#include "FS.h"
#include "WString.h"
#include <MD5Builder.h>
#include <base64.h>

#include "Uri.h"

#include "utility/WiFiDebug.h"

////////////////////////////////////////
////////////////////////////////////////

class FunctionRequestHandler : public RequestHandler
{
  public:

    FunctionRequestHandler(WiFiWebServer::THandlerFunction fn, WiFiWebServer::THandlerFunction ufn, const Uri &uri,
                           const HTTPMethod& method)
      : _fn(fn)
      , _ufn(ufn)
      , _uri(uri.clone())
      , _method(method)
    {
      _uri->initPathArgs(pathArgs);
    }

    ~FunctionRequestHandler()
    {
      delete _uri;
    }

    ////////////////////////////////////////

    bool canHandle(const HTTPMethod& requestMethod, const String& requestUri) override
    {
      if (_method != HTTP_ANY && _method != requestMethod)
        return false;

      return _uri->canHandle(requestUri, pathArgs);
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

    Uri *_uri;

    HTTPMethod _method;
};

////////////////////////////////////////
////////////////////////////////////////

class StaticRequestHandler : public RequestHandler
{
    using WebServerType = WiFiWebServer;

  public:

    StaticRequestHandler(FS& fs, const char* path, const char* uri, const char* cache_header)
      : _fs(fs)
      , _uri(uri)
      , _path(path)
      , _cache_header(cache_header)
    {
      _isFile = fs.exists(path);
      _baseUriLength = _uri.length();
    }

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

    bool handle(WiFiWebServer& server, const HTTPMethod& requestMethod, /*const*/ String& requestUri) override
    {
      if (!canHandle(requestMethod, requestUri))
        return false;

      WS_LOGDEBUG3(F("StaticRequestHandler::handle: request ="), requestUri, F(", _uri ="), _uri);

      String path(_path);

      if (!_isFile)
      {
        // Base URI doesn't point to a file.
        // If a directory is requested, look for index file.
        if (requestUri.endsWith("/"))
          requestUri += "index.htm";

        // Append whatever follows this URI in request to get the file path.
        path += requestUri.substring(_baseUriLength);
      }

      WS_LOGDEBUG3(F("StaticRequestHandler::handle: path ="), path, F(", _isFile ="), _isFile);

      String contentType = getContentType(path);

      using namespace mime_esp;

      // look for gz file, only if the original specified path is not a gz.
      // So part only works to send gzip via content encoding when a non compressed is asked for
      // if you point the the path to gzip you will serve the gzip as content type "application/x-gzip"
      // not text or javascript etc...
      if (!path.endsWith(FPSTR(mimeTable[gz].endsWith)) && !_fs.exists(path))
      {
        String pathWithGz = path + FPSTR(mimeTable[gz].endsWith);

        if (_fs.exists(pathWithGz))
          path += FPSTR(mimeTable[gz].endsWith);
      }

      File f = _fs.open(path, "r");

      if (!f || !f.available())
        return false;

      if (_cache_header.length() != 0)
        server.sendHeader("Cache-Control", _cache_header);

      server.streamFile(f, contentType);

      return true;
    }

    ////////////////////////////////////////

    static String getContentType(const String& path)
    {
      using namespace mime_esp;

      char buff[sizeof(mimeTable[0].mimeType)];

      // Check all entries but last one for match, return if found
      for (size_t i = 0; i < sizeof(mimeTable) / sizeof(mimeTable[0]) - 1; i++)
      {
        strcpy_P(buff, mimeTable[i].endsWith);

        if (path.endsWith(buff))
        {
          strcpy_P(buff, mimeTable[i].mimeType);

          return String(buff);
        }
      }

      // Fall-through and just return default type
      strcpy_P(buff, mimeTable[sizeof(mimeTable) / sizeof(mimeTable[0]) - 1].mimeType);

      return String(buff);
    }

    ////////////////////////////////////////

    bool validMethod(HTTPMethod requestMethod)
    {
      return (requestMethod == HTTP_GET) || (requestMethod == HTTP_HEAD);
    }

    ////////////////////////////////////////

  protected:
    FS      _fs;
    String  _uri;
    String  _path;
    String  _cache_header;
    bool    _isFile;
    size_t  _baseUriLength;
};

////////////////////////////////////////
////////////////////////////////////////


class StaticFileRequestHandler : public StaticRequestHandler
{
    using SRH = StaticRequestHandler;
    using WebServerType = WiFiWebServer;

  public:

    ////////////////////////////////////////

    StaticFileRequestHandler(FS& fs, const char* path, const char* uri, const char* cache_header)
      :
      StaticRequestHandler{fs, path, uri, cache_header}
    {
      File f = SRH::_fs.open(path, "r");
      MD5Builder calcMD5;
      calcMD5.begin();
      calcMD5.addStream(f, f.size());
      calcMD5.calculate();
      calcMD5.getBytes(_ETag_md5);
      f.close();
    }

    ////////////////////////////////////////

    bool canHandle(const HTTPMethod& requestMethod, const String& requestUri) override
    {
      return SRH::validMethod(requestMethod) && requestUri == SRH::_uri;
    }

    ////////////////////////////////////////

    bool handle(WiFiWebServer& server, const HTTPMethod& requestMethod, const String& requestUri)
    {
      if (!canHandle(requestMethod, requestUri))
        return false;


      const String etag = "\"" + base64::encode(_ETag_md5, 16) + "\"";

      if (server.header("If-None-Match") == etag)
      {
        server.send(304);
        return true;
      }

      File f = SRH::_fs.open(SRH::_path, "r");

      if (!f)
        return false;

      if (!_isFile)
      {
        f.close();
        return false;
      }

      if (SRH::_cache_header.length() != 0)
        server.sendHeader("Cache-Control", SRH::_cache_header);

      server.sendHeader("ETag", etag);

      server.streamFile(f, mime_esp::getContentType(SRH::_path), requestMethod);
      return true;
    }

    ////////////////////////////////////////

  protected:
    uint8_t _ETag_md5[16];
};


#endif    // ESP_RequestHandlersImpl_H

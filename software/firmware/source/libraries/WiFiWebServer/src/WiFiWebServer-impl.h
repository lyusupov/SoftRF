/*********************************************************************************************************************************
  WiFiWebServer-impl.h - Dead simple web-server.
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
 **********************************************************************************************************************************/

#pragma once

#ifndef WiFiWebServer_Impl_H
#define WiFiWebServer_Impl_H

#include <Arduino.h>
#include <libb64/cencode.h>

#include "WiFiWebServer.hpp"
#include "utility/RequestHandlersImpl.h"
#include "utility/WiFiDebug.h"
#include "utility/mimetable.h"

const char * AUTHORIZATION_HEADER = "Authorization";

// New to use WWString

////////////////////////////////////////

WWString fromString(const String& str)
{
  return str.c_str();
}

////////////////////////////////////////

WWString fromString(const String&& str)
{
  return str.c_str();
}

////////////////////////////////////////

String fromWWString(const WWString& str)
{
  return str.c_str();
}

////////////////////////////////////////

String fromWWString(const WWString&& str)
{
  return str.c_str();
}

////////////////////////////////////////
////////////////////////////////////////

#if USE_NEW_WEBSERVER_VERSION

////////////////////////////////////////

#if (ESP32 || ESP8266)
WiFiWebServer::WiFiWebServer(IPAddress addr, int port)
  : _corsEnabled(false)
  , _server(addr, port)
  , _currentMethod(HTTP_ANY)
  , _currentVersion(0)
  , _currentStatus(HC_NONE)
  , _statusChange(0)
  , _nullDelay(true)
  , _currentHandler(nullptr)
  , _firstHandler(nullptr)
  , _lastHandler(nullptr)
  , _currentArgCount(0)
  , _currentArgs(nullptr)
  , _postArgsLen(0)
  , _postArgs(nullptr)
  , _headerKeysCount(0)
  , _currentHeaders(nullptr)
  , _contentLength(0)
  , _clientContentLength(0)
  , _chunked(false)
{
}
#endif

////////////////////////////////////////

WiFiWebServer::WiFiWebServer(int port)
  : _corsEnabled(false)
  , _server(port)
  , _currentMethod(HTTP_ANY)
  , _currentVersion(0)
  , _currentStatus(HC_NONE)
  , _statusChange(0)
  , _nullDelay(true)
  , _currentHandler(nullptr)
  , _firstHandler(nullptr)
  , _lastHandler(nullptr)
  , _currentArgCount(0)
  , _currentArgs(nullptr)
  , _postArgsLen(0)
  , _postArgs(nullptr)
  , _headerKeysCount(0)
  , _currentHeaders(nullptr)
  , _contentLength(0)
  , _clientContentLength(0)
  , _chunked(false)
{
}

////////////////////////////////////////

WiFiWebServer::~WiFiWebServer()
{
#if (ESP32 || ESP8266)
  _server.close();
#endif

  if (_currentHeaders)
    delete[]_currentHeaders;

  RequestHandler* handler = _firstHandler;

  while (handler)
  {
    RequestHandler* next = handler->next();
    delete handler;
    handler = next;
  }
}

////////////////////////////////////////

void WiFiWebServer::begin()
{
  close();
  _server.begin();

#if (ESP32 || ESP8266)
  _server.setNoDelay(true);
#endif
}

////////////////////////////////////////

void WiFiWebServer::begin(uint16_t port)
{
  close();
  _server.begin(port);

#if (ESP32 || ESP8266)
  _server.setNoDelay(true);
#endif
}

////////////////////////////////////////

#else   // #if USE_NEW_WEBSERVER_VERSION

////////////////////////////////////////

WiFiWebServer::WiFiWebServer(int port)
  : _corsEnabled(false)
  , _server(port)
  , _currentMethod(HTTP_ANY)
  , _currentVersion(0)
  , _currentHandler(nullptr)
  , _firstHandler(nullptr)
  , _lastHandler(nullptr)
  , _currentArgCount(0)
  , _currentArgs(nullptr)
  , _headerKeysCount(0)
  , _currentHeaders(nullptr)
  , _contentLength(0)
  , _chunked(false)
{
}

////////////////////////////////////////

WiFiWebServer::~WiFiWebServer()
{
  if (_currentHeaders)
    delete[]_currentHeaders;

  _headerKeysCount = 0;
  RequestHandler* handler = _firstHandler;

  while (handler)
  {
    RequestHandler* next = handler->next();
    delete handler;
    handler = next;
  }

  close();
}

////////////////////////////////////////

void WiFiWebServer::begin()
{
  _currentStatus = HC_NONE;
  _server.begin();

  if (!_headerKeysCount)
    collectHeaders(0, 0);
}

////////////////////////////////////////

#endif    // #if USE_NEW_WEBSERVER_VERSION

////////////////////////////////////////

bool WiFiWebServer::authenticate(const char * username, const char * password)
{
  if (hasHeader(AUTHORIZATION_HEADER))
  {
    String authReq = header(AUTHORIZATION_HEADER);

    if (authReq.startsWith("Basic"))
    {
      authReq = authReq.substring(6);
      authReq.trim();
      char toencodeLen = strlen(username) + strlen(password) + 1;
      char *toencode = new char[toencodeLen + 1];

      if (toencode == NULL)
      {
        authReq = String();

        return false;
      }

      char *encoded = new char[base64_encode_expected_len(toencodeLen) + 1];

      if (encoded == NULL)
      {
        authReq = String();
        delete[] toencode;

        return false;
      }

      sprintf(toencode, "%s:%s", username, password);

      if (base64_encode_chars(toencode, toencodeLen, encoded) > 0 && authReq.equals(encoded))
      {
        authReq = String();
        delete[] toencode;
        delete[] encoded;

        return true;
      }

      delete[] toencode;
      delete[] encoded;
    }

    authReq = String();
  }

  return false;
}

////////////////////////////////////////

void WiFiWebServer::requestAuthentication()
{
  sendHeader("WWW-Authenticate", "Basic realm=\"Login Required\"");
  send(401);
}

////////////////////////////////////////

void WiFiWebServer::on(const String &uri, WiFiWebServer::THandlerFunction handler)
{
  on(uri, HTTP_ANY, handler);
}

////////////////////////////////////////

void WiFiWebServer::on(const String &uri, HTTPMethod method, WiFiWebServer::THandlerFunction fn)
{
  on(uri, method, fn, _fileUploadHandler);
}

////////////////////////////////////////

void WiFiWebServer::on(const String &uri, HTTPMethod method, WiFiWebServer::THandlerFunction fn,
                       WiFiWebServer::THandlerFunction ufn)
{
  _addRequestHandler(new FunctionRequestHandler(fn, ufn, uri, method));
}

////////////////////////////////////////

void WiFiWebServer::addHandler(RequestHandler* handler)
{
  _addRequestHandler(handler);
}

////////////////////////////////////////

void WiFiWebServer::_addRequestHandler(RequestHandler* handler)
{
  if (!_lastHandler)
  {
    _firstHandler = handler;
    _lastHandler = handler;
  }
  else
  {
    _lastHandler->next(handler);
    _lastHandler = handler;
  }
}

////////////////////////////////////////

#if USE_NEW_WEBSERVER_VERSION

void WiFiWebServer::handleClient()
{
  if (_currentStatus == HC_NONE)
  {
    WiFiClient client = _server.available();

    if (!client)
    {
      if (_nullDelay)
      {
        delay(1);
      }

      return;
    }

    WS_LOGDEBUG(F("handleClient: New Client"));

    _currentClient = client;
    _currentStatus = HC_WAIT_READ;
    _statusChange = millis();
  }

  bool keepCurrentClient = false;
  bool callYield = false;

  if (_currentClient.connected() || _currentClient.available())
  {
    switch (_currentStatus)
    {
      case HC_NONE:
        // No-op to avoid C++ compiler warning
        break;

      case HC_WAIT_READ:

        // Wait for data from client to become available
        if (_currentClient.available())
        {
          if (_parseRequest(_currentClient))
          {
            _currentClient.setTimeout(HTTP_MAX_SEND_WAIT);
            _contentLength = CONTENT_LENGTH_NOT_SET;
            _handleRequest();

#if USE_WIFI_NINA || ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_SAMD_NANO_33_IOT) )

            // Fix for issue with Chrome based browsers: https://github.com/espressif/arduino-esp32/issues/3652
            // Remove this will hang boards using WiFNINA, such as
            // Nano_RP2040_Connect with arduino_pico core, Nano_33_IoT
            if (_currentClient.connected())
            {
              _currentStatus = HC_WAIT_CLOSE;
              _statusChange = millis();
              keepCurrentClient = true;
            }

#endif
          }
        }
        else
        {
          // !_currentClient.available()
          if (millis() - _statusChange <= HTTP_MAX_DATA_WAIT)
          {
            keepCurrentClient = true;
          }

          callYield = true;
        }

        break;

      case HC_WAIT_CLOSE:

        // Wait for client to close the connection
        if (millis() - _statusChange <= HTTP_MAX_CLOSE_WAIT)
        {
          keepCurrentClient = true;
          callYield = true;
        }
    }
  }

  if (!keepCurrentClient)
  {
    WS_LOGDEBUG(F("handleClient: Don't keepCurrentClient"));
    _currentClient = WiFiClient();
    _currentStatus = HC_NONE;
    // KH
    //_currentUpload.reset();
  }

  if (callYield)
  {
    yield();
  }

#if (USE_WIFI_NINA || WIFI_USE_PORTENTA_H7)
  // KH, fix bug relating to New NINA FW 1.4.0. Have to close the connection
  _currentClient.stop();
  WS_LOGDEBUG(F("handleClient: Client disconnected"));
#endif
}

////////////////////////////////////////

#else   // #if USE_NEW_WEBSERVER_VERSION

////////////////////////////////////////

// KH, rewritten for Portenta H7 from v1.4.0
void WiFiWebServer::handleClient()
{
  if (_currentStatus == HC_NONE)
  {
    WiFiClient client = _server.available();

    if (!client)
    {
      return;
    }

    WS_LOGDEBUG(F("handleClient: New Client"));

    _currentClient = client;
    _currentStatus = HC_WAIT_READ;
    _statusChange = millis();
  }

  if (!_currentClient.connected())
  {
    _currentStatus = HC_NONE;

    goto stopClient;
  }

  // Wait for data from client to become available
  if (_currentStatus == HC_WAIT_READ)
  {
    if (!_currentClient.available())
    {
      if (millis() - _statusChange > HTTP_MAX_DATA_WAIT)
      {
        WS_LOGDEBUG(F("handleClient: HTTP_MAX_DATA_WAIT Timeout"));

        _currentStatus = HC_NONE;

        goto stopClient;
      }

      yield();
      return;
    }

    WS_LOGDEBUG(F("handleClient: Parsing Request"));

    if (!_parseRequest(_currentClient))
    {
      WS_LOGDEBUG(F("handleClient: Can't parse request"));

      _currentStatus = HC_NONE;

      goto stopClient;
    }

    _currentClient.setTimeout(HTTP_MAX_SEND_WAIT);
    _contentLength = CONTENT_LENGTH_NOT_SET;

    _handleRequest();

    if (!_currentClient.connected())
    {
      WS_LOGINFO(F("handleClient: Connection closed"));

      _currentStatus = HC_NONE;

      goto stopClient;
    }
    else
    {
      _currentStatus = HC_WAIT_CLOSE;
      _statusChange = millis();
      return;
    }
  }

  if (_currentStatus == HC_WAIT_CLOSE)
  {
    if (millis() - _statusChange > HTTP_MAX_CLOSE_WAIT)
    {
      _currentStatus = HC_NONE;

      WS_LOGDEBUG(F("handleClient: HTTP_MAX_CLOSE_WAIT Timeout"));

      yield();
    }
    else
    {
      yield();
      return;
    }
  }

stopClient:

#if (USE_WIFI_NINA || WIFI_USE_PORTENTA_H7)
  // To be used with New NINA FW 1.4.0 and Portenta_H7 WiFi. Have to close the connection
  _currentClient.stop();
  WS_LOGDEBUG(F("handleClient: Client disconnected"));
#endif
}

#endif    // #if USE_NEW_WEBSERVER_VERSION

////////////////////////////////////////

void WiFiWebServer::close()
{
#if (ESP32 || ESP8266)
  _server.close();
#endif

  _currentStatus = HC_NONE;

  if (!_headerKeysCount)
    collectHeaders(0, 0);
}

////////////////////////////////////////

void WiFiWebServer::stop()
{
  close();
}

////////////////////////////////////////

void WiFiWebServer::sendHeader(const String& name, const String& value, bool first)
{
  WWString headerLine = fromString(name);

  headerLine += ": ";
  headerLine += fromString(value);
  headerLine += RETURN_NEWLINE;

  if (first)
  {
    _responseHeaders = fromWWString(headerLine + fromString(_responseHeaders));
  }
  else
  {
    _responseHeaders = fromWWString(fromString(_responseHeaders) + headerLine);
  }
}

////////////////////////////////////////

void WiFiWebServer::setContentLength(size_t contentLength)
{
  _contentLength = contentLength;
}

////////////////////////////////////////

void WiFiWebServer::_prepareHeader(String& response, int code, const char* content_type, size_t contentLength)
{
  WWString aResponse = fromString(response);

  aResponse = "HTTP/1." + fromString(String(_currentVersion)) + " ";
  aResponse += fromString(String(code));
  aResponse += " ";
  aResponse += fromString(_responseCodeToString(code));
  aResponse += RETURN_NEWLINE;

#if (ESP32 || ESP8266)
  using namespace mime_esp;
#else
  using namespace mime;
#endif

  if (!content_type)
    content_type = mimeTable[html].mimeType;

  sendHeader("Content-Type", content_type, true);

  if (_contentLength == CONTENT_LENGTH_NOT_SET)
  {
    sendHeader("Content-Length", String(contentLength));
  }
  else if (_contentLength != CONTENT_LENGTH_UNKNOWN)
  {
    sendHeader("Content-Length", String(_contentLength));
  }
  else if (_contentLength == CONTENT_LENGTH_UNKNOWN && _currentVersion)
  {
    //HTTP/1.1 or above client
    //let's do chunked
    _chunked = true;
    sendHeader("Accept-Ranges", "none");
    sendHeader("Transfer-Encoding", "chunked");
  }

  if (_corsEnabled)
  {
    sendHeader("Access-Control-Allow-Origin",  "*");
    sendHeader("Access-Control-Allow-Methods", "*");
    sendHeader("Access-Control-Allow-Headers", "*");
  }

  WS_LOGDEBUG(F("_prepareHeader sendHeader Conn close"));

  sendHeader("Connection", "close");

  aResponse += fromString(_responseHeaders);
  aResponse += RETURN_NEWLINE;

  response = fromWWString(aResponse);

  _responseHeaders = String("");
}

////////////////////////////////////////

void WiFiWebServer::_prepareHeader(WWString& response, int code, const char* content_type, size_t contentLength)
{
  response = "HTTP/1." + fromString(String(_currentVersion)) + " ";
  response += fromString(String(code));
  response += " ";
  response += fromString(_responseCodeToString(code));
  response += RETURN_NEWLINE;

#if (ESP32 || ESP8266)
  using namespace mime_esp;
#else
  using namespace mime;
#endif

  if (!content_type)
    content_type = mimeTable[html].mimeType;

  sendHeader("Content-Type", content_type, true);

  if (_contentLength == CONTENT_LENGTH_NOT_SET)
  {
    sendHeader("Content-Length", String(contentLength));
  }
  else if (_contentLength != CONTENT_LENGTH_UNKNOWN)
  {
    sendHeader("Content-Length", String(_contentLength));
  }
  else if (_contentLength == CONTENT_LENGTH_UNKNOWN && _currentVersion)
  {
    //HTTP/1.1 or above client
    //let's do chunked
    _chunked = true;
    sendHeader("Accept-Ranges", "none");
    sendHeader("Transfer-Encoding", "chunked");
  }
  else if (_contentLength != CONTENT_LENGTH_UNKNOWN)
  {
    sendHeader("Content-Length", String(_contentLength));
  }
  else if (_contentLength == CONTENT_LENGTH_UNKNOWN && _currentVersion)
  {
    //HTTP/1.1 or above client
    //let's do chunked
    _chunked = true;
    sendHeader("Accept-Ranges", "none");
    sendHeader("Transfer-Encoding", "chunked");
  }

  if (_corsEnabled)
  {
    sendHeader("Access-Control-Allow-Origin",  "*");
    sendHeader("Access-Control-Allow-Methods", "*");
    sendHeader("Access-Control-Allow-Headers", "*");
  }

  WS_LOGDEBUG(F("_prepareHeader sendHeader Conn close"));

  sendHeader("Connection", "close");

  response += fromString(_responseHeaders);
  response += RETURN_NEWLINE;

  _responseHeaders = String("");
}

////////////////////////////////////////

void WiFiWebServer::send(int code, const char* content_type, const String& content)
{
  WWString header;

  _prepareHeader(header, code, content_type, content.length());

  _currentClient.write((const uint8_t *)header.c_str(), header.length());

  if (content.length())
  {
    sendContent(content, content.length());
  }
}

////////////////////////////////////////

void WiFiWebServer::send(int code, char* content_type, const String& content, size_t contentLength)
{
  WWString header;

  char type[64];

  memccpy((void*)type, content_type, 0, sizeof(type));
  _prepareHeader(header, code, (const char* )type, contentLength);

  _currentClient.write((const uint8_t *) header.c_str(), header.length());

  if (contentLength)
  {
    sendContent(content, contentLength);
  }
}

////////////////////////////////////////

void WiFiWebServer::send(int code, char* content_type, const String& content)
{
  send(code, (const char*)content_type, content);
}

////////////////////////////////////////

void WiFiWebServer::send(int code, const String& content_type, const String& content)
{
  send(code, (const char*)content_type.c_str(), content);
}

////////////////////////////////////////

// KH New

void WiFiWebServer::send(int code, const char* content_type, const char* content)
{
  send(code, content_type, content, content ? strlen(content) : 0);
}

////////////////////////////////////////

void WiFiWebServer::send(int code, const char* content_type, const char* content, size_t contentLength)
{
  String header;

  _prepareHeader(header, code, content_type, contentLength);

  _currentClient.write((const uint8_t *) header.c_str(), header.length());

  if (contentLength)
  {
    sendContent(content, contentLength);
  }
}

////////////////////////////////////////

void WiFiWebServer::sendContent(const char* content, size_t contentLength)
{
  const char * footer = RETURN_NEWLINE;

  if (_chunked)
  {
    char chunkSize[11];

    WS_LOGDEBUG1(F("sendContent_char: _chunked, _currentVersion ="), _currentVersion);

    sprintf(chunkSize, "%x%s", contentLength, footer);
    _currentClient.write(chunkSize, strlen(chunkSize));
  }

  _currentClient.write(content, contentLength);

  if (_chunked)
  {
    _currentClient.write(footer, 2);

    if (contentLength == 0)
    {
      _chunked = false;
    }
  }
}

////////////////////////////////////////

void WiFiWebServer::sendContent(const String& content)
{
  sendContent(content.c_str(), content.length());
}

////////////////////////////////////////

void WiFiWebServer::sendContent(const String& content, size_t contentLength)
{
  sendContent(content.c_str(), contentLength);
}

////////////////////////////////////////

// KH, Restore PROGMEM commands
void WiFiWebServer::send_P(int code, PGM_P content_type, PGM_P content)
{
  size_t contentLength = 0;

  if (content != NULL)
  {
    contentLength = strlen_P(content);
  }

  String header;
  char type[64];

  memccpy_P((void*)type, (PGM_VOID_P)content_type, 0, sizeof(type));
  _prepareHeader(header, code, (const char* )type, contentLength);

#if !( defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4) )
  WS_LOGDEBUG1(F("send_P: len = "), contentLength);
  WS_LOGDEBUG1(F("content = "), content);
  WS_LOGDEBUG1(F("send_P: hdrlen = "), header.length());
  WS_LOGDEBUG1(F("header = "), header);
#endif

  _currentClient.write((const uint8_t *) header.c_str(), header.length());

  if (contentLength)
  {
    sendContent_P(content);
  }
}

////////////////////////////////////////

void WiFiWebServer::send_P(int code, PGM_P content_type, PGM_P content, size_t contentLength)
{
  WWString header;

  char type[64];

  memccpy_P((void*)type, (PGM_VOID_P)content_type, 0, sizeof(type));
  _prepareHeader(header, code, (const char* )type, contentLength);

#if !( defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4) )
  WS_LOGDEBUG1(F("send_P: len = "), contentLength);
  WS_LOGDEBUG1(F("content = "), content);
  WS_LOGDEBUG1(F("send_P: hdrlen = "), header.length());
  WS_LOGDEBUG1(F("header = "), fromWWString(header));
#endif

  _currentClient.write((const uint8_t *) header.c_str(), header.length());

  if (contentLength)
  {
    sendContent_P(content, contentLength);
  }
}

////////////////////////////////////////

void WiFiWebServer::sendContent_P(PGM_P content)
{
  sendContent_P(content, strlen_P(content));
}

////////////////////////////////////////

void WiFiWebServer::sendContent_P(PGM_P content, size_t contentLength)
{
  const char * footer = RETURN_NEWLINE;

  if (_chunked)
  {
    char chunkSize[11];

    WS_LOGDEBUG1(F("sendContent_P: _chunked, _currentVersion ="), _currentVersion);

    sprintf(chunkSize, "%x%s", contentLength, footer);
    _currentClient.write(chunkSize, strlen(chunkSize));
  }

  uint8_t* _sendContentBuffer = new uint8_t[SENDCONTENT_P_BUFFER_SZ];

  if (_sendContentBuffer)
  {
    uint16_t count = contentLength / SENDCONTENT_P_BUFFER_SZ;
    uint16_t remainder = contentLength % SENDCONTENT_P_BUFFER_SZ;
    uint16_t i = 0;

    for (i = 0; i < count; i++)
    {
      /* code */
      memcpy_P(_sendContentBuffer, &content[i * SENDCONTENT_P_BUFFER_SZ], SENDCONTENT_P_BUFFER_SZ);
      _currentClient.write(_sendContentBuffer, SENDCONTENT_P_BUFFER_SZ);
    }

    memcpy_P(_sendContentBuffer, &content[i * SENDCONTENT_P_BUFFER_SZ], remainder);
    _currentClient.write(_sendContentBuffer, remainder);

    delete [] _sendContentBuffer;
  }
  else
  {
    WS_LOGERROR1(F("sendContent_P: Error, can't allocate _sendContentBuffer, Sz ="), SENDCONTENT_P_BUFFER_SZ);

    return;
  }

  if (_chunked)
  {
    _currentClient.write(footer, 2);

    _chunked = false;
  }
}

////////////////////////////////////////

#if (ESP32 || ESP8266)

#include "FS.h"

////////////////////////////////////////

void WiFiWebServer::serveStatic(const char* uri, FS& fs, const char* path, const char* cache_header)
{
  _addRequestHandler(new StaticFileRequestHandler(fs, path, uri, cache_header));
}

////////////////////////////////////////

void WiFiWebServer::_streamFileCore(const size_t fileSize, const String & fileName, const String & contentType,
                                    const int code)
{
#if (ESP32 || ESP8266)
  using namespace mime_esp;
#else
  using namespace mime;
#endif

  setContentLength(fileSize);

  if (fileName.endsWith(String(FPSTR(mimeTable[gz].endsWith))) &&
      contentType != String(FPSTR(mimeTable[gz].mimeType)) &&
      contentType != String(FPSTR(mimeTable[none].mimeType)))
  {
    sendHeader(F("Content-Encoding"), F("gzip"));
  }

  send(code, contentType, emptyString);
}

#endif

////////////////////////////////////////

String WiFiWebServer::arg(const String& name)
{
  for (int i = 0; i < _currentArgCount; ++i)
  {
    if ( _currentArgs[i].key == name )
      return _currentArgs[i].value;
  }

  return String();
}

////////////////////////////////////////

String WiFiWebServer::arg(int i)
{
  if (i < _currentArgCount)
    return _currentArgs[i].value;

  return String();
}

////////////////////////////////////////

String WiFiWebServer::argName(int i)
{
  if (i < _currentArgCount)
    return _currentArgs[i].key;

  return String();
}

////////////////////////////////////////

int WiFiWebServer::args()
{
  return _currentArgCount;
}

////////////////////////////////////////

bool WiFiWebServer::hasArg(const String& name)
{
  for (int i = 0; i < _currentArgCount; ++i)
  {
    if (_currentArgs[i].key == name)
      return true;
  }

  return false;
}

////////////////////////////////////////

String WiFiWebServer::header(const String& name)
{
  for (int i = 0; i < _headerKeysCount; ++i)
  {
    if (_currentHeaders[i].key == name)
      return _currentHeaders[i].value;
  }

  return String();
}

////////////////////////////////////////

void WiFiWebServer::collectHeaders(const char* headerKeys[], const size_t headerKeysCount)
{
  _headerKeysCount = headerKeysCount + 1;

  if (_currentHeaders)
    delete[]_currentHeaders;

  _currentHeaders = new RequestArgument[_headerKeysCount];
  _currentHeaders[0].key = AUTHORIZATION_HEADER;

  for (int i = 1; i < _headerKeysCount; i++)
  {
    _currentHeaders[i].key = headerKeys[i - 1];
  }
}

////////////////////////////////////////

String WiFiWebServer::header(int i)
{
  if (i < _headerKeysCount)
    return _currentHeaders[i].value;

  return String();
}

////////////////////////////////////////

String WiFiWebServer::headerName(int i)
{
  if (i < _headerKeysCount)
    return _currentHeaders[i].key;

  return String();
}

////////////////////////////////////////

int WiFiWebServer::headers()
{
  return _headerKeysCount;
}

////////////////////////////////////////

bool WiFiWebServer::hasHeader(const String& name)
{
  for (int i = 0; i < _headerKeysCount; ++i)
  {
    if ((_currentHeaders[i].key == name) &&  (_currentHeaders[i].value.length() > 0))
      return true;
  }

  return false;
}

////////////////////////////////////////

String WiFiWebServer::hostHeader()
{
  return _hostHeader;
}

////////////////////////////////////////

void WiFiWebServer::onFileUpload(THandlerFunction fn)
{
  _fileUploadHandler = fn;
}

////////////////////////////////////////

void WiFiWebServer::onNotFound(THandlerFunction fn)
{
  _notFoundHandler = fn;
}

////////////////////////////////////////

void WiFiWebServer::_handleRequest()
{
  bool handled = false;

  if (!_currentHandler)
  {
    WS_LOGDEBUG(F("_handleRequest: request handler not found"));
  }
  else
  {
    WS_LOGDEBUG(F("_handleRequest handle"));

    handled = _currentHandler->handle(*this, _currentMethod, _currentUri);

    if (!handled)
    {
      WS_LOGDEBUG(F("_handleRequest: _handleRequest failed"));
    }
    else
    {
      WS_LOGDEBUG(F("_handleRequest OK"));
    }
  }

  if (!handled && _notFoundHandler)
  {
    WS_LOGDEBUG(F("_handleRequest: Call _notFoundHandler"));

    _notFoundHandler();
    handled = true;
  }

  if (!handled)
  {
#if (ESP32 || ESP8266)
    using namespace mime_esp;
#else
    using namespace mime;
#endif

    WS_LOGDEBUG(F("_handleRequest: Send Not found"));

    send(404, mimeTable[html].mimeType, String("Not found: ") + _currentUri);
    handled = true;
  }

  if (handled)
  {
    WS_LOGDEBUG(F("_handleRequest: _finalizeResponse"));

    _finalizeResponse();
  }

#if WIFI_USE_PORTENTA_H7
  WS_LOGDEBUG(F("_handleRequest: Clear _currentUri"));
  //_currentUri = String();
  WS_LOGDEBUG(F("_handleRequest: Done Clear _currentUri"));
#else
  _responseHeaders = String("");
#endif
}

////////////////////////////////////////

void WiFiWebServer::_finalizeResponse()
{
  if (_chunked)
  {
    sendContent(String());
  }
}

////////////////////////////////////////

String WiFiWebServer::_responseCodeToString(int code)
{
  switch (code)
  {
    case 100:
      return F("Continue");

    case 101:
      return F("Switching Protocols");

    case 200:
      return F("OK");

    case 201:
      return F("Created");

    case 202:
      return F("Accepted");

    case 203:
      return F("Non-Authoritative Information");

    case 204:
      return F("No Content");

    case 205:
      return F("Reset Content");

    case 206:
      return F("Partial Content");

    case 300:
      return F("Multiple Choices");

    case 301:
      return F("Moved Permanently");

    case 302:
      return F("Found");

    case 303:
      return F("See Other");

    case 304:
      return F("Not Modified");

    case 305:
      return F("Use Proxy");

    case 307:
      return F("Temporary Redirect");

    case 400:
      return F("Bad Request");

    case 401:
      return F("Unauthorized");

    case 402:
      return F("Payment Required");

    case 403:
      return F("Forbidden");

    case 404:
      return F("Not Found");

    case 405:
      return F("Method Not Allowed");

    case 406:
      return F("Not Acceptable");

    case 407:
      return F("Proxy Authentication Required");

    case 408:
      return F("Request Time-out");

    case 409:
      return F("Conflict");

    case 410:
      return F("Gone");

    case 411:
      return F("Length Required");

    case 412:
      return F("Precondition Failed");

    case 413:
      return F("Request Entity Too Large");

    case 414:
      return F("Request-URI Too Large");

    case 415:
      return F("Unsupported Media Type");

    case 416:
      return F("Requested range not satisfiable");

    case 417:
      return F("Expectation Failed");

    case 500:
      return F("Internal Server Error");

    case 501:
      return F("Not Implemented");

    case 502:
      return F("Bad Gateway");

    case 503:
      return F("Service Unavailable");

    case 504:
      return F("Gateway Time-out");

    case 505:
      return F("HTTP Version not supported");

    default:
      return "";
  }
}

////////////////////////////////////////

#endif    // WiFiWebServer_Impl_H

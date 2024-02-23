/****************************************************************************************************************************
  WiFi_WebSocketClient.cpp - Dead simple HTTP WebSockets Client.
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

// (c) Copyright Arduino. 2016
// Released under Apache License, version 2.0

#define _WIFI_LOGLEVEL_     0

#include "libb64/base64.h"

#include "utility/WiFiDebug.h"
#include "WiFi_HTTPClient/WiFi_WebSocketClient.h"

////////////////////////////////////////

WiFiWebSocketClient::WiFiWebSocketClient(Client& aClient, const char* aServerName, uint16_t aServerPort)
  : WiFiHttpClient(aClient, aServerName, aServerPort),
    iTxStarted(false),
    iRxSize(0)
{
}

////////////////////////////////////////

WiFiWebSocketClient::WiFiWebSocketClient(Client& aClient, const String& aServerName, uint16_t aServerPort)
  : WiFiHttpClient(aClient, aServerName, aServerPort),
    iTxStarted(false),
    iRxSize(0)
{
}

////////////////////////////////////////

WiFiWebSocketClient::WiFiWebSocketClient(Client& aClient, const IPAddress& aServerAddress, uint16_t aServerPort)
  : WiFiHttpClient(aClient, aServerAddress, aServerPort),
    iTxStarted(false),
    iRxSize(0)
{
}

////////////////////////////////////////

int WiFiWebSocketClient::begin(const char* aPath)
{
  // start the GET request
  beginRequest();
  connectionKeepAlive();

  int status = get(aPath);

  if (status == 0)
  {
    uint8_t randomKey[16];
    char base64RandomKey[25];

    // create a random key for the connection upgrade
    for (int i = 0; i < (int)sizeof(randomKey); i++)
    {
      randomKey[i] = random(0x01, 0xff);
    }

    memset(base64RandomKey, 0x00, sizeof(base64RandomKey));
    base64_encode(randomKey, sizeof(randomKey), (unsigned char*)base64RandomKey, sizeof(base64RandomKey));

    // start the connection upgrade sequence
    sendHeader("Upgrade", "websocket");
    sendHeader("Connection", "Upgrade");
    sendHeader("Sec-WebSocket-Key", base64RandomKey);
    sendHeader("Sec-WebSocket-Version", "13");
    endRequest();

    status = responseStatusCode();

    if (status > 0)
    {
      skipResponseHeaders();
    }
  }

  iRxSize = 0;

  // status code of 101 means success
  return (status == 101) ? 0 : status;
}

////////////////////////////////////////

int WiFiWebSocketClient::begin(const String& aPath)
{
  return begin(aPath.c_str());
}

////////////////////////////////////////

int WiFiWebSocketClient::beginMessage(int aType)
{
  if (iTxStarted)
  {
    // fail TX already started
    return 1;
  }

  iTxStarted = true;
  iTxMessageType = (aType & 0xf);
  iTxSize = 0;

  return 0;
}

////////////////////////////////////////

int WiFiWebSocketClient::endMessage()
{
  if (!iTxStarted)
  {
    // fail TX not started
    return 1;
  }

  // send FIN + the message type (opcode)
  WiFiHttpClient::write(0x80 | iTxMessageType);

  // the message is masked (0x80)
  // send the length
  if (iTxSize < 126)
  {
    WiFiHttpClient::write(0x80 | (uint8_t)iTxSize);
  }
  else if (iTxSize < 0xffff)
  {
    WiFiHttpClient::write(0x80 | 126);
    WiFiHttpClient::write((iTxSize >> 8) & 0xff);
    WiFiHttpClient::write((iTxSize >> 0) & 0xff);
  }
  else
  {
    WiFiHttpClient::write(0x80 | 127);
    WiFiHttpClient::write((iTxSize >> 56) & 0xff);
    WiFiHttpClient::write((iTxSize >> 48) & 0xff);
    WiFiHttpClient::write((iTxSize >> 40) & 0xff);
    WiFiHttpClient::write((iTxSize >> 32) & 0xff);
    WiFiHttpClient::write((iTxSize >> 24) & 0xff);
    WiFiHttpClient::write((iTxSize >> 16) & 0xff);
    WiFiHttpClient::write((iTxSize >>  8) & 0xff);
    WiFiHttpClient::write((iTxSize >>  0) & 0xff);
  }

  uint8_t maskKey[4];

  // create a random mask for the data and send
  for (int i = 0; i < (int)sizeof(maskKey); i++)
  {
    maskKey[i] = random(0xff);
  }

  WiFiHttpClient::write(maskKey, sizeof(maskKey));

  // mask the data and send
  for (int i = 0; i < (int)iTxSize; i++)
  {
    iTxBuffer[i] ^= maskKey[i % sizeof(maskKey)];
  }

  size_t txSize = iTxSize;

  iTxStarted = false;
  iTxSize = 0;

  return (WiFiHttpClient::write(iTxBuffer, txSize) == txSize) ? 0 : 1;
}

////////////////////////////////////////

size_t WiFiWebSocketClient::write(uint8_t aByte)
{
  return write(&aByte, sizeof(aByte));
}

////////////////////////////////////////

size_t WiFiWebSocketClient::write(const uint8_t *aBuffer, size_t aSize)
{
  if (iState < eReadingBody)
  {
    // have not upgraded the connection yet
    return WiFiHttpClient::write(aBuffer, aSize);
  }

  if (!iTxStarted)
  {
    // fail TX not   started
    return 0;
  }

  // check if the write size, fits in the buffer
  if ((iTxSize + aSize) > sizeof(iTxBuffer))
  {
    aSize = sizeof(iTxSize) - iTxSize;
  }

  // copy data into the buffer
  memcpy(iTxBuffer + iTxSize, aBuffer, aSize);

  iTxSize += aSize;

  return aSize;
}

////////////////////////////////////////

int WiFiWebSocketClient::parseMessage()
{
  flushRx();

  // make sure 2 bytes (opcode + length)
  // are available
  if (WiFiHttpClient::available() < 2)
  {
    return 0;
  }

  // read open code and length
  uint8_t opcode = WiFiHttpClient::read();
  int length = WiFiHttpClient::read();

  if ((opcode & 0x0f) == 0)
  {
    // continuation, use previous opcode and update flags
    iRxOpCode |= opcode;
  }
  else
  {
    iRxOpCode = opcode;
  }

  iRxMasked = (length & 0x80);
  length   &= 0x7f;

  // read the RX size
  if (length < 126)
  {
    iRxSize = length;
  }
  else if (length == 126)
  {
    iRxSize = (WiFiHttpClient::read() << 8) | WiFiHttpClient::read();
  }
  else
  {
    iRxSize =   ((uint64_t)WiFiHttpClient::read() << 56) |
                ((uint64_t)WiFiHttpClient::read() << 48) |
                ((uint64_t)WiFiHttpClient::read() << 40) |
                ((uint64_t)WiFiHttpClient::read() << 32) |
                ((uint64_t)WiFiHttpClient::read() << 24) |
                ((uint64_t)WiFiHttpClient::read() << 16) |
                ((uint64_t)WiFiHttpClient::read() << 8)  |
                (uint64_t)WiFiHttpClient::read();
  }

  // read in the mask, if present
  if (iRxMasked)
  {
    for (int i = 0; i < (int)sizeof(iRxMaskKey); i++)
    {
      iRxMaskKey[i] = WiFiHttpClient::read();
    }
  }

  iRxMaskIndex = 0;

  if (TYPE_CONNECTION_CLOSE == messageType())
  {
    flushRx();
    stop();
    iRxSize = 0;
  }
  else if (TYPE_PING == messageType())
  {
    beginMessage(TYPE_PONG);

    while (available())
    {
      write(read());
    }

    endMessage();

    iRxSize = 0;
  }
  else if (TYPE_PONG == messageType())
  {
    flushRx();
    iRxSize = 0;
  }

  return iRxSize;
}

////////////////////////////////////////

int WiFiWebSocketClient::messageType()
{
  return (iRxOpCode & 0x0f);
}

////////////////////////////////////////

bool WiFiWebSocketClient::isFinal()
{
  return ((iRxOpCode & 0x80) != 0);
}

////////////////////////////////////////

String WiFiWebSocketClient::readString()
{
  int avail = available();
  String s;

  if (avail > 0)
  {
    s.reserve(avail);

    for (int i = 0; i < avail; i++)
    {
      s += (char)read();
    }
  }

  return s;
}

////////////////////////////////////////

int WiFiWebSocketClient::ping()
{
  uint8_t pingData[16];

  // create random data for the ping
  for (int i = 0; i < (int)sizeof(pingData); i++)
  {
    pingData[i] = random(0xff);
  }

  beginMessage(TYPE_PING);
  write(pingData, sizeof(pingData));

  return endMessage();
}

////////////////////////////////////////

int WiFiWebSocketClient::available()
{
  if (iState < eReadingBody)
  {
    return WiFiHttpClient::available();
  }

  return iRxSize;
}

////////////////////////////////////////

int WiFiWebSocketClient::read()
{
  byte b;

  if (read(&b, sizeof(b)))
  {
    return b;
  }

  return -1;
}

////////////////////////////////////////

int WiFiWebSocketClient::read(uint8_t *aBuffer, size_t aSize)
{
  int readCount = WiFiHttpClient::read(aBuffer, aSize);

  if (readCount > 0)
  {
    iRxSize -= readCount;

    // unmask the RX data if needed
    if (iRxMasked)
    {
      for (int i = 0; i < (int)aSize; i++, iRxMaskIndex++)
      {
        aBuffer[i] ^= iRxMaskKey[iRxMaskIndex % sizeof(iRxMaskKey)];
      }
    }
  }

  return readCount;
}

////////////////////////////////////////

int WiFiWebSocketClient::peek()
{
  int p = WiFiHttpClient::peek();

  if (p != -1 && iRxMasked)
  {
    // unmask the RX data if needed
    p = (uint8_t)p ^ iRxMaskKey[iRxMaskIndex % sizeof(iRxMaskKey)];
  }

  return p;
}

////////////////////////////////////////

void WiFiWebSocketClient::flushRx()
{
  while (available())
  {
    read();
  }
}

////////////////////////////////////////


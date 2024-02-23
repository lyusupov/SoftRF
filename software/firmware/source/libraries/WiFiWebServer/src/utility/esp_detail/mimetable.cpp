/****************************************************************************************************************************
  mimetable.cpp - Dead simple web-server.
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

#if (ESP32 || ESP8266)

#include "mimetable.h"
#include "pgmspace.h"
#include "WString.h"

////////////////////////////////////////

namespace mime_esp
{
static const char kHtmlSuffix[]     PROGMEM = ".html";
static const char kHtmSuffix[]      PROGMEM = ".htm";
static const char kTxtSuffix[]      PROGMEM = ".txt";

static const char kCssSuffix[]      PROGMEM = ".css";
static const char kJsSuffix[]       PROGMEM = ".js";
static const char kJsonSuffix[]     PROGMEM = ".json";
static const char kPngSuffix[]      PROGMEM = ".png";
static const char kGifSuffix[]      PROGMEM = ".gif";
static const char kJpgSuffix[]      PROGMEM = ".jpg";
static const char kJpegSuffix[]     PROGMEM = ".jpeg";
static const char kIcoSuffix[]      PROGMEM = ".ico";
static const char kSvgSuffix[]      PROGMEM = ".svg";
static const char kTtfSuffix[]      PROGMEM = ".ttf";
static const char kOtfSuffix[]      PROGMEM = ".otf";
static const char kWoffSuffix[]     PROGMEM = ".woff";
static const char kWoff2Suffix[]    PROGMEM = ".woff2";
static const char kEotSuffix[]      PROGMEM = ".eot";
static const char kSfntSuffix[]     PROGMEM = ".sfnt";
static const char kXmlSuffix[]      PROGMEM = ".xml";
static const char kPdfSuffix[]      PROGMEM = ".pdf";
static const char kZipSuffix[]      PROGMEM = ".zip";
static const char kAppcacheSuffix[] PROGMEM = ".appcache";

static const char kGzSuffix[]       PROGMEM = ".gz";
static const char kDefaultSuffix[]  PROGMEM = "";

////////////////////////////////////////

static const char kHtml[]           PROGMEM = "text/html";
static const char kTxt[]            PROGMEM = "text/plain";

static const char kCss[]            PROGMEM = "text/css";
static const char kJs[]             PROGMEM = "application/javascript";
static const char kJson[]           PROGMEM = "application/json";
static const char kPng[]            PROGMEM = "image/png";
static const char kGif[]            PROGMEM = "image/gif";
static const char kJpg[]            PROGMEM = "image/jpeg";
static const char kJpeg[]           PROGMEM = "image/jpeg";
static const char kIco[]            PROGMEM = "image/x-icon";
static const char kSvg[]            PROGMEM = "image/svg+xml";
static const char kTtf[]            PROGMEM = "application/x-font-ttf";
static const char kOtf[]            PROGMEM = "application/x-font-opentype";
static const char kWoff[]           PROGMEM = "application/font-woff";
static const char kWoff2[]          PROGMEM = "application/font-woff2";
static const char kEot[]            PROGMEM = "application/vnd.ms-fontobject";
static const char kSfnt[]           PROGMEM = "application/font-sfnt";
static const char kXml[]            PROGMEM = "text/xml";
static const char kPdf[]            PROGMEM = "application/pdf";
static const char kZip[]            PROGMEM = "application/zip";
static const char kAppcache[]       PROGMEM = "text/cache-manifest";

static const char kGz[]             PROGMEM = "application/x-gzip";
static const char kDefault[]        PROGMEM = "application/octet-stream";

////////////////////////////////////////

const Entry mimeTable[maxType]      PROGMEM =
{
  { kHtmlSuffix,      kHtml },
  { kHtmSuffix,       kHtml },
  { kCssSuffix,       kCss },
  { kTxtSuffix,       kTxt },
  { kJsSuffix,        kJs },
  { kJsonSuffix,      kJson },
  { kPngSuffix,       kPng },
  { kGifSuffix,       kGif },
  { kJpgSuffix,       kJpg },
  { kIcoSuffix,       kIco },
  { kSvgSuffix,       kSvg },
  { kTtfSuffix,       kTtf },
  { kOtfSuffix,       kOtf },
  { kWoffSuffix,      kWoff },
  { kWoff2Suffix,     kWoff2 },
  { kEotSuffix,       kEot },
  { kSfntSuffix,      kSfnt },
  { kXmlSuffix,       kXml },
  { kPdfSuffix,       kPdf },
  { kZipSuffix,       kZip },
  { kGzSuffix,        kGz },
  { kAppcacheSuffix,  kAppcache },
  { kDefaultSuffix,   kDefault }
};

////////////////////////////////////////

String getContentType(const String& path)
{
  for (size_t i = 0; i < maxType; i++)
  {
    if (path.endsWith(FPSTR(mimeTable[i].endsWith)))
    {
      return String(FPSTR(mimeTable[i].mimeType));
    }
  }

  // Fall-through and just return default type
  return String(FPSTR(kDefault));
}

////////////////////////////////////////

}   // namespace mime_esp

#endif

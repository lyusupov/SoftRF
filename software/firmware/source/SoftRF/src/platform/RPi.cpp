/*
 * Platform_RPi.cpp
 * Copyright (C) 2018-2025 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Usage example:
 *
 *  pi@raspberrypi $ make -f Makefile.RPi
 *
 *     < ... skipped ... >
 *
 *  pi@raspberrypi $ { echo "{class:SOFTRF,protocol:OGNTP}" ; cat /dev/ttyUSB0 ; } | sudo ./SoftRF
 *  SX1276 RFIC is detected.
 *  $GPGSA,A,3,02,30,05,06,07,09,,,,,,,5.09,3.19,3.97*04
 *  $GPRMC,145750.00,A,5XXX.XXX68,N,03XXX.XXX33,E,0.701,,051118,,,A*7E
 *  $GPGGA,145750.00,5XXX.XXX68,N,03XXX.XXX33,E,1,06,3.19,179.2,M,12.5,M,,*5E
 *  $PFLAA,3,0,0,0,2,C5D804!OGN_C5D804,0,,0,00000.0,1*60
 *  $PFLAU,1,1,2,1,3,-30,2,0,0*4E
 *
 *     < ... skipped ... >
 *
 *  Concurrent shell session:
 *
 *  pi@raspberrypi $ ps -ax | grep dump1090
 *  2381 pts/1    Sl+    1:53 dump1090 --interactive --net
 *
 *  pi@raspberrypi $ wget -q -O - http://localhost:8080/data/aircraft.json | nc -N localhost 30007
 *
 */

#if defined(RASPBERRY_PI)

#include "../system/SoC.h"
#include "../driver/EEPROM.h"
#include <TinyGPS++.h>
#if !defined(EXCLUDE_MAVLINK)
#include <aircraft.h>
#endif /* EXCLUDE_MAVLINK */
#include "../driver/RF.h"
#include "../driver/LED.h"
#include "../driver/Sound.h"
#include "../driver/Baro.h"
#include "../TrafficHelper.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "../driver/WiFi.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Bluetooth.h"
#include "../system/Time.h"

#include "TCPServer.h"

#include <stdio.h>
#include <sys/select.h>

#include <iostream>

#include <ArduinoJson.h>

// Dragino LoRa/GPS HAT or compatible SX1276 pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
#if !defined(USE_OGN_RF_DRIVER)
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#else
    .dio = {SOC_GPIO_PIN_DIO0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#endif
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

TTYSerial Serial1("/dev/ttyAMA0");
TTYSerial Serial2("/dev/ttyUSB0");

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
#if defined(USE_BASICMAC)
void os_getJoinEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
void os_getNwkKey (u1_t* buf) { }
//u1_t os_getRegion (void) { return REGCODE_EU868; }
#else
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

#if defined(USE_BASICMAC)
extern "C" void onLmicEvent (ev_t ev);
void onLmicEvent (ev_t ev) {
#else
void onEvent (ev_t ev) {
#endif
}

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#else
EEPROMClass EEPROM;
#endif /* EXCLUDE_EEPROM */

#if defined(USE_OLED)
#include "../driver/OLED.h"

extern U8X8 u8x8_i2c;
#endif /* USE_OLED */

ufo_t ThisAircraft;

#if !defined(EXCLUDE_MAVLINK)
aircraft the_aircraft;
#endif /* EXCLUDE_MAVLINK */

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming and outgoing packets

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE,
  .rtc      = RTC_NONE,
  .imu      = IMU_NONE,
  .mag      = MAG_NONE,
  .pmu      = PMU_NONE,
};

#define isTimeToExport() (millis() - ExportTimeMarker > 1000)
unsigned long ExportTimeMarker = 0;

std::string input_line;

TCPServer Traffic_TCP_Server;

#if defined(USE_EPAPER)
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> __attribute__ ((common)) epd_waveshare_W3(GxEPD2_270(/*CS=5*/ 8,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> __attribute__ ((common)) epd_waveshare_T91(GxEPD2_270_T91(/*CS=5*/ 8,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));
GxEPD2_GFX *display;
#endif /* USE_EPAPER */

ui_settings_t ui_settings = {
    .adapter      = 0,
    .connection   = 0,
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .baudrate     = 0,
    .server       = { 0 },
    .key          = { 0 },
    .rotate       = ROTATE_0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_REG,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .power_save   = 0,
    .team         = 0
};

ui_settings_t *ui;

const char *Hardware_Rev[] = {
  [0] = "Unknown"
};

static int RPi_hat = RPI_DRAGINO_LORA_GPS; /* default */

#include "mode-s.h"
#include "sdr/common.h"

mode_s_t state;

#if defined(USE_BRIDGE)
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <BridgeUdp.h>
#include <aWOT.h>
#include <../ui/Web.h>

#include <netdb.h>

BridgeServer WebServer(HTTP_SRV_PORT);
Application WebApp;
BridgeUDP Uni_Udp;

static IPAddress dest_IP;

void index_page(Request &req, Response &res) {
  char *content = Root_content();

  if (content) {
    res.set(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
    res.set(F("Pragma"), F("no-cache"));
    res.set(F("Expires"), F("-1"));
    res.set("Content-Type", "text/html;");
    res.write( (uint8_t *) content, strlen(content) );

    free(content);
  }
}

void settings_page(Request &req, Response &res) {
  char *content = Settings_content();

  if (content) {
    res.set(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
    res.set(F("Pragma"), F("no-cache"));
    res.set(F("Expires"), F("-1"));
    res.set("Content-Type", "text/html;");
    res.write( (uint8_t *) content, strlen(content) );

    free(content);
  }
}

#define MAX_PARAM_LEN   (32 + 1)

void input_page(Request &req, Response &res) {
  char buf[MAX_PARAM_LEN];

  if (req.query("mode", buf, MAX_PARAM_LEN)) {
    settings->mode = atoi(buf);
  }
  if (req.query("protocol", buf, MAX_PARAM_LEN)) {
    settings->rf_protocol = atoi(buf);
  }
  if (req.query("band", buf, MAX_PARAM_LEN)) {
    settings->band = atoi(buf);
  }
  if (req.query("acft_type", buf, MAX_PARAM_LEN)) {
    settings->aircraft_type = atoi(buf);
  }
  if (req.query("alarm", buf, MAX_PARAM_LEN)) {
    settings->alarm = atoi(buf);
  }
  if (req.query("txpower", buf, MAX_PARAM_LEN)) {
    settings->txpower = atoi(buf);
  }
  if (req.query("volume", buf, MAX_PARAM_LEN)) {
    settings->volume = atoi(buf);
  }
  if (req.query("pointer", buf, MAX_PARAM_LEN)) {
    settings->pointer = atoi(buf);
  }
  if (req.query("bluetooth", buf, MAX_PARAM_LEN)) {
    settings->bluetooth = atoi(buf);
  }
  if (req.query("nmea_g", buf, MAX_PARAM_LEN)) {
    settings->nmea_g = atoi(buf);
  }
  if (req.query("nmea_p", buf, MAX_PARAM_LEN)) {
    settings->nmea_p = atoi(buf);
  }
  if (req.query("nmea_l", buf, MAX_PARAM_LEN)) {
    settings->nmea_l = atoi(buf);
  }
  if (req.query("nmea_s", buf, MAX_PARAM_LEN)) {
    settings->nmea_s = atoi(buf);
  }
  if (req.query("nmea_out", buf, MAX_PARAM_LEN)) {
    settings->nmea_out = atoi(buf);
  }
  if (req.query("gdl90", buf, MAX_PARAM_LEN)) {
    settings->gdl90 = atoi(buf);
  }
  if (req.query("d1090", buf, MAX_PARAM_LEN)) {
    settings->d1090 = atoi(buf);
  }
  if (req.query("stealth", buf, MAX_PARAM_LEN)) {
    settings->stealth = atoi(buf);
  }
  if (req.query("no_track", buf, MAX_PARAM_LEN)) {
    settings->no_track = atoi(buf);
  }
  if (req.query("power_save", buf, MAX_PARAM_LEN)) {
    settings->power_save = atoi(buf);
  }
  if (req.query("rfc", buf, MAX_PARAM_LEN)) {
    settings->freq_corr = atoi(buf);
  }
#if defined(USE_OGN_ENCRYPTION)
  if (req.query("igc_key", buf, MAX_PARAM_LEN)) {
    buf[32] = 0;
    settings->igc_key[3] = strtoul(buf + 24, NULL, 16);
    buf[24] = 0;
    settings->igc_key[2] = strtoul(buf + 16, NULL, 16);
    buf[16] = 0;
    settings->igc_key[1] = strtoul(buf +  8, NULL, 16);
    buf[ 8] = 0;
    settings->igc_key[0] = strtoul(buf +  0, NULL, 16);
  }
#endif

  char *content = Input_content();

  if (content) {
    res.set(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
    res.set(F("Pragma"), F("no-cache"));
    res.set(F("Expires"), F("-1"));
    res.set("Content-Type", "text/html;");
    res.write( (uint8_t *) content, strlen(content) );

    delay(1000);
    free(content);

#if !defined(EXCLUDE_EEPROM)
    EEPROM_store();
#endif /* EXCLUDE_EEPROM */

    WebServer.end();

    Sound_fini();
    RF_Shutdown();

    delay(1000);
    SoC->reset();
  }
}

void about_page(Request &req, Response &res) {
  res.set("Content-Type", "text/html;");
  res.print(about_html);
}

void notFound(Request &req, Response &res) {
  res.set("Content-Type", "application/json");
  res.print("{\"error\":\"This is not the page you are looking for.\"}");
}
#endif /* USE_BRIDGE */

#if defined(USE_NEOPIXEL)
#include "clk.h"
#include "gpio.h"
#include "dma.h"
#include "pwm.h"

#include "ws2811.h"

#define ARRAY_SIZE(stuff)       (sizeof(stuff) / sizeof(stuff[0]))

#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                12
#define DMA                     10
//#define STRIP_TYPE            WS2811_STRIP_RGB		// WS2812/SK6812RGB integrated chip+leds
#define STRIP_TYPE              WS2811_STRIP_GBR		// WS2812/SK6812RGB integrated chip+leds
//#define STRIP_TYPE            SK6812_STRIP_RGBW		// SK6812RGBW (NOT SK6812RGB)

#define WIDTH                   12
#define HEIGHT                  1
#define LED_COUNT               (WIDTH * HEIGHT)

#define isTimeToDisplay() (millis() - LEDTimeMarker     > 1000)

unsigned long LEDTimeMarker = 0;

static int width = WIDTH;
static int height = HEIGHT;
static int led_count = LED_COUNT;

ws2811_t ledstring =
{
    .render_wait_time = 0,
    .device = NULL,
    .rpi_hw = NULL,
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .invert = 0,
            .count = LED_COUNT,
            .strip_type = STRIP_TYPE,
            .leds = 0,
            .brightness = 255,
            .wshift = 0,
            .rshift = 0,
            .gshift = 0,
            .bshift = 0,
            .gamma = NULL,
        },
        [1] =
        {
            .gpionum = 0,
            .invert = 0,
            .count = 0,
            .strip_type = 0,
            .leds = 0,
            .brightness = 0,
            .wshift = 0,
            .rshift = 0,
            .gshift = 0,
            .bshift = 0,
            .gamma = NULL,
        },
    },
};

static ws2811_led_t *matrix;
static int dotspos[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

void matrix_render(void)
{
    int x, y;

    for (x = 0; x < width; x++)
    {
        for (y = 0; y < height; y++)
        {
            ledstring.channel[0].leds[(y * width) + x] = matrix[y * width + x];
        }
    }
}

int ws281x_init(void)
{
    ws2811_return_t ret;

    matrix = (ws2811_led_t *) malloc(sizeof(ws2811_led_t) * width * height);

    if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
    {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }

    return ret;
}

void ws281x_show(void)
{
    ws2811_return_t ret;
    matrix_render();

    if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS)
    {
        fprintf(stderr, "ws2811_render failed: %s\n", ws2811_get_return_t_str(ret));
    }
}

int ws281x_numPixels(void)
{
    return ledstring.channel[0].count;
}

void ws281x_setPixelColor(int n, color_t c)
{
    matrix[dotspos[n] + (height - 1) * width] = c;
}

color_t ws281x_Color(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
#endif /* USE_NEOPIXEL */

//-------------------------------------------------------------------------
//
// The MIT License (MIT)
//
// Copyright (c) 2015 Andrew Duncan
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------

#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

static uint32_t SerialNumber = 0;

void RPi_SerialNumber(void)
{
    int fd = open("/dev/vcio", 0);
    if (fd == -1)
    {
        perror("open /dev/vcio");
        exit(EXIT_FAILURE);
    }

    uint32_t property[32] =
    {
        0x00000000,
        0x00000000,
        0x00010004,
        0x00000010,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000
    };

    property[0] = 10 * sizeof(property[0]);

    if (ioctl(fd, _IOWR(100, 0, char *), property) == -1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }

    close(fd);

    SerialNumber = property[5];
}

//----- end of MIT License ------------------------------------------------

static void RPi_setup()
{
#if defined(EXCLUDE_EEPROM)
  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol   = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band          = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_FULL;
  eeprom_block.field.settings.volume        = BUZZER_VOLUME_FULL;
  eeprom_block.field.settings.pointer       = DIRECTION_NORTH_UP;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_NONE;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_DISTANCE;

  eeprom_block.field.settings.nmea_g        = true;
  eeprom_block.field.settings.nmea_p        = false;
  eeprom_block.field.settings.nmea_l        = true;
  eeprom_block.field.settings.nmea_s        = true;
  eeprom_block.field.settings.nmea_out      = NMEA_UART;
  eeprom_block.field.settings.gdl90         = GDL90_OFF;
  eeprom_block.field.settings.d1090         = D1090_OFF;
  eeprom_block.field.settings.json          = JSON_OFF;
  eeprom_block.field.settings.stealth       = false;
  eeprom_block.field.settings.no_track      = false;
  eeprom_block.field.settings.power_save    = POWER_SAVE_NONE;
  eeprom_block.field.settings.freq_corr     = 0;
  eeprom_block.field.settings.igc_key[0]    = 0;
  eeprom_block.field.settings.igc_key[1]    = 0;
  eeprom_block.field.settings.igc_key[2]    = 0;
  eeprom_block.field.settings.igc_key[3]    = 0;
#endif /* EXCLUDE_EEPROM */

  ui = &ui_settings;

  RPi_SerialNumber();

#if !defined(USE_SPI1)
  pinMode(SOC_GPIO_PIN_WS_RST,  OUTPUT);
  pinMode(SOC_GPIO_PIN_WS_BUSY, INPUT);

  digitalWrite(SOC_GPIO_PIN_WS_RST, LOW);

  delay(10);

  if (digitalRead(SOC_GPIO_PIN_WS_BUSY) == HIGH) {
    digitalWrite(SOC_GPIO_PIN_WS_RST, HIGH);

    delay(50);

    if (digitalRead(SOC_GPIO_PIN_WS_BUSY) == LOW) {
      RPi_hat = RPI_WAVESHARE_LORA_GNSS;
    }
  }

  pinMode(SOC_GPIO_PIN_WS_RST, INPUT);
#endif /* USE_SPI1 */

  switch (RPi_hat)
  {
    case RPI_WAVESHARE_LORA_GNSS:
      lmic_pins.nss    = SOC_GPIO_PIN_WS_SS;
      lmic_pins.rst    = SOC_GPIO_PIN_WS_RST;
      lmic_pins.busy   = SOC_GPIO_PIN_WS_BUSY;
      if (SoC->getChipId() != 0xD3374780) {
        lmic_pins.tcxo = lmic_pins.rst; /* SX1262 with XTAL */
      }
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_WS_DIO1;
#endif /* USE_RADIOLIB */
      break;
    case RPI_DRAGINO_LORA_GPS:
    default:
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_DIO0;
#endif /* USE_RADIOLIB */
      break;
  }
}

static void RPi_post_init()
{

#if 0
  Serial.println();
  Serial.println(F("Raspberry Edition Power-on Self Test"));
  Serial.println();
  Serial.flush();

  Serial.println(F("Built-in components:"));

  Serial.print(F("RADIO   : ")); Serial.println(hw_info.rf      != RF_IC_NONE       ? F("PASS") : F("FAIL"));
  Serial.print(F("GNSS    : ")); Serial.println(hw_info.gnss    != GNSS_MODULE_NONE ? F("PASS") : F("FAIL"));
  Serial.print(F("DISPLAY : ")); Serial.println(hw_info.display != DISPLAY_NONE     ? F("PASS") : F("FAIL"));

  Serial.println();
  Serial.println(F("External components:"));
  Serial.print(F("BMx280  : ")); Serial.println(hw_info.baro    != BARO_MODULE_NONE ? F("PASS") : F("N/A"));

  Serial.println();
  Serial.println(F("Power-on Self Test is complete."));
  Serial.println();
  Serial.flush();
#endif

#if defined(USE_EPAPER)

  EPD_info1();

#endif /* USE_EPAPER */
}

static bool prev_PPS_state = LOW;

static void RPi_loop()
{
#if SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN
  if (digitalPinToInterrupt(SOC_GPIO_PIN_GNSS_PPS) == NOT_AN_INTERRUPT) {
    bool PPS_state = digitalRead(SOC_GPIO_PIN_GNSS_PPS);

    if (PPS_state == HIGH && prev_PPS_state == LOW) {
      PPS_TimeMarker = millis();
    }
    prev_PPS_state = PPS_state;
  }
#endif

#if defined(USE_BRIDGE)
  BridgeClient client = WebServer.available();

  if (client.connected()) {
    WebApp.process(&client);
    client.stop();
  }
#endif /* USE_BRIDGE */
}

static void RPi_fini(int reason)
{
  fprintf( stderr, "Program termination. Reason code: %d.\n", reason );
  exit(EXIT_SUCCESS);
}

static void RPi_reset()
{
  fprintf( stderr, "Program restart.\n" );
  exit(EXIT_SUCCESS + 2);
}

static uint32_t RPi_getChipId()
{
  uint32_t id = SerialNumber ? SerialNumber : gethostid();

  return DevID_Mapper(id);
}

static void* RPi_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static uint32_t RPi_getFreeHeap()
{
  return 0; /* TBD */
}

static long RPi_random(long howsmall, long howBig)
{
  return howsmall + random() % (howBig - howsmall);
}

#if defined(USE_LGPIO) && defined(USE_RADIOLIB) && !defined(EXCLUDE_LR11XX)
#include <hal/RPi/PiHal.h>

extern PiHal *RadioLib_HAL;
#endif /* USE_RADIOLIB */

static void RPi_Sound_test(int var)
{
#if defined(USE_LGPIO)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
#if defined(USE_RADIOLIB) && !defined(EXCLUDE_LR11XX)
    if (rf_chip && RadioLib_HAL && rf_chip->type == RF_IC_LR1121) {
      RadioLib_HAL->tone(SOC_GPIO_PIN_BUZZER, 440,  220); delay(500);
      RadioLib_HAL->tone(SOC_GPIO_PIN_BUZZER, 640,  320); delay(500);
      RadioLib_HAL->tone(SOC_GPIO_PIN_BUZZER, 840,  420); delay(500);
      RadioLib_HAL->tone(SOC_GPIO_PIN_BUZZER, 1040, 520); delay(600);
      RadioLib_HAL->noTone(SOC_GPIO_PIN_BUZZER);
      RadioLib_HAL->pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
    } else
#endif /* USE_RADIOLIB */
    {
      tone(SOC_GPIO_PIN_BUZZER, 440,  220); delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 640,  320); delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 840,  420); delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 1040, 520); delay(600);
      noTone(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
    }
  }
#endif /* USE_LGPIO */
}

static void RPi_Sound_tone(int hz, uint8_t volume)
{
#if defined(USE_LGPIO)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
#if defined(USE_RADIOLIB) && !defined(EXCLUDE_LR11XX)
    if (rf_chip && RadioLib_HAL && rf_chip->type == RF_IC_LR1121) {
      if (hz > 0) {
        RadioLib_HAL->tone(SOC_GPIO_PIN_BUZZER, hz, (hz * ALARM_TONE_MS) / 1000);
      } else {
        RadioLib_HAL->noTone(SOC_GPIO_PIN_BUZZER);
        RadioLib_HAL->pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
      }
    } else
#endif /* USE_RADIOLIB */
    {
      if (hz > 0) {
        tone(SOC_GPIO_PIN_BUZZER, hz, (hz * ALARM_TONE_MS) / 1000);
      } else {
        noTone(SOC_GPIO_PIN_BUZZER);
        pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
      }
    }
  }
#endif /* USE_LGPIO */
}

static void RPi_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
#if defined(USE_BRIDGE)
  // printf("%08X\n", (unsigned long) dest_IP);

  Uni_Udp.beginPacket(dest_IP, port);
  Uni_Udp.write(buf, size);
  Uni_Udp.endPacket();
#endif /* USE_BRIDGE */
}

static bool RPi_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();
#endif /* EXCLUDE_EEPROM */

  return true;
}

static void RPi_EEPROM_extension(int cmd)
{
  if (cmd == EEPROM_EXT_LOAD) {
    if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

    /* AUTO and UK RF bands are deprecated since Release v1.3 */
    if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
      settings->band = RF_BAND_EU;
    }
  }
}

static void RPi_SPI_begin()
{
  SPI.begin();
}

static void RPi_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void RPi_swSer_enableRx(boolean arg)
{
  /* NONE */
}

pthread_t RPi_EPD_update_thread;

#if defined(USE_OLED)
bool RPi_OLED_probe_func()
{
  bool ret = false;

#if defined(USE_LGPIO)
  uint8_t i2cDevice = 1;
  uint8_t buf[2] = { 0x00 };

  int i2cHandle = -1;
  if ((i2cHandle = lgI2cOpen(i2cDevice, SSD1306_OLED_I2C_ADDR, 0)) < 0) {
    fprintf(stderr, "Could not open I2C handle on 0: %s\n", lguErrorText(i2cHandle));
  } else {
    int status = lgI2cWriteDevice(i2cHandle, (char *) buf, 1);
    if (status < 0) { ret = false; } else { ret = true; }
    lgI2cClose(i2cHandle);
  }
#endif /* USE_LGPIO */

  return ret;
}
#endif /* USE_OLED */

static byte RPi_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_EPAPER)

#if !defined(USE_GDEY027T91)
  display = &epd_waveshare_W3;
#else
  display = &epd_waveshare_T91;
#endif /* USE_GDEY027T91 */

  if (EPD_setup(true)) {

    if ( pthread_create(&RPi_EPD_update_thread, NULL, &EPD_Task, (void *)0) != 0) {
      fprintf( stderr, "pthread_create(EPD_Task) Failed\n\n" );
      exit(EXIT_FAILURE);
    }

#if 0
    struct sched_param  param;
    param.sched_priority = 50;
    pthread_setschedparam(RPi_EPD_update_thread, SCHED_RR, &param);
#endif

    rval = DISPLAY_EPD_2_7;
  }
#endif /* USE_EPAPER */

#if defined(USE_OLED)
  if (rval == DISPLAY_NONE) {
    // u8x8_i2c.setI2CAddress(SH1106_OLED_I2C_ADDR_ALT << 1);
    rval = OLED_setup();
  }
#endif /* USE_OLED */

  return rval;
}

static void RPi_Display_loop()
{
  switch (hw_info.display)
  {
#if defined(USE_EPAPER)
  case DISPLAY_EPD_2_7:
    EPD_loop();
    break;
#endif /* USE_EPAPER */

#if defined(USE_OLED)
  case DISPLAY_OLED_1_3:
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
    OLED_loop();
    break;
#endif /* USE_OLED */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void RPi_Display_fini(int reason)
{
  switch (hw_info.display)
  {
#if defined(USE_EPAPER)
  case DISPLAY_EPD_2_7:
    EPD_fini(reason, false);

    if ( RPi_EPD_update_thread != (pthread_t) 0)
    {
      pthread_cancel( RPi_EPD_update_thread );
    }
    break;
#endif /* USE_EPAPER */

#if defined(USE_OLED)
  case DISPLAY_OLED_1_3:
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
    OLED_fini(reason);
    break;
#endif /* USE_OLED */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void RPi_Battery_setup()
{
  /* TBD */
}

static float RPi_Battery_param(uint8_t param)
{
  float rval;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = BATTERY_THRESHOLD_USB;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = BATTERY_CUTOFF_USB;
    break;

  case BATTERY_PARAM_CHARGE:
    /* TBD */

    rval = 100;
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    rval = BATTERY_THRESHOLD_USB + 0.05;
    break;
  }

  return rval;
}

void RPi_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long RPi_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool RPi_Baro_setup() {
  return true;
}

static void RPi_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
  UATSerial.dtr(false);
  UATSerial.rts(false);
}

static void RPi_UATModule_restart()
{
  UATSerial.dtr(false);

  delay(100);

#if DEBUG
  Serial.println("RTS on");
#endif

  UATSerial.rts(true);

  delay(100);

#if DEBUG
  Serial.println("RTS off");
#endif

  UATSerial.rts(false);
}

static void RPi_WDT_setup()
{
  /* TBD */
}

static void RPi_WDT_fini()
{
  /* TBD */
}

static void RPi_Button_setup()
{
  /* TODO */
}

static void RPi_Button_loop()
{
  /* TODO */
}

static void RPi_Button_fini()
{
  /* TODO */
}

const SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup,
  RPi_post_init,
  RPi_loop,
  RPi_fini,
  RPi_reset,
  RPi_getChipId,
  RPi_getResetInfoPtr,
  NULL,
  NULL,
  RPi_getFreeHeap,
  RPi_random,
  RPi_Sound_test,
  RPi_Sound_tone,
  NULL,
  NULL,
  RPi_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  RPi_EEPROM_begin,
  RPi_EEPROM_extension,
  RPi_SPI_begin,
  RPi_swSer_begin,
  RPi_swSer_enableRx,
  NULL,
  NULL,
  NULL,
  RPi_Display_setup,
  RPi_Display_loop,
  RPi_Display_fini,
  RPi_Battery_setup,
  RPi_Battery_param,
  NULL,
  RPi_get_PPS_TimeMarker,
  RPi_Baro_setup,
  RPi_UATSerial_begin,
  RPi_UATModule_restart,
  RPi_WDT_setup,
  RPi_WDT_fini,
  RPi_Button_setup,
  RPi_Button_loop,
  RPi_Button_fini,
  NULL
};

static bool inputAvailable()
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

static void parseNMEA(const char *str, int len)
{
  // NMEA input
  for (int i=0; i < len; i++) {
    gnss.encode(str[i]);
  }
  if (settings->nmea_g) {
    NMEA_Out(settings->nmea_out, (byte *) str, len, true);
  }

  GNSSTimeSync();

  if (isValidGNSSFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();

    /*
     * When geoidal separation is zero or not available - use approx. EGM96 value
     */
    if (ThisAircraft.geoid_separation == 0.0) {
      ThisAircraft.geoid_separation = (float) LookupSeparation(
                                                ThisAircraft.latitude,
                                                ThisAircraft.longitude
                                              );
      /* we can assume the GPS unit is giving ellipsoid height */
      ThisAircraft.altitude -= ThisAircraft.geoid_separation;
    }
  }
}

static void RPi_PickGNSSFix()
{
  if (inputAvailable()) {
    std::getline(std::cin, input_line);
    const char *str = input_line.c_str();
    int len = input_line.length();

    if (str[0] == '$' && str[1] == 'G') {
      // NMEA input
      parseNMEA(str, len);

    } else if (str[0] == '{') {
      // JSON input

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"TPV")) { // "TPV"
          parseTPV(root);
        } else if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings(root);

          RF_setup();
          Traffic_setup();
        }
      }

      if (root.containsKey("now") &&
          root.containsKey("messages") &&
          root.containsKey("aircraft")) {
        /* 'aircraft.json' output from 'dump1090' application */
        parseD1090(root);
      } else if (root.containsKey("aircraft")) {
        /* uAvionix PingStation */
        parsePING(root);
      }

      jsonBuffer.clear();

      if ((time(NULL) - now()) > 3) {
        hasValidGPSDFix = false;
      }
    }
  }
}

static void RPi_ReadTraffic()
{
  string traffic_input = Traffic_TCP_Server.getMessage();
  if (traffic_input != "") {
    const char *str = traffic_input.c_str();
    int len = traffic_input.length();

    if (str[0] == '{') {
      // JSON input

//    cout << "Traffic message:" << traffic_input << endl;

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings(root);

          RF_setup();
          Traffic_setup();
        }
      }

      if (root.containsKey("now") &&
          root.containsKey("messages") &&
          root.containsKey("aircraft")) {
        /* 'aircraft.json' output from 'dump1090' application */
        if (isValidFix()) {
          parseD1090(root);
        }
      } else if (root.containsKey("aircraft")) {
        /* uAvionix PingStation */
        if (isValidFix()) {
          parsePING(root);
        }
      }

      JsonVariant rawdata = root["rawdata"];
      if (rawdata.success()) {
        parseRAW(root);
      }

      jsonBuffer.clear();
    } else if (str[0] == 'q') {
      if (len >= 4 && str[1] == 'u' && str[2] == 'i' && str[3] == 't') {
        Traffic_TCP_Server.detach();
        fprintf( stderr, "Program termination.\n" );
        exit(EXIT_SUCCESS);
      }
    }

    Traffic_TCP_Server.clean();
  }
}

void normal_loop()
{
#if defined(USE_LGPIO)
    Baro_loop();
#endif /* USE_LGPIO */

    /* Read GNSS data from standard input */
    RPi_PickGNSSFix();

    /* Read NMEA data from GNSS module on GPIO pins */
//    PickGNSSFix();

    RPi_ReadTraffic();

    RF_loop();

    ThisAircraft.timestamp = now();

    if (isValidFix()) {
      RF_Transmit(RF_Encode(&ThisAircraft), true);
    }

    bool success = RF_Receive();

    if (success && isValidFix()) ParseData();

    if (isValidFix()) {
      Traffic_loop();
    }

#if defined(USE_NEOPIXEL)
    if (isTimeToDisplay()) {
      if (isValidFix()) {
        LED_DisplayTraffic();
      } else {
        LED_Clear();
      }
      LEDTimeMarker = millis();
    }
#endif /* USE_NEOPIXEL */

    if (isTimeToExport()) {

      NMEA_Export();
      GDL90_Export();
      D1090_Export();

      if (isValidFix()) {
        JSON_Export();
      }
      ExportTimeMarker = millis();
    }

    // Handle Air Connect
    NMEA_loop();

    SoC->Display_loop();

    ClearExpired();
}

void relay_loop()
{
    /* Read GNSS data from standard input */
    RPi_PickGNSSFix();

    /* Read NMEA data from GNSS module on GPIO pins */
//    PickGNSSFix();

    RPi_ReadTraffic();

    RF_loop();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      size_t size = RF_Payload_Size(settings->rf_protocol);
      size = size > sizeof(Container[i].raw) ? sizeof(Container[i].raw) : size;

      if (memcmp (Container[i].raw, EmptyFO.raw, size) != 0) {
        // Raw data
        size_t tx_size = sizeof(TxBuffer) > size ? size : sizeof(TxBuffer);
        memcpy(TxBuffer, Container[i].raw, tx_size);

        if (tx_size > 0) {
          /* Follow duty cycle rule */
          if (RF_Transmit(tx_size, true /* false */)) {
#if 0
            String str = Bin2Hex(TxBuffer, tx_size);
            printf("%s\n", str.c_str());
#endif
            Container[i] = EmptyFO;
          }
        }
      } else if (isValidFix() &&
                 Container[i].addr &&
                 Container[i].latitude  != 0.0 &&
                 Container[i].longitude != 0.0 &&
                 Container[i].altitude  != 0.0 &&
                 Container[i].distance < (ALARM_ZONE_NONE * 2) ) {

        fo = Container[i];
        fo.timestamp = now(); /* GNSS date&time */

        /* Follow duty cycle rule */
        if (RF_Transmit(RF_Encode(&fo), true /* false */)) {
#if 0
          printf("%06X %f %f %f %d %d %d\n",
              fo.addr,
              fo.latitude,
              fo.longitude,
              fo.altitude,
              fo.addr_type,
              (int) fo.vs,
              fo.aircraft_type);
#endif
          Container[i] = EmptyFO;
        }
      }
    }
}

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long baro_start_ms, baro_end_ms;
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
  unsigned long export_start_ms, export_end_ms;
  unsigned long oled_start_ms, oled_end_ms;
#endif

  setTime(time(NULL));

  RPi_ReadTraffic();

  RF_loop();

  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }

  ThisAircraft.altitude = TXRX_TEST_ALTITUDE;
  ThisAircraft.course = TXRX_TEST_COURSE;
  ThisAircraft.speed = TXRX_TEST_SPEED;
  ThisAircraft.vs = TXRX_TEST_VS;

#if defined(USE_LGPIO)
  Baro_loop();
#endif /* USE_LGPIO */

#if DEBUG_TIMING
  tx_start_ms = millis();
#endif

  RF_Transmit(RF_Encode(&ThisAircraft), true);

#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();
#endif

#if DEBUG_TIMING
  parse_start_ms = millis();
#endif
  if (success) ParseData();
#if DEBUG_TIMING
  parse_end_ms = millis();
#endif

  Traffic_loop();

#if defined(USE_NEOPIXEL)
  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }
#endif /* USE_NEOPIXEL */

#if DEBUG_TIMING
  export_start_ms = millis();
#endif
  if (isTimeToExport()) {
    NMEA_Position();
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }
#if DEBUG_TIMING
  export_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (tx_end_ms - tx_start_ms) {
    Serial.print(F("TX start: "));
    Serial.print(tx_start_ms);
    Serial.print(F(" TX stop: "));
    Serial.println(tx_end_ms);
  }
  if (rx_end_ms - rx_start_ms) {
    Serial.print(F("RX start: "));
    Serial.print(rx_start_ms);
    Serial.print(F(" RX stop: "));
    Serial.println(rx_end_ms);
  }
  if (parse_end_ms - parse_start_ms) {
    Serial.print(F("Parse start: "));
    Serial.print(parse_start_ms);
    Serial.print(F(" Parse stop: "));
    Serial.println(parse_end_ms);
  }

  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
#endif

  // Handle Air Connect
  NMEA_loop();

  SoC->Display_loop();

  ClearExpired();
}


void * traffic_tcpserv_loop(void * m)
{
  pthread_detach(pthread_self());
  Traffic_TCP_Server.receive();
}

extern "C" void *readerThreadEntryPoint(void *arg);
extern "C" void ModeS_demod_loop(mode_s_callback_t);

void on_msg(mode_s_t *self, struct mode_s_msg *mm) {

  MODES_NOTUSED(self);

  rx_packets_counter++;

/* When a new message is available, because it was decoded from the
 * SDR device, file, or received in the TCP input port, or any other
 * way we can receive a decoded message, we call this function in order
 * to use the message.
 *
 * Basically this function passes a raw message to the upper layers for
 * further processing and visualization. */

    if (self->check_crc == 0 || mm->crcok) {

//    printf("%02d %03d %02x%02x%02x\r\n", mm->msgtype, mm->msgbits, mm->aa1, mm->aa2, mm->aa3);

        int acfts_in_sight = 0;
        struct mode_s_aircraft *a = state.aircrafts;

        while (a) {
          acfts_in_sight++;
          a = a->next;
        }

        if (acfts_in_sight < MAX_TRACKING_OBJECTS) {
          interactiveReceiveData(self, mm);
        }
    }
}

int main()
{
#if defined(USE_BCMLIB)
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      exit(EXIT_FAILURE);
  }
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
  // Init GPIO lgpio
  if (!lgpio_init()) {
      fprintf( stderr, "lgpio_init() Failed\n\n" );
      exit(EXIT_FAILURE);
  }
#endif /* USE_LGPIO */

  Serial.begin(SERIAL_OUT_BR);

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Serial.println();
  Serial.print(F(SOFTRF_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2025 Linar Yusupov. All rights reserved."));
  Serial.flush();

#if !defined(EXCLUDE_EEPROM)
  EEPROM_setup();
#endif /* EXCLUDE_EEPROM */

  Serial.print(F("Radio & GNSS adapter: "));

  switch (RPi_hat)
  {
    case RPI_WAVESHARE_LORA_GNSS:
      Serial.println(F("Waveshare SX1262 LoRaWAN/GNSS HAT"));
      break;
    case RPI_DRAGINO_LORA_GPS:
    default:
      Serial.println(F("Dragino SX1276 LoRa/GPS HAT"));
      break;
  }

  mode_s_init(&state);

#if defined(ENABLE_RTLSDR) || defined(ENABLE_HACKRF) || defined(ENABLE_MIRISDR)
  sdrInitConfig();

  // Allocate the various buffers used by Modes
  state.trailing_samples = (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS + 16) * 1e-6 * state.sample_rate;

  if (!fifo_create(MODES_MAG_BUFFERS, MODES_MAG_BUF_SAMPLES + state.trailing_samples, state.trailing_samples)) {
      fprintf(stderr, "Out of memory allocating FIFO\n");
      exit(EXIT_FAILURE);
  }
#endif /* ENABLE_RTLSDR || ENABLE_HACKRF || ENABLE_MIRISDR */

#if defined(ENABLE_RTLSDR)
  if (state.sdr_type = SDR_RTLSDR, sdrOpen()) {
    hw_info.rf = RF_IC_R820T;
  } else
#endif /* ENABLE_RTLSDR */

#if defined(ENABLE_HACKRF)
  if (state.sdr_type = SDR_HACKRF, sdrOpen()) {
    hw_info.rf = RF_IC_MAX2837;
  } else
#endif /* ENABLE_HACKRF */

#if defined(ENABLE_MIRISDR)
  if (state.sdr_type = SDR_MIRI, sdrOpen()) {
    hw_info.rf = RF_IC_MSI001;
  } else
#endif /* ENABLE_MIRISDR */

  hw_info.rf = RF_setup();

#if 0
  if (hw_info.rf == RF_IC_NONE) {
      exit(EXIT_FAILURE);
  }
#endif

#if defined(ENABLE_RTLSDR) || defined(ENABLE_HACKRF) || defined(ENABLE_MIRISDR)
  if (hw_info.rf == RF_IC_R820T   ||
      hw_info.rf == RF_IC_MAX2837 ||
      hw_info.rf == RF_IC_MSI001) {
    // Create the thread that will read the data from the device.
    pthread_create(&state.reader_thread, NULL, readerThreadEntryPoint, NULL);
  }
#endif /* ENABLE_RTLSDR || ENABLE_HACKRF || ENABLE_MIRISDR */

#if defined(USE_LGPIO)
  hw_info.baro = Baro_setup();
#endif /* USE_LGPIO */

#if defined(USE_EPAPER) || defined(USE_OLED)
  Serial.print("Intializing display module (may take up to 10 seconds)... ");
  Serial.flush();
  hw_info.display = SoC->Display_setup();
  if (hw_info.display != DISPLAY_NONE) {
    Serial.println(" done.");
  } else {
    Serial.println(" failed!");
  }
#endif /* USE_EPAPER */

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

//  hw_info.gnss = GNSS_setup();

  Traffic_setup();

#if defined(USE_NEOPIXEL)
  LED_setup();
#endif /* USE_NEOPIXEL */

  NMEA_setup();

  Traffic_TCP_Server.setup(JSON_SRV_TCP_PORT);

  pthread_t traffic_tcpserv_thread;
  if ( pthread_create(&traffic_tcpserv_thread, NULL, traffic_tcpserv_loop, (void *)0) != 0) {
    fprintf( stderr, "pthread_create(traffic_tcpserv_thread) Failed\n\n" );
    exit(EXIT_FAILURE);
  }

#if defined(USE_BRIDGE)
  Bridge.begin();

  WebApp.get("/", &index_page);
  WebApp.get("/settings", &settings_page);
  WebApp.get("/input", &input_page);
  WebApp.get("/about", &about_page);
  WebApp.notFound(&notFound);

  WebServer.listenOnLocalhost();
  WebServer.begin();

  struct hostent *this_host = gethostbyname("pione.local");

  if (this_host == NULL) {
    dest_IP = IPAddress(255,255,255,255);
  } else {
    IPAddress this_IP = IPAddress((const uint8_t *)(this_host->h_addr_list[0]));
    dest_IP = IPAddress((uint32_t) this_IP | ~((uint32_t) 0x00FFFFFF));
  }

  Serial.print(F("HTTP server has started at port: "));
  Serial.println((unsigned long) HTTP_SRV_PORT);

  Uni_Udp.begin(RELAY_SRC_PORT);

  Serial.print(F("UDP  server has started at port: "));
  Serial.println((unsigned long) RELAY_SRC_PORT);
#endif /* USE_BRIDGE */

#if defined(USE_NEOPIXEL)
  LED_test();
#endif /* USE_NEOPIXEL */

  Sound_setup();
  SoC->Sound_test(reset_info.reason);

  SoC->post_init();

  SoC->WDT_setup();

  while (true) {
    switch (settings->mode)
    {
    case SOFTRF_MODE_TXRX_TEST:
      txrx_test_loop();
      break;
    case SOFTRF_MODE_RELAY:
      relay_loop();
      break;
    case SOFTRF_MODE_NORMAL:
    default:
      normal_loop();
      break;
    }

#if defined(ENABLE_RTLSDR) || defined(ENABLE_HACKRF) || defined(ENABLE_MIRISDR)
    ModeS_demod_loop(on_msg);
#endif /* ENABLE_RTLSDR || ENABLE_HACKRF || ENABLE_MIRISDR */

    SoC->loop();

    Time_loop();

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
    /* take care of millis() rollover on a long term run */
    if (millis() > (47 * 24 * 3600 * 1000UL)) {
      time_t current_time = time(NULL);
      struct tm timebuf;

      if (current_time == ((time_t)-1) ||
          localtime_r(&current_time, &timebuf) == NULL) {
        Traffic_TCP_Server.detach();
        fprintf(stderr, "Failure to obtain the current time.\n");
        exit(EXIT_FAILURE);
      }

      /* shut SoftRF down at night time only */
      if (timebuf.tm_hour >= 2 && timebuf.tm_hour <= 5) {
        Traffic_TCP_Server.detach();
        fprintf( stderr, "Program termination: millis() rollover prevention.\n" );
        exit(EXIT_SUCCESS);
      }
    }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */
  }

#if defined(USE_BRIDGE)
  WebServer.end();
  Uni_Udp.stop();
#endif /* USE_BRIDGE */

  Traffic_TCP_Server.detach();
  return 0;
}

void shutdown(int reason)
{
  SoC->WDT_fini();

  if (hw_info.display != DISPLAY_NONE) {
    SoC->Display_fini(reason);
  }

#if defined(USE_NEOPIXEL)
  ws2811_fini(&ledstring);
#endif /* USE_NEOPIXEL */

#if defined(USE_BRIDGE)
  WebServer.end();
  Uni_Udp.stop();
#endif /* USE_BRIDGE */

  Traffic_TCP_Server.detach();

  SoC->fini(reason);
}

#endif /* RASPBERRY_PI */

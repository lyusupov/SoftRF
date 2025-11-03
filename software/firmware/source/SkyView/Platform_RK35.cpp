/*
 * Platform_RK35.cpp
 * Copyright (C) 2025 Linar Yusupov
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
 *  pi@raspberrypi $ make -f Makefile.RK35
 *
 *     < ... skipped ... >
 *
 *  pi@raspberrypi $ sudo ./SkyView
 *
 */

#if defined(LUCKFOX_LYRA)

#include <stdio.h>
#include <string.h>
#include <sqlite3.h>

#include <ArduinoJson.h>

#include "SoCHelper.h"
#include "NMEAHelper.h"
#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "GDL90Helper.h"
#include "BatteryHelper.h"
#include "JSONHelper.h"
#include "EPDHelper.h"
#include "OLEDHelper.h"

#include "SkyView.h"

#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <string.h>

#include <iostream>

TTYSerial SerialInput("/dev/ttyACM0");

static const uint8_t SS    = 8;

/* Waveshare Pi HAT 2.7" */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare_W3(GxEPD2_270(/*CS=*/ SS,
                                       /*DC=*/ 9, /*RST=*/ 5, /*BUSY=*/ 4));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_waveshare_T91(GxEPD2_270_T91(/*CS=*/ SS,
                                       /*DC=*/ 9, /*RST=*/ 5, /*BUSY=*/ 4));


Adafruit_SSD1306 odisplay(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, /*DC=*/ 25, /*RST=*/ 17, /*CS=*/ SS);

lmic_pinmap lmic_pins = {
    .nss  = LMIC_UNUSED_PIN,
    .txe  = LMIC_UNUSED_PIN,
    .rxe  = LMIC_UNUSED_PIN,
    .rst  = LMIC_UNUSED_PIN,
    .dio  = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void radio_irq_handler (u1_t dio) {
}
u1_t radio_has_irq (void) {
    return 0;
}

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#else
EEPROMClass EEPROM;
#endif /* EXCLUDE_EEPROM */

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming packets

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_RASPBERRY,
  .revision = 0,
  .soc      = SOC_NONE,
  .display  = DISPLAY_NONE
};

bool screen_saver = false;

static sqlite3 *fln_db;
static sqlite3 *ogn_db;
static sqlite3 *icao_db;

std::string input_line;

#if defined(USE_BRIDGE)
#undef ARDUINO
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <BridgeUdp.h>
#include <aWOT.h>
#include "WebHelper.h"

#include <netdb.h>

BridgeServer WebServer(HTTP_SRV_PORT);
Application WebApp;
// BridgeUDP Uni_Udp;

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

  if (req.query("adapter", buf, MAX_PARAM_LEN)) {
    settings->adapter = atoi(buf);
  }
  if (req.query("connection", buf, MAX_PARAM_LEN)) {
    settings->connection = atoi(buf);
  }
  if (req.query("protocol", buf, MAX_PARAM_LEN)) {
    settings->protocol = atoi(buf);
  }
  if (req.query("baudrate", buf, MAX_PARAM_LEN)) {
    settings->baudrate = atoi(buf);
  }
  if (req.query("server", buf, MAX_PARAM_LEN)) {
    strncpy(settings->server, buf, sizeof(settings->server));
  }
  if (req.query("key", buf, MAX_PARAM_LEN)) {
    strncpy(settings->key, buf, sizeof(settings->key));
  }
  if (req.query("units", buf, MAX_PARAM_LEN)) {
    settings->units = atoi(buf);
  }
  if (req.query("rotation", buf, MAX_PARAM_LEN)) {
    settings->rotate = atoi(buf);
  }
  if (req.query("vmode", buf, MAX_PARAM_LEN)) {
    settings->vmode = atoi(buf);
  }
  if (req.query("orientation", buf, MAX_PARAM_LEN)) {
    settings->orientation = atoi(buf);
  }
  if (req.query("zoom", buf, MAX_PARAM_LEN)) {
    settings->zoom = atoi(buf);
  }
  if (req.query("adb", buf, MAX_PARAM_LEN)) {
    settings->adb = atoi(buf);
  }
  if (req.query("idpref", buf, MAX_PARAM_LEN)) {
    settings->idpref = atoi(buf);
  }
  if (req.query("voice", buf, MAX_PARAM_LEN)) {
    settings->voice = atoi(buf);
  }
  if (req.query("aghost", buf, MAX_PARAM_LEN)) {
    settings->aghost = atoi(buf);
  }
  if (req.query("filter", buf, MAX_PARAM_LEN)) {
    settings->filter = atoi(buf);
  }
  if (req.query("power_save", buf, MAX_PARAM_LEN)) {
    settings->power_save = atoi(buf);
  }
  if (req.query("team", buf, MAX_PARAM_LEN)) {
    settings->team = strtoul(buf, NULL, 16);
  }

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

void RK35_SerialNumber(void)
{
    FILE *fp = fopen("/sys/firmware/devicetree/base/serial-number", "r");

    if (fp == NULL)
    {
        perror("/sys/firmware/devicetree/base/serial-number");
        exit(EXIT_FAILURE);
    }

    char value[80];

    if (fgets(value, sizeof(value), fp) != NULL)
    {
        SerialNumber = strtoull(value, NULL, 16);
    }

    fclose(fp);
}

//----- end of MIT License ------------------------------------------------

static void RK35_setup()
{
#if defined(EXCLUDE_EEPROM)
#if defined(USE_GDEY027T91)
  eeprom_block.field.settings.adapter         = ADAPTER_WAVESHARE_PICO_2_7_V2;
#else
  eeprom_block.field.settings.adapter         = ADAPTER_WAVESHARE_PICO_2_7;
#endif /* USE_GDEY027T91 */

  eeprom_block.field.settings.connection      = CON_SERIAL_MAIN;
  eeprom_block.field.settings.baudrate        = B38400;
  eeprom_block.field.settings.protocol        = PROTOCOL_NMEA;
  eeprom_block.field.settings.orientation     = DIRECTION_NORTH_UP;

  strcpy(eeprom_block.field.settings.server,    DEFAULT_AP_SSID);
  strcpy(eeprom_block.field.settings.key,       DEFAULT_AP_PSK);

  eeprom_block.field.settings.units           = UNITS_METRIC;
  eeprom_block.field.settings.vmode           = VIEW_MODE_RADAR;
  eeprom_block.field.settings.zoom            = ZOOM_MEDIUM;
  eeprom_block.field.settings.adb             = DB_AUTO;
  eeprom_block.field.settings.idpref          = ID_REG;
  eeprom_block.field.settings.voice           = VOICE_1;
  eeprom_block.field.settings.aghost          = ANTI_GHOSTING_OFF;

  eeprom_block.field.settings.filter          = TRAFFIC_FILTER_OFF;
  eeprom_block.field.settings.power_save      = POWER_SAVE_NONE;
  eeprom_block.field.settings.team            = 0;
#endif /* EXCLUDE_EEPROM */

  RK35_SerialNumber();
}

static void RK35_post_init()
{

}

static void RK35_loop()
{
#if defined(USE_BRIDGE)
  BridgeClient client = WebServer.available();

  if (client.connected()) {
    WebApp.process(&client);
    client.stop();
  }
#endif /* USE_BRIDGE */
}

static void RK35_fini()
{
  fprintf( stderr, "Program termination.\n" );
  exit(EXIT_SUCCESS);
}

static void RK35_reset()
{

}

static uint32_t RK35_getChipId()
{
  return SerialNumber ? SerialNumber : gethostid();
}

static uint32_t RK35_getFreeHeap()
{
  return 0; /* TBD */
}

static bool RK35_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();
#endif /* EXCLUDE_EEPROM */

  return true;
}

static void RK35_EEPROM_extension(int cmd)
{
  /* TBD */
}

static void RK35_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void RK35_Battery_setup()
{
  /* TBD */
}

static float RK35_Battery_voltage()
{
  return 0.0;  /* TBD */
}

static byte RK35_Display_setup(bool splash_screen)
{
  switch (settings->adapter)
  {
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
    display = &epd_waveshare_T91;
    break;
  case ADAPTER_WAVESHARE_PICO_2_7:
  default:
    display = &epd_waveshare_W3;
    break;
  }
#if defined(USE_BCMLIB)
  display->epd2.selectSPI(SPI0, SPISettings(BCM2835_SPI_CLOCK_DIVIDER_64,
                                            BCM2835_SPI_BIT_ORDER_MSBFIRST,
                                            BCM2835_SPI_MODE0));
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
  display->epd2.selectSPI(SPI0, SPISettings(2000000, MSBFIRST, SPI_MODE0));
#endif /* USE_LGPIO */

  return EPD_setup(splash_screen);
}

static void RK35_Display_loop()
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    TFT_loop();
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
    EPD_loop();
    break;
  case DISPLAY_OLED_2_4:
    OLED_loop();
    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static void RK35_Display_fini(const char *msg, bool screen_saver)
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    TFT_fini();
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
    EPD_fini(msg, screen_saver);
    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static bool RK35_Display_is_ready()
{
  return true;
}

static void RK35_Display_update(int val)
{
  EPD_Update_Sync(val);
}

static size_t RK35_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; /* TBD */
}

static int RK35_WiFi_clients_count()
{
  return 0;
}

static bool RK35_DB_init()
{
  sqlite3_open("Aircrafts/fln.db", &fln_db);

  if (fln_db == NULL)
  {
    printf("Failed to open FlarmNet DB\n");
    return false;
  }

  sqlite3_open("Aircrafts/ogn.db", &ogn_db);

  if (ogn_db == NULL)
  {
    printf("Failed to open OGN DB\n");
    sqlite3_close(fln_db);
    return false;
  }

  sqlite3_open("Aircrafts/icao.db", &icao_db);

  if (icao_db == NULL)
  {
    printf("Failed to open ICAO DB\n");
    sqlite3_close(fln_db);
    sqlite3_close(ogn_db);
    return false;
  }

  return true;
}

static bool RK35_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  sqlite3_stmt *stmt;
  char *query = NULL;
  int error;
  bool rval = false;
  const char *reg_key, *db_key;
  sqlite3 *db;

  switch (type)
  {
  case DB_OGN:
    switch (settings->idpref)
    {
    case ID_TAIL:
      reg_key = "accn";
      break;
    case ID_MAM:
      reg_key = "acmodel";
      break;
    case ID_REG:
    default:
      reg_key = "acreg";
      break;
    }
    db_key  = "devices";
    db      = ogn_db;
    break;
  case DB_ICAO:
    switch (settings->idpref)
    {
    case ID_TAIL:
      reg_key = "owner";
      break;
    case ID_MAM:
      reg_key = "type";
      break;
    case ID_REG:
    default:
      reg_key = "registration";
      break;
    }
    db_key  = "aircrafts";
    db      = icao_db;
    break;
  case DB_FLN:
  default:
    switch (settings->idpref)
    {
    case ID_TAIL:
      reg_key = "tail";
      break;
    case ID_MAM:
      reg_key = "type";
      break;
    case ID_REG:
    default:
      reg_key = "registration";
      break;
    }
    db_key  = "aircrafts";
    db      = fln_db;
    break;
  }

  if (db == NULL) {
    return false;
  }

  error = asprintf(&query, "select %s from %s where id = %d",reg_key, db_key, id);

  if (error == -1) {
    return false;
  }

  sqlite3_prepare_v2(db, query, strlen(query), &stmt, NULL);

  while (sqlite3_step(stmt) != SQLITE_DONE) {
    if (sqlite3_column_type(stmt, 0) == SQLITE3_TEXT) {

      size_t len = strlen((char *) sqlite3_column_text(stmt, 0));

      if (len > 0) {
        len = len > size ? size : len;
        strncpy(buf, (char *) sqlite3_column_text(stmt, 0), len);
        if (len < size) {
          buf[len] = 0;
        } else if (len == size) {
          buf[len-1] = 0;
        }
        rval = true;
      }
    }
  }

  sqlite3_finalize(stmt);
  free(query);

  return rval;
}

static void RK35_DB_fini()
{
  if (fln_db != NULL) {
    sqlite3_close(fln_db);
  }

  if (ogn_db != NULL) {
    sqlite3_close(ogn_db);
  }

  if (icao_db != NULL) {
    sqlite3_close(icao_db);
  }
}

static void play_file(snd_pcm_t *pcm_handle, char *filename, short int* buf, snd_pcm_uframes_t frames)
{
    int pcmrc;
    int readcount;

    SF_INFO sfinfo;
    SNDFILE *infile = NULL;

    infile = sf_open(filename, SFM_READ, &sfinfo);

    while ((readcount = sf_readf_short(infile, buf, frames))>0) {

        pcmrc = snd_pcm_writei(pcm_handle, buf, readcount);
        if (pcmrc == -EPIPE) {
            fprintf(stderr, "Underrun!\n");
            snd_pcm_prepare(pcm_handle);
        }
        else if (pcmrc < 0) {
            fprintf(stderr, "Error writing to PCM device: %s\n", snd_strerror(pcmrc));
        }
        else if (pcmrc != readcount) {
            fprintf(stderr,"PCM write difffers from file read.\n");
        }
    }

    sf_close(infile);
}

static void RK35_TTS(char *message)
{
  snd_pcm_t *pcm_handle;
  snd_pcm_hw_params_t *params;
  snd_pcm_uframes_t frames;
  short int* buf = NULL;
  int dir;
  int channels = 1;

  char filename[MAX_FILENAME_LEN];

  if (!strcmp(message, "POST")) {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      /* keep boot-time SkyView logo on the screen for 7 seconds */
      delay(7000);
    }
  } else if (settings->voice != VOICE_OFF) {

    /* Open the PCM device in playback mode */
    snd_pcm_open(&pcm_handle, PCM_DEVICE, SND_PCM_STREAM_PLAYBACK, 0);

    /* Allocate parameters object and fill it with default values*/
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    /* Set parameters */
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_handle, params, 1);
    snd_pcm_hw_params_set_rate(pcm_handle, params, 22050, 0);

    /* Write parameters */
    snd_pcm_hw_params(pcm_handle, params);

    /* Allocate buffer to hold single period */
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);

    buf = (short int *) malloc(frames * channels * sizeof(short int));

    char *word = strtok (message, " ");

    while (word != NULL)
    {
        strcpy(filename, WAV_FILE_PREFIX);
        strcat(filename,  settings->voice == VOICE_1 ? VOICE1_SUBDIR :
                         (settings->voice == VOICE_2 ? VOICE2_SUBDIR :
                         (settings->voice == VOICE_3 ? VOICE3_SUBDIR :
                          "" )));
        strcat(filename, word);
        strcat(filename, WAV_FILE_SUFFIX);
        play_file(pcm_handle, filename, buf, frames);
        word = strtok (NULL, " ");

        /* Poll input source(s) */
        Input_loop();
    }

    snd_pcm_drain(pcm_handle);
    snd_pcm_close(pcm_handle);
    free(buf);
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_GPIO_PIN_KEY1);
AceButton button_up  (SOC_GPIO_PIN_KEY2);
AceButton button_down(SOC_GPIO_PIN_KEY0);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
  if        (button == &button_mode) {
    Serial.print(F("MODE "));
  } else if (button == &button_up) {
    Serial.print(F("UP   "));
  } else if (button == &button_down) {
    Serial.print(F("DOWN "));
  }

  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
#endif

  switch (eventType) {
    case AceButton::kEventPressed:
      if (button == &button_mode) {
        EPD_Mode();
      } else if (button == &button_up) {
        EPD_Up();
      } else if (button == &button_down) {
        EPD_Down();
      }
      break;
    case AceButton::kEventReleased:
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_mode) {
        shutdown("NORMAL OFF");
        Serial.println(F("This will never be printed."));
      }
      break;
  }
}

static void RK35_Button_setup()
{
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {

#if defined(USE_BCMLIB)
    // Sets the pins as input.
    bcm2835_gpio_fsel(SOC_GPIO_PIN_KEY0,     BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SOC_GPIO_PIN_KEY1,     BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SOC_GPIO_PIN_KEY2,     BCM2835_GPIO_FSEL_INPT);

    // Sets the Pull-up mode for the pins.
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY0,  BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY1,  BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY2,  BCM2835_GPIO_PUD_UP);
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
    pinMode(SOC_GPIO_PIN_KEY0, INPUT /* INPUT_PULLUP */);
    pinMode(SOC_GPIO_PIN_KEY1, INPUT /* INPUT_PULLUP */);
    pinMode(SOC_GPIO_PIN_KEY2, INPUT /* INPUT_PULLUP */);
#endif /* USE_LGPIO */

    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
    ModeButtonConfig->setEventHandler(handleEvent);
    ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    ModeButtonConfig->setDebounceDelay(15);
    ModeButtonConfig->setClickDelay(100);
    ModeButtonConfig->setDoubleClickDelay(1000);
    ModeButtonConfig->setLongPressDelay(2000);

    ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
    UpButtonConfig->setEventHandler(handleEvent);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    UpButtonConfig->setDebounceDelay(15);
    UpButtonConfig->setClickDelay(100);
    UpButtonConfig->setDoubleClickDelay(1000);
    UpButtonConfig->setLongPressDelay(2000);

    ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
    DownButtonConfig->setEventHandler(handleEvent);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    DownButtonConfig->setDebounceDelay(15);
    DownButtonConfig->setClickDelay(100);
    DownButtonConfig->setDoubleClickDelay(1000);
    DownButtonConfig->setLongPressDelay(2000);
  }
}

static void RK35_Button_loop()
{
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
    button_mode.check();
    button_up.check();
    button_down.check();
  }
}

static void RK35_Button_fini()
{
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
#if defined(USE_BCMLIB)
    // Clears the Pull-up mode for the pins.
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY0,  BCM2835_GPIO_PUD_OFF);
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY1,  BCM2835_GPIO_PUD_OFF);
    bcm2835_gpio_set_pud(SOC_GPIO_PIN_KEY2,  BCM2835_GPIO_PUD_OFF);
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
    pinMode(SOC_GPIO_PIN_KEY0, INPUT);
    pinMode(SOC_GPIO_PIN_KEY1, INPUT);
    pinMode(SOC_GPIO_PIN_KEY2, INPUT);
#endif /* USE_LGPIO */
  }
}

static void RK35_WDT_setup()
{
  /* TBD */
}

static void RK35_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t RK35_ops = {
  SOC_RK3506,
  "RK3506",
  RK35_setup,
  RK35_post_init,
  RK35_loop,
  RK35_fini,
  RK35_reset,
  RK35_getChipId,
  RK35_getFreeHeap,
  RK35_EEPROM_begin,
  RK35_EEPROM_extension,
  NULL,
  NULL,
  RK35_swSer_begin,
  NULL,
  NULL,
  NULL,
  RK35_Battery_setup,
  RK35_Battery_voltage,
  RK35_Display_setup,
  RK35_Display_loop,
  RK35_Display_fini,
  RK35_Display_is_ready,
  RK35_Display_update,
  RK35_WiFi_Receive_UDP,
  RK35_WiFi_clients_count,
  RK35_DB_init,
  RK35_DB_query,
  RK35_DB_fini,
  RK35_TTS,
  RK35_Button_setup,
  RK35_Button_loop,
  RK35_Button_fini,
  RK35_WDT_setup,
  RK35_WDT_fini,
  NULL,
  NULL,
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

static void RK35_ParseSettings()
{
  if (inputAvailable()) {
    std::getline(std::cin, input_line);
    const char *str = input_line.c_str();
    int len = input_line.length();

    if (str[0] == '{') {
      // JSON input

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"SKYVIEW")) {
          parseSettings(root);
        }
      }

      jsonBuffer.clear();
    }
  }
}

/* Poll input source(s) */
void Input_loop() {
  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_loop();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_loop();
    break;
  }
}

int main(int argc, char *argv[])
{
  bool isSysVinit = false;
  int opt;

  while ((opt = getopt(argc, argv, "b")) != -1) {
      switch (opt) {
      case 'b': isSysVinit = true; break;
      default: break;
      }
  }

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
  Serial.print(F(SKYVIEW_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SKYVIEW_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2019-2025 Linar Yusupov. All rights reserved."));
  Serial.flush();

#if !defined(EXCLUDE_EEPROM)
  EEPROM_setup();
#endif /* EXCLUDE_EEPROM */

  RK35_ParseSettings();

  Battery_setup();
  SoC->Button_setup();

  switch (settings->adapter)
  {
  case ADAPTER_WAVESHARE_PICO_2_7:
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
    Serial.print(F("Intializing E-ink display module (may take up to 10 seconds)... "));
    Serial.flush();

    hw_info.display = SoC->Display_setup(!isSysVinit);
    if (hw_info.display != DISPLAY_NONE) {
      Serial.println(F(" done."));
    } else {
      Serial.println(F(" failed!"));
    }
    break;
  case ADAPTER_OLED:
    /* 2.42" SPI OLED does not have MISO wire to probe the display */
    OLED_setup();
    hw_info.display = DISPLAY_OLED_2_4;
    break;
  default:
    break;
  }

  if (isSysVinit) {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      EPD_text_Draw_Message("PLEASE,", "WAIT");
      SoC->Display_update(EPD_UPDATE_SLOW);
    }

    SoC->Button_fini();
    SoC_fini();
  }

  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_setup();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_setup();
    break;
  }

  if (!SoC->DB_init()) {
      fprintf( stderr, "Unable to open aircrafts database(s)\n\n" );
      exit(EXIT_FAILURE);
  }

  char sentence[] = "POST";
  SoC->TTS(sentence);

  Traffic_setup();

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

  // Uni_Udp.begin(RELAY_SRC_PORT);

  // Serial.print(F("UDP  server has started at port: "));
  // Serial.println((unsigned long) RELAY_SRC_PORT);
#endif /* USE_BRIDGE */

  SoC->WDT_setup();

  while (true) {

    SoC->Button_loop();

    Input_loop();

    Traffic_loop();

    SoC->Display_loop();

    Traffic_ClearExpired();

    SoC->loop();
  }

#if defined(USE_BRIDGE)
  WebServer.end();
  // Uni_Udp.stop();
#endif /* USE_BRIDGE */

  return 0;
}

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  SoC->DB_fini();

  SoC->Display_fini(msg, screen_saver);

  SoC->Button_fini();

#if defined(USE_BRIDGE)
  WebServer.end();
  // Uni_Udp.stop();
#endif /* USE_BRIDGE */

  SoC_fini();
}

#endif /* LUCKFOX_LYRA */

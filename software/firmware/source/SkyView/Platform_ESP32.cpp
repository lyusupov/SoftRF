/*
 * Platform_ESP32.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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
#if defined(ESP32)

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <rom/spi_flash.h>
#include <flashchips.h>

#include "SoCHelper.h"
#include "EPDHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"

#include "SkyView.h"

#include <battery.h>
#include <sqlite3.h>
#include <SD.h>

#include "driver/i2s.h"

#include <esp_wifi.h>
#include <esp_bt.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  28        /* Time ESP32 will go to sleep (in seconds) */

WebServer server ( 80 );

/*
 * TTGO-T5S. Pin definition

#define BUSY_PIN        4
#define CS_PIN          5
#define RST_PIN         16
#define DC_PIN          17
#define SCK_PIN         18
#define MOSI_PIN        23

P1-1                    21
P1-2                    22 (LED)

I2S MAX98357A           26
                        25
                        19

I2S MIC                 27
                        32
                        33

B0                      RST
B1                      38
B2                      37
B3                      39

SD                      2
                        13
                        14
                        15

P2                      0
                        12
                        13
                        RXD
                        TXD
                        34
                        35 (BAT)
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_ttgo_t5s(GxEPD2_270(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

/*
 * Waveshare E-Paper ESP32 Driver Board

#define SCK_PIN         13
#define MOSI_PIN        14
#define CS_PIN          15
#define BUSY_PIN        25
#define RST_PIN         26
#define DC_PIN          27

B1                      0
LED                     2

RX0, TX0                3,1

P                       0,2,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34,35
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare(GxEPD2_270(/*CS=15*/ 15, /*DC=*/ 27, /*RST=*/ 26, /*BUSY=*/ 25));

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static sqlite3 *fln_db  = NULL;
static sqlite3 *ogn_db  = NULL;
static sqlite3 *icao_db = NULL;

static uint8_t sdcard_files_to_open = 0;

SPIClass SPI1(HSPI);

/* variables hold file, state of process wav file and wav file properties */
wavProperties_t wavProps;

//i2s configuration
int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
     .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
     .sample_rate          = 22050,
     .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
     .dma_buf_count        = 8,
     .dma_buf_len          = 128   //Interrupt level 1
    };

i2s_pin_config_t pin_config = {
    .bck_io_num   = SOC_GPIO_PIN_BCLK,
    .ws_io_num    = SOC_GPIO_PIN_LRCLK,
    .data_out_num = SOC_GPIO_PIN_DOUT,
    .data_in_num  = -1  // Not used
};

RTC_DATA_ATTR int bootCount = 0;

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

static void ESP32_fini()
{
  int mode_button_pin = SOC_BUTTON_MODE_DEF;

  if (settings && (settings->adapter == ADAPTER_TTGO_T5S)) {
    SPI1.end();

    mode_button_pin = SOC_BUTTON_MODE_T5S;
  }

  esp_wifi_stop();
  esp_bt_controller_disable();
  SPI.end();

  /*
   * manually apply this fix onto Arduino Core for ESP32:
   * https://github.com/espressif/arduino-esp32/pull/4272
   * to put SD card into idle state
   *
   *  SkyView EZ sleep current (from 3.7V battery source):
   *  ---------------------------------------------------
   *  SD card in  -            0.2 mA
   *  SD card out -            0.1 mA
   */
  esp_sleep_enable_ext1_wakeup(1ULL << mode_button_pin, ESP_EXT1_WAKEUP_ALL_LOW);

//  Serial.println("Going to sleep now");
//  Serial.flush();

  esp_deep_sleep_start();
}

static void ESP32_setup()
{
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ++bootCount;

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }

  uint32_t flash_id = ESP32_getFlashId();

  /*
   *    Board         |   Module   |  Flash memory IC
   *  ----------------+------------+--------------------
   *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V11    |            | BOYA_BY25Q32AL
   *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
   *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
   *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
   *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
   */

  if (psramFound()) {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      /* ESP32-WROVER module */
      hw_info.revision = HW_REV_T8_1_8;
      break;
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  } else {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
      hw_info.revision = HW_REV_DEVKIT;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
      hw_info.revision = HW_REV_T5S_1_9;
      break;
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
      hw_info.revision = HW_REV_T5S_2_8;
      break;
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  }
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static const int8_t ESP32_dB_to_power_level[21] = {
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) {
    dB = 20;
  }

  if (dB < 0) {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  SerialInput.setRxBufferSize(baud / 10); /* 1 second */
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static void ESP32_Battery_setup()
{
  calibrate_voltage(settings->adapter == ADAPTER_TTGO_T5S ?
                    ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
}

static float ESP32_Battery_voltage()
{
  float voltage = ((float) read_voltage()) * 0.001 ;

  /* T5 has voltage divider 100k/100k on board */
  return (settings->adapter == ADAPTER_TTGO_T5S ? 2 * voltage : voltage);
}

#define EPD_STACK_SZ      (256*4)
static TaskHandle_t EPD_Task_Handle = NULL;

static void ESP32_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_ESP32:
    display = &epd_waveshare;
    SPI.begin(SOC_GPIO_PIN_SCK_WS,
              SOC_GPIO_PIN_MISO_WS,
              SOC_GPIO_PIN_MOSI_WS,
              SOC_GPIO_PIN_SS_WS);
    break;
  case ADAPTER_TTGO_T5S:
  default:
    display = &epd_ttgo_t5s;
    SPI.begin(SOC_GPIO_PIN_SCK_T5S,
              SOC_GPIO_PIN_MISO_T5S,
              SOC_GPIO_PIN_MOSI_T5S,
              SOC_GPIO_PIN_SS_T5S);

    /* SD-SPI init */
    SPI1.begin(SOC_SD_PIN_SCK_T5S,
               SOC_SD_PIN_MISO_T5S,
               SOC_SD_PIN_MOSI_T5S,
               SOC_SD_PIN_SS_T5S);
    break;
  }

  xTaskCreateUniversal(EPD_Task, "EPD update", EPD_STACK_SZ, NULL, 1,
                       &EPD_Task_Handle, CONFIG_ARDUINO_RUNNING_CORE);
}

static void ESP32_EPD_fini()
{
  if( EPD_Task_Handle != NULL )
  {
    vTaskDelete( EPD_Task_Handle );
  }
}

static bool ESP32_EPD_is_ready()
{
//  return true;
  return (EPD_task_command == EPD_UPDATE_NONE);
}

static void ESP32_EPD_update(int val)
{
//  EPD_Update_Sync(val);
  EPD_task_command = val;
}

static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static int ESP32_WiFi_clients_count()
{
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    return infoList.num;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

static bool ESP32_DB_init()
{
  bool rval = false;

  if (settings->adapter != ADAPTER_TTGO_T5S) {
    return rval;
  }

  sdcard_files_to_open += (settings->adb   == DB_FLN    ? 1 : 0);
  sdcard_files_to_open += (settings->adb   == DB_OGN    ? 1 : 0);
  sdcard_files_to_open += (settings->adb   == DB_ICAO   ? 1 : 0);
  sdcard_files_to_open += (settings->voice != VOICE_OFF ? 1 : 0);

  if (!SD.begin(SOC_SD_PIN_SS_T5S, SPI1, 4000000, "/sd", sdcard_files_to_open)) {
    Serial.println(F("ERROR: Failed to mount microSD card."));
    return rval;
  }

  if (settings->adb == DB_NONE) {
    return rval;
  }

  sqlite3_initialize();

  if (settings->adb == DB_FLN) {
    sqlite3_open("/sd/Aircrafts/fln.db", &fln_db);

    if (fln_db == NULL)
    {
      Serial.println(F("Failed to open FlarmNet DB\n"));
    }  else {
      rval = true;
    }
  }

  if (settings->adb == DB_OGN) {
    sqlite3_open("/sd/Aircrafts/ogn.db", &ogn_db);

    if (ogn_db == NULL)
    {
      Serial.println(F("Failed to open OGN DB\n"));
    }  else {
      rval = true;
    }
  }

  if (settings->adb == DB_ICAO) {
    sqlite3_open("/sd/Aircrafts/icao.db", &icao_db);

    if (icao_db == NULL)
    {
      Serial.println(F("Failed to open ICAO DB\n"));
    }  else {
      rval = true;
    }
  }

  return rval;
}

static bool ESP32_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  sqlite3_stmt *stmt;
  char *query = NULL;
  int error;
  bool rval = false;
  const char *reg_key, *db_key;
  sqlite3 *db;

  if (settings->adapter != ADAPTER_TTGO_T5S) {
    return false;
  }

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

static void ESP32_DB_fini()
{
  if (settings->adapter == ADAPTER_TTGO_T5S) {

    if (settings->adb != DB_NONE) {
      if (fln_db != NULL) {
        sqlite3_close(fln_db);
      }

      if (ogn_db != NULL) {
        sqlite3_close(ogn_db);
      }

      if (icao_db != NULL) {
        sqlite3_close(icao_db);
      }

      sqlite3_shutdown();
    }

    SD.end();
  }
}

/* write sample data to I2S */
int i2s_write_sample_nb(uint32_t sample)
{
  return i2s_write_bytes((i2s_port_t)i2s_num, (const char *)&sample,
                          sizeof(uint32_t), 100);
}

/* read 4 bytes of data from wav file */
int read4bytes(File file, uint32_t *chunkId)
{
  int n = file.read((uint8_t *)chunkId, sizeof(uint32_t));
  return n;
}

/* these are functions to process wav file */
int readRiff(File file, wavRiff_t *wavRiff)
{
  int n = file.read((uint8_t *)wavRiff, sizeof(wavRiff_t));
  return n;
}
int readProps(File file, wavProperties_t *wavProps)
{
  int n = file.read((uint8_t *)wavProps, sizeof(wavProperties_t));
  return n;
}

static bool play_file(char *filename)
{
  headerState_t state = HEADER_RIFF;
  bool rval = false;

  File wavfile = SD.open(filename);

  if (wavfile) {
    int c = 0;
    int n;
    while (wavfile.available()) {
      switch(state){
        case HEADER_RIFF:
        wavRiff_t wavRiff;
        n = readRiff(wavfile, &wavRiff);
        if(n == sizeof(wavRiff_t)){
          if(wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff.format == CCCC('W', 'A', 'V', 'E')){
            state = HEADER_FMT;
//            Serial.println("HEADER_RIFF");
          }
        }
        break;
        case HEADER_FMT:
        n = readProps(wavfile, &wavProps);
        if(n == sizeof(wavProperties_t)){
          state = HEADER_DATA;
#if 0
            Serial.print("chunkID = "); Serial.println(wavProps.chunkID);
            Serial.print("chunkSize = "); Serial.println(wavProps.chunkSize);
            Serial.print("audioFormat = "); Serial.println(wavProps.audioFormat);
            Serial.print("numChannels = "); Serial.println(wavProps.numChannels);
            Serial.print("sampleRate = "); Serial.println(wavProps.sampleRate);
            Serial.print("byteRate = "); Serial.println(wavProps.byteRate);
            Serial.print("blockAlign = "); Serial.println(wavProps.blockAlign);
            Serial.print("bitsPerSample = "); Serial.println(wavProps.bitsPerSample);
#endif
        }
        break;
        case HEADER_DATA:
        uint32_t chunkId, chunkSize;
        n = read4bytes(wavfile, &chunkId);
        if(n == 4){
          if(chunkId == CCCC('d', 'a', 't', 'a')){
//            Serial.println("HEADER_DATA");
          }
        }
        n = read4bytes(wavfile, &chunkSize);
        if(n == 4){
//          Serial.println("prepare data");
          state = DATA;
        }
        //initialize i2s with configurations above
        i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
        i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
        //set sample rates of i2s to sample rate of wav file
        i2s_set_sample_rates((i2s_port_t)i2s_num, wavProps.sampleRate);
        break;
        /* after processing wav file, it is time to process music data */
        case DATA:
        uint32_t data;
        n = read4bytes(wavfile, &data);
        i2s_write_sample_nb(data);
        break;
      }
    }
    wavfile.close();
    rval = true;
  } else {
    Serial.println(F("error opening WAV file"));
  }
  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }

  return rval;
}

static void ESP32_TTS(char *message)
{
  char filename[MAX_FILENAME_LEN];

  if (strcmp(message, "POST")) {
    if (settings->voice != VOICE_OFF && settings->adapter == ADAPTER_TTGO_T5S) {

      if (SD.cardType() == CARD_NONE)
        return;

      EPD_Message("VOICE", "ALERT");

      bool wdt_status = loopTaskWDTEnabled;

      if (wdt_status) {
        disableLoopWDT();
      }

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
          play_file(filename);
          word = strtok (NULL, " ");

          yield();

          /* Poll input source(s) */
          Input_loop();
      }

      if (wdt_status) {
        enableLoopWDT();
      }
    }
  } else {
    if (settings->voice != VOICE_OFF && settings->adapter == ADAPTER_TTGO_T5S) {

      strcpy(filename, WAV_FILE_PREFIX);
      strcat(filename, "POST");
      strcat(filename, WAV_FILE_SUFFIX);

      if (SD.cardType() == CARD_NONE || !play_file(filename)) {
        /* keep boot-time SkyView logo on the screen for 7 seconds */
        delay(7000);
      }
    } else {
      if (hw_info.display == DISPLAY_EPD_2_7) {
        /* keep boot-time SkyView logo on the screen for 7 seconds */
        delay(7000);
      }
    }
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_BUTTON_MODE_T5S);
AceButton button_up  (SOC_BUTTON_UP_T5S);
AceButton button_down(SOC_BUTTON_DOWN_T5S);

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
      break;
    case AceButton::kEventReleased:
      if (button == &button_mode) {
        EPD_Mode();
      } else if (button == &button_up) {
        EPD_Up();
      } else if (button == &button_down) {
        EPD_Down();
      }
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_mode) {
        shutdown("NORMAL OFF");
        Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_mode.check();
}

void onUpButtonEvent() {
  button_up.check();
}

void onDownButtonEvent() {
  button_down.check();
}

static void ESP32_Button_setup()
{
  int mode_button_pin = settings->adapter == ADAPTER_TTGO_T5S ?
                        SOC_BUTTON_MODE_T5S : SOC_BUTTON_MODE_DEF;

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, INPUT);

  button_mode.init(mode_button_pin);

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

  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );

  if (settings->adapter == ADAPTER_TTGO_T5S) {

    // Button(s) uses external pull up resistor.
    pinMode(SOC_BUTTON_UP_T5S,   INPUT);
    pinMode(SOC_BUTTON_DOWN_T5S, INPUT);

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

    attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_UP_T5S),   onUpButtonEvent,   CHANGE );
    attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_DOWN_T5S), onDownButtonEvent, CHANGE );
  }
}

static void ESP32_Button_loop()
{
  button_mode.check();

  if (settings->adapter == ADAPTER_TTGO_T5S) {
    button_up.check();
    button_down.check();
  }
}

static void ESP32_Button_fini()
{

}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_fini,
  ESP32_getChipId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_hostname,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_WiFiUDP_stopAll,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_EPD_setup,
  ESP32_EPD_fini,
  ESP32_EPD_is_ready,
  ESP32_EPD_update,
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_clients_count,
  ESP32_DB_init,
  ESP32_DB_query,
  ESP32_DB_fini,
  ESP32_TTS,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  &ESP32_Bluetooth_ops
};

#endif /* ESP32 */

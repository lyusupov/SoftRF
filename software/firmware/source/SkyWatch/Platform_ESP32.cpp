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
#include <axp20x.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"
#include "BatteryHelper.h"

#include <battery.h>
#include <sqlite3.h>
#include <SD.h>

//#include "driver/i2s.h"

#include <esp_wifi.h>
#include <esp_bt.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  28        /* Time ESP32 will go to sleep (in seconds) */

WebServer server ( 80 );

AXP20X_Class  axp;
PCF8563_Class *rtc = nullptr;
BMA           *bma = nullptr;
I2CBus        *i2c = nullptr;

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static sqlite3 *fln_db;
static sqlite3 *ogn_db;
static sqlite3 *icao_db;

SPIClass uSD_SPI(HSPI);

#if 0
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
#endif

RTC_DATA_ATTR int bootCount   = 0;
static portMUX_TYPE PMU_mutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE BMA_mutex = portMUX_INITIALIZER_UNLOCKED;
volatile bool PMU_Irq         = false;
volatile bool BMA_Irq         = false;

static void IRAM_ATTR ESP32_PMU_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&PMU_mutex);
  PMU_Irq = true;
  portEXIT_CRITICAL_ISR(&PMU_mutex);
}

static void IRAM_ATTR ESP32_BMA_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&BMA_mutex);
  BMA_Irq = true;
  portEXIT_CRITICAL_ISR(&BMA_mutex);
}

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

static int ESP32_I2C_readBytes(uint8_t devAddress, uint8_t regAddress, uint8_t *data, uint8_t len)
{
    if (!i2c) return 0xFF;
    return i2c->readBytes(devAddress, regAddress, data, len);
}

static int ESP32_I2C_writeBytes(uint8_t devAddress, uint8_t regAddress, uint8_t *data, uint8_t len)
{
    if (!i2c) return 0xFF;
    return i2c->writeBytes(devAddress, regAddress, data, len);
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
   *    Board          |   Module   |  Flash memory IC
   *  -----------------+------------+--------------------
   *  DoIt ESP32       | WROOM      | GIGADEVICE_GD25Q32
   *  TTGO LoRa32 V2.0 | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T-Beam V06  |            | WINBOND_NEX_W25Q32_V
   *  TTGO T8  V1.8    | WROVER     | GIGADEVICE_GD25LQ32
   *  TTGO T5S V1.9    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T-Watch     |            | WINBOND_NEX_W25Q128_V
   */

  if (psramFound()) {
    switch(flash_id)
    {
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q128_V):
      hw_info.model = SOFTRF_MODEL_SKYWATCH;
      hw_info.revision = HW_REV_T_WATCH_19;
      break;
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      hw_info.model = SOFTRF_MODEL_WEBTOP;
      hw_info.revision = HW_REV_T8;
      break;
    default:
      hw_info.model = SOFTRF_MODEL_WEBTOP;
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  } else {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
      hw_info.model = SOFTRF_MODEL_WEBTOP;
      hw_info.revision = HW_REV_DEVKIT;
      break;
    default:
      hw_info.model = SOFTRF_MODEL_WEBTOP;
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  }

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    bool axp_present = false;
    bool bma_present = false;
    bool rtc_present = false;

    Wire1.begin(SOC_GPIO_PIN_TWATCH_SEN_SDA , SOC_GPIO_PIN_TWATCH_SEN_SCL);
    Wire1.beginTransmission(AXP202_SLAVE_ADDRESS);
    axp_present = (Wire1.endTransmission() == 0);

    Wire1.beginTransmission(BMA4_I2C_ADDR_SECONDARY);
    bma_present = (Wire1.endTransmission() == 0);

    Wire1.beginTransmission(PCF8563_SLAVE_ADDRESS);
    rtc_present = (Wire1.endTransmission() == 0);

    i2c = new I2CBus(Wire1, SOC_GPIO_PIN_TWATCH_SEN_SDA, SOC_GPIO_PIN_TWATCH_SEN_SCL);

    if (axp_present && (i2c != nullptr)) {
      axp.begin(ESP32_I2C_readBytes, ESP32_I2C_writeBytes, AXP202_SLAVE_ADDRESS);

      axp.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);
      axp.adc1Enable(0xFF, AXP202_OFF);
      axp.adc2Enable(0xFF, AXP202_OFF);

      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

      axp.setPowerOutPut(AXP202_LDO2, AXP202_ON); // BL
      axp.setPowerOutPut(AXP202_LDO3, AXP202_ON); // S76G (MCU + LoRa)
      axp.setLDO4Voltage(AXP202_LDO4_1800MV);
      axp.setPowerOutPut(AXP202_LDO4, AXP202_ON); // S76G (Sony GNSS)

      pinMode(SOC_GPIO_PIN_TWATCH_PMU_IRQ, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_PMU_IRQ),
                      ESP32_PMU_Interrupt_handler, FALLING);

#if DEBUG_POWER
      axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 |
                     AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);
#else
      axp.adc1Enable(AXP202_BATT_VOL_ADC1, AXP202_ON);
#endif

      axp.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
      axp.clearIRQ();
    }

    if (bma_present && (i2c != nullptr)) {
      bma = new BMA(*i2c);

      pinMode(SOC_GPIO_PIN_TWATCH_BMA_IRQ, INPUT);
      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_BMA_IRQ),
                      ESP32_BMA_Interrupt_handler, RISING);
    }

    if (rtc_present && (i2c != nullptr)) {
      rtc = new PCF8563_Class(*i2c);
    }
  }

  /* SD-SPI init */
  uSD_SPI.begin(SOC_GPIO_PIN_TWATCH_SD_SCK,
                SOC_GPIO_PIN_TWATCH_SD_MISO,
                SOC_GPIO_PIN_TWATCH_SD_MOSI,
                SOC_GPIO_PIN_TWATCH_SD_SS);
}

static void ESP32_loop()
{
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    bool is_irq = false;
    bool down = false;

    portENTER_CRITICAL_ISR(&PMU_mutex);
    is_irq = PMU_Irq;
    portEXIT_CRITICAL_ISR(&PMU_mutex);

    if (is_irq) {

      if (axp.readIRQ() == AXP_PASS) {

        if (axp.isPEKLongtPressIRQ()) {
          down = true;
#if 0
          Serial.println(F("Longt Press IRQ"));
          Serial.flush();
#endif
        }
        if (axp.isPEKShortPressIRQ()) {
#if 0
          Serial.println(F("Short Press IRQ"));
          Serial.flush();
#endif
        }

        axp.clearIRQ();
      }

      portENTER_CRITICAL_ISR(&PMU_mutex);
      PMU_Irq = false;
      portEXIT_CRITICAL_ISR(&PMU_mutex);

      if (down) {
        shutdown("  OFF  ");
      }
    }

    if (isTimeToBattery()) {
      if (Battery_voltage() > Battery_threshold()) {
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      } else {
        axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
      }
    }
  }
}

static void ESP32_fini()
{
  uSD_SPI.end();

  esp_wifi_stop();
  esp_bt_controller_disable();

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    axp.setChgLEDMode(AXP20X_LED_OFF);

    axp.setPowerOutPut(AXP202_LDO2, AXP202_OFF); // BL
#if !defined(EB_S76G_1_3)
    axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF); // S76G (Sony GNSS)
#endif
    axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF); // S76G (MCU + LoRa)

    delay(20);

    esp_sleep_enable_ext0_wakeup((gpio_num_t) SOC_GPIO_PIN_TWATCH_PMU_IRQ, 0); // 1 = High, 0 = Low
  }

//  Serial.println("Going to sleep now");
//  Serial.flush();

  esp_deep_sleep_start();
}

static void ESP32_reset()
{
  ESP.restart();
}

static void ESP32_sleep_ms(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
  esp_light_sleep_start();
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

static const int8_t ESP32_dBm_to_power_level[21] = {
  8,  /* 2    dBm, #0 */
  8,  /* 2    dBm, #1 */
  8,  /* 2    dBm, #2 */
  8,  /* 2    dBm, #3 */
  8,  /* 2    dBm, #4 */
  20, /* 5    dBm, #5 */
  20, /* 5    dBm, #6 */
  28, /* 7    dBm, #7 */
  28, /* 7    dBm, #8 */
  34, /* 8.5  dBm, #9 */
  34, /* 8.5  dBm, #10 */
  44, /* 11   dBm, #11 */
  44, /* 11   dBm, #12 */
  52, /* 13   dBm, #13 */
  52, /* 13   dBm, #14 */
  60, /* 15   dBm, #15 */
  60, /* 15   dBm, #16 */
  68, /* 17   dBm, #17 */
  74, /* 18.5 dBm, #18 */
  76, /* 19   dBm, #19 */
  78  /* 19.5 dBm, #20 */
};

static void ESP32_WiFi_set_param(int ndx, int value)
{
  uint32_t lt = value * 60; /* in minutes */

  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    if (value > 20) {
      value = 20; /* dBm */
    }

    if (value < 0) {
      value = 0; /* dBm */
    }

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dBm_to_power_level[value]));
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    tcpip_adapter_dhcps_option(
      (tcpip_adapter_option_mode_t) TCPIP_ADAPTER_OP_SET,
      (tcpip_adapter_option_id_t)   TCPIP_ADAPTER_IP_ADDRESS_LEASE_TIME,
      (void*) &lt, sizeof(lt));
    break;
  default:
    break;
  }
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static IPAddress ESP32_WiFi_get_broadcast()
{
  tcpip_adapter_ip_info_t info;
  IPAddress broadcastIp;

  if (WiFi.getMode() == WIFI_STA) {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
  } else {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &info);
  }
  broadcastIp = ~info.netmask.addr | info.ip.addr;

  return broadcastIp;
}

static void ESP32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;
  WiFiMode_t mode = WiFi.getMode();
  int i = 0;

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = ESP32_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    break;
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    while(i < infoList.num) {
      ClientIP = infoList.sta[i++].ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();
    }
    break;
  case WIFI_OFF:
  default:
    break;
  }
}

static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; // WiFi_Receive_UDP(buf, max_size);
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

static void ESP32_swSer_begin(unsigned long baud)
{
  if (settings->m.connection == CON_SERIAL_MAIN) {
    uint32_t config = hw_info.model == SOFTRF_MODEL_SKYWATCH  &&
                      baud          == SERIAL_IN_BR           ?
                      SERIAL_IN_BITS : SERIAL_8N1;
    SerialInput.begin(baud, config, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  } else {
    Serial.updateBaudRate(baud);
  }
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static void ESP32_Battery_setup()
{
  /* T-Beam v08 and T-Watch have PMU */

  /* TBD */
}

static float ESP32_Battery_voltage()
{
  float voltage = 0.0;

  /* T-Beam v08 and T-Watch have PMU */
  if (axp.isBatteryConnect()) {
    voltage = axp.getBattVoltage();
  }

  return (voltage * 0.001);
}

static bool ESP32_DB_init()
{

  if (!SD.begin(SOC_GPIO_PIN_TWATCH_SD_SS, uSD_SPI)) {
    Serial.println(F("ERROR: Failed to mount microSD card."));
    return false;
  }

  sqlite3_initialize();

  sqlite3_open("/sd/Aircrafts/fln.db", &fln_db);

  if (fln_db == NULL)
  {
    Serial.println(F("Failed to open FlarmNet DB\n"));
    return false;
  }

  sqlite3_open("/sd/Aircrafts/ogn.db", &ogn_db);

  if (ogn_db == NULL)
  {
    Serial.println(F("Failed to open OGN DB\n"));
    sqlite3_close(fln_db);
    return false;
  }

  sqlite3_open("/sd/Aircrafts/icao.db", &icao_db);

  if (icao_db == NULL)
  {
    Serial.println(F("Failed to open ICAO DB\n"));
    sqlite3_close(fln_db);
    sqlite3_close(ogn_db);
    return false;
  }

  return true;
}

static bool ESP32_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
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
    switch (settings->m.idpref)
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
    switch (settings->m.idpref)
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
    switch (settings->m.idpref)
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

    SD.end();
}

#if 0
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

static void play_file(char *filename)
{
  headerState_t state = HEADER_RIFF;

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
  } else {
    Serial.println(F("error opening WAV file"));
  }
  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
}

static void play_memory(const unsigned char *data, int size)
{
  headerState_t state = HEADER_RIFF;
  wavRiff_t *wavRiff;
  wavProperties_t *props;

  while (size > 0) {
    switch(state){
    case HEADER_RIFF:
      wavRiff = (wavRiff_t *) data;

      if(wavRiff->chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff->format == CCCC('W', 'A', 'V', 'E')){
        state = HEADER_FMT;
      }
      data += sizeof(wavRiff_t);
      size -= sizeof(wavRiff_t);
      break;

    case HEADER_FMT:
      props = (wavProperties_t *) data;
      state = HEADER_DATA;
      data += sizeof(wavProperties_t);
      size -= sizeof(wavProperties_t);
      break;

    case HEADER_DATA:
      uint32_t chunkId, chunkSize;
      chunkId = *((uint32_t *) data);
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      chunkSize = *((uint32_t *) data);
      state = DATA;
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);

      //initialize i2s with configurations above
      i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
      i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
      //set sample rates of i2s to sample rate of wav file
      i2s_set_sample_rates((i2s_port_t)i2s_num, props->sampleRate);
      break;

      /* after processing wav file, it is time to process music data */
    case DATA:
      i2s_write_sample_nb(*((uint32_t *) data));
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      break;
    }
  }

  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
}

#include "Melody.h"
#endif

static void ESP32_TTS(char *message)
{
#if 0
  char filename[MAX_FILENAME_LEN];

  if (strcmp(message, "POST")) {
    if (settings->m.voice != VOICE_OFF && settings->m.adapter == ADAPTER_TTGO_T5S) {

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
          strcat(filename,  settings->m.voice == VOICE_1 ? VOICE1_SUBDIR :
                           (settings->m.voice == VOICE_2 ? VOICE2_SUBDIR :
                           (settings->m.voice == VOICE_3 ? VOICE3_SUBDIR :
                            "" )));
          strcat(filename, word);
          strcat(filename, WAV_FILE_SUFFIX);
          play_file(filename);
          word = strtok (NULL, " ");

          yield();
      }

      if (wdt_status) {
        enableLoopWDT();
      }
    }
  } else {
    if (settings->m.voice != VOICE_OFF && settings->m.adapter == ADAPTER_TTGO_T5S) {
      play_memory(melody_wav, (int) melody_wav_len);
    } else {

    }
  }
#endif
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_GPIO_PIN_TWATCH_BUTTON);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
  if        (button == &button_mode) {
    Serial.print(F("MODE "));
  }

  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
#endif

  switch (eventType) {
    case AceButton::kEventPressed:
      if (button == &button_mode) {
//        EPD_Mode();
      }
      break;
    case AceButton::kEventReleased:
      break;
#if 0
    case AceButton::kEventLongPressed:
      if (button == &button_mode) {
        shutdown("NORMAL OFF");
        Serial.println(F("This will never be printed."));
      }
      break;
#endif
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_mode.check();
}

static void ESP32_Button_setup()
{
  // Button(s)) uses external pull up resistor.
  pinMode(SOC_GPIO_PIN_TWATCH_BUTTON, INPUT);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
//  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(100);
  ModeButtonConfig->setDoubleClickDelay(1000);
//  ModeButtonConfig->setLongPressDelay(2000);

  attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_BUTTON), onModeButtonEvent, CHANGE );
}

static void ESP32_Button_loop()
{
  button_mode.check();
}

static void ESP32_Button_fini()
{

}

static bool ESP32_Baro_setup() {

  bool rval = false;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    /* Pre-init 2nd ESP32 I2C bus to stick on these pins */
    Wire.begin(SOC_GPIO_PIN_TWATCH_EXT_SDA, SOC_GPIO_PIN_TWATCH_EXT_SCL);

    rval = true;
  }

  return rval;
}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

static void ESP32_Service_Mode(boolean arg)
{
  if (arg) {
//    Serial.begin(SERIAL_IN_BR, SERIAL_IN_BITS);
     Serial.updateBaudRate(SERIAL_IN_BR);
  WiFi_fini();
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_OUTPUT_LOW_MODE);  // MCU_reset
    delay(10);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_OUTPUT_HIGH_MODE); // BOOT0 high
    delay(100);
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_FLOATING_MODE);    // release MCU_reset (it has pull-up)
    delay(500);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_FLOATING_MODE);    // release BOOT0 (it has pull-down)

    inServiceMode = true;
  } else {
//    Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
    Serial.updateBaudRate(SERIAL_OUT_BR);
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_OUTPUT_LOW_MODE);  // MCU_reset
    delay(10);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_OUTPUT_LOW_MODE);  // BOOT0 low
    delay(100);
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_FLOATING_MODE);    // release MCU_reset (it has pull-up)
    delay(500);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_FLOATING_MODE);    // release BOOT0 (it has pull-down)

    inServiceMode = false;
  }
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_loop,
  ESP32_fini,
  ESP32_reset,
  ESP32_sleep_ms,
  ESP32_getChipId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_set_param,
  ESP32_WiFi_hostname,
  ESP32_WiFiUDP_stopAll,
  ESP32_WiFi_transmit_UDP,
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_clients_count,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_DB_init,
  ESP32_DB_query,
  ESP32_DB_fini,
  ESP32_TTS,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
  ESP32_Baro_setup,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  ESP32_Service_Mode,
  &ESP32_Bluetooth_ops
};

#endif /* ESP32 */

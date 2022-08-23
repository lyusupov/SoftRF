/*
 * BluetoothHelper.cpp
 * Copyright (C) 2018-2022 Linar Yusupov
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
#include "sdkconfig.h"
#endif

#if defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
#include "../system/SoC.h"
#include "EEPROM.h"
#include "Bluetooth.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#include <BluetoothSerial.h>

/*
    BLE code is based on Neil Kolban example for IDF:
      https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini    
    HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "esp_gap_bt_api.h"

#include "WiFi.h"   // HOSTNAME
#include "Battery.h"

#include <core_version.h>

BLEServer* pServer = NULL;
BLECharacteristic* pUARTCharacteristic = NULL;
BLECharacteristic* pBATCharacteristic  = NULL;

BLECharacteristic* pModelCharacteristic         = NULL;
BLECharacteristic* pSerialCharacteristic        = NULL;
BLECharacteristic* pFirmwareCharacteristic      = NULL;
BLECharacteristic* pHardwareCharacteristic      = NULL;
BLECharacteristic* pSoftwareCharacteristic      = NULL;
BLECharacteristic* pManufacturerCharacteristic  = NULL;

bool deviceConnected    = false;
bool oldDeviceConnected = false;

#if defined(USE_BLE_MIDI)
BLECharacteristic* pMIDICharacteristic = NULL;
#endif /* USE_BLE_MIDI */

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
BluetoothSerial SerialBT;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

String BT_name = HOSTNAME;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
    }
};

class UARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pUARTCharacteristic) {
      std::string rxValue = pUARTCharacteristic->getValue();

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

static void ESP32_Bluetooth_setup()
{
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch(settings->bluetooth)
  {
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  case BLUETOOTH_SPP:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str());
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

      // Create the BLE Device
      BLEDevice::init((BT_name+"-LE").c_str());

      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // BLEDevice::setMTU(23);

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(BLEUUID(UART_SERVICE_UUID));

      // Create a BLE Characteristic
      pUARTCharacteristic = pService->createCharacteristic(
                              BLEUUID(UART_CHARACTERISTIC_UUID),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      UserDescriptor.setValue("HMSoft");
      pUARTCharacteristic->addDescriptor(&UserDescriptor);
      pUARTCharacteristic->addDescriptor(new BLE2902());

      pUARTCharacteristic->setCallbacks(new UARTCallbacks());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_BATTERY));

      // Create a BLE Characteristic
      pBATCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_BATTERY_LEVEL),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
      pBATCharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_DEVICE_INFORMATION));

      // Create BLE Characteristics
      pModelCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MODEL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSerialCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SERIAL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pFirmwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_FIRMWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pHardwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_HARDWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSoftwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SOFTWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pManufacturerCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MANUFACTURER_NAME_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );

      const char *Model         = hw_info.model == SOFTRF_MODEL_STANDALONE ? "Standalone Edition" :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK2  ? "Prime Mark II"      :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK3  ? "Prime Mark III"     :
                                  "Unknown";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      const char *Firmware      = "Arduino Core ESP32 " ARDUINO_ESP32_RELEASE;

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SOFTRF_IDENT;
      const char *Software      = SOFTRF_FIRMWARE_VERSION;

      pModelCharacteristic->       setValue((uint8_t *) Model,        strlen(Model));
      pSerialCharacteristic->      setValue((uint8_t *) SerialNum,    strlen(SerialNum));
      pFirmwareCharacteristic->    setValue((uint8_t *) Firmware,     strlen(Firmware));
      pHardwareCharacteristic->    setValue((uint8_t *) Hardware,     strlen(Hardware));
      pSoftwareCharacteristic->    setValue((uint8_t *) Software,     strlen(Software));
      pManufacturerCharacteristic->setValue((uint8_t *) Manufacturer, strlen(Manufacturer));

      // Start the service
      pService->start();

#if defined(USE_BLE_MIDI)
      // Create the BLE Service
      pService = pServer->createService(BLEUUID(MIDI_SERVICE_UUID));

      // Create a BLE Characteristic
      pMIDICharacteristic = pService->createCharacteristic(
                              BLEUUID(MIDI_CHARACTERISTIC_UUID),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      // Create a BLE Descriptor
      pMIDICharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();
#endif /* USE_BLE_MIDI */

      // Start advertising
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(BLEUUID(UART_SERVICE_UUID));
      pAdvertising->addServiceUUID(BLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      pAdvertising->addServiceUUID(BLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
      BLEDevice::startAdvertising();

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
#if defined(ENABLE_BT_VOICE)
    void bt_app_main(void);

    bt_app_main();
#endif
    break;
  case BLUETOOTH_OFF:
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{
  switch(settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = (BLE_FIFO_TX->available() < BLE_MAX_WRITE_CHUNK_SIZE ?
                         BLE_FIFO_TX->available() : BLE_MAX_WRITE_CHUNK_SIZE);

          BLE_FIFO_TX->read((char *) chunk, size);

          pUARTCharacteristic->setValue(chunk, size);
          pUARTCharacteristic->notify();
          BLE_Notify_TimeMarker = millis();
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
      }
      if (deviceConnected && isTimeToBattery()) {
        uint8_t battery_level = Battery_charge();

        pBATCharacteristic->setValue(&battery_level, 1);
        pBATCharacteristic->notify();
      }
    }
    break;
  case BLUETOOTH_OFF:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void ESP32_Bluetooth_fini()
{
  /* TBD */
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch(settings->bluetooth)
  {
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  case BLUETOOTH_SPP:
    rval = SerialBT.available();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available();
    break;
  case BLUETOOTH_OFF:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch(settings->bluetooth)
  {
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  case BLUETOOTH_SPP:
    rval = SerialBT.read();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->read();
    break;
  case BLUETOOTH_OFF:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch(settings->bluetooth)
  {
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  case BLUETOOTH_SPP:
    rval = SerialBT.write(buffer, size);
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  case BLUETOOTH_OFF:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

IODev_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_fini,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#if defined(ENABLE_BT_VOICE)

/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

static void bt_app_task_handler(void *arg);
static bool bt_app_send_msg(bt_app_msg_t *msg);
static void bt_app_work_dispatched(bt_app_msg_t *msg);

static xQueueHandle bt_app_task_queue = NULL;
static xTaskHandle bt_app_task_handle = NULL;

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(&msg, msg.param, p_params);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

void bt_app_task_start_up(void)
{
    bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    xTaskCreate(bt_app_task_handler, "BtAppT", 2048, NULL, configMAX_PRIORITIES - 3, &bt_app_task_handle);
    return;
}

void bt_app_task_shut_down(void)
{
    if (bt_app_task_handle) {
        vTaskDelete(bt_app_task_handle);
        bt_app_task_handle = NULL;
    }
    if (bt_app_task_queue) {
        vQueueDelete(bt_app_task_queue);
        bt_app_task_queue = NULL;
    }
}

/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#define BT_AV_TAG               "BT_AV"

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* A2DP global state */
enum {
    APP_AV_STATE_IDLE,
    APP_AV_STATE_DISCOVERING,
    APP_AV_STATE_DISCOVERED,
    APP_AV_STATE_UNCONNECTED,
    APP_AV_STATE_CONNECTING,
    APP_AV_STATE_CONNECTED,
    APP_AV_STATE_DISCONNECTING,
};

/* sub states of APP_AV_STATE_CONNECTED */
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

#define BT_APP_HEART_BEAT_EVT                (0xff00)

/// handler for bluetooth stack enabled events
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/// callback function for A2DP source
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/// callback function for A2DP source audio data stream
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);

static void a2d_app_heart_beat(void *arg);

/// A2DP application state machine
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

/* A2DP application state machine handler for each state */
static void bt_app_av_state_unconnected(uint16_t event, void *param);
static void bt_app_av_state_connecting(uint16_t event, void *param);
static void bt_app_av_state_connected(uint16_t event, void *param);
static void bt_app_av_state_disconnecting(uint16_t event, void *param);

static esp_bd_addr_t peer_bda = {0};
static uint8_t peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static int m_a2d_state = APP_AV_STATE_IDLE;
static int m_media_state = APP_AV_MEDIA_STATE_IDLE;
static int m_intv_cnt = 0;
static int m_connecting_intv = 0;
static uint32_t m_pkt_cnt = 0;

TimerHandle_t tmr;

int m_sample = 0;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

void bt_app_main()
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes
    gpio_pullup_en(GPIO_NUM_15);
    gpio_pullup_en(GPIO_NUM_2);
    gpio_pullup_en(GPIO_NUM_4);
    gpio_pullup_en(GPIO_NUM_12);
    gpio_pullup_en(GPIO_NUM_13);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        return;
    }

    sdmmc_card_print_info(stdout, card);
    m_sample = open("/sdcard/voice.wav", O_RDONLY);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed\n", __func__);
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed\n", __func__);
        return;
    }

    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(BT_AV_TAG, "Scanned device: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(BT_AV_TAG, "--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(BT_AV_TAG, "--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        return;
    }

    /* search for device named "BT SPEAKER" in its extended inqury response */
    if (eir) {
        get_name_from_eir(eir, peer_bdname, NULL);
        if (strcmp((char *)peer_bdname, "raspberrypi") != 0) {
            return;
        }

        ESP_LOGI(BT_AV_TAG, "Found a target device, address %s, name %s", bda_str, peer_bdname);
        m_a2d_state = APP_AV_STATE_DISCOVERED;
        memcpy(peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        ESP_LOGI(BT_AV_TAG, "Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}


void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        filter_inquiry_scan_result(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (m_a2d_state == APP_AV_STATE_DISCOVERED) {
                m_a2d_state = APP_AV_STATE_CONNECTING;
                ESP_LOGI(BT_AV_TAG, "Device discovery stopped.");
                ESP_LOGI(BT_AV_TAG, "a2dp connecting to peer: %s", peer_bdname);
                esp_a2d_source_connect(peer_bda);
            } else {
                // not discovered, continue to discover
                ESP_LOGI(BT_AV_TAG, "Device discovery failed, continue to discover...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(BT_AV_TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT:
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = "ESP_A2DP_SRC";
        esp_bt_dev_set_device_name(dev_name);

        /* register GAP callback function */
        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize A2DP source */
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_source_init();

        /* set discoverable and connectable mode */
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);

        /* start device discovery */
        ESP_LOGI(BT_AV_TAG, "Starting device discovery...");
        m_a2d_state = APP_AV_STATE_DISCOVERING;
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

        /* create and start heart beat timer */
        do {
            int tmr_id = 0;
            tmr = xTimerCreate("connTmr", (10000 / portTICK_RATE_MS),
                               pdTRUE, (void *)tmr_id, a2d_app_heart_beat);
            xTimerStart(tmr, portMAX_DELAY);
        } while (0);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len)
{
    if (len < 0 || data == NULL) {
        return 0;
    }

    int l = read(m_sample, data, len);
    if (l < len) {
        lseek(m_sample, 0, SEEK_SET);
    }
    return len;
}

static void a2d_app_heart_beat(void *arg)
{
    bt_app_work_dispatch(bt_app_av_sm_hdlr, BT_APP_HEART_BEAT_EVT, NULL, 0, NULL);
}

static void bt_app_av_sm_hdlr(uint16_t event, void *param)
{
    ESP_LOGI(BT_AV_TAG, "%s state %d, evt 0x%x", __func__, m_a2d_state, event);
    switch (m_a2d_state) {
    case APP_AV_STATE_DISCOVERING:
    case APP_AV_STATE_DISCOVERED:
        break;
    case APP_AV_STATE_UNCONNECTED:
        bt_app_av_state_unconnected(event, param);
        break;
    case APP_AV_STATE_CONNECTING:
        bt_app_av_state_connecting(event, param);
        break;
    case APP_AV_STATE_CONNECTED:
        bt_app_av_state_connected(event, param);
        break;
    case APP_AV_STATE_DISCONNECTING:
        bt_app_av_state_disconnecting(event, param);
        break;
    default:
        ESP_LOGE(BT_AV_TAG, "%s invalid state %d", __func__, m_a2d_state);
        break;
    }
}

static void bt_app_av_state_unconnected(uint16_t event, void *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case BT_APP_HEART_BEAT_EVT: {
        uint8_t *p = peer_bda;
        ESP_LOGI(BT_AV_TAG, "a2dp connecting to peer: %02x:%02x:%02x:%02x:%02x:%02x",
                 p[0], p[1], p[2], p[3], p[4], p[5]);
        esp_a2d_source_connect(peer_bda);
        m_a2d_state = APP_AV_STATE_CONNECTING;
        m_connecting_intv = 0;
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

static void bt_app_av_state_connecting(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(BT_AV_TAG, "a2dp connected");
            m_a2d_state =  APP_AV_STATE_CONNECTED;
            m_media_state = APP_AV_MEDIA_STATE_IDLE;

        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            m_a2d_state =  APP_AV_STATE_UNCONNECTED;
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case BT_APP_HEART_BEAT_EVT:
        if (++m_connecting_intv >= 2) {
            m_a2d_state = APP_AV_STATE_UNCONNECTED;
            m_connecting_intv = 0;
        }
        break;
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

static void bt_app_av_media_proc(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (m_media_state) {
    case APP_AV_MEDIA_STATE_IDLE: {
        if (event == BT_APP_HEART_BEAT_EVT) {
            ESP_LOGI(BT_AV_TAG, "a2dp media ready checking ...");
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGI(BT_AV_TAG, "a2dp media ready, starting ...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                m_media_state = APP_AV_MEDIA_STATE_STARTING;
            }
        }
        break;
    }
    case APP_AV_MEDIA_STATE_STARTING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGI(BT_AV_TAG, "a2dp media start successfully.");
                m_intv_cnt = 0;
                m_media_state = APP_AV_MEDIA_STATE_STARTED;
            } else {
                // not started succesfully, transfer to idle state
                ESP_LOGI(BT_AV_TAG, "a2dp media start failed.");
                m_media_state = APP_AV_MEDIA_STATE_IDLE;
            }
        }
        break;
    }
    case APP_AV_MEDIA_STATE_STARTED: {
        if (event == BT_APP_HEART_BEAT_EVT) {
            if (++m_intv_cnt >= 10) {
                ESP_LOGI(BT_AV_TAG, "a2dp media stopping...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
                m_media_state = APP_AV_MEDIA_STATE_STOPPING;
                m_intv_cnt = 0;
            }
        }
        break;
    }
    case APP_AV_MEDIA_STATE_STOPPING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_STOP &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGI(BT_AV_TAG, "a2dp media stopped successfully, disconnecting...");
                m_media_state = APP_AV_MEDIA_STATE_IDLE;
                esp_a2d_source_disconnect(peer_bda);
                m_a2d_state = APP_AV_STATE_DISCONNECTING;
            } else {
                ESP_LOGI(BT_AV_TAG, "a2dp media stopping...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
            }
        }
        break;
    }
    }
}

static void bt_app_av_state_connected(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(BT_AV_TAG, "a2dp disconnected");
            m_a2d_state = APP_AV_STATE_UNCONNECTED;
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            m_pkt_cnt = 0;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT:
        // not suppposed to occur for A2DP source
        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case BT_APP_HEART_BEAT_EVT: {
        bt_app_av_media_proc(event, param);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

static void bt_app_av_state_disconnecting(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(BT_AV_TAG, "a2dp disconnected");
            m_a2d_state =  APP_AV_STATE_UNCONNECTED;
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case BT_APP_HEART_BEAT_EVT:
        break;
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

#endif /* ENABLE_BT_VOICE */

#elif defined(ARDUINO_ARCH_NRF52)

#include "../system/SoC.h"
#include "Bluetooth.h"

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <BLEUart_HM10.h>
#include <TinyGPS++.h>
#if defined(USE_BLE_MIDI)
#include <MIDI.h>
#endif /* USE_BLE_MIDI */

#include "WiFi.h"   // HOSTNAME
#include "Battery.h"
#include "GNSS.h"
#include "RF.h"
#include "../protocol/radio/Legacy.h"
#include "Baro.h"
#include "EEPROM.h"
#include "Sound.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

/*
 * SensorBox Serivce: aba27100-143b-4b81-a444-edcd0000f020
 * Navigation       : aba27100-143b-4b81-a444-edcd0000f022
 * Movement         : aba27100-143b-4b81-a444-edcd0000f023
 * GPS2             : aba27100-143b-4b81-a444-edcd0000f024
 * System           : aba27100-143b-4b81-a444-edcd0000f025
 */

const uint8_t SENSBOX_UUID_SERVICE[] =
{
    0x20, 0xF0, 0x00, 0x00, 0xCD, 0xED, 0x44, 0xA4,
    0x81, 0x4B, 0x3B, 0x14, 0x00, 0x71, 0xA2, 0xAB,
};

const uint8_t SENSBOX_UUID_NAVIGATION[] =
{
    0x22, 0xF0, 0x00, 0x00, 0xCD, 0xED, 0x44, 0xA4,
    0x81, 0x4B, 0x3B, 0x14, 0x00, 0x71, 0xA2, 0xAB,
};

const uint8_t SENSBOX_UUID_MOVEMENT[] =
{
    0x23, 0xF0, 0x00, 0x00, 0xCD, 0xED, 0x44, 0xA4,
    0x81, 0x4B, 0x3B, 0x14, 0x00, 0x71, 0xA2, 0xAB,
};

const uint8_t SENSBOX_UUID_GPS2[] =
{
    0x24, 0xF0, 0x00, 0x00, 0xCD, 0xED, 0x44, 0xA4,
    0x81, 0x4B, 0x3B, 0x14, 0x00, 0x71, 0xA2, 0xAB,
};

const uint8_t SENSBOX_UUID_SYSTEM[] =
{
    0x25, 0xF0, 0x00, 0x00, 0xCD, 0xED, 0x44, 0xA4,
    0x81, 0x4B, 0x3B, 0x14, 0x00, 0x71, 0xA2, 0xAB,
};

BLESensBox::BLESensBox(void) :
  BLEService   (SENSBOX_UUID_SERVICE),
  _sensbox_nav (SENSBOX_UUID_NAVIGATION),
  _sensbox_move(SENSBOX_UUID_MOVEMENT),
  _sensbox_gps2(SENSBOX_UUID_GPS2),
  _sensbox_sys (SENSBOX_UUID_SYSTEM)
{

}

err_t BLESensBox::begin(void)
{
  VERIFY_STATUS( BLEService::begin() );

  _sensbox_nav.setProperties(CHR_PROPS_NOTIFY);
  _sensbox_nav.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _sensbox_nav.setFixedLen(sizeof(sensbox_navigation_t));
  _sensbox_move.setProperties(CHR_PROPS_NOTIFY);
  _sensbox_move.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _sensbox_move.setFixedLen(sizeof(sensbox_movement_t));
  _sensbox_gps2.setProperties(CHR_PROPS_NOTIFY);
  _sensbox_gps2.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _sensbox_gps2.setFixedLen(sizeof(sensbox_gps2_t));
  _sensbox_sys.setProperties(CHR_PROPS_NOTIFY);
  _sensbox_sys.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _sensbox_sys.setFixedLen(sizeof(sensbox_system_t));
  VERIFY_STATUS( _sensbox_nav.begin()  );
  VERIFY_STATUS( _sensbox_move.begin() );
  VERIFY_STATUS( _sensbox_gps2.begin() );
  VERIFY_STATUS( _sensbox_sys.begin()  );

  return ERROR_NONE;
}

bool BLESensBox::notify_nav(uint8_t status)
{
  if (!_sensbox_nav.notifyEnabled(Bluefruit.connHandle()))
    return false;

  sensbox_navigation_t data = {0};

  data.timestamp = ThisAircraft.timestamp;
  data.lat       = (int32_t) (ThisAircraft.latitude  * 10000000);
  data.lon       = (int32_t) (ThisAircraft.longitude * 10000000);
  data.gnss_alt  = (int16_t) ThisAircraft.altitude;
  data.pres_alt  = (int16_t) ThisAircraft.pressure_altitude;
  data.status    = status;

  return _sensbox_nav.notify(&data, sizeof(sensbox_navigation_t)) > 0;
}

bool BLESensBox::notify_move(uint8_t status)
{
  if (!_sensbox_move.notifyEnabled(Bluefruit.connHandle()))
    return false;

  sensbox_movement_t data = {0};

  data.pres_alt  = (int32_t) (ThisAircraft.pressure_altitude * 100);
  data.vario     = (int16_t) ((ThisAircraft.vs * 10) / (_GPS_FEET_PER_METER * 6));
  data.gs        = (int16_t) (ThisAircraft.speed  * _GPS_MPS_PER_KNOT * 10);
  data.cog       = (int16_t) (ThisAircraft.course * 10);
  data.status    = status;

  return _sensbox_move.notify(&data, sizeof(sensbox_movement_t)) > 0;
}

bool BLESensBox::notify_gps2(uint8_t status)
{
  if (!_sensbox_gps2.notifyEnabled(Bluefruit.connHandle()))
    return false;

  sensbox_gps2_t data = {0};

  data.sats      = (uint8_t) gnss.satellites.value();
  data.status    = status;

  return _sensbox_gps2.notify(&data, sizeof(sensbox_gps2_t)) > 0;
}

bool BLESensBox::notify_sys(uint8_t status)
{
  if (!_sensbox_sys.notifyEnabled(Bluefruit.connHandle()))
    return false;

  sensbox_system_t data = {0};

  data.battery   = (uint8_t) Battery_charge();
  data.temp      = (int16_t) (Baro_temperature() * 10);
  data.status    = status;

  return _sensbox_sys.notify(&data, sizeof(sensbox_system_t)) > 0;
}

static unsigned long BLE_Notify_TimeMarker  = 0;
static unsigned long BLE_SensBox_TimeMarker = 0;

/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

// BLE Service
BLEDfu        bledfu;       // OTA DFU service
BLEDis        bledis;       // device information
BLEUart_HM10  bleuart_HM10; // TI UART over BLE
#if !defined(EXCLUDE_NUS)
BLEUart       bleuart_NUS;  // Nordic UART over BLE
#endif /* EXCLUDE_NUS */
BLEBas        blebas;       // battery
BLESensBox    blesens;      // SensBox

#if defined(USE_BLE_MIDI)
BLEMidi       blemidi;

MIDI_CREATE_INSTANCE(BLEMidi, blemidi, MIDI_BLE);
#endif /* USE_BLE_MIDI */

String BT_name = HOSTNAME;

#define UUID16_COMPANY_ID_NORDIC 0x0059

static uint8_t BeaconUuid[16] =
{
 /* https://openuuid.net: becf4a85-29b8-476e-928f-fce11f303344 */
  0xbe, 0xcf, 0x4a, 0x85, 0x29, 0xb8, 0x47, 0x6e,
  0x92, 0x8f, 0xfc, 0xe1, 0x1f, 0x30, 0x33, 0x44
};

// UUID, Major, Minor, RSSI @ 1M
BLEBeacon iBeacon(BeaconUuid, 0x0102, 0x0304, -64);

void startAdv(void)
{
  bool no_data = (settings->nmea_out != NMEA_BLUETOOTH  &&
                  settings->gdl90    != GDL90_BLUETOOTH &&
                  settings->d1090    != D1090_BLUETOOTH);

  // Advertising packet

#if defined(USE_IBEACON)
  if (no_data && settings->volume == BUZZER_OFF) {
    uint32_t id = SoC->getChipId();
    uint16_t major = (id >> 16) & 0x0000FFFF;
    uint16_t minor = (id      ) & 0x0000FFFF;

    // Manufacturer ID is required for Manufacturer Specific Data
    iBeacon.setManufacturer(UUID16_COMPANY_ID_NORDIC);
    iBeacon.setMajorMinor(major, minor);

    // Set the beacon payload using the BLEBeacon class
    Bluefruit.Advertising.setBeacon(iBeacon);
  } else
#endif /* USE_IBEACON */
  {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

#if defined(USE_BLE_MIDI)
    if (settings->volume != BUZZER_OFF) {
      Bluefruit.Advertising.addService(blemidi, bleuart_HM10);
    } else
#endif /* USE_BLE_MIDI */
    {
      Bluefruit.Advertising.addService(
#if !defined(EXCLUDE_NUS)
                                       bleuart_NUS,
#endif /* EXCLUDE_NUS */
                                       bleuart_HM10);
    }
  }

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
#if DEBUG_BLE
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
#endif
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
#if DEBUG_BLE
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
#endif
}

void nRF52_Bluetooth_setup()
{
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(LED_BLUE == SOC_GPIO_LED_BLE ? true : false);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName((BT_name+"-LE").c_str());
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer(nRF52_Device_Manufacturer);
  bledis.setModel(nRF52_Device_Model);
  bledis.setHardwareRev(hw_info.revision > 2 ?
                        Hardware_Rev[3] : Hardware_Rev[hw_info.revision]);
  bledis.setSoftwareRev(SOFTRF_FIRMWARE_VERSION);
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart_HM10.begin();
#if !defined(EXCLUDE_NUS)
  bleuart_NUS.begin();
  bleuart_NUS.bufferTXD(true);
#endif /* EXCLUDE_NUS */

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Start SensBox Service
  blesens.begin();

#if defined(USE_BLE_MIDI)
  // Initialize MIDI with no any input channels
  // This will also call blemidi service's begin()
  MIDI_BLE.begin(MIDI_CHANNEL_OFF);
#endif /* USE_BLE_MIDI */

  // Set up and start advertising
  startAdv();

#if DEBUG_BLE
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
#endif

  BLE_Notify_TimeMarker  = millis();
  BLE_SensBox_TimeMarker = millis();
}

/*********************************************************************
 End of Adafruit licensed text
*********************************************************************/

static void nRF52_Bluetooth_loop()
{
  // notify changed value
  // bluetooth stack will go into congestion, if too many packets are sent
  if ( Bluefruit.connected()              &&
       bleuart_HM10.notifyEnabled()       &&
       (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */
    bleuart_HM10.flushTXD();

    BLE_Notify_TimeMarker = millis();
  }

  if (isTimeToBattery()) {
    blebas.write(Battery_charge());
  }

  if (Bluefruit.connected() && isTimeToSensBox()) {
    uint8_t sens_status = isValidFix() ? GNSS_STATUS_3D_MOVING : GNSS_STATUS_NONE;
    blesens.notify_nav (sens_status);
    blesens.notify_move(sens_status);
    blesens.notify_gps2(sens_status);
    blesens.notify_sys (sens_status);
    BLE_SensBox_TimeMarker = millis();
  }
}

static void nRF52_Bluetooth_fini()
{
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  if ( Bluefruit.connected() ) {
    if ( bleuart_HM10.notifyEnabled() ) {
      // flush TXD since we use bufferTXD()
      bleuart_HM10.flushTXD();
    }

#if !defined(EXCLUDE_NUS)
    if ( bleuart_NUS.notifyEnabled() ) {
      // flush TXD since we use bufferTXD()
      bleuart_NUS.flushTXD();
    }
#endif /* EXCLUDE_NUS */
  }

  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }

  if (sd_en) sd_softdevice_disable();
}

static int nRF52_Bluetooth_available()
{
  int rval = 0;

  if ( !Bluefruit.connected() ) {
    return rval;
  }

  /* Give priority to HM-10 input */
  if ( bleuart_HM10.notifyEnabled() ) {
    return bleuart_HM10.available();
  }

#if !defined(EXCLUDE_NUS)
  if ( bleuart_NUS.notifyEnabled() ) {
    rval = bleuart_NUS.available();
  }
#endif /* EXCLUDE_NUS */

  return rval;
}

static int nRF52_Bluetooth_read()
{
  int rval = -1;

  if ( !Bluefruit.connected() ) {
    return rval;
  }

  /* Give priority to HM-10 input */
  if ( bleuart_HM10.notifyEnabled() ) {
    return bleuart_HM10.read();
  }

#if !defined(EXCLUDE_NUS)
  if ( bleuart_NUS.notifyEnabled() ) {
    rval = bleuart_NUS.read();
  }
#endif /* EXCLUDE_NUS */

  return rval;
}

static size_t nRF52_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if ( !Bluefruit.connected() ) {
    return rval;
  }

  /* Give priority to HM-10 output */
  if ( bleuart_HM10.notifyEnabled() ) {
    return bleuart_HM10.write(buffer, size);
  }

#if !defined(EXCLUDE_NUS)
  if ( bleuart_NUS.notifyEnabled() ) {
    rval = bleuart_NUS.write(buffer, size);
  }
#endif /* EXCLUDE_NUS */

  return rval;
}

IODev_ops_t nRF52_Bluetooth_ops = {
  "nRF52 Bluetooth",
  nRF52_Bluetooth_setup,
  nRF52_Bluetooth_loop,
  nRF52_Bluetooth_fini,
  nRF52_Bluetooth_available,
  nRF52_Bluetooth_read,
  nRF52_Bluetooth_write
};

#endif /* ESP32 or ARDUINO_ARCH_NRF52 */

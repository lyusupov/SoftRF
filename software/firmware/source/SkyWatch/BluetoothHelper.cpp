/*
 * BluetoothHelper.cpp
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

#if defined(ESP32)
#include "sdkconfig.h"
#endif

#if defined(ESP32)                     && \
   !defined(CONFIG_IDF_TARGET_ESP32S2) && \
   !defined(CONFIG_IDF_TARGET_ESP32P4)

#include "SoCHelper.h"

#if !defined(USE_ARDUINOBLE)

#if !defined(CONFIG_BT_ENABLED)
#error Bluetooth is not enabled!
#endif

#if defined(USE_NIMBLE)
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#else
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
#endif /* USE_NIMBLE */

#if !defined(CONFIG_IDF_TARGET_ESP32C5)
#include "esp_gap_bt_api.h"
#endif /* CONFIG_IDF_TARGET_ESP32C5 */

#include "EEPROMHelper.h"
#include "BluetoothHelper.h"

#if defined(USE_NIMBLE)
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pCharacteristic = NULL;
#else
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
#endif /* USE_NIMBLE */

bool deviceConnected = false;
bool oldDeviceConnected = false;

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif /* CONFIG_IDF_TARGET_ESP32 */
String BT_name;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

#if !defined(USE_NIMBLE)
BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));
#endif /* USE_NIMBLE */

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
      String rxValue = pCharacteristic->getValue();
#else
      std::string rxValue = pCharacteristic->getValue();
#endif /* ESP_IDF_VERSION_MAJOR */

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

static void ESP32_Bluetooth_setup()
{

  BT_name += hw_info.model == SOFTRF_MODEL_SKYWATCH ?
                              SKYWATCH_IDENT : SOFTRF_IDENT;
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->s.bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str());
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

#if defined(CONFIG_IDF_TARGET_ESP32)
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif /* CONFIG_IDF_TARGET_ESP32 */

      // Create the BLE Device
#if defined(USE_NIMBLE)
      NimBLEDevice::init((BT_name+"-LE").c_str());
#else
      BLEDevice::init((BT_name+"-LE").c_str());
#endif
      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // BLEDevice::setMTU(23);

      // Create the BLE Server
#if defined(USE_NIMBLE)
      pServer = NimBLEDevice::createServer();
#else
      pServer = BLEDevice::createServer();
#endif
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID16);

#if defined(USE_NIMBLE)
      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic(
                          NimBLEUUID(CHARACTERISTIC_UUID16),
                          NIMBLE_PROPERTY::READ   |
                          NIMBLE_PROPERTY::NOTIFY |
                          NIMBLE_PROPERTY::WRITE_NR
                        );
#else
      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID16,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_NOTIFY |
                          BLECharacteristic::PROPERTY_WRITE_NR
                        );

      UserDescriptor.setValue("HMSoft");
      pCharacteristic->addDescriptor(&UserDescriptor);
      pCharacteristic->addDescriptor(new BLE2902());
#endif

      pCharacteristic->setCallbacks(new MyCallbacks());

      // Start the service
      pService->start();

      // Start advertising
#if defined(USE_NIMBLE) && defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
      BLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(SERVICE_UUID16);
      pAdvertising->enableScanResponse(true);
      pAdvertising->setPreferredParams(0x06, 0x12);
      NimBLEDevice::startAdvertising();
#else
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(SERVICE_UUID16);
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
      BLEDevice::startAdvertising();
#endif /* ESP_IDF_VERSION_MAJOR */

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
#if defined(ENABLE_BT_VOICE)
    void bt_app_main(void);

    bt_app_main();
#endif
    break;
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{
  switch (settings->s.bluetooth)
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

          pCharacteristic->setValue(chunk, size);
          pCharacteristic->notify();
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
    }
    break;
  case BLUETOOTH_NONE:
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

  switch (settings->s.bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.available();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch (settings->s.bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.read();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->read();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->s.bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.write(buffer, size);
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  case BLUETOOTH_NONE:
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

#else

#include <ArduinoBLE.h>

#if defined(ESP32)
#include <core_version.h>
#include <cbuf.h>
#define ARDUINO_CORE_VERSION    ARDUINO_ESP32_RELEASE
#else
#include <api/RingBuffer.h>
#endif /* ESP32 */

#include "EEPROMHelper.h"
#include "BluetoothHelper.h"
#include "WiFiHelper.h"
#include "BatteryHelper.h"

#define UART_SERVICE_UUID16                 "FFE0"
#define UART_CHARACTERISTIC_UUID16          "FFE1"

#define UUID16_SVC_BATTERY                  "180F"
#define UUID16_CHR_BATTERY_LEVEL            "2A19"

#define UUID16_SVC_DEVICE_INFORMATION       "180A"
#define UUID16_CHR_MODEL_NUMBER_STRING      "2A24"
#define UUID16_CHR_SERIAL_NUMBER_STRING     "2A25"
#define UUID16_CHR_FIRMWARE_REVISION_STRING "2A26"
#define UUID16_CHR_HARDWARE_REVISION_STRING "2A27"
#define UUID16_CHR_SOFTWARE_REVISION_STRING "2A28"
#define UUID16_CHR_MANUFACTURER_NAME_STRING "2A29"

bool deviceConnected    = false;
bool oldDeviceConnected = false;

#if defined(ESP32)
cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;
#else
RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();
#endif /* ESP32 */

String BT_name;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

BLEService UARTService(UART_SERVICE_UUID16);
//BLEService BATService (UUID16_SVC_BATTERY);

BLECharacteristic UARTCharacteristic(UART_CHARACTERISTIC_UUID16,
                                     BLERead | BLEWriteWithoutResponse | BLENotify,
                                     BLE_MAX_WRITE_CHUNK_SIZE);
BLEDescriptor UARTDescriptor("2901", "HMSoft");

//BLEUnsignedCharCharacteristic BATCharacteristic(UUID16_CHR_BATTERY_LEVEL,
//                                                BLERead | BLENotify);

BLEService DevInfoService(UUID16_SVC_DEVICE_INFORMATION);
BLEStringCharacteristic ModelCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic SerialCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic FirmwareCharacteristic(UUID16_CHR_FIRMWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic HardwareCharacteristic(UUID16_CHR_HARDWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic SoftwareCharacteristic(UUID16_CHR_SOFTWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic ManufacturerCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);

void onConnect(BLEDevice central) {
//  Serial.print("Connected event, central: ");
//  Serial.println(central.address());

  deviceConnected = true;
};

void onDisconnect(BLEDevice central) {
//  Serial.print("Disconnected event, central: ");
//  Serial.println(central.address());

  deviceConnected = false;
  BLE_Advertising_TimeMarker = millis();
}

void UARTCharacteristicWritten(BLEDevice         central,
                               BLECharacteristic characteristic) {
  size_t valueLength = characteristic.valueLength();

  if (valueLength > 0) {
    char buf[BLE_MAX_WRITE_CHUNK_SIZE];
    size_t len = sizeof(buf);

    characteristic.readValue(buf, len);

    if (valueLength > len) { valueLength = len; }
#if defined(ESP32)
    BLE_FIFO_RX->write(buf, (BLE_FIFO_RX->room() > valueLength ?
                             valueLength : BLE_FIFO_RX->room()));
#else
    size_t size = BLE_FIFO_RX.availableForStore();
    size = (size < valueLength) ? size : valueLength;
    for (size_t i = 0; i < size; i++) {
      BLE_FIFO_RX.store_char(buf[i]);
    }
#endif /* ESP32 */
  }
}

static void ArdBLE_Bluetooth_setup()
{
  BT_name += hw_info.model == SOFTRF_MODEL_SKYWATCH ?
                              SKYWATCH_IDENT : SOFTRF_IDENT;
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
#if defined(ESP32)
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);
#endif /* ESP32 */

      BLE.begin();

      char LocalName[BLE_MAX_WRITE_CHUNK_SIZE];
      snprintf(LocalName, sizeof(LocalName), "%s-LE", BT_name.c_str());
      BLE.setLocalName(LocalName);
      BLE.setDeviceName(LocalName);

      BLE.setAdvertisedService(UARTService);
      UARTCharacteristic.setEventHandler(BLEWritten, UARTCharacteristicWritten);
      UARTCharacteristic.addDescriptor(UARTDescriptor);
      UARTService.addCharacteristic(UARTCharacteristic);
      BLE.addService(UARTService);

//      BLE.setAdvertisedService(BATService);
//      BATService.addCharacteristic(BATCharacteristic);
//      BLE.addService(BATService);

      const char *Model = hw_info.model == SOFTRF_MODEL_WEBTOP_SERIAL ? "WebTop Serial"   :
                          hw_info.model == SOFTRF_MODEL_WEBTOP_USB    ? "WebTop USB"      :
                          hw_info.model == SOFTRF_MODEL_STANDALONE ? "Standalone Edition" :
                          hw_info.model == SOFTRF_MODEL_PRIME_MK2  ? "Prime Mark II"      :
                          hw_info.model == SOFTRF_MODEL_PRIME_MK3  ? "Prime Mark III"     :
                          hw_info.model == SOFTRF_MODEL_HAM        ? "Ham Edition"        :
                          hw_info.model == SOFTRF_MODEL_MIDI       ? "Midi Edition"       :
                          hw_info.model == SOFTRF_MODEL_ACADEMY    ? "Academy Edition"    :
                          hw_info.model == SOFTRF_MODEL_ECO        ? "Eco Edition"        :
                          hw_info.model == SOFTRF_MODEL_INK        ? "Ink Edition"        :
                          "Unknown";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      char Firmware[32];
      snprintf(Firmware, sizeof(Firmware), "Core %s %s", SoC->name, ARDUINO_CORE_VERSION);

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SOFTRF_IDENT;
      const char *Software      = SKYWATCH_FIRMWARE_VERSION;

      ModelCharacteristic.       writeValue((char *) Model);
      SerialCharacteristic.      writeValue((char *) SerialNum);
      FirmwareCharacteristic.    writeValue((char *) Firmware);
      HardwareCharacteristic.    writeValue((char *) Hardware);
      SoftwareCharacteristic.    writeValue((char *) Software);
      ManufacturerCharacteristic.writeValue((char *) Manufacturer);

      DevInfoService.addCharacteristic(ModelCharacteristic);
      DevInfoService.addCharacteristic(SerialCharacteristic);
      DevInfoService.addCharacteristic(FirmwareCharacteristic);
      DevInfoService.addCharacteristic(HardwareCharacteristic);
      DevInfoService.addCharacteristic(SoftwareCharacteristic);
      DevInfoService.addCharacteristic(ManufacturerCharacteristic);
      BLE.addService(DevInfoService);

      BLE.setEventHandler(BLEConnected,    onConnect);
      BLE.setEventHandler(BLEDisconnected, onDisconnect);

      BLE.advertise();

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  default:
    break;
  }
}

static void ArdBLE_Bluetooth_loop()
{
  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE.poll();

      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */
          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];

#if defined(ESP32)
          size_t size = BLE_FIFO_TX->available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);
            UARTCharacteristic.writeValue(chunk, size);
          }
#else
          size_t size = BLE_FIFO_TX.available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
            for (int i=0; i < size; i++) {
              chunk[i] = BLE_FIFO_TX.read_char();
            }
            UARTCharacteristic.writeValue(chunk, size);
          }
#endif /* ESP32 */

          BLE_Notify_TimeMarker = millis();
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready

          // BLE.advertise(); // restart advertising

          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
      }
//      if (deviceConnected && isTimeToBattery()) {
//        uint8_t battery_level = Battery_charge();
//
//        BATCharacteristic.writeValue(battery_level);
//      }
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void ArdBLE_Bluetooth_fini()
{
  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    BLE.end();
#if defined(ESP32)
    delete BLE_FIFO_TX;
    delete BLE_FIFO_RX;
#endif /* ESP32 */
    break;
  case BLUETOOTH_A2DP_SOURCE:
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  default:
    break;
  }
}

static int ArdBLE_Bluetooth_available()
{
  int rval = 0;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
#if defined(ESP32)
    rval = BLE_FIFO_RX->available();
#else
    rval = BLE_FIFO_RX.available();
#endif /* ESP32 */
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int ArdBLE_Bluetooth_read()
{
  int rval = -1;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
#if defined(ESP32)
    rval = BLE_FIFO_RX->read();
#else
    rval = BLE_FIFO_RX.read_char();
#endif /* ESP32 */
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t ArdBLE_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
#if defined(ESP32)
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
#else
    {
      size_t avail = BLE_FIFO_TX.availableForStore();
      if (size > avail) {
        rval = avail;
      }
      for (size_t i = 0; i < rval; i++) {
        BLE_FIFO_TX.store_char(buffer[i]);
      }
    }
#endif /* ESP32 */
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

IODev_ops_t ArdBLE_Bluetooth_ops = {
  "Arduino BLE",
  ArdBLE_Bluetooth_setup,
  ArdBLE_Bluetooth_loop,
  ArdBLE_Bluetooth_fini,
  ArdBLE_Bluetooth_available,
  ArdBLE_Bluetooth_read,
  ArdBLE_Bluetooth_write
};

#endif /* USE_ARDUINOBLE */

#elif defined(ARDUINO_ARCH_RP2040)
#include "SoCHelper.h"
#if !defined(EXCLUDE_BLUETOOTH)

#include <SerialBT.h>
#include <api/RingBuffer.h>

#include "EEPROMHelper.h"
#include "WiFiHelper.h"   // HOSTNAME
#include "BluetoothHelper.h"

RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();

String BT_name;

static void CYW43_Bluetooth_setup()
{
  BT_name += hw_info.model == SOFTRF_MODEL_SKYWATCH ?
                              SKYWATCH_IDENT : SOFTRF_IDENT;
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_SPP:
    {
      SerialBT.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
    }
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    /* TBD */
    break;
  case BLUETOOTH_A2DP_SOURCE:
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static void CYW43_Bluetooth_loop()
{
  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    /* TBD */
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void CYW43_Bluetooth_fini()
{
  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_SPP:
    {
      SerialBT.end();
    }
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    /* TBD */
    break;
  case BLUETOOTH_A2DP_SOURCE:
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static int CYW43_Bluetooth_available()
{
  int rval = 0;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.available();
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX.available();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int CYW43_Bluetooth_read()
{
  int rval = -1;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.read();
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX.read_char();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t CYW43_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->s.bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.write(buffer, size);
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      size_t avail = BLE_FIFO_TX.availableForStore();
      if (size > avail) {
        rval = avail;
      }
      for (size_t i = 0; i < rval; i++) {
        BLE_FIFO_TX.store_char(buffer[i]);
      }
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

IODev_ops_t CYW43_Bluetooth_ops = {
  "CYW43 Bluetooth",
  CYW43_Bluetooth_setup,
  CYW43_Bluetooth_loop,
  CYW43_Bluetooth_fini,
  CYW43_Bluetooth_available,
  CYW43_Bluetooth_read,
  CYW43_Bluetooth_write
};
#endif /* EXCLUDE_BLUETOOTH */
#endif /* ESP32 or ARDUINO_ARCH_RP2040 */

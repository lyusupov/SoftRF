/*
 * BluetoothHelper.cpp
 * Copyright (C) 2019-2025 Linar Yusupov
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

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "BluetoothHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(USE_NIMBLE)
#include <NimBLEDevice.h>
#else
#include <BLEDevice.h>
#endif /* USE_NIMBLE */

#include "WiFiHelper.h"   // HOSTNAME

String BT_name = HOSTNAME;

Bluetooth_ctl_t ESP32_BT_ctl = {
  .mutex   = portMUX_INITIALIZER_UNLOCKED,
  .command = BT_CMD_NONE,
  .status  = BT_STATUS_NC
};

/* LE */
#if defined(USE_NIMBLE)
static NimBLERemoteCharacteristic* pRemoteCharacteristic;
static NimBLEAdvertisedDevice*     AppDevice;
static NimBLEClient*               pClient;

static NimBLEUUID                  serviceUUID(SERVICE_UUID16);
static NimBLEUUID                  charUUID(CHARACTERISTIC_UUID16);
#else
static BLERemoteCharacteristic*    pRemoteCharacteristic;
static BLEAdvertisedDevice*        AppDevice;
static BLEClient*                  pClient;

static BLEUUID                     serviceUUID(SERVICE_UUID16);
static BLEUUID                     charUUID(CHARACTERISTIC_UUID16);
#endif /* USE_NIMBLE */

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

static unsigned long BT_TimeMarker = 0;
static unsigned long BLE_Notify_TimeMarker = 0;

#if defined(USE_NIMBLE)
static void AppNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length > 0) {
      BLE_FIFO_RX->write((char *) pData, (BLE_FIFO_RX->room() > length ?
                                          length : BLE_FIFO_RX->room()));
    }
}

class AppClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
  }

  void onDisconnect(NimBLEClient* pclient) {
    ESP32_BT_ctl.status = BT_STATUS_NC;

    Serial.println(F("BLE: disconnected from Server."));
  }
};

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
class AppAdvertisedDeviceCallbacks: public NimBLEScanCallbacks {
#else
class AppAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
#endif /* ESP_IDF_VERSION_MAJOR */

  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {

    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID)) {

      NimBLEDevice::getScan()->stop();

      if (AppDevice) {
        AppDevice->~NimBLEAdvertisedDevice();
      }

      AppDevice = advertisedDevice;
      ESP32_BT_ctl.command = BT_CMD_CONNECT;
    }
  }
};

#else /* USE_NIMBLE */

static void AppNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length > 0) {
      BLE_FIFO_RX->write((char *) pData, (BLE_FIFO_RX->room() > length ?
                                          length : BLE_FIFO_RX->room()));
    }
}

class AppClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    ESP32_BT_ctl.status = BT_STATUS_NC;

    Serial.println(F("BLE: disconnected from Server."));
  }
};

class AppAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

  void onResult(BLEAdvertisedDevice advertisedDevice) {

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();

      if (AppDevice) {
        AppDevice->~BLEAdvertisedDevice();
      }

      AppDevice = new BLEAdvertisedDevice(advertisedDevice);
      ESP32_BT_ctl.command = BT_CMD_CONNECT;
    }
  }
};
#endif /* USE_NIMBLE */

#if defined(CONFIG_IDF_TARGET_ESP32)
static void ESP32_BT_SPP_Connection_Manager(void *parameter)
{
  int command;
  int status;

  while (true) {

    delay(1000);

    if (strnlen(settings->server, sizeof(settings->server)) == 0) continue;

    portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
    command = ESP32_BT_ctl.command;
    portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

    switch (command)
    {
    case BT_CMD_CONNECT:
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        status = ESP32_BT_ctl.status;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

        if (status == BT_STATUS_CON) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          break;
        }

        // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
        // to resolve name to address first, but it allows to connect to different devices with the same name.
        // Set CoreDebugLevel to Info to view devices bluetooth address and device names

        if (SerialBT.connect(settings->server)) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.status = BT_STATUS_CON;
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

          Serial.print(F("BT SPP: Connected to "));
        } else {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.status = BT_STATUS_NC;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

          Serial.print(F("BT SPP: Unable to connect to "));
        }
        Serial.println(settings->server);
        break;

    case BT_CMD_DISCONNECT:
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        status = ESP32_BT_ctl.status;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

        if (status != BT_STATUS_CON) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          break;
        }

        // disconnect() may take upto 10 secs max
        if (SerialBT.disconnect()) {
          Serial.print(F("BT SPP: Disconnected from "));
        } else {
          Serial.print(F("BT SPP: Unable to disconnect from "));
        }
        Serial.println(settings->server);

        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        ESP32_BT_ctl.status = BT_STATUS_NC;
        ESP32_BT_ctl.command = BT_CMD_NONE;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
        break;

    case BT_CMD_SHUTDOWN:
        vTaskDelete(NULL);
        break;
    default:
        break;
    }
  }
}
#endif /* CONFIG_IDF_TARGET_ESP32 */

static bool ESP32_BLEConnectToServer() {
    pClient->connect(AppDevice);

#if defined(USE_NIMBLE)
    NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
#else
    BLERemoteService*    pRemoteService = pClient->getService(serviceUUID);
#endif /* USE_NIMBLE */

    if (pRemoteService == nullptr) {
      Serial.print(F("BLE: Failed to find our service UUID: "));
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print(F("BLE: Failed to find our characteristic UUID: "));
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    if(pRemoteCharacteristic->canNotify())
#if defined(USE_NIMBLE)
      pRemoteCharacteristic->subscribe(true, AppNotifyCallback);
#else
      pRemoteCharacteristic->registerForNotify(AppNotifyCallback);
#endif /* USE_NIMBLE */

    ESP32_BT_ctl.status = BT_STATUS_CON;
    return true;
}

static void ESP32_Bluetooth_setup()
{
  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      char id_06x[8];
      snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
      BT_name += String(id_06x);
#if !defined(ESP_IDF_VERSION_MAJOR) || ESP_IDF_VERSION_MAJOR < 5
      SerialBT.setPin(settings->key);
#elif !defined(CONFIG_BT_SSP_ENABLED)
      SerialBT.setPin(settings->key);
#endif /* ESP_IDF_VERSION_MAJOR */
      SerialBT.begin(BT_name.c_str(), true);

      xTaskCreate(ESP32_BT_SPP_Connection_Manager, "BT SPP ConMgr Task", 2048, NULL, tskIDLE_PRIORITY, NULL);

      BT_TimeMarker = millis();
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

#if defined(CONFIG_IDF_TARGET_ESP32)
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(USE_NIMBLE)
      NimBLEDevice::init("");
      pClient = NimBLEDevice::createClient();
      pClient->setClientCallbacks(new AppClientCallback());

      NimBLEScan* pBLEScan = NimBLEDevice::getScan();
#else
      BLEDevice::init("");
      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new AppClientCallback());

      BLEScan* pBLEScan = BLEDevice::getScan();
#endif /* USE_NIMBLE */

#if defined(USE_NIMBLE) && defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
      pBLEScan->setScanCallbacks(new NimBLEScanCallbacks());
#else
      pBLEScan->setAdvertisedDeviceCallbacks(new AppAdvertisedDeviceCallbacks());
#endif /* ESP_IDF_VERSION_MAJOR */
      pBLEScan->setInterval(1349);
      pBLEScan->setWindow(449);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(3, false);

      BLE_Notify_TimeMarker = millis();
    }
    break;
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{

  bool hasData = false;

  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int command = ESP32_BT_ctl.command;
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_NC && command == BT_CMD_NONE) {
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        ESP32_BT_ctl.command = BT_CMD_CONNECT;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
      } else {
        switch (settings->protocol)
        {
        case PROTOCOL_GDL90:
          hasData = GDL90_isConnected();
          break;
        case PROTOCOL_NMEA:
        default:
          hasData = NMEA_isConnected();
          break;
        }

        if (hasData) {
          BT_TimeMarker = millis();
        } else if (millis() - BT_TimeMarker > BT_NODATA_TIMEOUT &&
                   command == BT_CMD_NONE) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_DISCONNECT;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

          BT_TimeMarker = millis();
        }
      }
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
      if (ESP32_BT_ctl.command == BT_CMD_CONNECT) {
        if (ESP32_BLEConnectToServer()) {
          Serial.println(F("BLE: connected to Server."));
        }
        ESP32_BT_ctl.command = BT_CMD_NONE;
      }

      switch (settings->protocol)
      {
      case PROTOCOL_GDL90:
        hasData = GDL90_isConnected();
        break;
      case PROTOCOL_NMEA:
      default:
        hasData = NMEA_isConnected();
        break;
      }

      if (hasData) {
        BT_TimeMarker = millis();
      } else if (millis() - BT_TimeMarker > BT_NODATA_TIMEOUT) {

        Serial.println(F("BLE: attempt to (re)connect..."));

        if (pClient) {
          if (pClient->isConnected()) {
            pClient->disconnect();
          }
        }

#if defined(USE_NIMBLE)
        NimBLEDevice::getScan()->start(3, false);
#else
        BLEDevice::getScan()->start(3, false);
#endif /* USE_NIMBLE */

#if 0
        /* approx. 170 bytes memory leak still remains */
        Serial.print("Free Heap: ");
        Serial.println(ESP.getFreeHeap());
#endif

        BT_TimeMarker = millis();
      }

      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (ESP32_BT_ctl.status == BT_STATUS_CON &&
          (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = (BLE_FIFO_TX->available() < BLE_MAX_WRITE_CHUNK_SIZE ?
                         BLE_FIFO_TX->available() : BLE_MAX_WRITE_CHUNK_SIZE);

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);
#if defined(USE_NIMBLE) && defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
            pRemoteCharacteristic->writeValue(chunk, size, false);
#else
            pRemoteCharacteristic->writeValue(chunk, size);
#endif /* ESP_IDF_VERSION_MAJOR */
            BLE_Notify_TimeMarker = millis();
          }
      }
    }
    break;
  default:
    break;
  }
}

static void ESP32_Bluetooth_fini()
{
  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      ESP32_BT_ctl.command = BT_CMD_SHUTDOWN;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      delay(100);

      SerialBT.end();
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
#if defined(USE_NIMBLE)
      NimBLEDevice::deinit();
#else
      BLEDevice::deinit();
#endif /* USE_NIMBLE */
    }
    break;
  default:
    break;
  }
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_CON) {
        rval = SerialBT.available();
      }
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_RX->available();
    }
    break;
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_CON) {
        rval = SerialBT.read();
      }
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_RX->read();
      break;
    }
    break;
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  int rval = 0;

  switch(settings->connection)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case CON_BLUETOOTH_SPP:
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_CON) {
        rval = SerialBT.write(buffer, size);
      }
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_TX->write((char *) buffer,
                          (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    }
    break;
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

#elif defined(ARDUINO_ARCH_RP2040)
#include "SoCHelper.h"
#if !defined(EXCLUDE_BLUETOOTH)

#include <queue>
#include <pico/cyw43_arch.h>
#include <pico/btstack_cyw43.h>
#include <CoreMutex.h>
#include <btstack.h>

#include <api/RingBuffer.h>

#include "EEPROMHelper.h"
#include "WiFiHelper.h"   // HOSTNAME
#include "BluetoothHelper.h"

/* ------- SPP BEGIN ------ */

#define INQUIRY_INTERVAL 5
#define NUM_SERVICES     10

//#define SPP_USE_COD
#define SPP_USE_NAME

#if defined(SPP_USE_COD)
#define SOFTRF_SPP_COD 0x02c110
//#define SOFTRF_SPP_COD 0x001f00 /* HC-05 */
#endif /* SPP_USE_COD */

//#define DEBUG_SPP      Serial.printf
#define DEBUG_SPP

typedef enum {
#if defined(SPP_USE_COD)
    W4_PEER_COD,
#else
    W4_PEER_NAME,
#endif /* SPP_USE_COD */
    W4_SCAN_COMPLETE,
    W4_SDP_RESULT,
    W2_SEND_SDP_QUERY,
    W4_RFCOMM_CHANNEL,
    SENDING,
    DONE
} state_t;

static bool _running                 = false;
static bool _overflow                = false;
static volatile bool _connected      = false;
static size_t _fifoSize              = 512;
static uint16_t _channelID           = 0;
static volatile int _writeLen        = 0;
static uint8_t rfcomm_server_channel = 1;
static char *pin_code                = (char *) "0000";
static uint8_t service_index         = 0;

static mutex_t     _mutex;
static uint32_t    _writer;
static uint32_t    _reader;
static uint8_t    *_queue = NULL;
static uint16_t    _mtu;
static bd_addr_t   peer_addr;
static state_t     spp_state;
static const void *_writeBuff;

static btstack_packet_callback_registration_t  _hci_event_callback_registration;
static btstack_context_callback_registration_t _handle_sdp_client_query_request;

// prototypes
static void hci_spp_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();

String BT_name = HOSTNAME;

static struct {
    uint8_t channel_nr;
    char    service_name[SDP_SERVICE_NAME_LEN+1];
} services[NUM_SERVICES];

static void start_scan(void){
    DEBUG_SPP("Starting inquiry scan..\r\n");
#if defined(SPP_USE_COD)
    spp_state = W4_PEER_COD;
#else
    spp_state = W4_PEER_NAME,
#endif /* SPP_USE_COD */
    gap_inquiry_start(INQUIRY_INTERVAL);
}
static void stop_scan(void){
    DEBUG_SPP("Stopping inquiry scan..\r\n");
    spp_state = W4_SCAN_COMPLETE;
    gap_inquiry_stop();
}

#if defined(SPP_USE_NAME)
#define MAX_DEVICES      20
#define MAX_NAMELEN      18 /* matches EEPROMHelper.h value */

enum DEVICE_STATE { REMOTE_NAME_REQUEST, REMOTE_NAME_INQUIRED, REMOTE_NAME_FETCHED };
struct device {
    bd_addr_t          address;
    uint8_t            pageScanRepetitionMode;
    uint16_t           clockOffset;
    enum DEVICE_STATE  state;
    char               name[MAX_NAMELEN];
};

struct device devices[MAX_DEVICES];
int deviceCount = 0;

static void CYW43_Bluetooth_setup();
static void CYW43_Bluetooth_fini();

static int getDeviceIndexForAddress( bd_addr_t addr){
    int j;
    for (j=0; j< deviceCount; j++){
        if (bd_addr_cmp(addr, devices[j].address) == 0){
            return j;
        }
    }
    return -1;
}

static int has_more_remote_name_requests(void){
    int i;
    for (i=0;i<deviceCount;i++) {
        if (devices[i].state == REMOTE_NAME_REQUEST) return 1;
    }
    return 0;
}

static void do_next_remote_name_request(void){
    int i;
    for (i=0;i<deviceCount;i++) {
        // remote name request
        if (devices[i].state == REMOTE_NAME_REQUEST){
            devices[i].state = REMOTE_NAME_INQUIRED;
            DEBUG_SPP("Get remote name of %s...\r\n", bd_addr_to_str(devices[i].address));
            gap_remote_name_request( devices[i].address, devices[i].pageScanRepetitionMode,  devices[i].clockOffset | 0x8000);
            return;
        }
    }
}

static void continue_remote_names(void){
    if (has_more_remote_name_requests()){
        do_next_remote_name_request();
        return;
    }
    start_scan();
}
#endif /* SPP_USE_NAME */

static void store_found_service(const char * name, uint8_t port){
    DEBUG_SPP("APP: Service name: '%s', RFCOMM port %u\r\n", name, port);
    if (service_index < NUM_SERVICES){
        services[service_index].channel_nr = port;
        btstack_strcpy(services[service_index].service_name, SDP_SERVICE_NAME_LEN + 1, name);
        service_index++;
    } else {
        DEBUG_SPP("APP: list full - ignore\r\n");
        return;
    }
}

static void report_found_services(void){
    DEBUG_SPP("Client query response done. ");
    if (service_index == 0) {
        DEBUG_SPP("No service found.\r\n");
    } else {
        DEBUG_SPP("Found following %d services:\r\n", service_index);
    }
    int i;
    for (i=0; i<service_index; i++) {
        DEBUG_SPP("     Service name %s, RFCOMM port %u\r\n", services[i].service_name, services[i].channel_nr);
    }
}

/*
 * @section SDP Query Packet Handler
 *
 * @text Store RFCOMM Channel for SPP service and initiates RFCOMM connection
 */
static void handle_query_rfcomm_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    switch (hci_event_packet_get_type(packet)){
        case SDP_EVENT_QUERY_RFCOMM_SERVICE:
            store_found_service(sdp_event_query_rfcomm_service_get_name(packet),
                                sdp_event_query_rfcomm_service_get_rfcomm_channel(packet));
            rfcomm_server_channel = sdp_event_query_rfcomm_service_get_rfcomm_channel(packet);
            break;
        case SDP_EVENT_QUERY_COMPLETE:
            if (sdp_event_query_complete_get_status(packet)){
                DEBUG_SPP("SDP query failed 0x%02x\r\n", sdp_event_query_complete_get_status(packet));
                break;
            }

            report_found_services();

            if (rfcomm_server_channel == 0) {
              DEBUG_SPP("No SPP service found\r\n");
              break;
            }
            DEBUG_SPP("SDP query done, channel %u.\r\n", rfcomm_server_channel);
            rfcomm_create_channel(hci_spp_event_handler, peer_addr, rfcomm_server_channel, NULL);
            break;
        default:
            break;
    }
}

static void handle_start_sdp_client_query(void * context){
    UNUSED(context);
    if (spp_state != W2_SEND_SDP_QUERY) return;
    spp_state = W4_RFCOMM_CHANNEL;
    sdp_client_query_rfcomm_channel_and_name_for_uuid(&handle_query_rfcomm_event, peer_addr, BLUETOOTH_ATTRIBUTE_PUBLIC_BROWSE_ROOT);
}

static void hci_spp_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint32_t class_of_device;
    int i;
#if defined(SPP_USE_NAME)
    int index;
#endif /* SPP_USE_NAME */

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                    start_scan();
                    break;

#if defined(SPP_USE_COD)
                case GAP_EVENT_INQUIRY_RESULT:
                    if (spp_state != W4_PEER_COD) break;
                    class_of_device = gap_event_inquiry_result_get_class_of_device(packet);
                    gap_event_inquiry_result_get_bd_addr(packet, event_addr);
                    if (class_of_device == SOFTRF_SPP_COD){
                        memcpy(peer_addr, event_addr, 6);
                        DEBUG_SPP("Peer found: %s\r\n", bd_addr_to_str(peer_addr));
                        stop_scan();
                    } else {
                        DEBUG_SPP("Device found: %s with COD: 0x%06x\r\n", bd_addr_to_str(event_addr), (int) class_of_device);
                    }
                    break;
#endif /* SPP_USE_COD */

#if defined(SPP_USE_NAME)
                case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE:
                    reverse_bd_addr(&packet[3], event_addr);
                    index = getDeviceIndexForAddress(event_addr);
                    if (index >= 0) {
                        if (packet[2] == 0) {
                            // fix for invalid remote names - terminate on 0xff
                            for (i=0; i<248;i++){
                                if (packet[9+i] == 0xff){
                                    packet[9+i] = 0;
                                    break;
                                }
                            }
                            packet[9+248] = 0;
                            DEBUG_SPP("Name: '%s'\r\n", &packet[9]);
                            memcpy(devices[index].name, &packet[9], MAX_NAMELEN);
                            devices[index].state = REMOTE_NAME_FETCHED;
                        } else {
                            DEBUG_SPP("Failed to get name: page timeout\r\n");
                        }
                    }
                    continue_remote_names();
                    break;

                case GAP_EVENT_INQUIRY_RESULT:
                    if (spp_state != W4_PEER_NAME) break;
                    if (deviceCount >= MAX_DEVICES) break;  // already full
                    gap_event_inquiry_result_get_bd_addr(packet, event_addr);
                    index = getDeviceIndexForAddress(event_addr);

                    // already in our list
                    if (index >= 0) {
                      size_t name_len = strnlen(settings->server, sizeof(settings->server));
                      if (name_len > 0 && devices[index].state == REMOTE_NAME_FETCHED &&
                          memcmp(settings->server, devices[index].name, name_len) == 0) {
                        memcpy(peer_addr, event_addr, 6);
                        DEBUG_SPP("Peer found: %s\r\n", bd_addr_to_str(peer_addr));
                        stop_scan();
                      }
                      break;
                    }

                    memcpy(devices[deviceCount].address, event_addr, 6);
                    devices[deviceCount].pageScanRepetitionMode = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
                    devices[deviceCount].clockOffset = gap_event_inquiry_result_get_clock_offset(packet);
                    // print info
                    DEBUG_SPP("Device found: %s ",  bd_addr_to_str(event_addr));
                    DEBUG_SPP("with COD: 0x%06x, ", (unsigned int) gap_event_inquiry_result_get_class_of_device(packet));
                    DEBUG_SPP("pageScan %d, ",      devices[deviceCount].pageScanRepetitionMode);
                    DEBUG_SPP("clock offset 0x%04x",devices[deviceCount].clockOffset);
                    if (gap_event_inquiry_result_get_rssi_available(packet)){
                        DEBUG_SPP(", rssi %d dBm", (int8_t) gap_event_inquiry_result_get_rssi(packet));
                    }
                    if (gap_event_inquiry_result_get_name_available(packet)){
                        char name_buffer[240];
                        int name_len = gap_event_inquiry_result_get_name_len(packet);
                        memcpy(name_buffer, gap_event_inquiry_result_get_name(packet), name_len);
                        name_buffer[name_len] = 0;
                        DEBUG_SPP(", name '%s'", name_buffer);
                        size_t srv_name_len = strnlen(settings->server, sizeof(settings->server));
                        if (srv_name_len > 0 &&
                            memcmp(settings->server, name_buffer, srv_name_len) == 0) {
                          memcpy(peer_addr, event_addr, 6);
                          DEBUG_SPP("Peer found: %s\r\n", bd_addr_to_str(peer_addr));
                          stop_scan();
                        }
                        devices[deviceCount].state = REMOTE_NAME_FETCHED;
                    } else {
                        devices[deviceCount].state = REMOTE_NAME_REQUEST;
                    }
                    DEBUG_SPP("\r\n");
                    deviceCount++;
                    break;
#endif /* SPP_USE_NAME */

                case GAP_EVENT_INQUIRY_COMPLETE:
                    switch (spp_state){
#if defined(SPP_USE_COD)
                        case W4_PEER_COD:
                            DEBUG_SPP("Inquiry complete\r\n");
                            DEBUG_SPP("Peer not found, starting scan again\r\n");
                            start_scan();
                            break;
#endif /* SPP_USE_COD */

#if defined(SPP_USE_NAME)
                        case W4_PEER_NAME:
                            for (i=0;i<deviceCount;i++) {
                                // retry remote name request
                                if (devices[i].state == REMOTE_NAME_INQUIRED)
                                    devices[i].state = REMOTE_NAME_REQUEST;
                            }
                            continue_remote_names();
                            break;
#endif /* SPP_USE_NAME */
                        case W4_SCAN_COMPLETE:
                            DEBUG_SPP("Start to connect and query for SPP service\r\n");
                            spp_state = W2_SEND_SDP_QUERY;
                            _handle_sdp_client_query_request.callback = &handle_start_sdp_client_query;
                            (void) sdp_client_register_query_callback(&_handle_sdp_client_query_request);
                            break;
                        default:
                            break;
                    }
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    if (strnlen(settings->key, sizeof(settings->key)) > 0) {
                      pin_code = settings->key;
                    }
                    DEBUG_SPP("Pin code request - using '%s'\r\n", pin_code);
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, pin_code);
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // inform about user confirmation request
                    DEBUG_SPP("SSP User Confirmation Request with numeric value '%06"PRIu32"'\r\n", little_endian_read_32(packet, 8));
                    DEBUG_SPP("SSP User Confirmation Auto accept\r\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    _channelID = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    DEBUG_SPP("RFCOMM channel %u requested for %s\r\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(_channelID);
                    break;

                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        DEBUG_SPP("RFCOMM channel open failed, status %u\r\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        _channelID = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        _mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        DEBUG_SPP("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\r\n", _channelID, _mtu);
                        _connected = true;

                        // disable page/inquiry scan to get max performance
                        gap_discoverable_control(0);
                        gap_connectable_control(0);
                    }
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    rfcomm_send(_channelID, (uint8_t *)_writeBuff, _writeLen);
                    _writeLen = 0;
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    DEBUG_SPP("RFCOMM channel closed\r\n");
                    _channelID = 0;
                    _connected = false;

#if defined(SPP_USE_COD)
                    spp_state = W4_PEER_COD;
#else
                    spp_state = W4_PEER_NAME,
                    deviceCount = 0;
#endif /* SPP_USE_COD */

                    service_index = 0;
                    rfcomm_server_channel = 1;

                    CYW43_Bluetooth_fini();
                    //rfcomm_deinit();

                    btstack_cyw43_init(cyw43_arch_async_context());
                    CYW43_Bluetooth_setup();
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            for (i = 0; i < size; i++) {
                auto next_writer = _writer + 1;
                if (next_writer == _fifoSize) {
                    next_writer = 0;
                }
                if (next_writer != _reader) {
                    _queue[_writer] = packet[i];
                    asm volatile("" ::: "memory"); // Ensure the queue is written before the written count advances
                    // Avoid using division or mod because the HW divider could be in use
                    _writer = next_writer;
                } else {
                    _overflow = true;
                }
            }
            break;

        default:
            break;
    }
}
/* ------- SPP END ------ */

/* ------- BLE BEGIN------ */

#define TEST_MODE_WRITE_WITHOUT_RESPONSE 1
#define TEST_MODE_ENABLE_NOTIFICATIONS   2
#define TEST_MODE_DUPLEX                 3

// configure test mode: send only, receive only, full duplex
#define TEST_MODE TEST_MODE_ENABLE_NOTIFICATIONS

#define REPORT_INTERVAL_MS 3000

#define DEBUG_BLE          0

// prototypes
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

typedef enum {
    TC_OFF,
    TC_IDLE,
    TC_W4_SCAN_RESULT,
    TC_W4_CONNECT,
    TC_W4_SERVICE_RESULT,
    TC_W4_CHARACTERISTIC_RXTX_RESULT,
    TC_W4_ENABLE_NOTIFICATIONS_COMPLETE,
    TC_W4_TEST_DATA
} gc_state_t;

// addr and type of device with correct name
static bd_addr_t      le_streamer_addr;
static bd_addr_type_t le_streamer_addr_type;

static hci_con_handle_t connection_handle;

static uint8_t le_streamer_service_uuid128[16]             = { 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
static uint8_t le_streamer_characteristic_rxtx_uuid128[16] = { 0x00, 0x00, 0xFF, 0xE1, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
static uint16_t le_streamer_service_uuid16             = 0xFFE0;
static uint16_t le_streamer_characteristic_rxtx_uuid16 = 0xFFE1;

static gatt_client_service_t le_streamer_service;
static gatt_client_characteristic_t le_streamer_characteristic_rxtx;

static gatt_client_notification_t notification_listener;
static int listener_registered;

static gc_state_t le_state = TC_OFF;

// support for multiple clients
typedef struct {
    char name;
    int le_notification_enabled;
    int  counter;
    char test_data[200];
    int  test_data_len;
    uint32_t test_data_sent;
    uint32_t test_data_start;
} le_streamer_connection_t;

static le_streamer_connection_t le_streamer_connection;

static void test_reset(le_streamer_connection_t * context){
    context->test_data_start = btstack_run_loop_get_time_ms();
    context->test_data_sent = 0;
}

static void test_track_data(le_streamer_connection_t * context, int bytes_sent){
    context->test_data_sent += bytes_sent;
    // evaluate
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t time_passed = now - context->test_data_start;
    if (time_passed < REPORT_INTERVAL_MS) return;
    // print speed
    int bytes_per_second = context->test_data_sent * 1000 / time_passed;
#if DEBUG_BLE
    Serial.printf("%c: %"PRIu32" bytes -> %u.%03u kB/s\r\n", context->name, context->test_data_sent, bytes_per_second / 1000, bytes_per_second % 1000);
#endif /* DEBUG_BLE */

    // restart
    context->test_data_start = now;
    context->test_data_sent  = 0;
}

// stramer
static void streamer(le_streamer_connection_t * context){
    if (connection_handle == HCI_CON_HANDLE_INVALID) return;

    // create test data
    context->counter++;
    if (context->counter > 'Z') context->counter = 'A';
    memset(context->test_data, context->counter, context->test_data_len);

    // send
    uint8_t status = gatt_client_write_value_of_characteristic_without_response(connection_handle, le_streamer_characteristic_rxtx.value_handle, context->test_data_len, (uint8_t*) context->test_data);
    if (status){
#if DEBUG_BLE
        Serial.printf("error %02x for write without response!\r\n", status);
#endif /* DEBUG_BLE */
        return;
    } else {
        test_track_data(&le_streamer_connection, context->test_data_len);
    }

    // request again
    gatt_client_request_can_write_without_response_event(handle_gatt_client_event, connection_handle);
}


// returns 1 if name is found in advertisement
static int advertisement_report_contains_name(const char * name, uint8_t * advertisement_report){
    // get advertisement from report event
    const uint8_t * adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint8_t         adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);
    uint16_t        name_len = (uint8_t) strlen(name);

    // iterate over advertisement data
    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t data_size    = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                // compare prefix
                if (data_size < name_len) break;
                if (memcmp(data, name, name_len) == 0) return 1;
                return 0;
            default:
                break;
        }
    }
    return 0;
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint16_t mtu;
    uint8_t att_status;
    switch(le_state){
        case TC_W4_SERVICE_RESULT:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_SERVICE_QUERY_RESULT:
                    // store service (we expect only one)
                    gatt_event_service_query_result_get_service(packet, &le_streamer_service);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS){
#if DEBUG_BLE
                        Serial.printf("SERVICE_QUERY_RESULT - Error status %x.\r\n", att_status);
#endif /* DEBUG_BLE */
                        gap_disconnect(connection_handle);
                        break;
                    }
                    // service query complete, look for characteristic
                    le_state = TC_W4_CHARACTERISTIC_RXTX_RESULT;
#if DEBUG_BLE
                    Serial.printf("Search for HM-10 UART RX/TX characteristic.\r\n");
#endif /* DEBUG_BLE */
//                    gatt_client_discover_characteristics_for_service_by_uuid128(handle_gatt_client_event, connection_handle, &le_streamer_service, le_streamer_characteristic_rxtx_uuid128);
                    gatt_client_discover_characteristics_for_service_by_uuid16(handle_gatt_client_event, connection_handle, &le_streamer_service, le_streamer_characteristic_rxtx_uuid16);
                    break;
                default:
                    break;
            }
            break;

        case TC_W4_CHARACTERISTIC_RXTX_RESULT:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
                    gatt_event_characteristic_query_result_get_characteristic(packet, &le_streamer_characteristic_rxtx);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS){
#if DEBUG_BLE
                        Serial.printf("CHARACTERISTIC_QUERY_RESULT - Error status %x.\r\n", att_status);
#endif /* DEBUG_BLE */
                        gap_disconnect(connection_handle);
                        break;
                    }
                    // register handler for notifications
                    listener_registered = 1;
                    gatt_client_listen_for_characteristic_value_updates(&notification_listener, handle_gatt_client_event, connection_handle, &le_streamer_characteristic_rxtx);
                    // setup tracking
                    le_streamer_connection.name = 'A';
                    le_streamer_connection.test_data_len = ATT_DEFAULT_MTU - 3;
                    test_reset(&le_streamer_connection);
                    gatt_client_get_mtu(connection_handle, &mtu);
                    le_streamer_connection.test_data_len = btstack_min(mtu - 3, sizeof(le_streamer_connection.test_data));
#if DEBUG_BLE
                    Serial.printf("%c: ATT MTU = %u => use test data of len %u\r\n", le_streamer_connection.name, mtu, le_streamer_connection.test_data_len);
#endif /* DEBUG_BLE */
                    // enable notifications
#if (TEST_MODE & TEST_MODE_ENABLE_NOTIFICATIONS)
#if DEBUG_BLE
                    Serial.printf("Start streaming - enable notify on test characteristic.\r\n");
#endif /* DEBUG_BLE */
                    le_state = TC_W4_ENABLE_NOTIFICATIONS_COMPLETE;
                    gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connection_handle,
                        &le_streamer_characteristic_rxtx, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    break;
#endif
                    le_state = TC_W4_TEST_DATA;
#if (TEST_MODE & TEST_MODE_WRITE_WITHOUT_RESPONSE)
#if DEBUG_BLE
                    Serial.printf("Start streaming - request can send now.\r\n");
#endif /* DEBUG_BLE */
                    gatt_client_request_can_write_without_response_event(handle_gatt_client_event, connection_handle);
#endif
                    break;
                default:
                    break;
            }
            break;

        case TC_W4_ENABLE_NOTIFICATIONS_COMPLETE:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_QUERY_COMPLETE:
#if DEBUG_BLE
                    Serial.printf("Notifications enabled, ATT status %02x\r\n", gatt_event_query_complete_get_att_status(packet));
#endif /* DEBUG_BLE */
                    if (gatt_event_query_complete_get_att_status(packet) != ATT_ERROR_SUCCESS) break;
                    le_state = TC_W4_TEST_DATA;
#if (TEST_MODE & TEST_MODE_WRITE_WITHOUT_RESPONSE)
#if DEBUG_BLE
                    Serial.printf("Start streaming - request can send now.\r\n");
#endif /* DEBUG_BLE */
                    gatt_client_request_can_write_without_response_event(handle_gatt_client_event, connection_handle);
#endif
                    break;
                default:
                    break;
            }
            break;

        case TC_W4_TEST_DATA:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_NOTIFICATION:
                    {
                      size_t payload_len = gatt_event_notification_get_value_length(packet);
                      if (payload_len > 0) {
                        size_t size = BLE_FIFO_RX.availableForStore();
                        size = (size < payload_len) ? size : payload_len;
                        for (size_t i = 0; i < size; i++) {
                          BLE_FIFO_RX.store_char(gatt_event_notification_get_value(packet)[i]);
                        }
                      }
                      test_track_data(&le_streamer_connection, payload_len);
                    }
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    break;
                case GATT_EVENT_CAN_WRITE_WITHOUT_RESPONSE:
                    streamer(&le_streamer_connection);
                    break;
                default:
#if DEBUG_BLE
                    Serial.printf("Unknown packet type %x\r\n", hci_event_packet_get_type(packet));
#endif /* DEBUG_BLE */
                    break;
            }
            break;

        default:
#if DEBUG_BLE
            Serial.printf("error\r\n");
#endif /* DEBUG_BLE */
            break;
    }
}

static void le_streamer_client_start(void){
#if DEBUG_BLE
    Serial.printf("Start scanning!\r\n");
#endif /* DEBUG_BLE */
    le_state = TC_W4_SCAN_RESULT;
    gap_set_scan_parameters(1, 0x0030, 0x0030);
    gap_start_scan();
}

#if DEBUG_BLE
static const char * ad_types[] = {
    "",
    "Flags",
    "Incomplete List of 16-bit Service Class UUIDs",
    "Complete List of 16-bit Service Class UUIDs",
    "Incomplete List of 32-bit Service Class UUIDs",
    "Complete List of 32-bit Service Class UUIDs",
    "Incomplete List of 128-bit Service Class UUIDs",
    "Complete List of 128-bit Service Class UUIDs",
    "Shortened Local Name",
    "Complete Local Name",
    "Tx Power Level",
    "",
    "",
    "Class of Device",
    "Simple Pairing Hash C",
    "Simple Pairing Randomizer R",
    "Device ID",
    "Security Manager TK Value",
    "Slave Connection Interval Range",
    "",
    "List of 16-bit Service Solicitation UUIDs",
    "List of 128-bit Service Solicitation UUIDs",
    "Service Data",
    "Public Target Address",
    "Random Target Address",
    "Appearance",
    "Advertising Interval"
};

static const char * flags[] = {
    "LE Limited Discoverable Mode",
    "LE General Discoverable Mode",
    "BR/EDR Not Supported",
    "Simultaneous LE and BR/EDR to Same Device Capable (Controller)",
    "Simultaneous LE and BR/EDR to Same Device Capable (Host)",
    "Reserved",
    "Reserved",
    "Reserved"
};

static void dump_advertisement_data(const uint8_t * adv_data, uint8_t adv_size){
    ad_context_t context;
    bd_addr_t address;
    uint8_t uuid_128[16];
    for (ad_iterator_init(&context, adv_size, (uint8_t *)adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t size         = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);

        if (data_type > 0 && data_type < 0x1B){
            Serial.printf("    %s: ", ad_types[data_type]);
        }
        int i;
        // Assigned Numbers GAP

        switch (data_type){
            case BLUETOOTH_DATA_TYPE_FLAGS:
                // show only first octet, ignore rest
                for (i=0; i<8;i++){
                    if (data[0] & (1<<i)){
                        Serial.printf("%s; ", flags[i]);
                    }
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_16_BIT_SERVICE_SOLICITATION_UUIDS:
                for (i=0; i<size;i+=2){
                    Serial.printf("%02X ", little_endian_read_16(data, i));
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_32_BIT_SERVICE_SOLICITATION_UUIDS:
                for (i=0; i<size;i+=4){
                    Serial.printf("%04"PRIX32, little_endian_read_32(data, i));
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_128_BIT_SERVICE_SOLICITATION_UUIDS:
                reverse_128(data, uuid_128);
                Serial.printf("%s", uuid128_to_str(uuid_128));
                break;
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                for (i=0; i<size;i++){
                    Serial.printf("%c", (char)(data[i]));
                }
                break;
            case BLUETOOTH_DATA_TYPE_TX_POWER_LEVEL:
                Serial.printf("%d dBm", *(int8_t*)data);
                break;
            case BLUETOOTH_DATA_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE:
                Serial.printf("Connection Interval Min = %u ms, Max = %u ms", little_endian_read_16(data, 0) * 5/4, little_endian_read_16(data, 2) * 5/4);
                break;
            case BLUETOOTH_DATA_TYPE_SERVICE_DATA:
//                printf_hexdump(data, size);
                break;
            case BLUETOOTH_DATA_TYPE_PUBLIC_TARGET_ADDRESS:
            case BLUETOOTH_DATA_TYPE_RANDOM_TARGET_ADDRESS:
                reverse_bd_addr(data, address);
                Serial.printf("%s", bd_addr_to_str(address));
                break;
            case BLUETOOTH_DATA_TYPE_APPEARANCE:
                // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
                Serial.printf("%02X", little_endian_read_16(data, 0) );
                break;
            case BLUETOOTH_DATA_TYPE_ADVERTISING_INTERVAL:
                Serial.printf("%u ms", little_endian_read_16(data, 0) * 5/8 );
                break;
            case BLUETOOTH_DATA_TYPE_3D_INFORMATION_DATA:
//                printf_hexdump(data, size);
                break;
            case BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA: // Manufacturer Specific Data
                break;
            case BLUETOOTH_DATA_TYPE_CLASS_OF_DEVICE:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_HASH_C:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_RANDOMIZER_R:
            case BLUETOOTH_DATA_TYPE_DEVICE_ID:
            case BLUETOOTH_DATA_TYPE_SECURITY_MANAGER_OUT_OF_BAND_FLAGS:
            default:
                Serial.printf("Advertising Data Type 0x%2x not handled yet", data_type);
                break;
        }
        Serial.printf("\r\n");
    }
    Serial.printf("\r\n");
}

#endif /* DEBUG_BLE */

static void hci_le_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint16_t conn_interval;
    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                le_streamer_client_start();
            } else {
                le_state = TC_OFF;
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (le_state != TC_W4_SCAN_RESULT) return;
#if DEBUG_BLE
            {
              bd_addr_t address;
              gap_event_advertising_report_get_address(packet, address);
              uint8_t length = gap_event_advertising_report_get_data_length(packet);
              const uint8_t * data = gap_event_advertising_report_get_data(packet);
              dump_advertisement_data(data, length);
            }
#endif /* DEBUG_BLE */
            // check name in advertisement
            if (strnlen(settings->server, sizeof(settings->server)) == 0) return;
            if (!advertisement_report_contains_name(settings->server, packet)) return;
            // store address and type
            gap_event_advertising_report_get_address(packet, le_streamer_addr);
            le_streamer_addr_type = (bd_addr_type_t) gap_event_advertising_report_get_address_type(packet);
            // stop scanning, and connect to the device
            le_state = TC_W4_CONNECT;
            gap_stop_scan();
#if DEBUG_BLE
            Serial.printf("Stop scan. Connect to %s with addr %s.\r\n", settings->server, bd_addr_to_str(le_streamer_addr));
#endif /* DEBUG_BLE */
            gap_connect(le_streamer_addr,le_streamer_addr_type);
            break;
        case HCI_EVENT_LE_META:
            // wait for connection complete
            if (hci_event_le_meta_get_subevent_code(packet) !=  HCI_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            if (le_state != TC_W4_CONNECT) return;
            connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            // print connection parameters (without using float operations)
            conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
#if DEBUG_BLE
            Serial.printf("Connection Interval: %u.%02u ms\r\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
            Serial.printf("Connection Latency: %u\r\n", hci_subevent_le_connection_complete_get_conn_latency(packet));
            // initialize gatt client context with handle, and add it to the list of active clients
            // query primary services
            Serial.printf("Search for HM-10 UART service.\r\n");
#endif /* DEBUG_BLE */
            le_state = TC_W4_SERVICE_RESULT;
//            gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, connection_handle, le_streamer_service_uuid128);
            gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connection_handle, le_streamer_service_uuid16);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            // unregister listener
            connection_handle = HCI_CON_HANDLE_INVALID;
            if (listener_registered){
                listener_registered = 0;
                gatt_client_stop_listening_for_characteristic_value_updates(&notification_listener);
            }
#if DEBUG_BLE
            Serial.printf("Disconnected %s\r\n", bd_addr_to_str(le_streamer_addr));
#endif /* DEBUG_BLE */
            if (le_state == TC_OFF) break;
            le_streamer_client_start();
            break;
        default:
            break;
    }
}

/* ------- BLE END ------ */

static void lockBluetooth() {
    async_context_acquire_lock_blocking(cyw43_arch_async_context());
}

static void unlockBluetooth() {
    async_context_release_lock(cyw43_arch_async_context());
}

static void CYW43_Bluetooth_setup()
{
  if (_running) return;

  char id_06x[8];
  snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
  BT_name += "-";
  BT_name += String(id_06x);

  switch (settings->connection)
  {
  case CON_BLUETOOTH_SPP:
    {
      mutex_init(&_mutex);
      _overflow = false;

      _queue = new uint8_t[_fifoSize];
      _writer = 0;
      _reader = 0;

      // register for HCI events
      _hci_event_callback_registration.callback = &hci_spp_event_handler;
      hci_add_event_handler(&_hci_event_callback_registration);

      l2cap_init();

      sm_init();

      rfcomm_init();

      // init SDP
      gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);

      hci_power_control(HCI_POWER_ON);

      _running = true;
    }
    break;
  case CON_BLUETOOTH_LE:
    {
      BT_name += "-LE";

      mutex_init(&_mutex);

      l2cap_init();

      sm_init();
      sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

      // sm_init needed before gatt_client_init
      gatt_client_init();

      _hci_event_callback_registration.callback = &hci_le_event_handler;
      hci_add_event_handler(&_hci_event_callback_registration);

      hci_power_control(HCI_POWER_ON);

      _running = true;
    }
    break;
  default:
    break;
  }
}

static void CYW43_Bluetooth_loop()
{
  switch (settings->connection)
  {
  case CON_BLUETOOTH_LE:
    /* TBD */
    break;
  case CON_BLUETOOTH_SPP:
  default:
    break;
  }
}

static void CYW43_Bluetooth_fini()
{
  switch (settings->connection)
  {
  case CON_BLUETOOTH_SPP:
  case CON_BLUETOOTH_LE:
    {
      if (!_running) {
          return;
      }
      _running = false;

      sm_deinit();
      l2cap_deinit();

      btstack_cyw43_deinit(cyw43_arch_async_context());

      lockBluetooth();
      if (_queue != NULL) delete[] _queue;
      unlockBluetooth();
    }
    break;
  default:
    break;
  }
}

static int CYW43_Bluetooth_available()
{
  int rval = 0;

  switch (settings->connection)
  {
  case CON_BLUETOOTH_SPP:
    {
      CoreMutex m(&_mutex);
      if (_running && m) {
        rval = (_fifoSize + _writer - _reader) % _fifoSize;
      }
    }
    break;
  case CON_BLUETOOTH_LE:
    rval = BLE_FIFO_RX.available();
    break;
  default:
    break;
  }

  return rval;
}

static int CYW43_Bluetooth_read()
{
  int rval = -1;

  switch (settings->connection)
  {
  case CON_BLUETOOTH_SPP:
    {
      CoreMutex m(&_mutex);
      if (_running && m && _writer != _reader) {
          auto ret = _queue[_reader];
          asm volatile("" ::: "memory"); // Ensure the value is read before advancing
          auto next_reader = (_reader + 1) % _fifoSize;
          asm volatile("" ::: "memory"); // Ensure the reader value is only written once, correctly
          _reader = next_reader;
          rval = ret;
      }
    }
    break;
  case CON_BLUETOOTH_LE:
    rval = BLE_FIFO_RX.read_char();
    break;
  default:
    break;
  }

  return rval;
}

static size_t CYW43_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->connection)
  {
  case CON_BLUETOOTH_SPP:
    {
      CoreMutex m(&_mutex);
      if (!_running || !m || !_connected || !size)  {
          return 0;
      }
      _writeBuff = buffer;
      _writeLen = size;
      lockBluetooth();
      rfcomm_request_can_send_now_event(_channelID);
      unlockBluetooth();
      while (_connected && _writeLen) {
          /* noop busy wait */
      }
    }
    break;
  case CON_BLUETOOTH_LE:
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

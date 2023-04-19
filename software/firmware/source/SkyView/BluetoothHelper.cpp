/*
 * BluetoothHelper.cpp
 * Copyright (C) 2019-2023 Linar Yusupov
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

#include <BLEDevice.h>

#include "WiFiHelper.h"   // HOSTNAME

String BT_name = HOSTNAME;

Bluetooth_ctl_t ESP32_BT_ctl = {
  .mutex   = portMUX_INITIALIZER_UNLOCKED,
  .command = BT_CMD_NONE,
  .status  = BT_STATUS_NC
};

/* LE */
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* AppDevice;
static BLEClient* pClient;

static BLEUUID  serviceUUID(SERVICE_UUID);
static BLEUUID  charUUID(CHARACTERISTIC_UUID);

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

static unsigned long BT_TimeMarker = 0;
static unsigned long BLE_Notify_TimeMarker = 0;

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

#if defined(CONFIG_IDF_TARGET_ESP32)
static void ESP32_BT_SPP_Connection_Manager(void *parameter)
{
  int command;
  int status;

  while (true) {

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

    delay(1000);
  }
}
#endif /* CONFIG_IDF_TARGET_ESP32 */

static bool ESP32_BLEConnectToServer() {
    pClient->connect(AppDevice);

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
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
      pRemoteCharacteristic->registerForNotify(AppNotifyCallback);

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

      BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

      SerialBT.setPin(settings->key);
      SerialBT.begin(BT_name.c_str(), true);

      xTaskCreate(ESP32_BT_SPP_Connection_Manager, "BT SPP ConMgr Task", 1024, NULL, tskIDLE_PRIORITY, NULL);

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

      BLEDevice::init("");

      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new AppClientCallback());

      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new AppAdvertisedDeviceCallbacks());
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

        BLEDevice::getScan()->start(3, false);

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

            pRemoteCharacteristic->writeValue(chunk, size);

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
      BLEDevice::deinit();
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
#include <CoreMutex.h>
#include <btstack.h>

#include <BTstack.h>
#include <api/RingBuffer.h>

#include "EEPROMHelper.h"
#include "WiFiHelper.h"   // HOSTNAME
#include "BluetoothHelper.h"

#define SOFTRF_SPP_COD 0x02c110

//#define DEBUG_SPP      Serial.printf
#define DEBUG_SPP

typedef enum {
    W4_PEER_COD,
    W4_SCAN_COMPLETE,
    W4_SDP_RESULT,
    W2_SEND_SDP_QUERY,
    W4_RFCOMM_CHANNEL,
    SENDING,
    DONE
} state_t;

static bool _running            = false;
static bool _overflow           = false;
static volatile bool _connected = false;
static size_t _fifoSize         = 512;
static uint16_t _channelID      = 0;
static volatile int _writeLen   = 0;

static mutex_t     _mutex;
static uint32_t    _writer;
static uint32_t    _reader;
static uint8_t    *_queue;
static uint16_t    _mtu;
static uint8_t     rfcomm_server_channel;
static bd_addr_t   peer_addr;
static state_t     state;
static const void *_writeBuff;

static btstack_packet_callback_registration_t  _hci_event_callback_registration;
static btstack_context_callback_registration_t _handle_sdp_client_query_request;

// prototypes
static void packetHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();

String BT_name = HOSTNAME;

UUID uuid("E2C56DB5-DFFB-48D2-B060-D0F5A71096E0");

/*
 * Find remote peer by COD
 */
#define INQUIRY_INTERVAL 5
static void start_scan(void){
    DEBUG_SPP("Starting inquiry scan..\r\n");
    state = W4_PEER_COD;
    gap_inquiry_start(INQUIRY_INTERVAL);
}
static void stop_scan(void){
    DEBUG_SPP("Stopping inquiry scan..\r\n");
    state = W4_SCAN_COMPLETE;
    gap_inquiry_stop();
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
            rfcomm_server_channel = sdp_event_query_rfcomm_service_get_rfcomm_channel(packet);
            break;
        case SDP_EVENT_QUERY_COMPLETE:
            if (sdp_event_query_complete_get_status(packet)){
                DEBUG_SPP("SDP query failed 0x%02x\r\n", sdp_event_query_complete_get_status(packet));
                break;
            }
            if (rfcomm_server_channel == 0){
                DEBUG_SPP("No SPP service found\r\n");
                break;
            }
            DEBUG_SPP("SDP query done, channel %u.\r\n", rfcomm_server_channel);
            rfcomm_create_channel(packetHandler, peer_addr, rfcomm_server_channel, NULL);
            break;
        default:
            break;
    }
}

static void handle_start_sdp_client_query(void * context){
    UNUSED(context);
    if (state != W2_SEND_SDP_QUERY) return;
    state = W4_RFCOMM_CHANNEL;
    sdp_client_query_rfcomm_channel_and_name_for_uuid(&handle_query_rfcomm_event, peer_addr, BLUETOOTH_ATTRIBUTE_PUBLIC_BROWSE_ROOT);
}

/*
 * @section Gerenal Packet Handler
 *
 * @text Handles startup (BTSTACK_EVENT_STATE), inquiry, pairing, starts SDP query for SPP service, and RFCOMM connection
 */

static void packetHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint32_t class_of_device;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                    start_scan();
                    break;

                case GAP_EVENT_INQUIRY_RESULT:
                    if (state != W4_PEER_COD) break;
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

                case GAP_EVENT_INQUIRY_COMPLETE:
                    switch (state){
                        case W4_PEER_COD:
                            DEBUG_SPP("Inquiry complete\r\n");
                            DEBUG_SPP("Peer not found, starting scan again\r\n");
                            start_scan();
                            break;
                        case W4_SCAN_COMPLETE:
                            DEBUG_SPP("Start to connect and query for SPP service\r\n");
                            state = W2_SEND_SDP_QUERY;
                            _handle_sdp_client_query_request.callback = &handle_start_sdp_client_query;
                            (void) sdp_client_register_query_callback(&_handle_sdp_client_query_request);
                            break;
                        default:
                            break;
                    }
                    if (state == W4_PEER_COD){
                    }
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    DEBUG_SPP("Pin code request - using '0000'\r\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
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

                    // re-enable page/inquiry scan again
                    gap_discoverable_control(1);
                    gap_connectable_control(1);
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

static void lockBluetooth() {
    async_context_acquire_lock_blocking(cyw43_arch_async_context());
}

static void unlockBluetooth() {
    async_context_release_lock(cyw43_arch_async_context());
}

static void CYW43_Bluetooth_setup()
{
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

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
      _hci_event_callback_registration.callback = &packetHandler;
      hci_add_event_handler(&_hci_event_callback_registration);

      l2cap_init();

#ifdef ENABLE_BLE
      // Initialize LE Security Manager. Needed for cross-transport key derivation
      sm_init();
#endif

      rfcomm_init();

      // init SDP
      gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);

      // turn on!
      hci_power_control(HCI_POWER_ON);

      _running = true;
    }
    break;
  case CON_BLUETOOTH_LE:
    BTstack.setup();
    BTstack.iBeaconConfigure(&uuid, 4711, 2);
    BTstack.startAdvertising();
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
    BTstack.loop();
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
    {
      if (!_running) {
          return;
      }
      _running = false;

      hci_power_control(HCI_POWER_OFF);
      lockBluetooth();
      delete[] _queue;
      unlockBluetooth();
    }
    break;
  case CON_BLUETOOTH_LE:
    /* TBD */
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

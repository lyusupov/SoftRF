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

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)

#include "../../system/SoC.h"

#if !defined(EXCLUDE_BLUETOOTH) && !defined(USE_ARDUINOBLE)

#include <queue>
#include <pico/cyw43_arch.h>
#include <CoreMutex.h>
#include <btstack.h>

#include <api/RingBuffer.h>

#include "../../driver/EEPROM.h"
#include "../../driver/WiFi.h"
#include "../../driver/Bluetooth.h"
#include "../../driver/Battery.h"

#if defined(ENABLE_BT_VOICE)
#include "AudioTools.h"
#include "BTstack_A2DP.h"

SineWaveGenerator<int16_t> sineWave(32000);
GeneratedSoundStream<int16_t> in(sineWave);
#endif /* ENABLE_BT_VOICE */

static bool _running = false;
static mutex_t _mutex;
static bool _overflow = false;
static volatile bool _connected = false;

static uint32_t _writer;
static uint32_t _reader;
static size_t   _fifoSize = 32;
static uint8_t *_queue = NULL;

static const int RFCOMM_SERVER_CHANNEL = 1;

static uint16_t _channelID;
static uint8_t  _spp_service_buffer[150];
static btstack_packet_callback_registration_t _hci_event_callback_registration;

static volatile int _writeLen = 0;
static const void *_writeBuff;

RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();

String BT_name = HOSTNAME;

/* ------- SPP BEGIN ------ */

static void hci_spp_packet_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    bd_addr_t event_addr;
    //uint8_t   rfcomm_channel_nr;
    //uint16_t  mtu;
    int i;

    switch (type) {
    case HCI_EVENT_PACKET:
        switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_PIN_CODE_REQUEST:
            //Serial.printf("Pin code request - using '0000'\n");
            hci_event_pin_code_request_get_bd_addr(packet, event_addr);
            gap_pin_code_response(event_addr, "0000");
            break;

        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            // ssp: inform about user confirmation request
            //Serial.printf("SSP User Confirmation Request with numeric value '%06" PRIu32 "'\n", little_endian_read_32(packet, 8));
            //Serial.printf("SSP User Confirmation Auto accept\n");
            break;

        case RFCOMM_EVENT_INCOMING_CONNECTION:
            rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
            //rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
            _channelID = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
            //Serial.printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
            rfcomm_accept_connection(_channelID);
            break;

        case RFCOMM_EVENT_CHANNEL_OPENED:
            if (rfcomm_event_channel_opened_get_status(packet)) {
                //Serial.printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
            } else {
                _channelID = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                //mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                //Serial.printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                _connected = true;
            }
            break;
        case RFCOMM_EVENT_CAN_SEND_NOW:
            rfcomm_send(_channelID, (uint8_t *)_writeBuff, _writeLen);
            _writeLen = 0;
            break;
        case RFCOMM_EVENT_CHANNEL_CLOSED:
            //Serial.printf("RFCOMM channel closed\n");
            _channelID = 0;
            _connected = false;
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

#define REPORT_INTERVAL_MS 3000
#define MAX_NR_CONNECTIONS 3
#define APP_AD_FLAGS       0x06

#define DEBUG_BLE          0

uint8_t *_advData   = nullptr;
uint8_t _advDataLen = 0;
uint8_t *_attdb     = nullptr;
size_t _attdbLen    = 0;

void _buildAdvData(const char *completeLocalName) {
    free(_advData);
    _advDataLen = 9 + strlen(completeLocalName);
    _advData = (uint8_t*) malloc(_advDataLen);
    int i = 0;
    // Flags general discoverable, BR/EDR not supported
    // 0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    _advData[i++] = 0x02;
    _advData[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
    _advData[i++] = 0x06;
    // Name
    // 0x0d, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    _advData[i++] = 1 + strlen(completeLocalName);
    _advData[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(_advData + i, completeLocalName, strlen(completeLocalName));
    i += strlen(completeLocalName);
    // 16-bit Service UUIDs
    _advData[i++] = 0x03;
    _advData[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS;
    _advData[i++] = 0xe0;
    _advData[i++] = 0xff;
}

static constexpr const uint8_t _attdb_head[] = {
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,
};

static constexpr const uint8_t _attdb_tail[] =  {
    // Specification Type org.bluetooth.service.battery_service
    // https://www.bluetooth.com/api/gatt/xmlfile?xmlFileName=org.bluetooth.service.battery_service.xml
    // Battery Service 180F
    // 0x0004 PRIMARY_SERVICE-ORG_BLUETOOTH_SERVICE_BATTERY_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x0f, 0x18,
    // 0x0005 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    0x0d, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x12, 0x06, 0x00, 0x19, 0x2a,
    // 0x0006 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x06, 0x00, 0x19, 0x2a,
    // 0x0007 CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x07, 0x00, 0x02, 0x29, 0x00, 0x00,

    // Specification Type org.bluetooth.service.device_information
    // https://www.bluetooth.com/api/gatt/xmlfile?xmlFileName=org.bluetooth.service.device_information.xml
    // Device Information 180A
    // 0x0008 PRIMARY_SERVICE-ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION
    0x0a, 0x00, 0x02, 0x00, 0x08, 0x00, 0x00, 0x28, 0x0a, 0x18,
    // 0x0009 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_MANUFACTURER_NAME_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x09, 0x00, 0x03, 0x28, 0x02, 0x0a, 0x00, 0x29, 0x2a,
    // 0x000a VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_MANUFACTURER_NAME_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x0a, 0x00, 0x29, 0x2a,
    // 0x000b CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_MODEL_NUMBER_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x0b, 0x00, 0x03, 0x28, 0x02, 0x0c, 0x00, 0x24, 0x2a,
    // 0x000c VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_MODEL_NUMBER_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x0c, 0x00, 0x24, 0x2a,
    // 0x000d CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SERIAL_NUMBER_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x03, 0x28, 0x02, 0x0e, 0x00, 0x25, 0x2a,
    // 0x000e VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SERIAL_NUMBER_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x0e, 0x00, 0x25, 0x2a,
    // 0x000f CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_HARDWARE_REVISION_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x0f, 0x00, 0x03, 0x28, 0x02, 0x10, 0x00, 0x27, 0x2a,
    // 0x0010 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_HARDWARE_REVISION_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x10, 0x00, 0x27, 0x2a,
    // 0x0011 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_FIRMWARE_REVISION_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x11, 0x00, 0x03, 0x28, 0x02, 0x12, 0x00, 0x26, 0x2a,
    // 0x0012 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_FIRMWARE_REVISION_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x12, 0x00, 0x26, 0x2a,
    // 0x0013 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SOFTWARE_REVISION_STRING - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x13, 0x00, 0x03, 0x28, 0x02, 0x14, 0x00, 0x28, 0x2a,
    // 0x0014 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SOFTWARE_REVISION_STRING - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x14, 0x00, 0x28, 0x2a,
    // 0x0015 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SYSTEM_ID - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x15, 0x00, 0x03, 0x28, 0x02, 0x16, 0x00, 0x23, 0x2a,
    // 0x0016 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_SYSTEM_ID - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x16, 0x00, 0x23, 0x2a,
    // 0x0017 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_IEEE_11073_20601_REGULATORY_CERTIFICATION_DATA_LIST - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x17, 0x00, 0x03, 0x28, 0x02, 0x18, 0x00, 0x2a, 0x2a,
    // 0x0018 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_IEEE_11073_20601_REGULATORY_CERTIFICATION_DATA_LIST - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x18, 0x00, 0x2a, 0x2a,
    // 0x0019 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_PNP_ID - DYNAMIC | READ
    0x0d, 0x00, 0x02, 0x00, 0x19, 0x00, 0x03, 0x28, 0x02, 0x1a, 0x00, 0x50, 0x2a,
    // 0x001a VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_PNP_ID - DYNAMIC | READ
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x1a, 0x00, 0x50, 0x2a,

    // 0x001b PRIMARY_SERVICE-FFE0
    0x0a, 0x00, 0x02, 0x00, 0x1b, 0x00, 0x00, 0x28, 0xe0, 0xff,
    // 0x001c CHARACTERISTIC-FFE1 - READ | WRITE_WITHOUT_RESPONSE | NOTIFY | DYNAMIC
    0x0d, 0x00, 0x02, 0x00, 0x1c, 0x00, 0x03, 0x28, 0x16, 0x1d, 0x00, 0xe1, 0xff,
    // 0x001d VALUE CHARACTERISTIC-FFE1 - READ | WRITE_WITHOUT_RESPONSE | NOTIFY | DYNAMIC
    0x08, 0x00, 0x06, 0x01, 0x1d, 0x00, 0xe1, 0xff,
    // 0x001e CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x1e, 0x00, 0x02, 0x29, 0x00, 0x00,
    // 0x001f USER_DESCRIPTION-READ-HMSoft
    // READ_ANYBODY, WRITE_ANYBODY
    0x08, 0x00, 0x0a, 0x01, 0x1f, 0x00, 0x01, 0x29,
    // END
    0x00, 0x00,
};

void _buildAttdb(const char *Name) {
    free(_attdb);
    _attdbLen = sizeof(_attdb_head) + 8 + strlen(Name) + sizeof(_attdb_tail);
    _attdb = (uint8_t *) malloc(_attdbLen);
    memcpy(_attdb, _attdb_head, sizeof(_attdb_head));
    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ
    // READ_ANYBODY
    // 0x11, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x48, 0x49, 0x44, 0x20, 0x4d, 0x6f, 0x75, 0x73, 0x65,
    int i = sizeof(_attdb_head);
    _attdb[i++] = 8 + strlen(Name);
    _attdb[i++] = 0x00;
    _attdb[i++] = 0x02;
    _attdb[i++] = 0x00;
    _attdb[i++] = 0x03;
    _attdb[i++] = 0x00;
    _attdb[i++] = 0x00;
    _attdb[i++] = 0x2a;
    memcpy(_attdb + i, Name, strlen(Name));
    i += strlen(Name);
    memcpy(_attdb + i, _attdb_tail, sizeof(_attdb_tail));
}

// support for multiple clients
typedef struct {
    char name;
    int le_notification_enabled;
    uint16_t value_handle;
    hci_con_handle_t connection_handle;
    int  counter;
    char test_data[200];
    int  test_data_len;
    uint32_t test_data_sent;
    uint32_t test_data_start;
} le_streamer_connection_t;

static le_streamer_connection_t le_streamer_connections[MAX_NR_CONNECTIONS];

// round robin sending
static int connection_index;

static void init_connections(void){
    // track connections
    int i;
    for (i=0;i<MAX_NR_CONNECTIONS;i++){
        le_streamer_connections[i].connection_handle = HCI_CON_HANDLE_INVALID;
        le_streamer_connections[i].name = 'A' + i;
    }
}

static le_streamer_connection_t * connection_for_conn_handle(hci_con_handle_t conn_handle){
    int i;
    for (i=0;i<MAX_NR_CONNECTIONS;i++){
        if (le_streamer_connections[i].connection_handle == conn_handle) return &le_streamer_connections[i];
    }
    return NULL;
}

static void next_connection_index(void){
    connection_index++;
    if (connection_index == MAX_NR_CONNECTIONS){
        connection_index = 0;
    }
}

static void test_reset(le_streamer_connection_t * context){
    context->test_data_start = btstack_run_loop_get_time_ms();
    context->test_data_sent = 0;
}

static void test_track_sent(le_streamer_connection_t * context, int bytes_sent){
    context->test_data_sent += bytes_sent;
    // evaluate
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t time_passed = now - context->test_data_start;
    if (time_passed < REPORT_INTERVAL_MS) return;
    // print speed
    int bytes_per_second = context->test_data_sent * 1000 / time_passed;
#if DEBUG_BLE
    Serial.printf("%c: %"PRIu32" bytes sent-> %u.%03u kB/s\r\n", context->name, context->test_data_sent, bytes_per_second / 1000, bytes_per_second % 1000);
#endif /* DEBUG_BLE */

    // restart
    context->test_data_start = now;
    context->test_data_sent  = 0;
}

static void streamer(void){

    // find next active streaming connection
    int old_connection_index = connection_index;
    while (true) {
        // active found?
        if ((le_streamer_connections[connection_index].connection_handle != HCI_CON_HANDLE_INVALID) &&
            (le_streamer_connections[connection_index].le_notification_enabled)) break;

        // check next
        next_connection_index();

        // none found
        if (connection_index == old_connection_index) return;
    }

    le_streamer_connection_t * context = &le_streamer_connections[connection_index];

    size_t size = BLE_FIFO_TX.available();
    size = size < context->test_data_len ? size : context->test_data_len;
    size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

    if (size > 0) {
      for (int i=0; i < size; i++) {
        context->test_data[i] = BLE_FIFO_TX.read_char();
      }

      // send
      att_server_notify(context->connection_handle, context->value_handle, (uint8_t*) context->test_data, size);

      // request next send event
      att_server_request_can_send_now_event(context->connection_handle);
    }

    // track
    test_track_sent(context, size);

    // check next
    next_connection_index();
}

static void hci_le_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint16_t conn_interval;
    hci_con_handle_t con_handle;
    static const char * const phy_names[] = {
        "1 M", "2 M", "Codec"
    };

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                //Serial.printf("To start the streaming, please run the le_streamer_client example on other device, or use some GATT Explorer, e.g. LightBlue, BLExplr.\r\n");
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
#if DEBUG_BLE
            Serial.printf("- LE Connection 0x%04x: disconnect, reason %02x\r\n", con_handle, hci_event_disconnection_complete_get_reason(packet));
#endif /* DEBUG_BLE */
            break;
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    // print connection parameters (without using float operations)
                    con_handle    = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
#if DEBUG_BLE
                    Serial.printf("- LE Connection 0x%04x: connected - connection interval %u.%02u ms, latency %u\r\n", con_handle, conn_interval * 125 / 100,
                        25 * (conn_interval & 3), hci_subevent_le_connection_complete_get_conn_latency(packet));

                    // request min con interval 15 ms for iOS 11+
                    Serial.printf("- LE Connection 0x%04x: request 15 ms connection interval\r\n", con_handle);
#endif /* DEBUG_BLE */
                    gap_request_connection_parameter_update(con_handle, 12, 12, 0, 0x0048);
                    break;
                case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
                    // print connection parameters (without using float operations)
                    con_handle    = hci_subevent_le_connection_update_complete_get_connection_handle(packet);
                    conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
#if DEBUG_BLE
                    Serial.printf("- LE Connection 0x%04x: connection update - connection interval %u.%02u ms, latency %u\r\n", con_handle, conn_interval * 125 / 100,
                        25 * (conn_interval & 3), hci_subevent_le_connection_update_complete_get_conn_latency(packet));
#endif /* DEBUG_BLE */
                    break;
                case HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE:
                    con_handle = hci_subevent_le_data_length_change_get_connection_handle(packet);
#if DEBUG_BLE
                    Serial.printf("- LE Connection 0x%04x: data length change - max %u bytes per packet\r\n", con_handle,
                                  hci_subevent_le_data_length_change_get_max_tx_octets(packet));
#endif /* DEBUG_BLE */
                    break;
                case HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE:
                    con_handle = hci_subevent_le_phy_update_complete_get_connection_handle(packet);
                    Serial.printf("- LE Connection 0x%04x: PHY update - using LE %s PHY now\r\n", con_handle,
                                  phy_names[hci_subevent_le_phy_update_complete_get_tx_phy(packet)]);
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

static void att_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    int mtu;
    le_streamer_connection_t * context;
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case ATT_EVENT_CONNECTED:
                    // setup new
                    context = connection_for_conn_handle(HCI_CON_HANDLE_INVALID);
                    if (!context) break;
                    context->counter = 'A';
                    context->connection_handle = att_event_connected_get_handle(packet);
                    context->test_data_len = btstack_min(att_server_get_mtu(context->connection_handle) - 3, sizeof(context->test_data));
#if DEBUG_BLE
                    Serial.printf("%c: ATT connected, handle %04x, test data len %u\r\n", context->name, context->connection_handle, context->test_data_len);
#endif /* DEBUG_BLE */
                    break;
                case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
                    mtu = att_event_mtu_exchange_complete_get_MTU(packet) - 3;
                    context = connection_for_conn_handle(att_event_mtu_exchange_complete_get_handle(packet));
                    if (!context) break;
                    context->test_data_len = btstack_min(mtu - 3, sizeof(context->test_data));
#if DEBUG_BLE
                    Serial.printf("%c: ATT MTU = %u => use test data of len %u\r\n", context->name, mtu, context->test_data_len);
#endif /* DEBUG_BLE */
                    break;
                case ATT_EVENT_CAN_SEND_NOW:
                    streamer();
                    break;
                case ATT_EVENT_DISCONNECTED:
                    context = connection_for_conn_handle(att_event_disconnected_get_handle(packet));
                    if (!context) break;
                    // free connection
#if DEBUG_BLE
                    Serial.printf("%c: ATT disconnected, handle %04x\r\n", context->name, context->connection_handle);
#endif /* DEBUG_BLE */
                    context->le_notification_enabled = 0;
                    context->connection_handle = HCI_CON_HANDLE_INVALID;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(offset);

#if DEBUG_BLE
    Serial.printf("att_write_callback att_handle %04x, transaction mode %u size %u offset %u\r\n", att_handle, transaction_mode, buffer_size, offset);
#endif /* DEBUG_BLE */
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) return 0;
    le_streamer_connection_t * context = connection_for_conn_handle(con_handle);
    switch(att_handle){
        case ATT_CHARACTERISTIC_FFE1_01_CLIENT_CONFIGURATION_HANDLE:
            context->le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            //Serial.printf("%c: Notifications enabled %u\r\n", context->name, context->le_notification_enabled);
            if (context->le_notification_enabled){
                switch (att_handle){
                    case ATT_CHARACTERISTIC_FFE1_01_CLIENT_CONFIGURATION_HANDLE:
                        context->value_handle = ATT_CHARACTERISTIC_FFE1_01_VALUE_HANDLE;
                        break;
                    default:
                        break;
                }
                att_server_request_can_send_now_event(context->connection_handle);
            }
            test_reset(context);
            break;
        case ATT_CHARACTERISTIC_FFE1_01_VALUE_HANDLE:
#if DEBUG_BLE
            Serial.printf("Write to 0x%04x, len %u offset %u\r\n", att_handle, buffer_size, offset);
#endif /* DEBUG_BLE */
            if (buffer_size > 0 && offset == 0) {
              size_t size = BLE_FIFO_RX.availableForStore();
              size = (size < buffer_size) ? size : buffer_size;
              for (size_t i = 0; i < size; i++) {
                BLE_FIFO_RX.store_char(buffer[i]);
              }
            }
            break;
        default:
            Serial.printf("Write to 0x%04x, len %u\r\n", att_handle, buffer_size);
            break;
    }
    return 0;
}

#define ATT_VALUE_MAX_LEN  50
#define ATT_NUM_ATTRIBUTES 10

typedef struct {
    uint16_t handle;
    uint16_t len;
    uint8_t  value[ATT_VALUE_MAX_LEN];
} attribute_t;

static attribute_t att_attributes[ATT_NUM_ATTRIBUTES];

// handle == 0 finds free attribute
static int att_attribute_for_handle(uint16_t aHandle){
    int i;
    for (i=0;i<ATT_NUM_ATTRIBUTES;i++){
        if (att_attributes[i].handle == aHandle) {
            return i;
        }
    }
    return -1;
}

static void att_setup_attribute(uint16_t attribute_handle, const uint8_t * value, uint16_t len){
    int index = att_attribute_for_handle(attribute_handle);
    if (index < 0){
        index = att_attribute_for_handle(0);
    }
#if DEBUG_BLE
    Serial.printf("Setup Attribute %04x, len %u, value: %s\r\n", attribute_handle, len, value);
#endif /* DEBUG_BLE */
    att_attributes[index].handle = attribute_handle;
    att_attributes[index].len    = len;
    memcpy(att_attributes[index].value, value, len);
}

static void att_attributes_init(void){
    int i;
    for (i=0;i<ATT_NUM_ATTRIBUTES;i++){
        att_attributes[i].handle = 0;
    }

    // preset some attributes
    att_setup_attribute(ATT_CHARACTERISTIC_FFE1_01_USER_DESCRIPTION_HANDLE,
                        (const uint8_t *) "HMSoft", strlen("HMSoft"));
}

static unsigned long BLE_Aux_Tx_TimeMarker = 0;

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

  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    {
      mutex_init(&_mutex);
      _overflow = false;

      _queue = new uint8_t[_fifoSize];
      _writer = 0;
      _reader = 0;

      // register for HCI events
      _hci_event_callback_registration.callback = &hci_spp_packet_handler;
      hci_add_event_handler(&_hci_event_callback_registration);

      l2cap_init();

#ifdef ENABLE_BLE
      // Initialize LE Security Manager. Needed for cross-transport key derivation
      sm_init();
#endif

      rfcomm_init();
      rfcomm_register_service(hci_spp_packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

      // init SDP, create record for SPP and register with SDP
      sdp_init();
      bzero(_spp_service_buffer, sizeof(_spp_service_buffer));
      spp_create_sdp_record(_spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "CYW43_SPP");
      sdp_register_service(_spp_service_buffer);

      gap_discoverable_control(1);
      gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
      gap_set_local_name(BT_name.c_str());

      hci_power_control(HCI_POWER_ON);

      _running = true;
    }
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      mutex_init(&_mutex);

      BT_name += "-LE";
      _buildAdvData(BT_name.c_str());
      _buildAttdb(BT_name.c_str());

      l2cap_init();
//      l2cap_set_max_le_mtu(26);

      // setup SM: Display only
      sm_init();

      // setup ATT server
      att_server_init(_attdb, NULL, att_write_callback);

      // Setup battery service
      battery_service_server_init((uint8_t) Battery_charge());

      static char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());
      static char Hardware[9];
      snprintf(Hardware,  sizeof(Hardware),  "%08X", hw_info.revision);

      const char *Firmware = "Arduino RP2040 " ARDUINO_PICO_VERSION_STR;

      // Setup device information service
      device_information_service_server_init();
      device_information_service_server_set_manufacturer_name(RP2xxx_Device_Manufacturer);
      device_information_service_server_set_model_number(RP2xxx_Device_Model);
      device_information_service_server_set_serial_number(SerialNum);
      device_information_service_server_set_hardware_revision(Hardware);
      device_information_service_server_set_firmware_revision(Firmware);
      device_information_service_server_set_software_revision(SOFTRF_FIRMWARE_VERSION);

      // register for HCI events
      _hci_event_callback_registration.callback = &hci_le_packet_handler;
      hci_add_event_handler(&_hci_event_callback_registration);

      att_attributes_init();

      // register for ATT events
      att_server_register_packet_handler(att_packet_handler);

      // setup advertisements
      uint16_t adv_int_min = 0x0030;
      uint16_t adv_int_max = 0x0030;
      uint8_t adv_type = 0;
      bd_addr_t null_addr;
      memset(null_addr, 0, 6);
      gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
      gap_advertisements_set_data(_advDataLen, _advData);
      gap_advertisements_enable(1);

      // init client state
      init_connections();

      hci_power_control(HCI_POWER_ON);

       BLE_Aux_Tx_TimeMarker = millis();
      _running = true;
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
#if defined(ENABLE_BT_VOICE)
    A2DPSource.setVolume(50);
    A2DPSource.begin(in);
#endif
    break;
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static void CYW43_Bluetooth_loop()
{
  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    if (_running && (millis() - BLE_Aux_Tx_TimeMarker > 100)) {
      if (BLE_FIFO_TX.available() > 0) {
        streamer();
      }
      BLE_Aux_Tx_TimeMarker = millis();
    }

    if (isTimeToBattery()) {
      battery_service_server_set_battery_value((uint8_t) Battery_charge());
    }
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
  switch (settings->bluetooth)
  {
  case BLUETOOTH_SPP:
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      if (!_running) {
          return;
      }
      _running = false;

      hci_power_control(HCI_POWER_OFF);
      lockBluetooth();
      if (_queue != NULL) delete[] _queue;
      unlockBluetooth();
    }
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

  switch (settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    {
      CoreMutex m(&_mutex);
      if (_running && m) {
        rval = (_fifoSize + _writer - _reader) % _fifoSize;
      }
    }
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

  switch (settings->bluetooth)
  {
  case BLUETOOTH_SPP:
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

  switch (settings->bluetooth)
  {
  case BLUETOOTH_SPP:
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
#endif /* ARDUINO_ARCH_RP2XXX */

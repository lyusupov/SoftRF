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

#if defined(ARDUINO_ARCH_NRF52)

#include "../../system/SoC.h"
#include "../../driver/Bluetooth.h"

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <BLEUart_HM10.h>
#include <TinyGPS++.h>
#if defined(USE_BLE_MIDI)
#include <MIDI.h>
#endif /* USE_BLE_MIDI */

#include "../../driver/WiFi.h"
#include "../../driver/Battery.h"
#include "../../driver/GNSS.h"
#include "../../driver/RF.h"
#include "../../protocol/radio/Legacy.h"
#include "../../driver/Baro.h"
#include "../../driver/EEPROM.h"
#include "../../driver/Sound.h"
#include "../../protocol/data/NMEA.h"
#include "../../protocol/data/GDL90.h"
#include "../../protocol/data/D1090.h"

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

const uint16_t UUID16_SVC_DFU_OTA = 0xFE59;

const uint8_t UUID128_CHR_DFU_CONTROL[16] = {0x50, 0xEA, 0xDA, 0x30, 0x88, 0x83, 0xB8, 0x9F,
                                             0x60, 0x4F, 0x15, 0xF3, 0x03, 0x00, 0xC9, 0x8E};

extern "C" void bootloader_util_app_start(uint32_t start_addr);

static uint16_t crc16(const uint8_t *data_p, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return crc;
}

static void bledfu_control_wr_authorize_cb(uint16_t conn_hdl, BLECharacteristic *chr, ble_gatts_evt_write_t *request)
{
    if ((request->handle == chr->handles().value_handle) && (request->op != BLE_GATTS_OP_PREP_WRITE_REQ) &&
        (request->op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) && (request->op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
        BLEConnection *conn = Bluefruit.Connection(conn_hdl);

        ble_gatts_rw_authorize_reply_params_t reply = {.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE};

        if (!chr->indicateEnabled(conn_hdl)) {
            reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_CCCD_CONFIG_ERROR;
            sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
            return;
        }

        reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);

        enum { START_DFU = 1 };
        if (request->data[0] == START_DFU) {
            // Peer data information so that bootloader could re-connect after reboot
            typedef struct {
                ble_gap_addr_t addr;
                ble_gap_irk_t irk;
                ble_gap_enc_key_t enc_key;
                uint8_t sys_attr[8];
                uint16_t crc16;
            } peer_data_t;

            VERIFY_STATIC(offsetof(peer_data_t, crc16) == 60);

            /* Save Peer data
             * Peer data address is defined in bootloader linker @0x20007F80
             * - If bonded : save Security information
             * - Otherwise : save Address for direct advertising
             *
             * TODO may force bonded only for security reason
             */
            peer_data_t *peer_data = (peer_data_t *)(0x20007F80UL);
            varclr(peer_data);

            // Get CCCD
            uint16_t sysattr_len = sizeof(peer_data->sys_attr);
            sd_ble_gatts_sys_attr_get(conn_hdl, peer_data->sys_attr, &sysattr_len, BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS);

            // Get Bond Data or using Address if not bonded
            peer_data->addr = conn->getPeerAddr();

            if (conn->secured()) {
                bond_keys_t bkeys;
                if (conn->loadBondKey(&bkeys)) {
                    peer_data->addr = bkeys.peer_id.id_addr_info;
                    peer_data->irk = bkeys.peer_id.id_info;
                    peer_data->enc_key = bkeys.own_enc;
                }
            }

            // Calculate crc
            peer_data->crc16 = crc16((uint8_t *)peer_data, offsetof(peer_data_t, crc16));

            // Initiate DFU Sequence and reboot into DFU OTA mode
            Bluefruit.Advertising.restartOnDisconnect(false);
            conn->disconnect();

            NRF_POWER->GPREGRET = 0xB1;
            NVIC_SystemReset();
        }
    }
}

BLEDfuSecure::BLEDfuSecure(void) : BLEService(UUID16_SVC_DFU_OTA), _chr_control(UUID128_CHR_DFU_CONTROL) {}

err_t BLEDfuSecure::begin(void)
{
    // Invoke base class begin()
    VERIFY_STATUS(BLEService::begin());

    _chr_control.setProperties(CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
    _chr_control.setMaxLen(23);
    _chr_control.setWriteAuthorizeCallback(bledfu_control_wr_authorize_cb);
    VERIFY_STATUS(_chr_control.begin());

    return ERROR_NONE;
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
BLEDfuSecure  bledfusecure;
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

#if defined(ENABLE_REMOTE_ID)
#include "../../protocol/radio/RemoteID.h"

#define UUID16_COMPANY_ID_ASTM 0xFFFA

static unsigned long RID_Time_Marker = 0;

BLEService    BLE_ODID_service;

static const uint8_t ODID_Uuid[] = {0x00, 0x00, 0xff, 0xfa, 0x00, 0x00, 0x10, 0x00,
                                    0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
#endif /* ENABLE_REMOTE_ID */

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

#if defined(ENABLE_REMOTE_ID)
    if (rid_enabled()) {
      Bluefruit.Advertising.addService(BLE_ODID_service);
      Bluefruit.Advertising.addName();
    } else
#endif /* ENABLE_REMOTE_ID */
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
#if defined(ENABLE_REMOTE_ID)
  if (!rid_enabled()) 
#endif /* ENABLE_REMOTE_ID */
  {
    Bluefruit.ScanResponse.addName();
  }

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
  char id_06x[8];
  snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
  BT_name += "-";
  BT_name += String(id_06x);

#if defined(ENABLE_REMOTE_ID)
  rid_init();
  RID_Time_Marker = millis();
#endif /* ENABLE_REMOTE_ID */

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
  //if (hw_info.model == SOFTRF_MODEL_CARD) {
  //  bledfusecure.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
  //  bledfusecure.begin();
  //} else {
    bledfu.begin();
  //}

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

#if defined(ENABLE_REMOTE_ID)
  if (rid_enabled()) {
    BLE_ODID_service.setUuid(BLEUuid(ODID_Uuid /* UUID16_COMPANY_ID_ASTM */));
    BLE_ODID_service.begin();
  }
#endif /* ENABLE_REMOTE_ID */

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

#if defined(ENABLE_REMOTE_ID)
  if (rid_enabled() && isValidFix()) {
    if ((millis() - RID_Time_Marker) > 74) {
      rid_encode((void *) &utm_data, &ThisAircraft);
      squitter.transmit(&utm_data);

      RID_Time_Marker = millis();
    }
  }
#endif /* ENABLE_REMOTE_ID */
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
  if ( bleuart_HM10.notifyEnabled() && size > 0) {
    return bleuart_HM10.write(buffer, size);
  }

#if !defined(EXCLUDE_NUS)
  if ( bleuart_NUS.notifyEnabled() && size > 0) {
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

#endif /* ARDUINO_ARCH_NRF52 */

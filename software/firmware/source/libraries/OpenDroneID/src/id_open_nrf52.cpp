/* -*- tab-width: 2; mode: c; -*-
 * 
 * C++ class for Arduino to function as a wrapper around opendroneid.
 * This file has nRF52 specific code.
 *
 * Copyright (c) 2022, Steve Jack.
 *
 * December '22  
 *
 * MIT licence.
 *
 * NOTES
 *
 * Experimental.
 *
 * BLE_OPTION == 1
 * 
 * Uses Adafruit's BLE libraries. Works for BT4. It doesn't work for BT5 coded.
 * Adafruit use Nordic's S140 and support for coded in S410 is experimental.
 *
 * BLE_OPTION == 2
 *
 * To do. Uses the nRF functions directly.
 * Probably easier and better to use Zephyr rather than Arduino for this.
 * 
 */

#define DIAGNOSTICS    0
#define BLE_OPTION     3

//

#if defined(ARDUINO_ARCH_NRF52)

#pragma GCC diagnostic warning "-Wunused-variable"

#include <Arduino.h>

#include "id_open.h"
#include "id_open_nrf52.h"

#if ID_OD_WIFI

#endif // WIFI

#if ID_OD_BT

#if BLE_OPTION == 1

// Use Adafruit's BLE libraries.
// nRF SDK, Soft Device S140.

#define PATCHED_BLEADV 0

BLEService BLE_ODID_service;

#elif BLE_OPTION == 2

#elif BLE_OPTION == 3

#define PATCHED_BLEADV 0

#endif

//

const uint8_t    ODID_Uuid[] = {0x00, 0x00, 0xff, 0xfa, 0x00, 0x00, 0x10, 0x00,
                                0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
static char      BT_type_toggle[32];

#endif

extern "C" {
  ;
}

static void check_error(char *);
static void connect_callback(uint16_t);
static void disconnect_callback(uint16_t,uint8_t);

static Stream *Debug_Serial = NULL;
#if DIAGNOSTICS
static char    text[128];
#endif

/*
 *
 */

void construct2() {

  return;
}

/*
 *
 */

void init2(char *ssid,int ssid_length,uint8_t *WiFi_mac_addr,uint8_t wifi_channel) {

  memset(BT_type_toggle,4,sizeof(BT_type_toggle));
  
#if DIAGNOSTICS
  text[0] = text[63] = 0;

  Debug_Serial = &Serial;
#endif

#if ID_OD_WIFI

#endif // WIFI
  
#if ID_OD_BT

#if BLE_OPTION == 1

  err_t error;
  
  Bluefruit.begin();
  Bluefruit.setTxPower(8);
  Bluefruit.setName(ssid);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  BLE_ODID_service.setUuid(BLEUuid(ODID_Uuid));
  Bluefruit.Advertising.addService(BLE_ODID_service);
  error = BLE_ODID_service.begin();

  Bluefruit.Advertising.addName();
  // Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(100,100);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode

  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

#elif BLE_OPTION == 2

#elif BLE_OPTION == 3

#endif
  
#endif // BT
  
  return;
}

/*
 *
 */

uint8_t *capability() {

  static uint8_t capa[2] = {0x11,0x05};
  
  return capa;
}

//

int tag_rates(uint8_t *beacon_frame,int beacon_offset) {

  return beacon_offset;
}

//

int tag_ext_rates(uint8_t *beacon_frame,int beacon_offset) {

  return beacon_offset;
}

//

int misc_tags(uint8_t *beacon_frame,int beacon_offset) {

  return beacon_offset;
}

/*
 *
 */

int transmit_wifi2(uint8_t *buffer,int length) {

#if ID_OD_WIFI

  ;

#endif

  return 0;
}

/*
 * To do:
 *
 * Alternate BT4 & BT5 ?
 *
 */

int transmit_ble2(uint8_t *ble_message,int length) {

  short int                    BT_type = 5, index = 0;
  ODID_BasicID_encoded        *basic_encoded;
  ODID_Auth_encoded_page_zero *auth_encoded;
  
  basic_encoded = (ODID_BasicID_encoded *)        &ble_message[6];
  auth_encoded  = (ODID_Auth_encoded_page_zero *) &ble_message[6];

#if ID_OD_BT

#if DIAGNOSTICS
  static bool diag = 0;

  if ((!diag)&&(Debug_Serial)&&(millis() > 5000)) {

    diag = 1;
    
    // This should go in init2, but doing it this way gives us chance to 
    // open the serial port.

    ble_version_t version;

    if (text[0]) {
      Debug_Serial->print(text);
    }
    
    sd_ble_version_get(&version);

    sprintf(text,"nRF sd_ble_version_get() = 0x%04x %d %d \n",
            version.company_id,version.version_number,version.subversion_number);
    Debug_Serial->print(text);
  }
#endif

  if (length < (ODID_MESSAGE_SIZE + 10)) {

    // Toggle between BT4 and BT5.

    switch (basic_encoded->MessageType) {

    case ODID_MESSAGETYPE_BASIC_ID:

      index = (basic_encoded->IDType < 6) ? basic_encoded->IDType: 0;
      break;

    case ODID_MESSAGETYPE_LOCATION:
    case ODID_MESSAGETYPE_SELF_ID:
    case ODID_MESSAGETYPE_SYSTEM:
    case ODID_MESSAGETYPE_OPERATOR_ID:
      
      index = basic_encoded->MessageType + 6;
      break;

    case ODID_MESSAGETYPE_AUTH:
      index = auth_encoded->DataPage + 14;
      break;

    default:

      index = 0;
      break;
    }

    BT_type               =  BT_type_toggle[index];
    BT_type_toggle[index] = (BT_type_toggle[index] == 4) ? 5: 4;
  } 

#if BLE_OPTION == 1 || BLE_OPTION == 3

  Bluefruit.Advertising.stop();

	if (BT_type == 4) {

		Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);
#if PATCHED_BLEADV
		Bluefruit.Advertising.setPhy(BLE_GAP_PHY_AUTO);
#endif
	} else {

		Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
#if PATCHED_BLEADV
		Bluefruit.Advertising.setPhy(BLE_GAP_PHY_CODED);
#endif
	}

  Bluefruit.Advertising.setData(ble_message,length);
  Bluefruit.Advertising.start(0);
  
#if DIAGNOSTICS
  if (Debug_Serial) {

    sprintf(text,"\r%s %08x %2d BT%d ",__func__,ble_message,length,BT_type);
    Debug_Serial->print(text);

    for (int j = 0; j < 12; ++j) {

      sprintf(text,"%02x ",ble_message[j]);
      Debug_Serial->print(text);
    }
  }
#endif

#elif BLE_OPTION == 2

#endif  

#endif

  return 0;
}

/*
 *
 */

void connect_callback(uint16_t conn_handle) {

  return;
}

//

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {

  return;
}

/*
 * Misc. utility functions.
 */

void check_error(char * name) {

  char text[128];

  if ((Debug_Serial)) {

    sprintf(text,"%s\n",name);
    Debug_Serial->print(text);
  }
  
  return;
}

/*
 *
 */

#endif // NRF52

/* -*- tab-width: 2; mode: c; -*-
 *
 * C++ class for Arduino to function as a wrapper around opendroneid.
 *
 * Copyright (c) 2020-2023, Steve Jack.
 *
 * MIT licence.
 *
 */

#ifndef ID_OPENDRONE_H
#define ID_OPENDRONE_H

/*
 *  Using an ESP32 and enabling both WiFi and Bluetooth will almost certainly 
 *  require a partition scheme with > 1.2M for the application.
 */

#if defined(ARDUINO_ARCH_ESP32)

#define ID_OD_WIFI_NAN    0
#define ID_OD_WIFI_BEACON 1
#define ID_OD_BT          0        // ASTM F3411-19 / ASD-STAN 4709-002.

#define USE_BEACON_FUNC   0
//#define ESP32_WIFI_OPTION 0
#define ESP32_WIFI_OPTION 2

#define WIFI_CHANNEL      1

#elif defined(ARDUINO_ARCH_ESP8266)

#define ID_OD_WIFI_NAN    0
#define ID_OD_WIFI_BEACON 1
#define ID_OD_BT          0

#define USE_BEACON_FUNC   0

#elif defined(ARDUINO_ARCH_RP2040)

// The Pico doesn't have BT and the NAN/OD beacon code needs work to get it to compile for the Pico.

#define ID_OD_WIFI_NAN    0
#define ID_OD_WIFI_BEACON 1
#define ID_OD_BT          0

#define USE_BEACON_FUNC   0

#elif defined(ARDUINO_ARCH_NRF52)

#define ID_OD_WIFI_NAN    0
#define ID_OD_WIFI_BEACON 0
#define ID_OD_BT          1

#define USE_BEACON_FUNC   0

#else

error "No configuration for this processor."

#endif

// National/regional specific RIDs.

#define ID_JAPAN          0        // Experimental

#if (ID_JAPAN) && (ID_OD_WIFI_NAN || USE_BEACON_FUNC || !ID_OD_WIFI_BEACON)
#warning "National IDs will only work with WIFI_BEACON"
#define ID_NATIONAL       0
#else
#define ID_NATIONAL       ID_JAPAN
#endif

#if ID_OD_WIFI_NAN || ID_OD_WIFI_BEACON
#define ID_OD_WIFI        1
#else
#define ID_OD_WIFI        0
#endif

#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL      6        // Be careful changing this.
#endif /* WIFI_CHANNEL */
#define BEACON_FRAME_SIZE 512
#define BEACON_INTERVAL   0        // ms, defaults to 500. Android apps would prefer 100ms.

// Used by the id_open_beacon and id_open_esp32.
 
#if ID_JAPAN
#define WIFI_COUNTRY_CC    "JP"
#define WIFI_COUNTRY_NCHAN 13
#else
#define WIFI_COUNTRY_CC    "ZZ"
#define WIFI_COUNTRY_NCHAN 11
#endif

#define ID_OD_AUTH_DATUM  1546300800LU

//

#include "utm.h"

#include "opendroneid.h"

//
// Functions in a processor specific file.
//

void     construct2(void);
void     init2(char *,int,uint8_t *,uint8_t);
uint8_t *capability(void);
int      tag_rates(uint8_t *,int);
int      tag_ext_rates(uint8_t *,int);
int      misc_tags(uint8_t *,int);
int      transmit_wifi2(uint8_t *,int);
int      transmit_ble2(uint8_t *,int);

//

class ID_OpenDrone {

public:
           ID_OpenDrone();
  void     init(struct UTM_parameters *);
  void     set_self_id(char *);
  void     set_auth(char *);
  void     set_auth(uint8_t *,short int,uint8_t);
  int      transmit(struct UTM_data *);
#if ID_NATIONAL
  void     init_national(struct UTM_parameters *);
  void     auth_key_national(uint8_t *,int,uint8_t *,int);
#endif

private:

  void     init_beacon(void);
#if ID_NATIONAL
  int      pack_encrypt_national(uint8_t *);
#endif
  int      transmit_wifi(struct UTM_data *,int);
  int      transmit_ble(uint8_t *,int);

  int                     auth_page = 0, auth_page_count = 0, key_length = 0, iv_length = 0;
  char                   *UAS_operator;
  uint8_t                 msg_counter[16];
  uint16_t                wifi_interval = 0, ble_interval = 0;
  Stream                 *Debug_Serial = NULL;

  char                    ssid[32];
  size_t                  ssid_length = 0;
  uint8_t                 WiFi_mac_addr[6], wifi_channel = WIFI_CHANNEL,
                         *auth_key = NULL, *auth_iv = NULL;
#if ID_OD_WIFI
  uint16_t                sequence = 1, beacon_interval = 0x200;
#if ID_OD_WIFI_BEACON
  int                     beacon_offset = 0, beacon_max_packed = 30;
  uint8_t                 beacon_frame[BEACON_FRAME_SIZE],
#if USE_BEACON_FUNC
                          beacon_counter = 0;
#else
                         *beacon_payload, *beacon_timestamp, *beacon_counter, *beacon_length, *beacon_seq;
#endif
#endif
#endif

#if ID_OD_BT
  uint8_t                 ble_message[36], counter = 0;
#endif

  ODID_UAS_Data           UAS_data;
  ODID_BasicID_data      *basicID_data;
  ODID_Location_data     *location_data;
  ODID_Auth_data         *auth_data[ODID_AUTH_MAX_PAGES];
  ODID_SelfID_data       *selfID_data;
  ODID_System_data       *system_data;
  ODID_OperatorID_data   *operatorID_data;

  ODID_BasicID_encoded    basicID_enc[2];
  ODID_Location_encoded   location_enc;
  ODID_Auth_encoded       auth_enc;
  ODID_SelfID_encoded     selfID_enc;
  ODID_System_encoded     system_enc;
  ODID_OperatorID_encoded operatorID_enc;
};

#endif

/*
 *
 */

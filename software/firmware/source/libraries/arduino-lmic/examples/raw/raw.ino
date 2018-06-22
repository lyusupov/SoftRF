/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2017-2018 Linar Yusupov
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "lib_crc.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#endif

#if defined(ESP32)
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>

#include <protocol.h>

// Expose Espressif SDK functionality - wrapped in ifdef so that it still
// compiles on other platforms
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

const char* ssid = "";
const char* password = "";

#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif

#define DO_FREQ_SWEEP

#if defined(DO_FREQ_SWEEP)
#include <TimeLib.h>
#define SWEEP_START 810000000 /* 810 MHz */
#define NTP_TIME_SYNC
//#define RECEIVE_ONLY
#endif

#define  USE_P3I
//#define  USE_FANET
//#define  USE_LEGACY
//#define  USE_OGNTP


// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc 
#define TX_INTERVAL 200 /* 2000 */

// Pin mapping
#if defined(ESP8266)
const lmic_pinmap lmic_pins = {
    .nss = /* 15 */ D8 ,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = LMIC_UNUSED_PIN,
    .dio = {/* D0 */ LMIC_UNUSED_PIN , LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};
#endif

#if defined(ESP32)

#define SOC_GPIO_PIN_MOSI     27
#define SOC_GPIO_PIN_MISO     19
#define SOC_GPIO_PIN_SCK      5
#define SOC_GPIO_PIN_SS       18
#define SOC_GPIO_PIN_DIO0     26
#define SOC_GPIO_PIN_RST      14

#define LED_BUILTIN           2 /* DoIt */

const lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = { /* SOC_GPIO_PIN_DIO0 */ LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};
#endif

#if defined(NTP_TIME_SYNC)
extern void Time_setup(void);
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);

#if defined(USE_P3I)   /* P3I */
unsigned char tx_data[] = {
  0x24, 0x81, 0x47, 0x37, 0x9a, 0x99, 0x1b, 0x42, 0x00, 0x00, 0x62, 0x42,
  0x8a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3a
};
#elif defined(USE_FANET) /* FANET */
unsigned char tx_data[] = {
  0x41, 0xAA, 0xBC, 0xBB, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x64, 0x00, 0x14, 0x00, 0x00
};
#elif defined(USE_LEGACY) /* LEGACY */
unsigned char tx_data[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#elif defined(USE_OGNTP) /* OGNTP */
unsigned char tx_data[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#else
#error "RF protocol is not defined"
#endif

#define LEGACY_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_55
#define LEGACY_PREAMBLE_SIZE   1
/*  IEEE Manchester(F531FAB6) = 55 99 A5 A9 55 66 65 96 */
#define LEGACY_SYNCWORD        {0x55, 0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96}
#define LEGACY_SYNCWORD_SIZE   8
#define LEGACY_PAYLOAD_SIZE    24
#define LEGACY_CRC_TYPE        RF_CHECKSUM_TYPE_CCITT_FFFF
#define LEGACY_CRC_SIZE        2

#define OGNTP_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define OGNTP_PREAMBLE_SIZE   1 /* Warmup: 6 bits, preamble: 8 bits, value:  0xAA */
/* IEEE  Manchester(0AF3656C) = AA 66 55 A5 96 99 96 5A */
#define OGNTP_SYNCWORD        {0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A}
#define OGNTP_SYNCWORD_SIZE   8
#define OGNTP_PAYLOAD_SIZE    20
#define OGNTP_CRC_TYPE        RF_CHECKSUM_TYPE_GALLAGER
#define OGNTP_CRC_SIZE        6

#define P3I_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_55
#define P3I_PREAMBLE_SIZE   5
#define P3I_SYNCWORD        {0x2d, 0xd4}
#define P3I_SYNCWORD_SIZE   2
#define P3I_NET_ID          0x00000000
#define P3I_PAYLOAD_SIZE    24
#define P3I_PAYLOAD_OFFSET  5
#define P3I_CRC_TYPE        RF_CHECKSUM_TYPE_CCITT_0000
#define P3I_CRC_SIZE        2

#define FANET_PAYLOAD_SIZE    15

const rf_proto_desc_t legacy_proto_desc  = {
  .type             = RF_PROTOCOL_LEGACY,
  .modulation_type  = RF_MODULATION_TYPE_2FSK,
  .preamble_type    = LEGACY_PREAMBLE_TYPE,
  .preamble_size    = LEGACY_PREAMBLE_SIZE,
  .syncword         = LEGACY_SYNCWORD,
  .syncword_size    = LEGACY_SYNCWORD_SIZE,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_INVERTED,
  .payload_size     = LEGACY_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = LEGACY_CRC_TYPE,
  .crc_size         = LEGACY_CRC_SIZE,

  .bitrate          = RF_BITRATE_100KBPS,
  .deviation        = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening        = RF_WHITENING_MANCHESTER,
  .bandwidth        = RF_RX_BANDWIDTH_SS_125KHZ
};

const rf_proto_desc_t ogntp_proto_desc = {
  .type             = RF_PROTOCOL_OGNTP,
  .modulation_type  = RF_MODULATION_TYPE_2FSK,
  .preamble_type    = OGNTP_PREAMBLE_TYPE,
  .preamble_size    = OGNTP_PREAMBLE_SIZE,
  .syncword         = OGNTP_SYNCWORD,
  .syncword_size    = OGNTP_SYNCWORD_SIZE,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_INVERTED,
  .payload_size     = OGNTP_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = OGNTP_CRC_TYPE,
  .crc_size         = OGNTP_CRC_SIZE,

  .bitrate          = RF_BITRATE_100KBPS,
  .deviation        = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening        = RF_WHITENING_MANCHESTER,
  .bandwidth        = RF_RX_BANDWIDTH_SS_125KHZ
};

const rf_proto_desc_t p3i_proto_desc  = {
  .type             = RF_PROTOCOL_P3I,
  .modulation_type  = RF_MODULATION_TYPE_2FSK,
  .preamble_type    = P3I_PREAMBLE_TYPE,
  .preamble_size    = P3I_PREAMBLE_SIZE,
  .syncword         = P3I_SYNCWORD,
  .syncword_size    = P3I_SYNCWORD_SIZE,
  .net_id           = P3I_NET_ID,
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = P3I_PAYLOAD_SIZE,
  .payload_offset   = P3I_PAYLOAD_OFFSET,
  .crc_type         = P3I_CRC_TYPE,
  .crc_size         = P3I_CRC_SIZE,

  .bitrate          = RF_BITRATE_38400,
  .deviation        = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening        = RF_WHITENING_NICERF,
  .bandwidth        = RF_RX_BANDWIDTH_SS_100KHZ
};

const rf_proto_desc_t fanet_proto_desc = {
  .type             = RF_PROTOCOL_FANET,
  .modulation_type  = RF_MODULATION_TYPE_LORA,
  .preamble_type    = 0 /* INVALID FOR LORA */,
  .preamble_size    = 0 /* INVALID FOR LORA */,
//  .syncword         = { 0x12 },  // sx127x default value, valid for FANET
  .syncword       = { 0xF1 },  // FANET+
  .syncword_size    = 1,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = FANET_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = RF_CHECKSUM_TYPE_NONE,
  .crc_size         = 0 /* INVALID FOR LORA */,

  .bitrate          = DR_SF7B,
  .deviation        = 0 /* INVALID FOR LORA */,
  .whitening        = RF_WHITENING_NONE,
  .bandwidth        = 0 /* INVALID FOR LORA */
};

// every row represents a parity check to be performed on the received codeword
static const uint32_t LDPC_ParityCheck_n208k160[48][7]
= { // parity check vectors: 48 vectors for 48 parity checks
    // Eaech vector applied to the data packet should yield even number of bits
 { 0x00000805, 0x00000020, 0x04000000, 0x20000000, 0x00000040, 0x00044020, 0x00000000 },
 { 0x00000001, 0x00800800, 0x00000000, 0x00000000, 0x00000000, 0x10010000, 0x00008C98 },
 { 0x00004001, 0x01000080, 0x80000400, 0x00000000, 0x08000200, 0x00200000, 0x00000005 },
 { 0x00000101, 0x20000200, 0x00000022, 0x00000000, 0x00000000, 0xCC008000, 0x00005002 },
 { 0x00000401, 0x00000000, 0x00004900, 0x00000020, 0x00000000, 0x20C00349, 0x00000020 },
 { 0x03140001, 0x00000002, 0x00000000, 0x40000001, 0x41534100, 0x00102C00, 0x00002000 },
 { 0x04008800, 0x82000642, 0x00000000, 0x00000020, 0x88040020, 0x03000010, 0x00000400 },
 { 0x00000802, 0x20000000, 0x02000014, 0x01200000, 0x04000403, 0x00800004, 0x0000A004 },
 { 0x02020820, 0x00000000, 0x80020820, 0x10190040, 0x30000000, 0x00000002, 0x00000900 },
 { 0x40804950, 0x00090000, 0x00000000, 0x00021204, 0x40001000, 0x10001100, 0x00000000 },
 { 0x08000A00, 0x00020008, 0x00040000, 0x02400010, 0x01002000, 0x40280280, 0x00000010 },
 { 0x00000000, 0x00008010, 0x118000A0, 0x00040080, 0x01000084, 0x00040100, 0x00000444 },
 { 0x20040108, 0x18000000, 0x08608800, 0x0000000A, 0x08000010, 0x00040080, 0x00008000 },
 { 0x00004080, 0x00422201, 0x00010000, 0x0000A400, 0x00400800, 0x00840000, 0x00000800 },
 { 0x00000000, 0x60200000, 0x80100240, 0x08000021, 0x02800000, 0x100C0000, 0x00000000 },
 { 0x00001000, 0x01010002, 0x00082001, 0x04000000, 0x00000001, 0x00040002, 0x00004030 },
 { 0x00002300, 0x04000000, 0xA0080000, 0x20004000, 0x00028000, 0x00800000, 0x00000400 },
 { 0x00004000, 0x00104100, 0x40041028, 0x24000020, 0x00200000, 0x00100000, 0x00008000 },
 { 0x08011000, 0x20040000, 0x00000000, 0xA0800000, 0x08090000, 0x00000100, 0x00000A00 },
 { 0x10180000, 0x00000204, 0x00002800, 0x20400800, 0x00000000, 0x10000000, 0x00000004 },
 { 0x00000000, 0xC0000000, 0x10200000, 0x20028000, 0x20000000, 0x80000008, 0x00002011 },
 { 0x82004000, 0x20000000, 0x04202000, 0x00000000, 0x00000000, 0x00020200, 0x00000400 },
 { 0x08600000, 0x00001200, 0x94000000, 0x00000000, 0x40000008, 0x00000000, 0x00008020 },
 { 0x04040000, 0x04010000, 0x04100000, 0x00000100, 0x00200000, 0x40000008, 0x00000804 },
 { 0x00000200, 0x00000110, 0x04000100, 0x00000000, 0x28400400, 0x10000000, 0x00004000 },
 { 0x00080000, 0x00000080, 0x04001000, 0x01882007, 0x00008024, 0x04000001, 0x00000010 },
 { 0x20200000, 0x00000020, 0x00010040, 0x81000800, 0x10001000, 0x00300008, 0x00004400 },
 { 0x90000010, 0x89841021, 0x00000118, 0x08080000, 0x00020000, 0x40000000, 0x00000040 },
 { 0x04C20000, 0x10404034, 0x00000000, 0x00004000, 0x00810001, 0x04000200, 0x00000009 },
 { 0x40102000, 0x020020A0, 0x40100000, 0x00100080, 0x00080400, 0x80030080, 0x00000020 },
 { 0x00010000, 0x04020920, 0x00000200, 0x00060000, 0x00000218, 0x01002007, 0x00001000 },
 { 0x00020008, 0x00A08040, 0x00080000, 0x40001400, 0x04200040, 0x80200001, 0x00000200 },
 { 0x40000402, 0x01100000, 0x20808000, 0x00008000, 0x10100060, 0x00080000, 0x00001008 },
 { 0x200010A0, 0x00000000, 0x01040100, 0x00000104, 0x02040042, 0x08012000, 0x00000001 },
 { 0x01000000, 0x50000880, 0x00000092, 0x14400000, 0x00001840, 0x02400000, 0x00000000 },
 { 0x00000010, 0x02000000, 0x00014000, 0x00200018, 0x00000240, 0x04000800, 0x00000180 },
 { 0x00008000, 0x00880008, 0x08000044, 0x00100000, 0x00000004, 0x00400820, 0x00001001 },
 { 0x01000000, 0x00002000, 0x02004001, 0x00000042, 0x00000000, 0x09201020, 0x00000048 },
 { 0x00800000, 0x01000400, 0x00400002, 0xC0002000, 0x00002080, 0x00010064, 0x00000100 },
 { 0x00000400, 0x08400840, 0x00000400, 0x00000890, 0x00008102, 0x00000020, 0x00000002 },
 { 0x00200040, 0x00000081, 0x00000000, 0x02050000, 0x04940000, 0x20008020, 0x00000080 },
 { 0x00000404, 0x00800000, 0x00001000, 0x00014000, 0x00082200, 0x0A000400, 0x00000000 },
 { 0x0000A024, 0x00000000, 0x00000402, 0x08A01000, 0x00004010, 0x20000000, 0x00000008 },
 { 0x00480046, 0x00008000, 0x00000208, 0x00000048, 0x00000000, 0x00410010, 0x00000002 },
 { 0x0000008C, 0x00044C00, 0x00824004, 0x00000200, 0x00000000, 0x00028000, 0x00000000 },
 { 0x10010004, 0x00080000, 0x43008000, 0x10000400, 0x80000100, 0x00000040, 0x00000080 },
 { 0x80000000, 0x0020000C, 0x20420480, 0x00000100, 0x00000008, 0x00005410, 0x00000080 },
 { 0x00000000, 0x00101000, 0x08000001, 0x02000200, 0x82004A80, 0x00004000, 0x00000202 }
} ;

const uint8_t ByteCount1s[256] = {
 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
} ;

inline uint8_t Count1s(uint8_t Byte) { return ByteCount1s[Byte]; }

uint8_t LDPC_Check(const uint8_t *Data) // 20 data bytes followed by 6 parity bytes
{ uint8_t Errors=0;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint8_t *Check = (uint8_t *)LDPC_ParityCheck_n208k160[Row];
    for(uint8_t Idx=0; Idx<26; Idx++)
    { uint8_t And = Data[Idx]&Check[Idx]; Count+=Count1s(And); }
    if(Count&1) Errors++; }
  return Errors; }

const uint8_t whitening_pattern[] = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
  0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
  0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
  0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
  0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };

void tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u2_t crc16;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;
    break;
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (u1_t i=0; i < size; i++) {

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[LMIC.dataLen] = buf[i] ^ whitening_pattern[i];
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      LMIC.frame[LMIC.dataLen] = buf[i];
      break;
    }

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    }

    LMIC.dataLen++;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
    LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;
    break;
  }

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
#if !defined(DO_FREQ_SWEEP)
  Serial.println("TX");
#endif
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX for LoRa only (e.g. without a timeout,
  // still stops after receiving a packet)
  os_radio(LMIC.protocol &&
           LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
          RADIO_RXON : RADIO_RX);
#if !defined(DO_FREQ_SWEEP)
  Serial.println("RX");
#endif
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {

  u2_t crc16;
  u1_t i;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

#if !defined(DO_FREQ_SWEEP)
  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);
#endif

#if !defined(DO_FREQ_SWEEP)
  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
#endif

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (i = LMIC.protocol->payload_offset;
       i < (LMIC.dataLen - LMIC.protocol->crc_size);
       i++)
  {

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
      break;
    }

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[i] ^= whitening_pattern[i - LMIC.protocol->payload_offset];
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      break;
    }
#if !defined(DO_FREQ_SWEEP)
    Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) &LMIC.frame[0])) {
#if !defined(DO_FREQ_SWEEP)
      Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FCS",
        LMIC.frame[i], LMIC.frame[i+1], LMIC.frame[i+2],
        LMIC.frame[i+3], LMIC.frame[i+4], LMIC.frame[i+5]);
#endif
    };
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    u2_t pkt_crc = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
#if !defined(DO_FREQ_SWEEP)
    if (crc16 == pkt_crc ) {
      Serial.printf(" %04x is valid crc", pkt_crc);
    } else {
      Serial.printf(" %04x is wrong crc", pkt_crc);
    }
#endif
    break;
  }

#if 0
  int rssi = LMIC.rssi ; // (LMIC.rssi - 64 + 125) - 157;
	uint8_t value = LMIC.snr;	// 0x19;
  int8_t snr;

  if( value & 0x80 ) {                      // The SNR sign bit is 1
    value = ( ( ~value + 1 ) & 0xFF ) >> 2; // Invert and divide by 4
    snr = -value;
  } else {
    // Divide by 4
    snr = ( value & 0xFF ) >> 2;
  }
#endif

#if !defined(DO_FREQ_SWEEP)
  Serial.printf(" RSSI: %d SNR: %d ", LMIC.rssi, LMIC.snr);
  Serial.println();
#else
  Serial.printf("RX FREQ: %d RSSI: %d SNR: %d ", LMIC.freq / 1000000, LMIC.rssi, LMIC.snr);
  Serial.println();
#endif

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  rx(rx_func);
}

#if !defined(DO_FREQ_SWEEP)
// log text to USART and toggle LED
static void tx_func (osjob_t* job) {

  tx(tx_data, sizeof(tx_data), txdone_func);

  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}
#else
static void tx_func (osjob_t* job) {

  LMIC.freq = SWEEP_START + ((minute() & 1) * 60 + second()) * 1000000;

#if !defined(RECEIVE_ONLY)
  Serial.printf("TX FREQ: %d", LMIC.freq / 1000000);
  Serial.println();

  tx(tx_data, sizeof(tx_data), txdone_func);
#else
  rx(rx_func);
#endif

  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL), tx_func);
}
#endif

// application entry point
void setup() {

  Serial.begin(38400);
  Serial.println("Starting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  #if defined(NTP_TIME_SYNC)
  Time_setup();
  #endif

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  os_init();

#if defined(USE_OGNTP)
  LMIC.protocol = &ogntp_proto_desc;
#elif defined(USE_P3I)
  LMIC.protocol = &p3i_proto_desc;
#elif defined(USE_LEGACY)
  LMIC.protocol = &legacy_proto_desc;
#elif defined(USE_FANET)
  LMIC.protocol = &fanet_proto_desc;
#else
#error "RF protocol is not defined"
#endif

  // Set up these settings once, and use them for both TX and RX

#if defined(CFG_eu868)

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_P3I:
    LMIC.freq = 869920000; /* 869525000 */
    break;
  case RF_PROTOCOL_FANET:
    LMIC.freq = 868200000;
    break;
  case RF_PROTOCOL_LEGACY:
  case RF_PROTOCOL_OGNTP:
  default:
    LMIC.freq = 868400000;
    break;
  }

#elif defined(CFG_us915)
  LMIC.freq = 902300000;
#endif

  // Maximum TX power
//  LMIC.txpow = 27;
//  LMIC.txpow = 15;
//  LMIC.txpow = 5;
  LMIC.txpow = 2;

  if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA) {
    LMIC.datarate = LMIC.protocol->bitrate;
  } else {
    LMIC.datarate = DR_FSK;
  }

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET) {
    /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
    setCr(LMIC.rps, CR_4_8);
  }

  Serial.println("Started");
  Serial.flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);
}

void loop() {
  // execute scheduled jobs and events
  os_runloop_once();

  ArduinoOTA.handle();
}

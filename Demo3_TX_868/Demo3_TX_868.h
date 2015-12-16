#ifndef SOFTRF_DEMO3_H
#define SOFTRF_DEMO3_H

#include <nRF905.h>

#define RF_FREQ   NRF905_FREQ
#define PKT_SIZE  NRF905_PAYLOAD_SIZE

#define RXADDR {0x31, 0xfa , 0xb6} // Address of this device (4 bytes)
#define TXADDR {0x31, 0xfa , 0xb6} // Address of device to send to (4 bytes)

#define TIMEOUT   1000 // 1 second ping timeout

#define RRB_SIZE  10

#define LATITUDE    43.21
#define LONGTITUDE  5.43
#define ALTITUDE    12

#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define ARGUS_HOSTNAME "192.168.157.129"
#define ARGUS_PORT  7777

#define XCSOAR_HOSTNAME "192.168.157.129" // "192.168.157.248"
#define XCSOAR_PORT 10110

#define CLOUD_HOSTNAME "192.168.157.129" // glidern1.glidernet.org
#define CLOUD_PORT 7 /* echo TCP/IP service to test response */ // 14580
#define CLOUD_MODE  0

#define STATION_ID  "EGHL"

typedef struct UFO {
    String    raw;
    time_t    timestamp;

    uint32_t  addr;
    float     latitude;
    float     longtitude;
    int32_t   altitude;
    unsigned int type;

    int32_t   vs;
    float     distance;

    bool      stealth;
    bool      no_track;

    int8_t ns[4];
    int8_t ew[4];
} ufo_t;

enum tx_state {
  TX_CLEAR,
  TX_DATA_READY,
  TX_SENT
};

#endif /* SOFTRF_DEMO3_H */


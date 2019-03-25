#ifndef GDL90_H_
#define GDL90_H_

#if defined(RASPBERRY_PI)
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#elif defined(ARDUINO)
#include <Arduino.h>
#endif /* RASPBERRY_PI || ARDUINO */

#include "portable_endian.h"

#define retrieve_4bits_upper(data, start_idx) (((uint8_t)data[start_idx] >> 4) & 0x0F)
#define retrieve_4bits_lower(data, start_idx) ((uint8_t)data[start_idx] & 0x0F)
#define retrieve_8bits(data, start_idx) (data[start_idx])
#define retrieve_12bits_upper(data, start_idx) ((data[start_idx]<<4)+(data[start_idx+1]>>4))
#define retrieve_12bits_lower(data, start_idx) (((data[start_idx] & 0x0F)<<8)+data[start_idx+1])
#define retrieve_12bits_lower_s(data, start_idx) (((int16_t)(retrieve_12bits_lower(data, start_idx)<<4))>>4)
#define retrieve_24bits(data, start_idx) ((data[start_idx]<<16) + (data[start_idx+1]<<8) + (data[start_idx+2]))
#define retrieve_24bits_s(data, start_idx) (((int32_t)(retrieve_24bits(data, start_idx)<<8))>>8)

#define GDL90_COUNTS_TO_DEGREES             (180.0f/pow(2, 23))
#define GDL90_ALTITUDE_FACTOR               (25.0f)
#define GDL90_ALTITUDE_OFFSET               (-1000.0f)
#define GDL90_HORZ_VELOCITY_FACTOR          (1.0f)
#define GDL90_VERT_VELOCITY_FACTOR          (64.0f)
#define GDL90_COUNTS_TO_HEADING             (360.0f/256.0f)

#define GDL90_GEO_ALTITUDE_FACTOR           (5.0f)


#define GDL90_DECODE_TRAFFIC_ALERT(msg)     (retrieve_4bits_upper(msg, 0))
#define GDL90_DECODE_ADDRESS_TYPE(msg)      (retrieve_4bits_lower(msg, 0))
#define GDL90_DECODE_ADDRESS(msg)           (retrieve_24bits(msg, 1))
#define GDL90_DECODE_ALTITUDE(msg)          ((retrieve_12bits_upper(msg, 10)*GDL90_ALTITUDE_FACTOR)+GDL90_ALTITUDE_OFFSET)
#define GDL90_DECODE_LATITUDE(msg)          (retrieve_24bits_s(msg, 4)*GDL90_COUNTS_TO_DEGREES)
#define GDL90_DECODE_LONGITUDE(msg)         (retrieve_24bits_s(msg, 7)*GDL90_COUNTS_TO_DEGREES)
#define GDL90_DECODE_AIRBORNE(msg)          ((retrieve_4bits_lower(msg, 11) & 0b00001000) >> 3)
#define GDL90_DECODE_REPORT_TYPE(msg)       ((retrieve_4bits_lower(msg, 11) & 0b00000100) >> 2)
#define GDL90_DECODE_HEADING_TRACK_TYPE(msg) (retrieve_4bits_lower(msg, 11) & 0b00000011)
#define GDL90_DECODE_NIC(msg)               (retrieve_4bits_upper(msg, 12))
#define GDL90_DECODE_NACP(msg)              (retrieve_4bits_lower(msg, 12))
#define GDL90_DECODE_HORZ_VELOCITY(msg)     ((float)retrieve_12bits_upper(msg, 13)*GDL90_HORZ_VELOCITY_FACTOR)
#define GDL90_DECODE_VERT_VELOCITY(msg)     ((float)retrieve_12bits_lower_s(msg, 14)*GDL90_VERT_VELOCITY_FACTOR)  // Vert velocity factor of 64fpm/count
#define GDL90_DECODE_HEADING(msg)           ((float)retrieve_8bits(msg, 16)*GDL90_COUNTS_TO_HEADING)
#define GDL90_DECODE_EMITTER_CATEGORY(msg)  (retrieve_8bits(msg, 17))
#define GDL90_DECODE_EMERGENCY_CODE(msg)    (retrieve_4bits_upper(msg, 26))
#define GDL90_DECODE_CALLSIGN_START_IDX     (18)
#define GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE (8)

// These are the lengths of the payloads, so doesn't include frame, message ID, or CRC
#define GDL90_MSG_LEN_HEARTBEAT             (6)
#define GDL90_MSG_LEN_OWNSHIP_REPORT        (27)
#define GDL90_MSG_LEN_TRAFFIC_REPORT        GDL90_MSG_LEN_OWNSHIP_REPORT
#define GDL90_MSG_LEN_OWNSHIP_GEOMETRIC     (4)
#define GDL90_MSG_LEN_SHORT_UAT             (21)
#define GDL90_MSG_LEN_LONG_UAT              (37)
#define GDL90_MSG_LEN_UPLINK_DATA           (435)

#define GDL90_FLAG_BYTE         0x7E
#define GDL90_CONTROL_ESCAPE    0x7D
#define GDL90_ESCAPE_BYTE       0x20

#define GDL90_UPLINK_PAYLOAD_SIZE           (432)
#define GDL90_SHORT_UAT_PAYLOAD_SIZE        (18)
#define GDL90_LONG_UAT_PAYLOAD_SIZE         (34)

typedef enum {
    NO_ALERT        = 0,
    TRAFFIC_ALERT   = 1
    // Values 2-15 are reserved
} traffic_alert_status_t;

typedef enum {
    ADS_B_WITH_ICAO_ADDRESS     = 0,
    ADS_B_WITH_SELF_ASSIGNED    = 1,
    TIS_B_WITH_ICAO_ADDRESS     = 2,
    TIS_B_WITH_TRACK_ID         = 3,
    SURFACE_VEHICLE             = 4,
    GROUND_STATION_BEACON       = 5
    // 6-15 are reserved
} address_type_t;

typedef enum {
    REPORT_UPDATED          = 0,
    REPORT_EXTRAPOLATED     = 1
} traffic_report_type_t;

typedef enum {
    TT_TYPE_INVALID         = 0,
    TT_TYPE_TRUE_TRACK      = 1,
    TT_TYPE_MAG_HEADING     = 2,
    TT_TYPE_TRUE_HEADING    = 3
} heading_or_track_type_t;

typedef enum {
    NIC_UNKNOWN                 = 0,
    NIC_LESS_20NM               = 1,
    NIC_LESS_8NM                = 2,
    NIC_LESS_4NM                = 3,
    NIC_LESS_2NM                = 4,
    NIC_LESS_1NM                = 5,
    NIC_LESS_0_6NM              = 6,
    NIC_LESS_0_2NM              = 7,
    NIC_LESS_0_1NM              = 8,
    NIC_HPL_75M_AND_VPL_112M    = 9,
    NIC_HPL_25M_AND_VPL_37M     = 10,
    NIC_HPL_7M_AND_VPL_11M      = 11
    // 12-15 are unused
} nic_t;

typedef enum {
    NACP_UNKNOWN                = 0,
    NACP_LESS_10NM              = 1,
    NACP_LESS_4NM               = 2,
    NACP_LESS_2NM               = 3,
    NACP_LESS_1NM               = 4,
    NACP_LESS_0_5NM             = 5,
    NACP_LESS_0_3NM             = 6,
    NACP_LESS_0_1NM             = 7,
    NACP_LESS_0_05NM            = 8,
    NACP_HFOM_30M_AND_VFOM_45M  = 9,
    NACP_HFOM_10M_AND_VFOM_15M  = 10,
    NACP_HFOM_3M_AND_VFOM_4M    = 11
    // 12-15 are unused
} nacp_t;

typedef enum {
    EMIITER_NO_INFO         = 0,
    EMITTER_LIGHT           = 1,
    EMITTER_SMALL           = 2,
    EMITTER_LARGE           = 3,
    EMITTER_HIGH_VORTEX     = 4,
    EMITTER_HEAVY           = 5,
    EMITTER_HIGH_MANUEVER   = 6,
    EMITTER_ROTORCRAFT      = 7,
    //EMITTER_UNASSIGNED8     = 8,
    EMITTER_GLIDER          = 9,
    EMITTER_LIGHTER_THAN_AIR= 10,
    EMITTER_PARACHUTIST     = 11,
    EMITTER_ULTRA_LIGHT     = 12,
    //EMITTER_UNASSIGNED13    = 13,
    EMITTER_UAV             = 14,
    EMITTER_SPACE           = 15,
    //EMITTER_UNASSIGNED16    = 16,
    EMITTER_SURFACE_EMERG   = 17,
    EMITTER_SURFACE_SERVICE = 18,
    EMITTER_POINT_OBSTACLE  = 19,
    EMITTER_CLUSTER_OBST    = 20,
    EMITTER_LINE_OBSTACLE   = 21
    //22-39 are reserved
} emitter_category_t;

typedef enum {
    EMERGENCY_NONE          = 0,
    EMERGENCY_GENERAL       = 1,
    EMERGENCY_MEDICAL       = 2,
    EMERGENCY_MIN_FUEL      = 3,
    EMERGENCY_NO_COMM       = 4,
    EMERGENCY_UNLAWFUL_INT  = 5,
    EMERGENCY_DOWNED        = 6
    // 7-15 are reserved
} emergency_code_t;


typedef struct {
    traffic_alert_status_t trafficAlertStatus;
    address_type_t addressType;

    uint32_t address;

    float latitude;
    float longitude;
    float altitude;

    bool airborne;
    traffic_report_type_t reportType;
    heading_or_track_type_t ttType;

    nic_t nic;
    nacp_t nacp;

    float horizontalVelocity;
    float verticalVelocity;
    float trackOrHeading;
    emitter_category_t emitterCategory;
    uint8_t callsign[GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE];

    emergency_code_t emergencyCode;
} gdl90_msg_traffic_report_t;

typedef struct {
    float ownshipGeoAltitude;
    bool verticalWarningIndicator;
    float verticalFigureOfMerit;
} gdl90_msg_ownship_geo_altitude;

typedef struct {
    bool gpsPosValid;
    bool maintReq;
    bool ident;
    bool addrType;
    bool gpsBattLow;
    bool ratcs;
    bool uatInitialized;

    bool csaRequested;
    bool csaNotAvailable;
    bool utcOK;

    uint32_t timestamp;
    uint16_t messageCounts;
} gdl90_msg_heartbeat;

typedef enum {
    MSG_ID_HEARTBEAT            = 0,
    MSG_ID_INIT                 = 2,
    MSG_ID_UPLINK_DATA          = 7,
    MSG_ID_HEIGHT_ABOVE_TERRAIN = 9,
    MSG_ID_OWNSHIP_REPORT       = 10,
    MSG_ID_OWNSHIP_GEOMETRIC    = 11,
    MSG_ID_TRAFFIC_REPORT       = 20,
    MSG_ID_BASIC_REPORT         = 30,
    MSG_ID_LONG_REPORT          = 31
} gdl_msg_id_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t flag0;          // Fixed, 0x7E
    uint8_t messageId;

    uint8_t data[GDL90_MSG_LEN_UPLINK_DATA + 3];  // Max data size is Uplink Data Message, plus CRC and flags
} gdl_message_t;
#pragma pack(pop)

typedef struct {
    uint16_t length;
    uint8_t data[GDL90_MSG_LEN_UPLINK_DATA*2];  // Give ourselves lots of breathing room for padding
} gdl_message_escaped_t;

void gdl90_crcInit();
uint16_t gdl90_crcCompute(uint8_t *block, uint32_t length);
bool gdl90_verifyCrc(gdl_message_t *rawMsg, uint32_t length);
void gdl90_insertCrc(gdl_message_t *rawMsg, uint32_t length);

void decode_gdl90_message(gdl_message_t *rawMsg);
void print_gdl90_traffic_report(gdl90_msg_traffic_report_t *decodedMsg);
bool decode_gdl90_traffic_report(gdl_message_t *rawMsg, gdl90_msg_traffic_report_t *decodedMsg);
void encode_gdl90_traffic_report(gdl_message_t *rawMsg, gdl90_msg_traffic_report_t *decodedMsg);
bool decode_gdl90_ownship_geo_altitude(gdl_message_t *rawMsg, gdl90_msg_ownship_geo_altitude *decodedMsg);
void encode_gdl90_ownship_geo_altitude(gdl_message_t *rawMsg, gdl90_msg_ownship_geo_altitude *decodedMsg);
void print_gdl90_ownship_geo_altitude(gdl90_msg_ownship_geo_altitude *decodedMsg);
bool decode_gdl90_heartbeat(gdl_message_t *rawMsg, gdl90_msg_heartbeat *decodedMsg);
void print_gdl90_heartbeat(gdl90_msg_heartbeat *decodedMsg);
void encode_gdl90_heartbeat(gdl_message_t *rawMsg, gdl90_msg_heartbeat *decodedMsg);
void encode_gdl90_uplink_data(gdl_message_t *rawMsg, uint8_t *payload, uint16_t payload_size);
void encode_gdl90_basic_uat_report(gdl_message_t *rawMsg, uint8_t *payload, uint8_t payload_size);
void encode_gdl90_long_uat_report(gdl_message_t *rawMsg, uint8_t *payload, uint8_t payload_size);
void gdl90_escape_message_for_tx(gdl_message_t *rawMsg, gdl_message_escaped_t *escapedMsg);

#endif  // GDL90_H_

#include "gdl90.h"

#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#endif

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <avr/pgmspace.h>
#endif

// Copied from the GDL90 ICD
#if !defined(ESP8266) && !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
uint16_t crc16table[256];
#else
uint16_t crc16table[256] PROGMEM = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
#endif

void decode_gdl90_message(gdl_message_t *rawMsg) {
    gdl90_msg_heartbeat heartbeatMsg;
    gdl90_msg_traffic_report_t trafficReportMsg;
    gdl90_msg_ownship_geo_altitude ownshipGeoAltitude;

    switch (rawMsg->messageId) {
        case(MSG_ID_HEARTBEAT):
            fprintf(stdout, "Processing Heartbeat Message\n");
            decode_gdl90_heartbeat(rawMsg, &heartbeatMsg);
            print_gdl90_heartbeat(&heartbeatMsg);
            break;

        case(MSG_ID_TRAFFIC_REPORT):
            fprintf(stdout, "Processing Traffic Report Message!\n");
            decode_gdl90_traffic_report(rawMsg, &trafficReportMsg);
            print_gdl90_traffic_report(&trafficReportMsg);
            break;

        case(MSG_ID_OWNSHIP_REPORT):
            fprintf(stdout, "Processing Ownship Report Message!\n");
            decode_gdl90_traffic_report(rawMsg, &trafficReportMsg);
            print_gdl90_traffic_report(&trafficReportMsg);
            break;

        case(MSG_ID_OWNSHIP_GEOMETRIC):
            fprintf(stdout, "Processing Ownship Geometric Altitude Message!\n");
            decode_gdl90_ownship_geo_altitude(rawMsg, &ownshipGeoAltitude);
            print_gdl90_ownship_geo_altitude(&ownshipGeoAltitude);
            break;

        default:
            fprintf(stdout, "Unknown message ID = %d!\n", rawMsg->messageId);
    }
}

void print_gdl90_traffic_report(gdl90_msg_traffic_report_t *decodedMsg) {
    // Try and replicate the contents in section 3.5.4 of the GDL90 ICD
    switch(decodedMsg->trafficAlertStatus) {
        case(NO_ALERT):
        fprintf(stdout, "No Traffic Alert\n");
        break;

        case(TRAFFIC_ALERT):
        default:
        fprintf(stdout, "Traffic Alert\n");
        break;
    }

    switch(decodedMsg->addressType){
        case(ADS_B_WITH_ICAO_ADDRESS):
        fprintf(stdout, "ICAO ADS-B ");
        break;

        case(ADS_B_WITH_SELF_ASSIGNED):
        fprintf(stdout, "Self-Assigned ADS-B ");
        break;

        case(TIS_B_WITH_ICAO_ADDRESS):
        fprintf(stdout, "ICAO TIS-B ");
        break;

        case(TIS_B_WITH_TRACK_ID):
        fprintf(stdout, "Track ID TIS-B ");
        break;

        case(SURFACE_VEHICLE):
        fprintf(stdout, "Surface Vehicle ");
        break;

        case(GROUND_STATION_BEACON):
        fprintf(stdout, "Ground Station ");
        break;

        default:
        fprintf(stdout, "Unknown ");
        break;
    }

    fprintf(stdout, "Address (octal): %o\n", decodedMsg->address);
    fprintf(stdout, "Latitude: %f\n", decodedMsg->latitude);
    fprintf(stdout, "Longitude: %f\n", decodedMsg->longitude);
    fprintf(stdout, "Pressure Altitude: %f\n", decodedMsg->altitude);

    if(decodedMsg->airborne == true) {
        fprintf(stdout, "Airborne with ");
    } else {
        fprintf(stdout, "Grounded with ");
    }

    switch(decodedMsg->ttType) {
        case(TT_TYPE_TRUE_TRACK):
        fprintf(stdout, "True Track\n");
        break;

        case(TT_TYPE_MAG_HEADING):
        fprintf(stdout, "Magnetic Heading\n");
        break;

        case(TT_TYPE_TRUE_HEADING):
        fprintf(stdout, "True Heading\n");
        break;

        case(TT_TYPE_INVALID):
        default:
        fprintf(stdout, "Invalid track/heading type\n");
    }

    switch(decodedMsg->reportType) {
        case(REPORT_UPDATED):
        fprintf(stdout, "Report Updated\n");
        break;

        case(REPORT_EXTRAPOLATED):
        fprintf(stdout, "Report Extrapolated\n");
        break;

        default:
        fprintf(stdout, "Report Unknown = %d\n", decodedMsg->reportType);
        break;
    }

    switch(decodedMsg->nic) {
        case(NIC_LESS_20NM):
        fprintf(stderr, "HPL = 20nm, ");
        break;

        case(NIC_LESS_8NM):
        fprintf(stderr, "HPL = 8nm, ");
        break;

        case(NIC_LESS_4NM):
        fprintf(stderr, "HPL = 4nm, ");
        break;
        
        case(NIC_LESS_2NM):
        fprintf(stderr, "HPL = 2nm, ");
        break;
        
        case(NIC_LESS_1NM):
        fprintf(stderr, "HPL = 1nm, ");
        break;
        
        case(NIC_LESS_0_6NM):
        fprintf(stderr, "HPL = 0.6nm, ");
        break;

        case(NIC_LESS_0_2NM):
        fprintf(stderr, "HPL = 0.2nm, ");
        break;

        case(NIC_LESS_0_1NM):
        fprintf(stderr, "HPL = 0.1nm, ");
        break;

        case(NIC_HPL_75M_AND_VPL_112M):
        fprintf(stderr, "HPL = 75m, ");
        break;

        case(NIC_HPL_25M_AND_VPL_37M):
        fprintf(stderr, "HPL = 25m, ");
        break;
        
        case(NIC_HPL_7M_AND_VPL_11M):
        fprintf(stderr, "HPL = 7m, ");
        break;
        
        case(NIC_UNKNOWN):
        default:
        fprintf(stderr, "HPL = Unknown, ");
        break;
    }

    switch(decodedMsg->nacp) {
        case(NACP_LESS_10NM):
        fprintf(stderr, "HFOM = 10nm ");
        break;

        case(NACP_LESS_4NM):
        fprintf(stderr, "HFOM = 4nm ");
        break;

        case(NACP_LESS_2NM):
        fprintf(stderr, "HFOM = 2nm ");
        break;

        case(NACP_LESS_0_5NM):
        fprintf(stderr, "HFOM = 0.5nm ");
        break;

        case(NACP_LESS_0_3NM):
        fprintf(stderr, "HFOM = 0.3nm ");
        break;

        case(NACP_LESS_0_1NM):
        fprintf(stderr, "HFOM = 0.1nm ");
        break;

        case(NACP_LESS_0_05NM):
        fprintf(stderr, "HFOM = 0.05nm ");
        break;

        case(NACP_HFOM_30M_AND_VFOM_45M):
        fprintf(stderr, "HFOM = 30m ");
        break;

        case(NACP_HFOM_10M_AND_VFOM_15M):
        fprintf(stderr, "HFOM = 10m ");
        break;

        case(NACP_HFOM_3M_AND_VFOM_4M):
        fprintf(stderr, "HFOM = 3m ");
        break;

        case(NACP_UNKNOWN):
        default:
        fprintf(stderr, "HFOM = Unknown ");
        break;
    }

    fprintf(stdout, "(NIC = %d, NACp = %d)\n", decodedMsg->nic, decodedMsg->nacp);

    fprintf(stdout, "Horizontal Velocity: %f knots at %f degrees ", decodedMsg->horizontalVelocity, decodedMsg->trackOrHeading);

    switch(decodedMsg->ttType) {
        case(TT_TYPE_TRUE_TRACK):
        fprintf(stdout, "(True Track)\n");
        break;

        case(TT_TYPE_MAG_HEADING):
        fprintf(stdout, "(Magnetic Heading)\n");
        break;

        case(TT_TYPE_TRUE_HEADING):
        fprintf(stdout, "(True Heading)\n");
        break;

        case(TT_TYPE_INVALID):
        default:
        fprintf(stdout, "(Invalid Heading)\n");
        break;
    }

    fprintf(stdout, "Vertical Velocity: %f FPM\n", decodedMsg->verticalVelocity);


    switch(decodedMsg->emergencyCode) {
        case(EMERGENCY_NONE):
        fprintf(stdout, "Emergency Code: None\n");
        break;

        case(EMERGENCY_GENERAL):
        fprintf(stdout, "Emergency Code: General\n");
        break;

        case(EMERGENCY_MEDICAL):
        fprintf(stdout, "Emergency Code: Medical\n");
        break;

        case(EMERGENCY_MIN_FUEL):
        fprintf(stdout, "Emergency Code: Min Fuel\n");
        break;

        case(EMERGENCY_NO_COMM):
        fprintf(stdout, "Emergency Code: No Comm\n");
        break;

        case(EMERGENCY_UNLAWFUL_INT):
        fprintf(stdout, "Emergency Code: Unlawful Interference\n");
        break;

        case(EMERGENCY_DOWNED):
        fprintf(stdout, "Emergency Code: Downed\n");
        break;

        default:
        fprintf(stdout, "Emergency Code: Invalid\n");
        break;
    }

    switch(decodedMsg->emitterCategory) {
        case(EMIITER_NO_INFO):
        fprintf(stdout, "Emitter/Category: No Info\n");
        break;

        case(EMITTER_LIGHT):
        fprintf(stdout, "Emitter/Category: Light\n");
        break;

        case(EMITTER_SMALL):
        fprintf(stdout, "Emitter/Category: Small\n");
        break;

        case(EMITTER_LARGE):
        fprintf(stdout, "Emitter/Category: Large\n");
        break;

        case(EMITTER_HIGH_VORTEX):
        fprintf(stdout, "Emitter/Category: High Vortex\n");
        break;

        case(EMITTER_HEAVY):
        fprintf(stdout, "Emitter/Category: Heavy\n");
        break;

        case(EMITTER_HIGH_MANUEVER):
        fprintf(stdout, "Emitter/Category: High Manueverability\n");
        break;

        case(EMITTER_ROTORCRAFT):
        fprintf(stdout, "Emitter/Category: Rotorcraft\n");
        break;

        case(EMITTER_GLIDER):
        fprintf(stdout, "Emitter/Category: Glider\n");
        break;

        case(EMITTER_LIGHTER_THAN_AIR):
        fprintf(stdout, "Emitter/Category: Lighter Than Air\n");
        break;

        case(EMITTER_PARACHUTIST):
        fprintf(stdout, "Emitter/Category: Parachutist\n");
        break;

        case(EMITTER_ULTRA_LIGHT):
        fprintf(stdout, "Emitter/Category: Ultra-light\n");
        break;

        case(EMITTER_UAV):
        fprintf(stdout, "Emitter/Category: UAV\n");
        break;

        case(EMITTER_SPACE):
        fprintf(stdout, "Emitter/Category: Space\n");
        break;

        case(EMITTER_SURFACE_EMERG):
        fprintf(stdout, "Emitter/Category: Surface Emergency\n");
        break;

        case(EMITTER_SURFACE_SERVICE):
        fprintf(stdout, "Emitter/Category: Surface Service\n");
        break;

        case(EMITTER_POINT_OBSTACLE):
        fprintf(stdout, "Emitter/Category: Point Obstacle\n");
        break;

        case(EMITTER_CLUSTER_OBST):
        fprintf(stdout, "Emitter/Category: Cluster Obstacle\n");
        break;

        case(EMITTER_LINE_OBSTACLE):
        fprintf(stdout, "Emitter/Category: Line Obstacle\n");
        break;

        default:
        fprintf(stdout, "Emitter/Category: Unknown\n");
        break;
    }

    fprintf(stdout, "Tail Number: ");
    for(int i=0; i < GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE; i++) {
        fprintf(stdout, "%c", decodedMsg->callsign[i]);
    }
    fprintf(stdout, "\n");
}

bool decode_gdl90_traffic_report(gdl_message_t *rawMsg, gdl90_msg_traffic_report_t *decodedMsg) {
    bool rval = gdl90_verifyCrc(rawMsg, GDL90_MSG_LEN_TRAFFIC_REPORT);

    decodedMsg->trafficAlertStatus = GDL90_DECODE_TRAFFIC_ALERT(rawMsg->data);
    decodedMsg->addressType = GDL90_DECODE_ADDRESS_TYPE(rawMsg->data);
    decodedMsg->address = GDL90_DECODE_ADDRESS(rawMsg->data);
    decodedMsg->latitude = GDL90_DECODE_LATITUDE(rawMsg->data);
    decodedMsg->longitude = GDL90_DECODE_LONGITUDE(rawMsg->data);
    decodedMsg->altitude = GDL90_DECODE_ALTITUDE(rawMsg->data);

    decodedMsg->airborne = GDL90_DECODE_AIRBORNE(rawMsg->data);
    decodedMsg->reportType = GDL90_DECODE_REPORT_TYPE(rawMsg->data);
    decodedMsg->ttType = GDL90_DECODE_HEADING_TRACK_TYPE(rawMsg->data);

    decodedMsg->nic = GDL90_DECODE_NIC(rawMsg->data);
    decodedMsg->nacp = GDL90_DECODE_NACP(rawMsg->data);

    decodedMsg->horizontalVelocity = GDL90_DECODE_HORZ_VELOCITY(rawMsg->data);
    decodedMsg->verticalVelocity = GDL90_DECODE_VERT_VELOCITY(rawMsg->data);
    decodedMsg->trackOrHeading = GDL90_DECODE_HEADING(rawMsg->data);
    decodedMsg->emitterCategory = GDL90_DECODE_EMITTER_CATEGORY(rawMsg->data);

    for (int i=0; i < GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE; i++) {
        decodedMsg->callsign[i] = rawMsg->data[GDL90_DECODE_CALLSIGN_START_IDX + i];
    }

    decodedMsg->emergencyCode = GDL90_DECODE_EMERGENCY_CODE(rawMsg->data);

    return rval;
}

void encode_gdl90_traffic_report(gdl_message_t *rawMsg, gdl90_msg_traffic_report_t *decodedMsg) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_TRAFFIC_REPORT;
    /*0-st  Traffic Alert Status, Address Type
    1-aa    Address
    2-aa    Address
    3-aa    Address
    4-ll    Latitude
    5-ll    Latitude
    6-ll    Latitude
    7-nn    Longitude
    8-nn    Longitude
    9-nn    Longitude
    10-dd   Altitude
    11-dm   Altitude, Misc
    12-ia   NIC, NACp
    13-hh   Horizontal Velocity
    14-hv   Horizontal Velocity, Vertical Velocity 
    15-vv   Vertical Velocity
    16-tt   Track/Heading
    17-ee   Emitter Category
    18-cc   Callsign
    19-cc   Callsign
    20-cc   Callsign
    21-cc   Callsign
    22-cc   Callsign
    23-cc   Callsign
    24-cc   Callsign
    25-cc   Callsign
    26-px   Emergency/Priority Code */

    // Traffic Alert Status, Address Type
    rawMsg->data[0]     = ((decodedMsg->trafficAlertStatus & 0x0F) << 4) + \
                          (decodedMsg->addressType & 0x0F);
    // Address
    rawMsg->data[1]     = (decodedMsg->address >> 16) & 0xFF;
    rawMsg->data[2]     = (decodedMsg->address >> 8) & 0xFF;
    rawMsg->data[3]     = decodedMsg->address & 0xFF;

    // Latitude
    int32_t convertedLatitude = (int)(decodedMsg->latitude / GDL90_COUNTS_TO_DEGREES);
    rawMsg->data[4]     = (convertedLatitude >> 16) & 0xFF;
    rawMsg->data[5]     = (convertedLatitude >> 8) & 0xFF;
    rawMsg->data[6]     = convertedLatitude & 0xFF;

    // Longitude
    int32_t convertedLongitude = (int)(decodedMsg->longitude / GDL90_COUNTS_TO_DEGREES);
    rawMsg->data[7]     = (convertedLongitude >> 16) & 0xFF;
    rawMsg->data[8]     = (convertedLongitude >> 8) & 0xFF;
    rawMsg->data[9]     = convertedLongitude & 0xFF;

    // Altitude
    int32_t convertedAltitude = (int)((decodedMsg->altitude - GDL90_ALTITUDE_OFFSET) / GDL90_ALTITUDE_FACTOR);
    rawMsg->data[10]    = (convertedAltitude >> 4) & 0xFF;

    // Altitude, Misc
    uint8_t misc = ((decodedMsg->airborne & 0x01) << 3) + ((decodedMsg->reportType & 0x01) << 2) + (decodedMsg->ttType & 0x03);
    rawMsg->data[11]    = ((convertedAltitude & 0x0F) << 4) + misc;

    // NIC, NACp
    rawMsg->data[12]    = ((decodedMsg->nic & 0x0F) << 4) + (decodedMsg->nacp & 0x0F);

    // Horizontal Velocity
    uint16_t convertedHorzVelocity = (int)(decodedMsg->horizontalVelocity / GDL90_HORZ_VELOCITY_FACTOR);
    rawMsg->data[13]    = (convertedHorzVelocity >> 4) & 0xFF;

    // Horizontal Velocity, Vertical Velocity
    int16_t convertedVertVelocity = (int)(decodedMsg->verticalVelocity / GDL90_VERT_VELOCITY_FACTOR);
    rawMsg->data[14]    = ((convertedHorzVelocity & 0x0F) << 4) + ((convertedVertVelocity >> 8) & 0x0F);

    // Vertical Velocity
    rawMsg->data[15]    = (convertedVertVelocity & 0xFF);

    // Track or Heading
    rawMsg->data[16]    = (uint8_t)(decodedMsg->trackOrHeading / GDL90_COUNTS_TO_HEADING);

    // Emitter Type
    rawMsg->data[17]    = (uint8_t)decodedMsg->emitterCategory;

    // Callsign
    for(int i = 0; i < GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE; i++) {
        rawMsg->data[18+i] = decodedMsg->callsign[i];
    }

    // Emergency
    rawMsg->data[26]    = (uint8_t)(decodedMsg->emergencyCode << 4);

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_TRAFFIC_REPORT);

    // Stuff the trailing flag byte after the CRC
    rawMsg->data[GDL90_MSG_LEN_TRAFFIC_REPORT + 2] = GDL90_FLAG_BYTE;
}

bool decode_gdl90_ownship_geo_altitude(gdl_message_t *rawMsg, gdl90_msg_ownship_geo_altitude *decodedMsg) {
    bool rval = gdl90_verifyCrc(rawMsg, GDL90_MSG_LEN_OWNSHIP_GEOMETRIC);
    // pg 34 of GDL90 ICD
    decodedMsg->ownshipGeoAltitude = ((int16_t)((rawMsg->data[0] << 8) + rawMsg->data[1])) * GDL90_GEO_ALTITUDE_FACTOR;
    decodedMsg->verticalWarningIndicator = (bool)(rawMsg->data[2] >> 7);
    decodedMsg->verticalFigureOfMerit = (float)((rawMsg->data[2] << 8) + (rawMsg->data[3]) & 0x7FFF);

    return rval;
}

void print_gdl90_ownship_geo_altitude(gdl90_msg_ownship_geo_altitude *decodedMsg) {
    fprintf(stdout, "Geometric Altitude: %f\n", decodedMsg->ownshipGeoAltitude);
    fprintf(stdout, "Vertical Warning Indicator: %d\n", decodedMsg->verticalWarningIndicator);
    fprintf(stdout, "Vertial FOM: %f\n", decodedMsg->verticalFigureOfMerit);
}

void encode_gdl90_ownship_geo_altitude(gdl_message_t *rawMsg, gdl90_msg_ownship_geo_altitude *decodedMsg) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_OWNSHIP_GEOMETRIC;

    int16_t normalizedGoAltitude = (int16_t)(decodedMsg->ownshipGeoAltitude / GDL90_GEO_ALTITUDE_FACTOR);
    rawMsg->data[0] = (uint8_t)(normalizedGoAltitude >> 8);
    rawMsg->data[1] = (uint8_t)(normalizedGoAltitude & 0xFF);

    uint16_t normalizedVerticalFOM = (uint16_t)decodedMsg->verticalFigureOfMerit;
    rawMsg->data[2] = (decodedMsg->verticalWarningIndicator << 7) + (normalizedVerticalFOM >> 8);
    rawMsg->data[3] = (normalizedVerticalFOM & 0xFF);

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_OWNSHIP_GEOMETRIC);
    // Stuff the trailing flag byte after the CRC
    rawMsg->data[GDL90_MSG_LEN_OWNSHIP_GEOMETRIC + 2] = GDL90_FLAG_BYTE;
}

bool decode_gdl90_heartbeat(gdl_message_t *rawMsg, gdl90_msg_heartbeat *decodedMsg) {
    bool rval = gdl90_verifyCrc(rawMsg, GDL90_MSG_LEN_HEARTBEAT);

    decodedMsg->gpsPosValid = (bool)(rawMsg->data[0] >> 7);
    decodedMsg->maintReq = (bool)(rawMsg->data[0] >> 6);
    decodedMsg->ident = (bool)(rawMsg->data[0] >> 5);
    decodedMsg->addrType = (bool)(rawMsg->data[0] >> 4);
    decodedMsg->gpsBattLow = (bool)(rawMsg->data[0] >> 3);
    decodedMsg->ratcs  = (bool)(rawMsg->data[0] >> 2);
    // Bit 1 is reserved
    decodedMsg->uatInitialized = (bool)(rawMsg->data[0] >> 0);

    decodedMsg->csaRequested = (bool)(rawMsg->data[1] >> 6);
    decodedMsg->csaNotAvailable = (bool)(rawMsg->data[1] >> 5);
    decodedMsg->utcOK = (bool)(rawMsg->data[1] >> 0);

    decodedMsg->timestamp = (uint32_t)(((rawMsg->data[1] >> 7)  << 16) +
                                        (rawMsg->data[2]        << 8) +
                                        rawMsg->data[3]);
    decodedMsg->messageCounts = (uint16_t)((rawMsg->data[4] << 8) + rawMsg->data[5]);

    return rval;
}

void print_gdl90_heartbeat(gdl90_msg_heartbeat *decodedMsg) {
    fprintf(stdout, "GPS Pos Valid: %d\n", decodedMsg->gpsPosValid);
    fprintf(stdout, "Maintenance Req: %d\n", decodedMsg->maintReq);
    fprintf(stdout, "Ident: %d\n", decodedMsg->ident);
    fprintf(stdout, "Address Type: %d\n", decodedMsg->addrType);
    fprintf(stdout, "GPS Battery Low: %d\n", decodedMsg->gpsBattLow);
    fprintf(stdout, "RATCS: %d\n", decodedMsg->ratcs);
    fprintf(stdout, "Timestamp: %d\n", decodedMsg->timestamp);
    fprintf(stdout, "CSA Requested: %d\n", decodedMsg->csaRequested);
    fprintf(stdout, "CSA Not Available: %d\n", decodedMsg->csaNotAvailable);
    fprintf(stdout, "UTC OK: %d\n", decodedMsg->utcOK);
    fprintf(stdout, "Message Counts: %d\n", decodedMsg->messageCounts);
}

void encode_gdl90_heartbeat(gdl_message_t *rawMsg, gdl90_msg_heartbeat *decodedMsg) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_HEARTBEAT;

    rawMsg->data[0] = (decodedMsg->gpsPosValid   << 7) +
                      (decodedMsg->maintReq      << 6) +
                      (decodedMsg->ident         << 5) +
                      (decodedMsg->addrType      << 4) +
                      (decodedMsg->gpsBattLow    << 3) +
                      (decodedMsg->ratcs         << 2) +
                      // Bit 1 is reserved
                      (decodedMsg->uatInitialized<< 0);

    rawMsg->data[1] = (((decodedMsg->timestamp >> 16) & 0x01) << 7) +
                      (decodedMsg->csaRequested      << 6) +
                      (decodedMsg->csaNotAvailable   << 5) +
                      // Bits 4-1 are reserved
                      (decodedMsg->utcOK             << 0);

    rawMsg->data[2] = decodedMsg->timestamp >> 8;
    rawMsg->data[3] = decodedMsg->timestamp & 0xFF;

    rawMsg->data[4] = decodedMsg->messageCounts >> 8;
    rawMsg->data[5] = decodedMsg->messageCounts & 0xFF;

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_HEARTBEAT);
    // Stuff the trailing flag byte after the CRC
    rawMsg->data[GDL90_MSG_LEN_HEARTBEAT+2] = GDL90_FLAG_BYTE;
}

void encode_gdl90_basic_uat_report(gdl_message_t *rawMsg, uint8_t *payload, uint8_t payload_size) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_BASIC_REPORT;

    // Time of Reception, bytes 0-2
    // TODO(rdavid): Actually populate with the right timestamp
    rawMsg->data[0] = 0;
    rawMsg->data[1] = 0;
    rawMsg->data[2] = 0;

    // Make sure we clamp the payload size
    if (payload_size > GDL90_SHORT_UAT_PAYLOAD_SIZE) {
        payload_size = GDL90_SHORT_UAT_PAYLOAD_SIZE;
    }

    memcpy(&(rawMsg->data[3]), payload, payload_size);

    // Zero out the rest of the payload, if necessary
    memset(&(rawMsg->data[3 + payload_size]), 0, GDL90_SHORT_UAT_PAYLOAD_SIZE - payload_size);

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_SHORT_UAT);
    rawMsg->data[GDL90_SHORT_UAT_PAYLOAD_SIZE + 2] = GDL90_FLAG_BYTE;
}

void encode_gdl90_long_uat_report(gdl_message_t *rawMsg, uint8_t *payload, uint8_t payload_size) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_LONG_REPORT;

    // Time of Reception, bytes 0-2
    // TODO(rdavid): Actually populate with the right timestamp
    rawMsg->data[0] = 0;
    rawMsg->data[1] = 0;
    rawMsg->data[2] = 0;

    // Make sure we clamp the payload size
    if (payload_size > GDL90_LONG_UAT_PAYLOAD_SIZE) {
        payload_size = GDL90_LONG_UAT_PAYLOAD_SIZE;
    }

    memcpy(&(rawMsg->data[3]), payload, payload_size);

    // Zero out the rest of the payload, if necessary
    memset(&(rawMsg->data[3 + payload_size]), 0, GDL90_LONG_UAT_PAYLOAD_SIZE - payload_size);

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_LONG_UAT);
    rawMsg->data[GDL90_MSG_LEN_LONG_UAT + 2] = GDL90_FLAG_BYTE;
}

void encode_gdl90_uplink_data(gdl_message_t *rawMsg, uint8_t *payload, uint16_t payload_size) {
    rawMsg->flag0 = GDL90_FLAG_BYTE;
    rawMsg->messageId = MSG_ID_UPLINK_DATA;

    // Time of Reception, bytes 0-2
    // TODO(rdavid): Actually populate with the right timestamp
    rawMsg->data[0] = 0;
    rawMsg->data[1] = 0;
    rawMsg->data[2] = 0;

    // Make sure we clamp the payload size
    if (payload_size > GDL90_UPLINK_PAYLOAD_SIZE) {
        payload_size = GDL90_UPLINK_PAYLOAD_SIZE;
    }

    memcpy(&(rawMsg->data[3]), payload, payload_size);

    // Zero out the rest of the payload, if necessary
    memset(&(rawMsg->data[3 + payload_size]), 0, GDL90_UPLINK_PAYLOAD_SIZE - payload_size);

    gdl90_insertCrc(rawMsg, GDL90_MSG_LEN_UPLINK_DATA);
    rawMsg->data[GDL90_MSG_LEN_UPLINK_DATA + 2] = GDL90_FLAG_BYTE;
}

// Copied from the GDL90 ICD
void gdl90_crcInit() {
#if !defined(ESP8266) && !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
    uint32_t i, bitctr, crc;

    for (i = 0; i < 256; i++) {
        crc = (i << 8);
        for (bitctr = 0; bitctr < 8; bitctr++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
        crc16table[i] = crc;
    }
#endif
}

// Copied from the GDL90 ICD
uint16_t gdl90_crcCompute(uint8_t *block, uint32_t length) {
    uint32_t i;
    uint16_t crc = 0;

    for (i = 0; i < length; i++) {
#if !defined(ESP8266) && !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
        crc = crc16table[crc >> 8] ^ (crc << 8) ^ block[i];
#else
        crc = pgm_read_word(&crc16table[crc >> 8]) ^ (crc << 8) ^ block[i];
#endif
    }

    return crc;
}

bool gdl90_verifyCrc(gdl_message_t *rawMsg, uint32_t length) {
    bool crcMatch = false;
    uint16_t calc_crc = gdl90_crcCompute(&(rawMsg->messageId), length+1);
    uint16_t rx_crc = (rawMsg->data[length+1] << 8) + (rawMsg->data[length]);

    if (calc_crc == rx_crc) {
        crcMatch = true;
#if 0
    } else {
        fprintf(stdout, "CRC Mismatch! (Calc = %04X, Expected = %04X)\n", calc_crc, rx_crc);
#endif
    }

    return crcMatch;
}

void gdl90_insertCrc(gdl_message_t *rawMsg, uint32_t length) {
    uint16_t calc_crc = gdl90_crcCompute(&(rawMsg->messageId), length+1);

    rawMsg->data[length]        = (uint8_t)(calc_crc & 0xFF);
    rawMsg->data[length + 1]    = (uint8_t)(calc_crc >> 8);
}

void gdl90_escape_message_for_tx(gdl_message_t *rawMsg, gdl_message_escaped_t *escapedMsg) {
    uint16_t originalLen = 0;

    switch(rawMsg->messageId) {
        case(MSG_ID_HEARTBEAT):
            originalLen = GDL90_MSG_LEN_HEARTBEAT;
            break;
        case(MSG_ID_OWNSHIP_REPORT):
            originalLen = GDL90_MSG_LEN_OWNSHIP_REPORT;
            break;
        case(MSG_ID_TRAFFIC_REPORT):
            originalLen = GDL90_MSG_LEN_TRAFFIC_REPORT;
            break;
        case(MSG_ID_OWNSHIP_GEOMETRIC):
            originalLen = GDL90_MSG_LEN_OWNSHIP_GEOMETRIC;
            break;
        case(MSG_ID_BASIC_REPORT):
            originalLen = GDL90_MSG_LEN_SHORT_UAT;
            break;
        case(MSG_ID_LONG_REPORT):
            originalLen = GDL90_MSG_LEN_LONG_UAT;
            break;
        case(MSG_ID_UPLINK_DATA):
            originalLen = GDL90_MSG_LEN_UPLINK_DATA;
            break;
    }

    originalLen += 5; // Add (2) Frame bytes, (2) CRC bytes, and (1) msg ID byte

    // Init the escaped message
    escapedMsg->length = 0;
    memset(&(escapedMsg->data[0]), 0, sizeof(escapedMsg->data));
    escapedMsg->data[0] = GDL90_FLAG_BYTE;

    // Start escaping! Start at index 1 and stop 1 byte short so we skip the true frame bytes
    uint16_t paddedIndex = 1;
    for(size_t i = 1; i < originalLen - 1; i++) {
        if ((rawMsg->data[i] == GDL90_FLAG_BYTE) || (rawMsg->data[i] == GDL90_CONTROL_ESCAPE)) {
            escapedMsg->data[paddedIndex++] = GDL90_CONTROL_ESCAPE;
            escapedMsg->data[paddedIndex++] = rawMsg->data[i] ^ GDL90_ESCAPE_BYTE;
        } else {
            escapedMsg->data[paddedIndex++] = rawMsg->data[i];
        }
    }

    // Update the length of our now escaped message
    escapedMsg->length = paddedIndex - 1;
}

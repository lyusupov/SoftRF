/*
 * TrafficHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
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

#ifndef TRAFFICHELPER_H
#define TRAFFICHELPER_H

#include "SoCHelper.h"

extern "C" {
#include <gdl90.h>
}


typedef struct traffic_struct {
    time_t    timestamp;

/* -------------------------------+------------------------------ */
/*            FLARM FTD-12        |      GDL90 equivalent         */
/* -------------------------------+------------------------------ */
    int8_t    AlarmLevel;         // trafficAlertStatus
                                  //
    int8_t    IDType;             // addressType
    uint32_t  ID;                 // address
    uint16_t  Track;              // trackOrHeading
    int16_t   TurnRate;
    uint16_t  GroundSpeed;        // horizontalVelocity
    float     ClimbRate;          // verticalVelocity
    int8_t    AcftType;           // emitterCategory

/*            Legacy              */
    int16_t   RelativeNorth;
    int16_t   RelativeEast;
    int16_t   RelativeVertical;

/*            GDL90      */
    float     latitude;
    float     longitude;
    float     altitude;
    uint8_t   callsign [GDL90_TRAFFICREPORT_MSG_CALLSIGN_SIZE];
} traffic_t;

typedef struct traffic_by_dist_struct {
  traffic_t *fop;
  float     distance;
} traffic_by_dist_t;

#define ENTRY_EXPIRATION_TIME   5 /* seconds */
#define TRAFFIC_VECTOR_UPDATE_INTERVAL 2 /* seconds */
#define TRAFFIC_UPDATE_INTERVAL_MS (TRAFFIC_VECTOR_UPDATE_INTERVAL * 1000)
#define isTimeToUpdateTraffic() (millis() - UpdateTrafficTimeMarker > \
                                  TRAFFIC_UPDATE_INTERVAL_MS)

#define isTimeToVoice()         (millis() - Traffic_Voice_TimeMarker > 2000)
#define VOICE_EXPIRATION_TIME   5 /* seconds */

void Traffic_Update       (int);
void Traffic_setup        (void);
void Traffic_loop         (void);
void Traffic_ClearExpired (void);
int  Traffic_Count        (void);
int traffic_cmp_by_distance(const void *, const void *);

extern traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
extern traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

#endif /* TRAFFICHELPER_H */

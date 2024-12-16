/*
 * Protocol_RID.h
 *
 * Encoder for Remote ID radio protocol
 * Copyright (C) 2023-2025 Linar Yusupov
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

#ifndef PROTOCOL_RID_H
#define PROTOCOL_RID_H

#define RID_TX_INTERVAL_MIN  490  /* in ms */
#define RID_TX_INTERVAL_MAX  510

typedef struct {

  /* Dummy type definition. */

} rid_packet_t;

bool   rid_init();
bool   rid_enabled();
size_t rid_encode(void *, ufo_t *);
bool   rid_decode(void *, ufo_t *, ufo_t *);

#if defined(ENABLE_REMOTE_ID)
#include <id_open.h>

extern ID_OpenDrone          squitter;
extern struct UTM_parameters utm_parameters;
extern struct UTM_data       utm_data;

extern char   RID_Operator_ID[ID_SIZE];
extern char   RID_Drone_ID   [ID_SIZE];

extern double RID_Base_Lat;
extern double RID_Base_Lon;
extern float  RID_Base_Alt;

#endif /* ENABLE_REMOTE_ID */
#endif /* PROTOCOL_RID_H */

/*
 * GDL90Helper.h
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

#ifndef GDL90HELPER_H
#define GDL90HELPER_H

#define GDL90_DST_PORT    4000

#define GDL90_EXP_TIME  3500 /* 3.5 seconds */

void GDL90_setup(void);
void GDL90_loop(void);
bool GDL90_isConnected(void);
bool GDL90_hasHeartBeat(void);
bool GDL90_hasOwnShip(void);

#define GDL90_TO_AT(x)  ((x) > 15 ? \
   AIRCRAFT_TYPE_UNKNOWN : pgm_read_byte(&gdl90_to_aircraft_type[(x)]))

extern const uint8_t gdl90_to_aircraft_type[] PROGMEM;

#endif /* GDL90HELPER_H */

#ifndef ARD_MAV_TO_FRSKY_MAVLINK_H_INCLUDED
#define ARD_MAV_TO_FRSKY_MAVLINK_H_INCLUDED

/*
 Copyright (c) 2012 - 2013 Andy Little 

(
  Some parts of this work are based on: 
  http://code.google.com/p/arducam-osd/source/browse/trunk/ArduCAM_OSD/MAVLink.ino
  Copyright (c) 2011. Sandro Benigno
)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define MAVLINK10

uint32_t get_num_heartbeats();
void read_mavlink();

#include "include/mavlink/v1.0/mavlink_types.h"

extern mavlink_system_t mavlink_system;

#include "include/mavlink/v1.0/ardupilotmega/mavlink.h"

#endif // ARD_MAV_TO_FRSKY_MAVLINK_H_INCLUDED

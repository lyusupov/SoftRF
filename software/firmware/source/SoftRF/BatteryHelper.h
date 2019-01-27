/*
 * BatteryHelper.h
 * Copyright (C) 2016-2019 Linar Yusupov
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

#ifndef BATTERYHELPER_H
#define BATTERYHELPER_H

#define BATTERY_THRESHOLD_NIMHX2  2.3
#define BATTERY_THRESHOLD_LIPO    3.5
#define BATTERY_THRESHOLD_LIFE    3.0

void  Battery_setup(void);
float Battery_voltage(void);
float Battery_threshold(void);

#endif /* BATTERYHELPER_H */
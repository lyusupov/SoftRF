/*
 * BatteryHelper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#define isTimeToBattery()         (millis() - Battery_TimeMarker > 5000)

#define BATTERY_THRESHOLD_INVALID 1.8

#define BATTERY_THRESHOLD_NIMHX2  2.3
#define BATTERY_THRESHOLD_LIPO    3.5
#define BATTERY_THRESHOLD_LIFE    3.0
#define BATTERY_THRESHOLD_NIZNX2  3.0
#define BATTERY_THRESHOLD_USB     5.0

#define BATTERY_CUTOFF_NIMHX2     2.1
#define BATTERY_CUTOFF_LIPO       3.2
#define BATTERY_CUTOFF_NIZNX2     2.9
#define BATTERY_CUTOFF_USB        4.9

enum
{
	POWER_SAVE_NONE      = 0,
	POWER_SAVE_WIFI      = 1,
	POWER_SAVE_GNSS      = 2,
	POWER_SAVE_NORECEIVE = 4
};

enum
{
	BATTERY_PARAM_VOLTAGE,
	BATTERY_PARAM_THRESHOLD,
	BATTERY_PARAM_CUTOFF,
	BATTERY_PARAM_CHARGE
};

void    Battery_setup(void);
void    Battery_loop(void);
float   Battery_voltage(void);
float   Battery_threshold(void);
float   Battery_cutoff(void);
uint8_t Battery_charge(void);

extern unsigned long Battery_TimeMarker;

#endif /* BATTERYHELPER_H */

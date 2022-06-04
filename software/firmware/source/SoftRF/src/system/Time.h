/*
 * TimeHelper.h
 * Copyright (C) 2016-2022 Linar Yusupov
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

#ifndef TIMEHELPER_H
#define TIMEHELPER_H

#include <stdint.h>

enum
{
  RTC_NONE,
  RTC_PCF8563
};

typedef struct UpTime_struct {
  uint8_t days;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} UpTime_t;


void Time_setup(void);
void Time_loop(void);

extern UpTime_t UpTime;

#endif /* TIMEHELPER_H */

/*
 * SoundHelper.cpp
 * Copyright (C) 2016-2017 Linar Yusupov
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

#include "SoundHelper.h"

#if 1
#include <ets_sys.h>
#include <osapi.h>
#include <gpio.h>
#include <os_type.h>
#include <user_interface.h>

void ICACHE_FLASH_ATTR user_init()
{   
  // init gpio subsytem
  gpio_init();
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
}
#endif

void Sound_setup(void)
{

}


void Sound_test(void)
{
  swSer.enableRx(false);
  tone(10, 440, 500);delay(500);
  tone(10, 640, 500);delay(500);
  tone(10, 840, 500);delay(500);
  tone(10, 1040, 500);
  delay(1000); 
/*
  tone(10, 1040, 500);delay(500);
  tone(10, 840, 500);delay(500); 
  tone(10, 640, 500);delay(500);
  tone(10, 440, 500);delay(500);
*/
  swSer.enableRx(true);
}
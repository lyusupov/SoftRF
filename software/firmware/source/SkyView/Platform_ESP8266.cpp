/*
 * Platform_ESP8266.cpp
 * Copyright (C) 2019 Linar Yusupov
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
#if defined(ESP8266)

#include "SoCHelper.h"
#include "EPDHelper.h"

Exp_SoftwareSerial SerialInput(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX , false, 256);

/* Waveshare E-Paper ESP8266 Driver Board */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(/*CS=D8*/ SS, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));

static void ESP8266_setup()
{

}

const SoC_ops_t ESP8266_ops = {
  SOC_ESP8266,
  "ESP8266",
  ESP8266_setup
};

#endif /* ESP8266 */

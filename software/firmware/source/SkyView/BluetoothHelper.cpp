/*
 * BluetoothHelper.cpp
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
#if defined(ESP32)

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "BluetoothHelper.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#include <BTSPP.h>

#include "WiFiHelper.h"   // HOSTNAME

BluetoothSerial SerialBT;
String BT_name = HOSTNAME;

static void ESP32_Bluetooth_setup()
{
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  SerialBT.begin(BT_name.c_str(), ESP_SPP_ROLE_MASTER, settings->bt_name);
}

static void ESP32_Bluetooth_loop()
{

}

static int ESP32_Bluetooth_available()
{
  return SerialBT.available();
}

static int ESP32_Bluetooth_read()
{
  return SerialBT.read();
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  return SerialBT.write(buffer, size);
}

Bluetooth_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#endif /* ESP32 */
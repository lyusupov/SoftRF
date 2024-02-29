/*
 * ArduinoBLE.cpp
 * Copyright (C) 2024 Linar Yusupov
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

#if defined(ARDUINO_ARCH_RENESAS)

#include "../../system/SoC.h"

#if !defined(EXCLUDE_BLUETOOTH)

#include <ArduinoBLE.h>

#include <api/RingBuffer.h>

#include "../../driver/EEPROM.h"
#include "../../driver/Bluetooth.h"
#include "../../driver/WiFi.h"
#include "../../driver/Battery.h"

// #include <core_version.h> // TODO
#define ARDUINO_RA4M1_RELEASE "1.1.0"

bool deviceConnected    = false;
bool oldDeviceConnected = false;

RingBufferN<BLE_FIFO_TX_SIZE> BLE_FIFO_TX = RingBufferN<BLE_FIFO_TX_SIZE>();
RingBufferN<BLE_FIFO_RX_SIZE> BLE_FIFO_RX = RingBufferN<BLE_FIFO_RX_SIZE>();

String BT_name = HOSTNAME;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

BLEService UARTService(UART_SERVICE_UUID16);
BLEService BATService (UUID16_SVC_BATTERY);

BLECharacteristic UARTCharacteristic(UART_CHARACTERISTIC_UUID16,
                                     BLERead | BLEWriteWithoutResponse | BLENotify,
                                     BLE_MAX_WRITE_CHUNK_SIZE);
BLEDescriptor UARTDescriptor("2901", "HMSoft");

BLEUnsignedCharCharacteristic BATCharacteristic(UUID16_CHR_BATTERY_LEVEL,
                                                BLERead | BLENotify);

BLEService DevInfoService(UUID16_SVC_DEVICE_INFORMATION);
BLEStringCharacteristic ModelCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic SerialCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic FirmwareCharacteristic(UUID16_CHR_FIRMWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic HardwareCharacteristic(UUID16_CHR_HARDWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic SoftwareCharacteristic(UUID16_CHR_SOFTWARE_REVISION_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);
BLEStringCharacteristic ManufacturerCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING,
                                            BLERead, BLE_MAX_WRITE_CHUNK_SIZE);

void onConnect(BLEDevice central) {
//  Serial.print("Connected event, central: ");
//  Serial.println(central.address());

  deviceConnected = true;
};

void onDisconnect(BLEDevice central) {
//  Serial.print("Disconnected event, central: ");
//  Serial.println(central.address());

  deviceConnected = false;
  BLE_Advertising_TimeMarker = millis();
}

void UARTCharacteristicWritten(BLEDevice         central,
                               BLECharacteristic characteristic) {
  size_t valueLength = characteristic.valueLength();

  if (valueLength > 0) {
    char buf[BLE_MAX_WRITE_CHUNK_SIZE];
    size_t len = sizeof(buf);

    characteristic.readValue(buf, len);

    if (valueLength > len) { valueLength = len; }

    size_t size = BLE_FIFO_RX.availableForStore();
    size = (size < valueLength) ? size : valueLength;
    for (size_t i = 0; i < size; i++) {
      BLE_FIFO_RX.store_char(buf[i]);
    }
  }
}

static void RA4M1_Bluetooth_setup()
{
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE.begin();

      char LocalName[BLE_MAX_WRITE_CHUNK_SIZE];
      snprintf(LocalName, sizeof(LocalName), "%s-LE", BT_name.c_str());
      BLE.setLocalName(LocalName);
      BLE.setDeviceName(LocalName);

      BLE.setAdvertisedService(UARTService);
      UARTCharacteristic.setEventHandler(BLEWritten, UARTCharacteristicWritten);
      UARTCharacteristic.addDescriptor(UARTDescriptor);
      UARTService.addCharacteristic(UARTCharacteristic);
      BLE.addService(UARTService);

//      BLE.setAdvertisedService(BATService);
      BATService.addCharacteristic(BATCharacteristic);
      BLE.addService(BATService);

      const char *Model         = hw_info.model == SOFTRF_MODEL_STANDALONE ? "Standalone Edition" :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK2  ? "Prime Mark II"      :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK3  ? "Prime Mark III"     :
                                  hw_info.model == SOFTRF_MODEL_HAM        ? "Ham Edition"        :
                                  hw_info.model == SOFTRF_MODEL_MIDI       ? "Midi Edition"       :
                                  hw_info.model == SOFTRF_MODEL_ACADEMY    ? "Academy Edition"    :
                                  "Unknown";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      const char *Firmware      = "Arduino RA4M1 " ARDUINO_RA4M1_RELEASE;

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SOFTRF_IDENT;
      const char *Software      = SOFTRF_FIRMWARE_VERSION;

      ModelCharacteristic.       writeValue((char *) Model);
      SerialCharacteristic.      writeValue((char *) SerialNum);
      FirmwareCharacteristic.    writeValue((char *) Firmware);
      HardwareCharacteristic.    writeValue((char *) Hardware);
      SoftwareCharacteristic.    writeValue((char *) Software);
      ManufacturerCharacteristic.writeValue((char *) Manufacturer);

      DevInfoService.addCharacteristic(ModelCharacteristic);
      DevInfoService.addCharacteristic(SerialCharacteristic);
      DevInfoService.addCharacteristic(FirmwareCharacteristic);
      DevInfoService.addCharacteristic(HardwareCharacteristic);
      DevInfoService.addCharacteristic(SoftwareCharacteristic);
      DevInfoService.addCharacteristic(ManufacturerCharacteristic);
      BLE.addService(DevInfoService);

      BLE.setEventHandler(BLEConnected,    onConnect);
      BLE.setEventHandler(BLEDisconnected, onDisconnect);

      BLE.advertise();

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
    break;
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static void RA4M1_Bluetooth_loop()
{
  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE.poll();

      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */
          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];

          size_t size = BLE_FIFO_TX.available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
            for (int i=0; i < size; i++) {
              chunk[i] = BLE_FIFO_TX.read_char();
            }
            UARTCharacteristic.writeValue(chunk, size);
          }

          BLE_Notify_TimeMarker = millis();
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready

          // BLE.advertise(); // restart advertising

          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
      }
      if (deviceConnected && isTimeToBattery()) {
        uint8_t battery_level = Battery_charge();

        BATCharacteristic.writeValue(battery_level);
      }
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void RA4M1_Bluetooth_fini()
{
  /* TBD */

  BLE.end();
}

static int RA4M1_Bluetooth_available()
{
  int rval = 0;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX.available();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int RA4M1_Bluetooth_read()
{
  int rval = -1;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX.read_char();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t RA4M1_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      size_t avail = BLE_FIFO_TX.availableForStore();
      if (size > avail) {
        rval = avail;
      }
      for (size_t i = 0; i < rval; i++) {
        BLE_FIFO_TX.store_char(buffer[i]);
      }
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

IODev_ops_t RA4M1_Bluetooth_ops = {
  "RA4M1 Bluetooth",
  RA4M1_Bluetooth_setup,
  RA4M1_Bluetooth_loop,
  RA4M1_Bluetooth_fini,
  RA4M1_Bluetooth_available,
  RA4M1_Bluetooth_read,
  RA4M1_Bluetooth_write
};

#endif /* EXCLUDE_BLUETOOTH */
#endif /* ARDUINO_ARCH_RENESAS */

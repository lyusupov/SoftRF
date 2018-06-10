/*
 * BluetoothHelper.cpp
 * Copyright (C) 2018 Linar Yusupov
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
#include "EEPROMHelper.h"
#include "BluetoothHelper.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#include <BluetoothSerial.h>
/*
    BLE code is based on Neil Kolban example for IDF:
      https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini    
    HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "WiFiHelper.h"   // HOSTNAME

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;
BluetoothSerial SerialBT;
String BT_name = HOSTNAME;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

static void ESP32_Bluetooth_setup()
{
#if !defined(SOFTRF_ADDRESS)
  union {
    uint8_t efuse_mac[6];
    uint64_t chipmacid;
  };

  chipmacid = ESP.getEfuseMac();

  BT_name += String((uint32_t) efuse_mac[5] | (efuse_mac[4] << 8) | \
                              (efuse_mac[3] << 16), HEX);
#else
  BT_name += String(SOFTRF_ADDRESS & 0x00FFFFFFU, HEX);
#endif

  switch(settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    {
      SerialBT.begin(BT_name.c_str());
    }
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

      // Create the BLE Device
      BLEDevice::init((BT_name+"-LE").c_str());

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_NOTIFY |
                          BLECharacteristic::PROPERTY_WRITE_NR
                        );

      UserDescriptor.setValue("HMSoft");
      pCharacteristic->addDescriptor(&UserDescriptor);
      pCharacteristic->addDescriptor(new BLE2902());

      pCharacteristic->setCallbacks(new MyCallbacks());

      pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);

      // Start the service
      pService->start();

      // Start advertising
      pServer->getAdvertising()->start();
      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_OFF:
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{
  switch(settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 30)) { /* < 6000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = (BLE_FIFO_TX->available() < BLE_MAX_WRITE_CHUNK_SIZE ?
                         BLE_FIFO_TX->available() : BLE_MAX_WRITE_CHUNK_SIZE);

          BLE_FIFO_TX->read((char *) chunk, size);

          pCharacteristic->setValue(chunk, size);
          pCharacteristic->notify();
          BLE_Notify_TimeMarker = millis();
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
      }
    }
    break;
  case BLUETOOTH_OFF:
  case BLUETOOTH_SPP:
  default:
    break;
  }
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch(settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.available();
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available();
    break;
  case BLUETOOTH_OFF:
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch(settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.read();
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->read();
    break;
  case BLUETOOTH_OFF:
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch(settings->bluetooth)
  {
  case BLUETOOTH_SPP:
    rval = SerialBT.write(buffer, size);
    break;
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  case BLUETOOTH_OFF:
  default:
    break;
  }

  return rval;
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
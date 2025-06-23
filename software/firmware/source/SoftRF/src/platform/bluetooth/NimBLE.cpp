/*
 * NimBLE.cpp
 * Copyright (C) 2024-2025 Linar Yusupov
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
#include "sdkconfig.h"

#include "../../system/SoC.h"

#if defined(USE_NIMBLE) && !defined(EXCLUDE_BLUETOOTH)
/*
 *  BLE code is based on Neil Kolban example for IDF:
 *    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
 *  Ported to Arduino ESP32 by Evandro Copercini
 *  HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
 */
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>

#if !defined(CONFIG_IDF_TARGET_ESP32P4)
#include "esp_gap_bt_api.h"
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

#include "../../driver/EEPROM.h"
#include "../../driver/Bluetooth.h"
#include "../../driver/WiFi.h"
#include "../../driver/Battery.h"

#include <core_version.h>
#include <cbuf.h>

NimBLEServer* pServer = NULL;
NimBLECharacteristic* pUARTCharacteristic = NULL;
NimBLECharacteristic* pBATCharacteristic  = NULL;

NimBLECharacteristic* pModelCharacteristic         = NULL;
NimBLECharacteristic* pSerialCharacteristic        = NULL;
NimBLECharacteristic* pFirmwareCharacteristic      = NULL;
NimBLECharacteristic* pHardwareCharacteristic      = NULL;
NimBLECharacteristic* pSoftwareCharacteristic      = NULL;
NimBLECharacteristic* pManufacturerCharacteristic  = NULL;

bool deviceConnected    = false;
bool oldDeviceConnected = false;

#if defined(USE_BLE_MIDI)
NimBLECharacteristic* pMIDICharacteristic = NULL;
#endif /* USE_BLE_MIDI */

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

String BT_name = HOSTNAME;

static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

// NimBLEDescriptor UserDescriptor(NimBLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(NimBLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
    }
};

class UARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pUARTCharacteristic) {
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
      String rxValue = pUARTCharacteristic->getValue();
#else
      std::string rxValue = pUARTCharacteristic->getValue();
#endif /* ESP_IDF_VERSION_MAJOR */

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

static void ESP32_Bluetooth_setup()
{
  char id_06x[8];
  snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
  BT_name += "-";
  BT_name += String(id_06x);

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

      // Create the BLE Device
      NimBLEDevice::init((BT_name+"-LE").c_str());

      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // NimBLEDevice::setMTU(23);

      // Create the BLE Server
      pServer = NimBLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      NimBLEService *pService = pServer->createService(NimBLEUUID(UART_SERVICE_UUID16));

      // Create a BLE Characteristic
      pUARTCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UART_CHARACTERISTIC_UUID16),
                              NIMBLE_PROPERTY::READ   |
                              NIMBLE_PROPERTY::NOTIFY |
                              NIMBLE_PROPERTY::WRITE_NR
                            );

//      UserDescriptor.setValue("HMSoft");
//      pUARTCharacteristic->addDescriptor(&UserDescriptor);

      pUARTCharacteristic->setCallbacks(new UARTCallbacks());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(NimBLEUUID(UUID16_SVC_BATTERY));

      // Create a BLE Characteristic
      pBATCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_BATTERY_LEVEL),
                              NIMBLE_PROPERTY::READ   |
                              NIMBLE_PROPERTY::NOTIFY
                            );

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(NimBLEUUID(UUID16_SVC_DEVICE_INFORMATION));

      // Create BLE Characteristics
      pModelCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_MODEL_NUMBER_STRING),
                              NIMBLE_PROPERTY::READ
                            );
      pSerialCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_SERIAL_NUMBER_STRING),
                              NIMBLE_PROPERTY::READ
                            );
      pFirmwareCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_FIRMWARE_REVISION_STRING),
                              NIMBLE_PROPERTY::READ
                            );
      pHardwareCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_HARDWARE_REVISION_STRING),
                              NIMBLE_PROPERTY::READ
                            );
      pSoftwareCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_SOFTWARE_REVISION_STRING),
                              NIMBLE_PROPERTY::READ
                            );
      pManufacturerCharacteristic = pService->createCharacteristic(
                              NimBLEUUID(UUID16_CHR_MANUFACTURER_NAME_STRING),
                              NIMBLE_PROPERTY::READ
                            );

      const char *Model         = hw_info.model == SOFTRF_MODEL_STANDALONE ? "Standalone Edition" :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK2  ? "Prime Mark II"      :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK3  ? "Prime Mark III"     :
                                  hw_info.model == SOFTRF_MODEL_HAM        ? "Ham Edition"        :
                                  hw_info.model == SOFTRF_MODEL_MIDI       ? "Midi Edition"       :
                                  hw_info.model == SOFTRF_MODEL_ECO        ? "Eco Edition"        :
                                  hw_info.model == SOFTRF_MODEL_INK        ? "Ink Edition"        :
                                  hw_info.model == SOFTRF_MODEL_GIZMO      ? "Gizmo Edition"      :
                                  hw_info.model == SOFTRF_MODEL_NANO       ? "Nano Edition"       :
                                  "Unknown";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      const char *Firmware      = "Arduino ESP32 " ARDUINO_ESP32_RELEASE;

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SOFTRF_IDENT;
      const char *Software      = SOFTRF_FIRMWARE_VERSION;

      pModelCharacteristic->       setValue((uint8_t *) Model,        strlen(Model));
      pSerialCharacteristic->      setValue((uint8_t *) SerialNum,    strlen(SerialNum));
      pFirmwareCharacteristic->    setValue((uint8_t *) Firmware,     strlen(Firmware));
      pHardwareCharacteristic->    setValue((uint8_t *) Hardware,     strlen(Hardware));
      pSoftwareCharacteristic->    setValue((uint8_t *) Software,     strlen(Software));
      pManufacturerCharacteristic->setValue((uint8_t *) Manufacturer, strlen(Manufacturer));

      // Start the service
      pService->start();

#if defined(USE_BLE_MIDI)
      // Create the BLE Service
      pService = pServer->createService(NimBLEUUID(MIDI_SERVICE_UUID));

      // Create a BLE Characteristic
      pMIDICharacteristic = pService->createCharacteristic(
                              NimBLEUUID(MIDI_CHARACTERISTIC_UUID),
                              NIMBLE_PROPERTY::READ   |
                              NIMBLE_PROPERTY::WRITE  |
                              NIMBLE_PROPERTY::NOTIFY |
                              NIMBLE_PROPERTY::WRITE_NR
                            );

      // Start the service
      pService->start();
#endif /* USE_BLE_MIDI */

      // Start advertising
      NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
#if 0
      pAdvertising->addServiceUUID(NimBLEUUID(UART_SERVICE_UUID16));
      pAdvertising->addServiceUUID(NimBLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      pAdvertising->addServiceUUID(NimBLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
#else
      /* work around https://github.com/espressif/arduino-esp32/issues/6750 */
      NimBLEAdvertisementData BLEAdvData;
      BLEAdvData.setFlags(0x06);
      BLEAdvData.setCompleteServices(NimBLEUUID(UART_SERVICE_UUID16));
      BLEAdvData.setCompleteServices(NimBLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      BLEAdvData.setCompleteServices(NimBLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
      pAdvertising->setAdvertisementData(BLEAdvData);
#endif
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
      pAdvertising->enableScanResponse(true);
      pAdvertising->setPreferredParams(0x06, 0x12);
#else
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
#endif /* ESP_IDF_VERSION_MAJOR */
      NimBLEDevice::startAdvertising();

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{
  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = BLE_FIFO_TX->available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);

            pUARTCharacteristic->setValue(chunk, size);
            pUARTCharacteristic->notify();
          }

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
      if (deviceConnected && isTimeToBattery()) {
        uint8_t battery_level = Battery_charge();

        pBATCharacteristic->setValue(&battery_level, 1);
        pBATCharacteristic->notify();
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

static void ESP32_Bluetooth_fini()
{
  /* TBD */
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  case BLUETOOTH_SPP:
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->read();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  case BLUETOOTH_SPP:
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  case BLUETOOTH_SPP:
  default:
    break;
  }

  return rval;
}

IODev_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_fini,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#endif /* USE_NIMBLE */
#endif /* ESP32 */

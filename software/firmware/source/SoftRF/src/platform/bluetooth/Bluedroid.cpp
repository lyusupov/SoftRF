/*
 * BluetoothHelper.cpp
 * Copyright (C) 2018-2025 Linar Yusupov
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

#if !defined(EXCLUDE_BLUETOOTH) && !defined(USE_NIMBLE) && !defined(USE_ARDUINOBLE)
/*
 *  BLE code is based on Neil Kolban example for IDF:
 *    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
 *  Ported to Arduino ESP32 by Evandro Copercini
 *  HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp_gap_bt_api.h"
#endif /* CONFIG_IDF_TARGET_ESP32 */

#include "../../driver/EEPROM.h"
#include "../../driver/Bluetooth.h"
#include "../../driver/WiFi.h"
#include "../../driver/Battery.h"

#include <core_version.h>
#include <cbuf.h>

BLEServer* pServer = NULL;
BLECharacteristic* pUARTCharacteristic = NULL;
BLECharacteristic* pBATCharacteristic  = NULL;

BLECharacteristic* pModelCharacteristic         = NULL;
BLECharacteristic* pSerialCharacteristic        = NULL;
BLECharacteristic* pFirmwareCharacteristic      = NULL;
BLECharacteristic* pHardwareCharacteristic      = NULL;
BLECharacteristic* pSoftwareCharacteristic      = NULL;
BLECharacteristic* pManufacturerCharacteristic  = NULL;

bool deviceConnected    = false;
bool oldDeviceConnected = false;

#if defined(USE_BLE_MIDI)
BLECharacteristic* pMIDICharacteristic = NULL;
#endif /* USE_BLE_MIDI */

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(ENABLE_BT_VOICE)
#include "BluetoothA2DPSource.h"
#include "piano16bit.h"

BluetoothA2DPSource a2dp_source;
SoundData *sound_data = new OneChannelSoundData((int16_t*)piano16bit_raw, piano16bit_raw_len/2);
#endif /* ENABLE_BT_VOICE */

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

class UARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pUARTCharacteristic) {
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
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str());
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

#if defined(CONFIG_IDF_TARGET_ESP32)
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif /* CONFIG_IDF_TARGET_ESP32 */

      // Create the BLE Device
      BLEDevice::init((BT_name+"-LE").c_str());

      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // BLEDevice::setMTU(23);

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(BLEUUID(UART_SERVICE_UUID16));

      // Create a BLE Characteristic
      pUARTCharacteristic = pService->createCharacteristic(
                              BLEUUID(UART_CHARACTERISTIC_UUID16),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      UserDescriptor.setValue("HMSoft");
      pUARTCharacteristic->addDescriptor(&UserDescriptor);
      pUARTCharacteristic->addDescriptor(new BLE2902());

      pUARTCharacteristic->setCallbacks(new UARTCallbacks());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_BATTERY));

      // Create a BLE Characteristic
      pBATCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_BATTERY_LEVEL),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
      pBATCharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_DEVICE_INFORMATION));

      // Create BLE Characteristics
      pModelCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MODEL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSerialCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SERIAL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pFirmwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_FIRMWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pHardwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_HARDWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSoftwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SOFTWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pManufacturerCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MANUFACTURER_NAME_STRING),
                              BLECharacteristic::PROPERTY_READ
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
                                  hw_info.model == SOFTRF_MODEL_AIRVENTURE ? "Airventure Edition" :
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
      pService = pServer->createService(BLEUUID(MIDI_SERVICE_UUID));

      // Create a BLE Characteristic
      pMIDICharacteristic = pService->createCharacteristic(
                              BLEUUID(MIDI_CHARACTERISTIC_UUID),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      // Create a BLE Descriptor
      pMIDICharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();
#endif /* USE_BLE_MIDI */

      // Start advertising
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32P4)
      pAdvertising->addServiceUUID(BLEUUID(UART_SERVICE_UUID16));
      pAdvertising->addServiceUUID(BLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      pAdvertising->addServiceUUID(BLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
#else
      /* work around https://github.com/espressif/arduino-esp32/issues/6750 */
      BLEAdvertisementData BLEAdvData;
      BLEAdvData.setFlags(0x06);
      BLEAdvData.setCompleteServices(BLEUUID(UART_SERVICE_UUID16));
      BLEAdvData.setCompleteServices(BLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      BLEAdvData.setCompleteServices(BLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
      pAdvertising->setAdvertisementData(BLEAdvData);
#endif
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
      BLEDevice::startAdvertising();

      BLE_Advertising_TimeMarker = millis();
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
#if defined(ENABLE_BT_VOICE)
    //a2dp_source.set_auto_reconnect(false);
    a2dp_source.start("BT SPEAKER");
    a2dp_source.set_volume(100);
    a2dp_source.write_data(sound_data);
#endif
    break;
  case BLUETOOTH_NONE:
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
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.available();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
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
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.read();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->read();
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
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
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP:
    rval = SerialBT.write(buffer, size);
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
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

#endif /* CONFIG_BLUEDROID_ENABLED */
#endif /* ESP32 */

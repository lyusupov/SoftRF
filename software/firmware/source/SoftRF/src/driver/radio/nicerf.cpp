/*
 * nicerf.cpp
 * Copyright (C) 2023-2025 Linar Yusupov
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

#include "../RF.h"

#include <LibAPRSesp.h>

AX25Msg Incoming_APRS_Packet;

#if defined(USE_SA8X8)

#include <SA818.h>
#include <SA818Controller.h>
#include <cppQueue.h>

#include "../EEPROM.h"
#include "../Battery.h"

static bool sa8x8_probe(void);
static void sa8x8_setup(void);
static void sa8x8_channel(int8_t);
static bool sa8x8_receive(void);
static bool sa8x8_transmit(void);
static void sa8x8_shutdown(void);

const rfchip_ops_t sa8x8_ops = {
  RF_IC_SA8X8,
  "SA8X8",
  sa8x8_probe,
  sa8x8_setup,
  sa8x8_channel,
  sa8x8_receive,
  sa8x8_transmit,
  sa8x8_shutdown
};

SA818 sa868(&SA8X8_Serial);
SA818Controller controller(&sa868);

OpenEdition OE(&SA8X8_Serial, SOC_GPIO_PIN_TWR2_RADIO_RX, SOC_GPIO_PIN_TWR2_RADIO_TX);

bool afskSync            = false;
int mVrms                = 0;
uint32_t Data_Frequency  = 0;
uint32_t Voice_Frequency = 0;
byte     SA8X8_SQL       = 1;

cppQueue PacketBuffer(sizeof(AX25Msg), 5, FIFO);

void aprs_msg_callback(struct AX25Msg *msg) {
  AX25Msg pkg;
  memcpy(&pkg, msg, sizeof(AX25Msg));

  // trim off a multiline message
  for (int i = 0; i < pkg.len; i++) {
    if (pkg.info[i] == '\n' || pkg.info[i] == '\r') {
      pkg.info[i] = 0;
      pkg.len = i;
      break;
    }
  }

  PacketBuffer.push(&pkg);
}

#include "../LED.h"
#if defined(USE_NEOPIXELBUS_LIBRARY)
extern NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> TWR2_Pixel;
#endif /* USE_NEOPIXELBUS_LIBRARY */
#if defined(USE_ADAFRUIT_NEO_LIBRARY)
extern Adafruit_NeoPixel TWR2_Pixel;
#endif /* USE_ADAFRUIT_NEO_LIBRARY */

void sa868_Tx_LED_state(bool val) {
  if (hw_info.model == SOFTRF_MODEL_HAM) {
#if defined(USE_NEOPIXELBUS_LIBRARY)
    color_t inactive_color = hw_info.revision > 0 ? LED_COLOR_BLACK :
                             settings->power_save & POWER_SAVE_NORECEIVE ?
                             LED_COLOR_BLACK : LED_COLOR_GREEN;
    TWR2_Pixel.SetPixelColor(0, val ? LED_COLOR_RED : inactive_color);
    TWR2_Pixel.Show();
#endif /* USE_NEOPIXELBUS_LIBRARY */
#if defined(USE_ADAFRUIT_NEO_LIBRARY)
    color_t inactive_color = hw_info.revision > 0 ? LED_COLOR_BLACK :
                             settings->power_save & POWER_SAVE_NORECEIVE ?
                             LED_COLOR_BLACK : LED_COLOR_GREEN;
    TWR2_Pixel.setPixelColor(0, val ? LED_COLOR_RED : inactive_color);
    TWR2_Pixel.show();
#endif /* USE_ADAFRUIT_NEO_LIBRARY */
  }
}

static bool sa8x8_probe()
{
  bool success = false;

  if (hw_info.model != SOFTRF_MODEL_HAM) {
    return success;
  }

  SoC->UATSerial_begin(9600);

//  sa868.verbose(); // verbose mode

  controller.setPins(SOC_GPIO_PIN_TWR2_RADIO_PTT,
                     SOC_GPIO_PIN_TWR2_RADIO_PD,
                     SOC_GPIO_PIN_TWR2_RADIO_HL);

  controller.wake();
  controller.lowPower();
  controller.receive();

  delay(500);

  bool connect = controller.connect();
  if (connect) {
    String version = controller.version();

    Serial.print("SA8X8 firmware version: "); Serial.println(version);

    if (version.startsWith("SA868")) {
      controller.setModel(Model::SA_868_NiceRF);
    }

    success = connect;
  } else {
    String model = controller.model();

    Serial.print("SA868 model: "); Serial.println(model);

    if (model.startsWith("SA868S-VHF")) {
      controller.setModel(Model::SA_868_OpenEdition);
      controller.setBand(Band::VHF);
      success = true;
    }

    if (model.startsWith("SA868S-UHF")) {
      controller.setModel(Model::SA_868_OpenEdition);
      controller.setBand(Band::UHF);
      success = true;
    }
  }

  return success;
}

static void sa8x8_channel(int8_t channel)
{
  /* Nothing to do */
}

static void sa8x8_setup()
{
  unsigned long aprs_preamble = 350UL;
  unsigned long aprs_tail     =   0UL;

  /* Enforce radio settings to follow APRS protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_APRS;

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
  float TxF_MHz = frequency / 1000000.0;

  if (Data_Frequency) {
    TxF_MHz = Data_Frequency / 1000000.0;
  } else {
    if (controller.getBand() == Band::UHF) {
      switch (settings->band)
      {
        case RF_BAND_US:
          TxF_MHz = 445.925;
          break;
        case RF_BAND_AU:
          TxF_MHz = 439.1;
          break;
        case RF_BAND_NZ:
          TxF_MHz = 432.575;
          break;
        case RF_BAND_EU:
        default:
          TxF_MHz = 432.5; /* IARU R1 */
          // TxF_MHz = 433.8;
          break;
      }
    }
  }

  bool rx = (settings->power_save & POWER_SAVE_NORECEIVE ? false : true);
  float RxF_MHz = Voice_Frequency && !rx ? Voice_Frequency/1000000.0 : TxF_MHz;
  byte sq = rx || Voice_Frequency ? SA8X8_SQL : 8;

  switch (controller.getModel())
  {
    case Model::SA_868_OpenEdition:
      OE.setAudio(false);
      OE.init();
      OE.setTxFrequency((uint32_t) (TxF_MHz * 1000000));
      OE.setRxFrequency((uint32_t) (RxF_MHz * 1000000));
      // OE.setSqlThresh(sq);

      if (settings->txpower == RF_TX_POWER_FULL) {
        OE.setHighPower();
      } else {
        OE.setLowPower();
      }

      // OE.setBandwidth(1); /* 25 KHz */
      OE.setAudio(true);
      break;

    case Model::SA_818:
    case Model::SA_868_NiceRF:
    default:
      if (controller.getModel() == Model::SA_868_NiceRF) {
        controller.setBW(settings->txpower == RF_TX_POWER_FULL ? 0 : 1);
        aprs_preamble = 350UL * 3;
        aprs_tail     = 250UL;
      } else {
        controller.setBW(0 /* 12.5 KHz */);
      }

      controller.setTXF(TxF_MHz);
      controller.setRXF(RxF_MHz);
      controller.setTXSub(0);
      controller.setSQ(sq);
      controller.setRXSub(0);

      controller.update();

      controller.setFilter(1,1,1);
      if (controller.getModel() == Model::SA_818) {
        controller.closeTail();
      }

      byte volume = 1;
      if (hw_info.revision == 1 && rx == false && Voice_Frequency) {
        volume = 3;
      }
      /*
       * Owners of T-TWR Plus V2.0 board with no R22 fix
       * may consider to increase the volume setting
       * up to 6 for a better 'APRS sensitivity' at an expense of
       * very loud sound from the speaker.
       * A disconnect of the speaker is one of the options to think about.
       */
      controller.setVolume(volume);

      break;
  }

  protocol_encode = &aprs_encode;
  protocol_decode = &ax25_decode;

  if (rx) {
    PacketBuffer.begin();
    PacketBuffer.clean();
  }

  APRS_init(rx);
  APRS_setCallsign("NOCALL", 0 /* 11 - balloons, aircraft, spacecraft, etc */ );
  APRS_setPath1("WIDE1", 1);
  APRS_setPreamble(aprs_preamble);
  APRS_setTail(aprs_tail);
  APRS_setSymbol('\'');

  /* AX25 library used is not 100% ready for SSID feature yet */
  //if (strlen(APRS_FromCall) > 6) {
  //  APRS_FromCall[6] = 0;
  //}

  // We don't need to set the destination identifier, but
  // if you want to, this is how you do it:
  // APRS_setDestination("APSRF1", 0);

  if (hw_info.model == SOFTRF_MODEL_HAM) {
#if defined(USE_NEOPIXELBUS_LIBRARY)
    TWR2_Pixel.SetPixelColor(0, rx && hw_info.revision == 0 ?
                                LED_COLOR_GREEN : LED_COLOR_BLACK);
    TWR2_Pixel.Show();
#endif /* USE_NEOPIXELBUS_LIBRARY */
#if defined(USE_ADAFRUIT_NEO_LIBRARY)
    TWR2_Pixel.setPixelColor(0, rx && hw_info.revision == 0 ?
                                LED_COLOR_GREEN : LED_COLOR_BLACK);
    TWR2_Pixel.show();
#endif /* USE_ADAFRUIT_NEO_LIBRARY */
  }

  APRS_setTxLEDCallback(sa868_Tx_LED_state);

  if (rx) {
    AFSK_TimerEnable(true);
  }
}

static bool sa8x8_receive()
{
  bool success = false;

  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return success;
  }

  switch (controller.getModel())
  {
    case Model::SA_868_OpenEdition:
      if (OE.settings().mode != OpenEdition_Mode::RX) {
        OE.RxOn();
        OE.setVolume(3);
      }
      break;

    case Model::SA_818:
    case Model::SA_868_NiceRF:
    default:
//    controller.receive();
      break;
  }

  uint8_t powerPin = SOC_GPIO_PIN_TWR2_RADIO_HL;
  AFSK_Poll(true, LOW, powerPin);

  if (PacketBuffer.getCount() > 0) {
    PacketBuffer.pop(&Incoming_APRS_Packet);

    rx_packets_counter++;
    success = true;
  }

  return success;
}

static bool sa8x8_transmit()
{
  switch (controller.getModel())
  {
    case Model::SA_868_OpenEdition:
      if (OE.settings().mode != OpenEdition_Mode::TX) {
        if (OE.settings().mode == OpenEdition_Mode::RX) {
          OE.RxOff();
          delay(300);
        }
        OE.TxOn();
        delay(300);
      } else {
        return false;
      }
      break;

    case Model::SA_818:
    case Model::SA_868_NiceRF:
    default:
      if (controller.getTxStatus()) return false;
      break;
  }

  uint8_t powerPin = SOC_GPIO_PIN_TWR2_RADIO_HL;
  AFSK_TimerEnable(false);

  APRS_sendTNC2Pkt(String((char *) TxBuffer));

  do {
    delay(5);
    AFSK_Poll(true, settings->txpower == RF_TX_POWER_FULL ? HIGH : LOW, powerPin);
  } while (AFSK_modem->sending);

  switch (controller.getModel())
  {
    case Model::SA_868_OpenEdition:
      if (OE.settings().mode == OpenEdition_Mode::TX) {
        OE.TxOff();
      }
      break;

    case Model::SA_818:
    case Model::SA_868_NiceRF:
    default:
      break;
  }

  if ((settings->power_save & POWER_SAVE_NORECEIVE) == 0) {
    AFSK_TimerEnable(true);
  }

  return true;
}

static void sa8x8_shutdown()
{
  switch (controller.getModel())
  {
    case Model::SA_868_OpenEdition:
      /* TBD */
      break;

    case Model::SA_818:
    case Model::SA_868_NiceRF:
    default:
      /* TBD */
      controller.lowPower();
      controller.receive();
      controller.sleep();
      break;
  }
}

#endif /* USE_SA8X8 */

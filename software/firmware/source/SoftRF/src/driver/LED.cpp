/*
 * LEDHelper.cpp
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

#include "../system/SoC.h"

#include <TimeLib.h>

#include "LED.h"
#include "Battery.h"
#include "../TrafficHelper.h"

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;

#define isTimeToToggle() (millis() - status_LED_TimeMarker > 300)
static int status_LED = SOC_UNUSED_PIN;
static unsigned long status_LED_TimeMarker = 0;

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void LED_setup() {
#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    uni_begin();
    uni_show(); // Initialize all pixels to 'off'
  }
#endif /* EXCLUDE_LED_RING */

  status_LED = SOC_GPIO_PIN_STATUS;

  if (status_LED != SOC_UNUSED_PIN) {
    pinMode(status_LED, OUTPUT);
    /* Indicate positive power supply */
    digitalWrite(status_LED, LED_STATE_ON);
  }
}

#if !defined(EXCLUDE_LED_RING)
// Fill the dots one after the other with a color
static void colorWipe(color_t c, uint8_t wait) {
  for (uint16_t i = 0; i < uni_numPixels(); i++) {
    uni_setPixelColor(i, c);
    uni_show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
static void theaterChase(color_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < uni_numPixels(); i = i + 3) {
        uni_setPixelColor(i + q, c);  //turn every third pixel on
      }
      uni_show();

      delay(wait);

      for (int i = 0; i < uni_numPixels(); i = i + 3) {
        uni_setPixelColor(i + q, LED_COLOR_BLACK);      //turn every third pixel off
      }
    }
  }
}
#endif /* EXCLUDE_LED_RING */

void LED_test() {
#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    // Some example procedures showing how to display to the pixels:
    colorWipe(uni_Color(255, 0, 0), 50); // Red
    colorWipe(uni_Color(0, 255, 0), 50); // Green
    colorWipe(uni_Color(0, 0, 255), 50); // Blue
    // Send a theater pixel chase in...
    theaterChase(uni_Color(127, 127, 127), 50); // White
    theaterChase(uni_Color(127, 0, 0), 50); // Red
    theaterChase(uni_Color(0, 0, 127), 50); // Blue

    //  rainbow(20);
    //  rainbowCycle(20);
    //  theaterChaseRainbow(50);
    colorWipe(uni_Color(0, 0, 0), 50); // clear
  }
#endif /* EXCLUDE_LED_RING */
}

#if !defined(EXCLUDE_LED_RING)
static void LED_Clear_noflush() {
    for (uint16_t i = 0; i < RING_LED_NUM; i++) {
      uni_setPixelColor(i, LED_COLOR_BACKLIT);
    }

    if (rx_packets_counter > prev_rx_packets_counter) {
      uni_setPixelColor(LED_STATUS_RX, LED_COLOR_MI_GREEN);
      prev_rx_packets_counter = rx_packets_counter;

      if (settings->mode == SOFTRF_MODE_WATCHOUT) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          uni_setPixelColor(i, LED_COLOR_RED);
        }
      } else if (settings->mode == SOFTRF_MODE_BRIDGE) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          uni_setPixelColor(i, LED_COLOR_MI_RED);
        }
      }

    }  else {
      uni_setPixelColor(LED_STATUS_RX, LED_COLOR_BLACK);
    }

    if (tx_packets_counter > prev_tx_packets_counter) {
      uni_setPixelColor(LED_STATUS_TX, LED_COLOR_MI_GREEN);
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      uni_setPixelColor(LED_STATUS_TX, LED_COLOR_BLACK);
    }

    uni_setPixelColor(LED_STATUS_POWER,
      Battery_voltage() > Battery_threshold() ? LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
    uni_setPixelColor(LED_STATUS_SAT,
      isValidFix() ? LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
}
#endif /* EXCLUDE_LED_RING */

void LED_Clear() {
#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    SoC->swSer_enableRx(false);
    uni_show();
    SoC->swSer_enableRx(true);
  }
#endif /* EXCLUDE_LED_RING */
}

void LED_DisplayTraffic() {
#if !defined(EXCLUDE_LED_RING)
  int bearing, distance;
  int led_num;
  color_t color;

  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].addr && (now() - Container[i].timestamp) <= LED_EXPIRATION_TIME) {

        bearing  = (int) Container[i].bearing;
        distance = (int) Container[i].distance;

        if (settings->pointer == DIRECTION_TRACK_UP) {
          bearing = (360 + bearing - (int)ThisAircraft.course) % 360;
        }

        led_num = ((bearing + LED_ROTATE_ANGLE + SECTOR_PER_LED/2) % 360) / SECTOR_PER_LED;
//      Serial.print(bearing);
//      Serial.print(" , ");
//      Serial.println(led_num);
//      Serial.println(distance);
        if (distance < LED_DISTANCE_FAR) {
          if (distance >= 0 && distance <= LED_DISTANCE_CLOSE) {
            color =  LED_COLOR_RED;
          } else if (distance > LED_DISTANCE_CLOSE && distance <= LED_DISTANCE_NEAR) {
            color =  LED_COLOR_YELLOW;
          } else if (distance > LED_DISTANCE_NEAR && distance <= LED_DISTANCE_FAR) {
            color =  LED_COLOR_BLUE;
          }
          uni_setPixelColor(led_num, color);
        }
      }
    }

    SoC->swSer_enableRx(false);
    uni_show();
    SoC->swSer_enableRx(true);

  }
#endif /* EXCLUDE_LED_RING */
}

void LED_loop() {
  if (status_LED != SOC_UNUSED_PIN) {
    if (Battery_voltage() > Battery_threshold() ) {
      /* Indicate positive power supply */
      if (digitalRead(status_LED) != LED_STATE_ON) {
        digitalWrite(status_LED, LED_STATE_ON);
      }
    } else {
      if (isTimeToToggle()) {
        digitalWrite(status_LED, !digitalRead(status_LED) ? HIGH : LOW);  // toggle state
        status_LED_TimeMarker = millis();
      }
    }
  }
}

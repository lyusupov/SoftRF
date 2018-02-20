/*
 * LEDHelper.cpp
 * Copyright (C) 2016-2018 Linar Yusupov
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

#include "LEDHelper.h"
#include "BatteryHelper.h"
#include "SoCHelper.h"

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void LED_setup() {
  if (settings->pointer != LED_OFF) {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
  }
}

// Fill the dots one after the other with a color
static void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
static void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

void LED_test() {
  if (settings->pointer != LED_OFF) {
    // Some example procedures showing how to display to the pixels:
    colorWipe(strip.Color(255, 0, 0), 50); // Red
    colorWipe(strip.Color(0, 255, 0), 50); // Green
    colorWipe(strip.Color(0, 0, 255), 50); // Blue
    // Send a theater pixel chase in...
    theaterChase(strip.Color(127, 127, 127), 50); // White
    theaterChase(strip.Color(127, 0, 0), 50); // Red
    theaterChase(strip.Color(0, 0, 127), 50); // Blue

    //  rainbow(20);
    //  rainbowCycle(20);
    //  theaterChaseRainbow(50);
    colorWipe(strip.Color(0, 0, 0), 50); // clear
  }
}

static void LED_Clear_noflush() {
    for (uint16_t i = 0; i < RING_LED_NUM; i++) {
      strip.setPixelColor(i, LED_COLOR_BACKLIT);
    }

    if (rx_packets_counter > prev_rx_packets_counter) {
      strip.setPixelColor(LED_STATUS_RX, LED_COLOR_MI_GREEN);
      prev_rx_packets_counter = rx_packets_counter;

      if (settings->mode == SOFTRF_MODE_WATCHOUT) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          strip.setPixelColor(i, LED_COLOR_RED);
        }
      } else if (settings->mode == SOFTRF_MODE_BRIDGE) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          strip.setPixelColor(i, LED_COLOR_MI_RED);
        }
      }

    }  else {
      strip.setPixelColor(LED_STATUS_RX, LED_COLOR_BLACK);
    }

    if (tx_packets_counter > prev_tx_packets_counter) {
      strip.setPixelColor(LED_STATUS_TX, LED_COLOR_MI_GREEN);
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      strip.setPixelColor(LED_STATUS_TX, LED_COLOR_BLACK);
    }

    strip.setPixelColor(LED_STATUS_POWER,
      Battery_voltage() > 2.3 ? LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
    strip.setPixelColor(LED_STATUS_SAT,
      gnss.location.isValid() && (gnss.location.age()) <= 3000 ?
      LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
}

void LED_Clear() {
  if (settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    SoC->swSer_enableRx(false);
    strip.show();
    SoC->swSer_enableRx(true);
  }
}

void LED_DisplayTraffic() {
  int bearing, distance;
  int led_num;
  uint32_t color;

  if (settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].addr && (now() - Container[i].timestamp) <= LED_EXPIRATION_TIME) {
        bearing = (int) gnss.courseTo(ThisAircraft.latitude,
          ThisAircraft.longitude, Container[i].latitude, Container[i].longitude);
        distance = (int) gnss.distanceBetween(ThisAircraft.latitude,
          ThisAircraft.longitude, Container[i].latitude, Container[i].longitude);
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
          strip.setPixelColor(led_num, color);
        }
      }
    }

    SoC->swSer_enableRx(false);
    strip.show();
    SoC->swSer_enableRx(true);

  }
}

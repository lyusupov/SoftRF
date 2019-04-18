/*
 * OLEDHelper.cpp
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

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "OLEDHelper.h"

#include <Fonts/Picopixel.h>
#include <Fonts/Org_01.h>

#include <Fonts/FreeMonoBold12pt7b.h>

const char SkyView_text1[] = "SkyView";
const char SkyView_text2[] = "Presented by";
const char SkyView_text3[] = "SoftRF project";

void OLED_setup() {

  int16_t  tbx1, tby1;
  uint16_t tbw1, tbh1;
  int16_t  tbx2, tby2;
  uint16_t tbw2, tbh2;
  int16_t  tbx3, tby3;
  uint16_t tbw3, tbh3;

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!odisplay.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  odisplay.clearDisplay();

  odisplay.setFont(&FreeMonoBold12pt7b);
  odisplay.setTextColor(WHITE);

  odisplay.getTextBounds(SkyView_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);

  odisplay.fillScreen(BLACK);
  uint16_t x = (odisplay.width()  - tbw1) / 2;
  uint16_t y = (odisplay.height() + tbh1) / 2;
  odisplay.setCursor(x, y - tbh1);
  odisplay.print(SkyView_text1);

  odisplay.setFont(&Picopixel);
  odisplay.getTextBounds(SkyView_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);
  x = (odisplay.width()  - tbw2) / 2;
  y = (odisplay.height() + tbh2) / 2;
  odisplay.setCursor(x, y + tbh2);
  odisplay.print(SkyView_text2);
  odisplay.setFont(&Org_01);
  odisplay.getTextBounds(SkyView_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);
  x = (odisplay.width()  - tbw3) / 2;
  y = (odisplay.height() + tbh3) / 2;
  odisplay.setCursor(x, y + tbh2 + tbh3);
  odisplay.print(SkyView_text3);

  odisplay.display();

  delay(7000); // Pause for 7 seconds

  odisplay.clearDisplay();
  odisplay.display();
}

void OLED_loop()
{

}

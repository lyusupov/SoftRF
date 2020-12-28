/*
 * OLEDHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#if defined(RASPBERRY_PI)

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "OLEDHelper.h"

#include <Fonts/Picopixel.h>
#include <Fonts/Org_01.h>

#include <Fonts/FreeMonoBold12pt7b.h>

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "SkyView.h"

#define isTimeToDisplay() (millis() - OLEDTimeMarker > 2000)
unsigned long OLEDTimeMarker = 0;

const char OLED_SkyView_text1[] = "SkyView";
const char OLED_SkyView_text2[] = "Presented by";
const char OLED_SkyView_text3[] = "SoftRF project";

static bool OLED_display_frontpage = false;

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

  odisplay.getTextBounds(OLED_SkyView_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);

  odisplay.fillScreen(BLACK);
  uint16_t x = (odisplay.width()  - tbw1) / 2;
  uint16_t y = (odisplay.height() + tbh1) / 2;
  odisplay.setCursor(x, y - tbh1);
  odisplay.print(OLED_SkyView_text1);

  odisplay.setFont(&Picopixel);
  odisplay.getTextBounds(OLED_SkyView_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);
  x = (odisplay.width()  - tbw2) / 2;
  y = (odisplay.height() + tbh2) / 2;
  odisplay.setCursor(x, y + tbh2);
  odisplay.print(OLED_SkyView_text2);
  odisplay.setFont(&Org_01);
  odisplay.getTextBounds(OLED_SkyView_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);
  x = (odisplay.width()  - tbw3) / 2;
  y = (odisplay.height() + tbh3) / 2;
  odisplay.setCursor(x, y + tbh2 + tbh3);
  odisplay.print(OLED_SkyView_text3);

  odisplay.display();

  delay(7000); // Pause for 7 seconds

  odisplay.clearDisplay();
  odisplay.display();

  OLEDTimeMarker = millis();
}

void OLED_loop()
{
  if (hw_info.display == DISPLAY_OLED_2_4) {

    int16_t  tbx1, tby1;
    uint16_t tbw1, tbh1;

    if (!OLED_display_frontpage) {

      odisplay.setFont(&Org_01);
      odisplay.getTextBounds(COLUMN1_TITLE, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);

      uint16_t x = 0;
      uint16_t y = tbh1;
      odisplay.setCursor(x, y);
      odisplay.print(COLUMN1_TITLE);

      x += tbw1;

      odisplay.getTextBounds(COLUMN2_TITLE, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
      odisplay.setCursor(x, y);
      odisplay.print(COLUMN2_TITLE);

      x += tbw1;

      odisplay.getTextBounds(COLUMN3_TITLE, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
      odisplay.setCursor(x, y);
      odisplay.print(COLUMN3_TITLE);

      x += tbw1;

      odisplay.getTextBounds(COLUMN4_TITLE, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
      odisplay.setCursor(x, y);
      odisplay.print(COLUMN4_TITLE);

      odisplay.drawFastHLine( 0, tbh1 + 3, odisplay.width(), WHITE);

      odisplay.display();

      OLED_display_frontpage = true;

    } else {

      if (isTimeToDisplay()) {

        int j=0;
        uint16_t x = 0;
        uint16_t y = 9;
        char id_str  [9];
        char dist_str[5];
        char brg_str [4];
        char elev_str[6];

        for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
          if (Container[i].ID && (now() - Container[i].timestamp) <= OLED_EXPIRATION_TIME) {

            traffic[j].fop = &Container[i];
            traffic[j].distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                                        Container[i].RelativeEast  * Container[i].RelativeEast);
            j++;
          }
        }

        qsort(traffic, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

        odisplay.fillRect(x, y, odisplay.width(), odisplay.height() - y, BLACK);

        odisplay.setFont(&Org_01);

        for (int i=0; i < OLED_LINES_PER_PAGE; i++) {

          if (i >= j) {
            break;
          }

          x = 0;
          y += 10;

          snprintf(id_str, sizeof(id_str), "%06X", traffic[i].fop->ID);

          odisplay.setCursor(x, y);
          odisplay.print(id_str);

          float distance = constrain(traffic[i].distance / 1000.0, 0, 19.9);
          snprintf(dist_str, sizeof(dist_str), "%04.1f", distance);

          if (dist_str[0] == '0') dist_str[0] = ' ';

          x = odisplay.width() / 3;

          odisplay.setCursor(x, y);
          odisplay.print(dist_str);

          float bearing = atan2f(traffic[i].fop->RelativeNorth,
                                 traffic[i].fop->RelativeEast) * 180.0 / PI;  /* -180 ... 180 */

          /* convert from math angle into course relative to north */
          bearing = (bearing <= 90.0 ? 90.0 - bearing :
                                      450.0 - bearing);

          snprintf(brg_str, sizeof(brg_str), "%03d", (int) bearing);

          if (brg_str[0] == '0') {
            brg_str[0] = ' ';
            if (brg_str[1] == '0') {
              brg_str[1] = ' ';
            }
          }

          x = odisplay.width() / 2 + 5;

          odisplay.setCursor(x, y);
          odisplay.print(brg_str);

          if (traffic[i].fop->RelativeVertical > 0) {
            elev_str[0] = '+';
          } else if (traffic[i].fop->RelativeVertical < 0) {
            elev_str[0] = '-';
          } else {
            elev_str[0] = ' ';
          }

          int16_t elevation = constrain(abs(traffic[i].fop->RelativeVertical), 0, 9999);
          snprintf(&elev_str[1], sizeof(elev_str) - 1, "%04d", elevation);

          if (elev_str[1] == '0') {
            elev_str[1] = ' ';
            if (elev_str[2] == '0') {
              elev_str[2] = ' ';
              if (elev_str[3] == '0') {
                elev_str[3] = ' ';
              }
            }
          }

          x = (3 * odisplay.width()) / 4;

          odisplay.setCursor(x, y);
          odisplay.print(elev_str);
        }

        odisplay.display();

        OLEDTimeMarker = millis();
      }
    }
  }
}

#endif /* RASPBERRY_PI */

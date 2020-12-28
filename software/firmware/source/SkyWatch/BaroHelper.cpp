/*
 * BaroHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
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

#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPL3115A2.h>
#include <TinyGPS++.h>

#include "SoCHelper.h"
#include "BaroHelper.h"

barochip_ops_t *baro_chip = NULL;

Adafruit_BMP085 bmp180;
Adafruit_BMP280 bmp280;
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();

static unsigned long BaroTimeMarker = 0;
static float prev_pressure_altitude = 0;

#define VS_AVERAGING_FACTOR   4
static float Baro_VS[VS_AVERAGING_FACTOR];
static int avg_ndx = 0;

/* 4 baro sensor readings per second */
#define isTimeToBaro() ((millis() - BaroTimeMarker) > (1000 / VS_AVERAGING_FACTOR))

static bool bmp180_probe()
{
  return bmp180.begin();
}

static void bmp180_setup()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp180.readTemperature());
  Serial.println(F(" *C"));
  
  Serial.print(F("Pressure = "));
  Serial.print(bmp180.readPressure());
  Serial.println(F(" Pa"));
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print(F("Altitude = "));
  Serial.print(bmp180.readAltitude());
  Serial.println(F(" meters"));

  Serial.print(F("Pressure at sealevel (calculated) = "));
  Serial.print(bmp180.readSealevelPressure());
  Serial.println(F(" Pa"));

// you can get a more precise measurement of altitude
// if you know the current sea level pressure which will
// vary with weather and such. If it is 1015 millibars
// that is equal to 101500 Pascals.
  Serial.print(F("Real altitude = "));
  Serial.print(bmp180.readAltitude(101500));
  Serial.println(F(" meters"));
  
  Serial.println();
  delay(500);
}

static float bmp180_altitude(float sealevelPressure)
{
  return bmp180.readAltitude(sealevelPressure * 100);
}

barochip_ops_t bmp180_ops = {
  BARO_MODULE_BMP180,
  "BMP180",
  bmp180_probe,
  bmp180_setup,
  bmp180_altitude
};

static bool bmp280_probe()
{
  return (
          bmp280.begin(BMP280_ADDRESS,     BMP280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS,     BME280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS_ALT, BME280_CHIPID)
         );
}

static void bmp280_setup()
{
    Serial.print(F("Temperature = "));
    Serial.print(bmp280.readTemperature());
    Serial.println(F(" *C"));
    
    Serial.print(F("Pressure = "));
    Serial.print(bmp280.readPressure());
    Serial.println(F(" Pa"));

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp280.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(F(" m"));
    
    Serial.println();
    delay(500);
}

static float bmp280_altitude(float sealevelPressure)
{
    return bmp280.readAltitude(sealevelPressure);
}

barochip_ops_t bmp280_ops = {
  BARO_MODULE_BMP280,
  "BMP280",
  bmp280_probe,
  bmp280_setup,
  bmp280_altitude
};

static bool mpl3115a2_probe()
{
  return mpl3115a2.begin();
}

static void mpl3115a2_setup()
{
  float pascals = mpl3115a2.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(F(" Inches (Hg)"));

  float altm = mpl3115a2.getAltitude();
  Serial.print(altm); Serial.println(F(" meters"));

  float tempC = mpl3115a2.getTemperature();
  Serial.print(tempC); Serial.println(F("*C"));

  delay(250);
}

static float mpl3115a2_altitude(float sealevelPressure)
{
  mpl3115a2.setSeaPressure(sealevelPressure * 100);
  return mpl3115a2.getAltitude();
}

barochip_ops_t mpl3115a2_ops = {
  BARO_MODULE_MPL3115A2,
  "MPL3115A2",
  mpl3115a2_probe,
  mpl3115a2_setup,
  mpl3115a2_altitude
};

bool Baro_probe()
{
  return (
#if !defined(EXCLUDE_BMP180)
           (baro_chip = &bmp180_ops,    baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
           (baro_chip = &bmp280_ops,    baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_MPL3115A2)
           (baro_chip = &mpl3115a2_ops, baro_chip->probe())
#else
           false
#endif /* EXCLUDE_MPL3115A2 */
         );
}

byte Baro_setup()
{
  if ( SoC->Baro_setup() && Baro_probe() ) {

    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();

    prev_pressure_altitude = baro_chip->altitude(1013.25);
    BaroTimeMarker = millis();

    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      Baro_VS[i] = 0;
    }

    return baro_chip->type;

  } else {
    baro_chip = NULL;
    Serial.println(F("WARNING! Barometric pressure sensor is NOT detected."));

    return BARO_MODULE_NONE;
  }
}

void Baro_loop()
{
  if (baro_chip && isTimeToBaro()) {

    /* Draft of pressure altitude and vertical speed calculation */
    ThisDevice.pressure_altitude = baro_chip->altitude(1013.25);

    Baro_VS[avg_ndx] = (ThisDevice.pressure_altitude - prev_pressure_altitude) /
      (millis() - BaroTimeMarker) * 1000;  /* in m/s */

    ThisDevice.vs = 0;
    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      ThisDevice.vs += Baro_VS[i];
    }
    ThisDevice.vs /= VS_AVERAGING_FACTOR;

    if (ThisDevice.vs > -0.1 && ThisDevice.vs < 0.1) {
      ThisDevice.vs = 0;
    }

    ThisDevice.vs *= (_GPS_FEET_PER_METER * 60.0) ; /* feet per minute */

    prev_pressure_altitude = ThisDevice.pressure_altitude;
    BaroTimeMarker = millis();
    avg_ndx = (avg_ndx + 1) % VS_AVERAGING_FACTOR;

#if 0
    Serial.print(F("P.Alt. = ")); Serial.print(ThisDevice.pressure_altitude);
    Serial.print(F(" , VS avg. = ")); Serial.println(ThisDevice.vs);
#endif
  }
}
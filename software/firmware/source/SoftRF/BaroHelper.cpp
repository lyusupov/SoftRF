/*
 * BaroHelper.cpp
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

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPL3115A2.h>

#include "SoCHelper.h"
#include "BaroHelper.h"

barochip_ops_t *baro_chip = NULL;

#if defined(SOFTRF_LORA_PCB_1_2_PROTO)

Adafruit_BMP085 bmp180;
Adafruit_BMP280 bmp280;
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();

static bool bmp180_probe()
{
  return bmp180.begin();
}

static void bmp180_setup()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp180.readTemperature());
  Serial.println(" *C");
  
  Serial.print(F("Pressure = "));
  Serial.print(bmp180.readPressure());
  Serial.println(" Pa");
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print(F("Altitude = "));
  Serial.print(bmp180.readAltitude());
  Serial.println(" meters");

  Serial.print(F("Pressure at sealevel (calculated) = "));
  Serial.print(bmp180.readSealevelPressure());
  Serial.println(" Pa");

// you can get a more precise measurement of altitude
// if you know the current sea level pressure which will
// vary with weather and such. If it is 1015 millibars
// that is equal to 101500 Pascals.
  Serial.print(F("Real altitude = "));
  Serial.print(bmp180.readAltitude(101500));
  Serial.println(" meters");
  
  Serial.println();
  delay(500);
}

barochip_ops_t bmp180_ops = {
  BARO_MODULE_BMP180,
  "BMP180",
  bmp180_probe,
  bmp180_setup
};

static bool bmp280_probe()
{
  return bmp280.begin() || bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
}

static void bmp280_setup()
{
    Serial.print(F("Temperature = "));
    Serial.print(bmp280.readTemperature());
    Serial.println(" *C");
    
    Serial.print(F("Pressure = "));
    Serial.print(bmp280.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp280.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(" m");
    
    Serial.println();
    delay(500);
}

barochip_ops_t bmp280_ops = {
  BARO_MODULE_BMP280,
  "BMP280",
  bmp280_probe,
  bmp280_setup
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
  Serial.print(altm); Serial.println(" meters");

  float tempC = mpl3115a2.getTemperature();
  Serial.print(tempC); Serial.println("*C");

  delay(250);
}

barochip_ops_t mpl3115a2_ops = {
  BARO_MODULE_MPL3115A2,
  "MPL3115A2",
  mpl3115a2_probe,
  mpl3115a2_setup
};

#endif /* SOFTRF_LORA_PCB_1_2_PROTO */

void Baro_setup()
{

#if defined(SOFTRF_LORA_PCB_1_2_PROTO)

  Wire.pins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  baro_chip = &bmp180_ops;

  if (baro_chip->probe()) {
    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();
    return;
  }

  baro_chip = &bmp280_ops;

  if (baro_chip->probe()) {
    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();
    return;
  }

  baro_chip = &mpl3115a2_ops;

  if (baro_chip->probe()) {
    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();
  } else {
    Serial.println(F("WARNING! Barometric pressure sensor is NOT detected."));
  }

#endif /* SOFTRF_LORA_PCB_1_2_PROTO */
}

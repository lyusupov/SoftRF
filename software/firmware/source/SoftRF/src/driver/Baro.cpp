/*
 * BaroHelper.cpp
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

#include "../system/SoC.h"

#include "Baro.h"

#if defined(EXCLUDE_BMP180) && defined(EXCLUDE_BMP280)    && \
    defined(EXCLUDE_BME680) && defined(EXCLUDE_BME280AUX) && \
    defined(EXCLUDE_MPL3115A2)
byte  Baro_setup()        {return BARO_MODULE_NONE;}
void  Baro_loop()         {}
void  Baro_fini()         {}
float Baro_altitude()     {return 0;}
float Baro_pressure()     {return 0;}
float Baro_temperature()  {return 0;}
#else

#if !defined(EXCLUDE_BMP180)
#include <Adafruit_BMP085.h>
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
#include <Adafruit_BMP280.h>
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_BME680)
#include <Adafruit_BME680.h>
#endif /* EXCLUDE_BME680 */
#if !defined(EXCLUDE_BME280AUX)
#include <SensorBHI260AP.hpp>
#endif /* EXCLUDE_BME280AUX */
#if !defined(EXCLUDE_MPL3115A2)
#include <Adafruit_MPL3115A2.h>
#endif /* EXCLUDE_MPL3115A2 */

#include <TinyGPS++.h>

barochip_ops_t *baro_chip = NULL;

#if !defined(EXCLUDE_BMP180)
Adafruit_BMP085 bmp180;
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
Adafruit_BMP280 bmp280;
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_BME680)
Adafruit_BME680 bme680;
#endif /* EXCLUDE_BME680 */
#if !defined(EXCLUDE_BME280AUX)
SensorBHI260AP bhy;
#endif /* EXCLUDE_BME280AUX */
#if !defined(EXCLUDE_MPL3115A2)
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();
#endif /* EXCLUDE_MPL3115A2 */

static float Baro_altitude_cache            = 0;
static float Baro_pressure_cache            = 0;
static float Baro_temperature_cache         = 0;

static unsigned long BaroAltitudeTimeMarker = 0;
static unsigned long BaroPresTempTimeMarker = 0;
static float prev_pressure_altitude         = 0;

static float Baro_VS[VS_AVERAGING_FACTOR];
static int avg_ndx = 0;

#if !defined(EXCLUDE_BMP180)
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

  Serial.println();
  delay(500);
}

static void bmp180_fini()
{
  /* TBD */
}

static float bmp180_altitude(float sealevelPressure)
{
  return bmp180.readAltitude(sealevelPressure * 100);
}

static float bmp180_pressure()
{
  return (float) bmp180.readPressure();
}

static float bmp180_temperature()
{
  return bmp180.readTemperature();
}

barochip_ops_t bmp180_ops = {
  BARO_MODULE_BMP180,
  "BMP180",
  bmp180_probe,
  bmp180_setup,
  bmp180_fini,
  bmp180_altitude,
  bmp180_pressure,
  bmp180_temperature
};
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
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

static void bmp280_fini()
{
    bmp280.reset();
}

static float bmp280_altitude(float sealevelPressure)
{
    return bmp280.readAltitude(sealevelPressure);
}

static float bmp280_pressure()
{
    return bmp280.readPressure();
}

static float bmp280_temperature()
{
    return bmp280.readTemperature();
}

barochip_ops_t bmp280_ops = {
  BARO_MODULE_BMP280,
  "BMx280",
  bmp280_probe,
  bmp280_setup,
  bmp280_fini,
  bmp280_altitude,
  bmp280_pressure,
  bmp280_temperature
};
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_BME680)
static bool bme680_probe()
{
  return bme680.begin(BME68X_DEFAULT_ADDRESS);
}

static void bme680_setup()
{
    Serial.print(F("Temperature = "));
    Serial.print(bme680.readTemperature());
    Serial.println(F(" *C"));

    Serial.print(F("Pressure = "));
    Serial.print(bme680.readPressure());
    Serial.println(F(" Pa"));

    Serial.print(F("Approx altitude = "));
    Serial.print(bme680.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(F(" m"));

    Serial.println();
    delay(500);
}

static void bme680_fini()
{
  /* TBD */
}

static float bme680_altitude(float sealevelPressure)
{
    return bme680.readAltitude(sealevelPressure);
}

static float bme680_pressure()
{
    return bme680.readPressure();
}

static float bme680_temperature()
{
    return bme680.readTemperature();
}

barochip_ops_t bme680_ops = {
  BARO_MODULE_BME680,
  "BME68x",
  bme680_probe,
  bme680_setup,
  bme680_fini,
  bme680_altitude,
  bme680_pressure,
  bme680_temperature
};
#endif /* EXCLUDE_BME680 */

#if !defined(EXCLUDE_BME280AUX)
static float aux_temperature = 0;
static float aux_pressure    = 0; /* hPa */

void parse_bme280_sensor_data(uint8_t sensor_id, uint8_t *data_ptr,
                              uint32_t len, uint64_t *timestamp, void *user_data)
{
    switch (sensor_id) {
    case SensorBHI260AP::TEMPERATURE:
        bhy2_parse_temperature_celsius(data_ptr, &aux_temperature);
        Serial.print("temperature: ");
        Serial.print(aux_temperature);
        Serial.println(" *C");
        break;
    case SensorBHI260AP::BAROMETER:
        bhy2_parse_pressure(data_ptr, &aux_pressure);
        Serial.print("pressure: ");
        Serial.print(aux_pressure);
        Serial.println(" hPa");
        break;
    default:
        break;
    }
}

static bool bme280aux_probe()
{
    return false; /* TBD */
}

static void bme280aux_setup()
{
    bhy.setPins(SOC_GPIO_PIN_IMU_TULTIMA_RST);
    bhy.setBootFromFlash(true);

    if (!bhy.begin(SPI,
                   SOC_GPIO_PIN_IMU_TULTIMA_SS, SOC_GPIO_PIN_TULTIMA_MOSI,
                   SOC_GPIO_PIN_TULTIMA_MISO,   SOC_GPIO_PIN_TULTIMA_SCK)) {
        Serial.print("Failed to init BHI260AP - ");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init BHI260AP Sensor success!");

    // Output all sensors info to Serial
    BoschSensorInfo info = bhy.getSensorInfo();
#ifdef PLATFORM_HAS_PRINTF
    info.printInfo(Serial);
#else
    info.printInfo();
#endif

    float sample_rate          = 0.0; /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0;   /* Report immediately */

    /*
     * Enable BME280 function
     * Function depends on BME280.
     * If the hardware is not connected to BME280, the function cannot be used.
     */
    bhy.configure(SensorBHI260AP::TEMPERATURE, sample_rate, report_latency_ms);
    bhy.configure(SensorBHI260AP::BAROMETER, sample_rate, report_latency_ms);

    bhy.onResultEvent(SensorBHI260AP::TEMPERATURE, parse_bme280_sensor_data);
    bhy.onResultEvent(SensorBHI260AP::BAROMETER, parse_bme280_sensor_data);
}

static void bme280aux_fini()
{
    /* TBD */
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
static float bme280aux_altitude(float seaLevelhPa)
{
    return 44330 * (1.0 - pow(aux_pressure / seaLevelhPa, 0.1903));
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
static float bme280aux_pressure()
{
    return aux_pressure * 100;
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degress celcius.
 */
static float bme280aux_temperature()
{
    return aux_temperature;
}

barochip_ops_t bme280aux_ops = {
  BARO_MODULE_BME280AUX,
  "BME280",
  bme280aux_probe,
  bme280aux_setup,
  bme280aux_fini,
  bme280aux_altitude,
  bme280aux_pressure,
  bme280aux_temperature
};
#endif /* EXCLUDE_BME280AUX */

#if !defined(EXCLUDE_MPL3115A2)
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

static void mpl3115a2_fini()
{
  /* TBD */
}

static float mpl3115a2_altitude(float sealevelPressure)
{
  mpl3115a2.setSeaPressure(sealevelPressure * 100);
  return mpl3115a2.getAltitude();
}

static float mpl3115a2_pressure()
{
  return mpl3115a2.getPressure();
}

static float mpl3115a2_temperature()
{
  return mpl3115a2.getTemperature();
}

barochip_ops_t mpl3115a2_ops = {
  BARO_MODULE_MPL3115A2,
  "MPL3115A2",
  mpl3115a2_probe,
  mpl3115a2_setup,
  mpl3115a2_fini,
  mpl3115a2_altitude,
  mpl3115a2_pressure,
  mpl3115a2_temperature
};
#endif /* EXCLUDE_MPL3115A2 */

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

#if !defined(EXCLUDE_BME680)
           (baro_chip = &bme680_ops,    baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BME680 */

#if !defined(EXCLUDE_BME280AUX)
           (baro_chip = &bme280aux_ops, baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BME280AUX */

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

    Baro_pressure_cache    = baro_chip->pressure();
    Baro_temperature_cache = baro_chip->temperature();
    BaroPresTempTimeMarker = millis();

    Baro_altitude_cache    = baro_chip->altitude(1013.25);
    ThisAircraft.pressure_altitude = prev_pressure_altitude = Baro_altitude_cache;
    BaroAltitudeTimeMarker = millis();

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
  if (baro_chip == NULL) return;

  if (isTimeToBaroAltitude()) {

#if !defined(EXCLUDE_BME280AUX)
    if (baro_chip->type == BARO_MODULE_BME280AUX) {
      bhy.update();
    }
#endif /* EXCLUDE_BME280AUX */

    /* Draft of pressure altitude and vertical speed calculation */
    Baro_altitude_cache = baro_chip->altitude(1013.25);

    ThisAircraft.pressure_altitude = Baro_altitude_cache;

    Baro_VS[avg_ndx] = (Baro_altitude_cache - prev_pressure_altitude) /
                       (millis() - BaroAltitudeTimeMarker) * 1000;  /* in m/s */

    ThisAircraft.vs = 0;
    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      ThisAircraft.vs += Baro_VS[i];
    }
    ThisAircraft.vs /= VS_AVERAGING_FACTOR;

    if (ThisAircraft.vs > -0.1 && ThisAircraft.vs < 0.1) {
      ThisAircraft.vs = 0;
    }

    ThisAircraft.vs *= (_GPS_FEET_PER_METER * 60.0) ; /* feet per minute */

    prev_pressure_altitude = Baro_altitude_cache;
    BaroAltitudeTimeMarker = millis();
    avg_ndx = (avg_ndx + 1) % VS_AVERAGING_FACTOR;

#if 0
    Serial.print(F("P.Alt. = ")); Serial.print(ThisAircraft.pressure_altitude);
    Serial.print(F(" , VS avg. = ")); Serial.println(ThisAircraft.vs);
#endif
  }

  if (isTimeToBaroPresTemp()) {
    Baro_pressure_cache    = baro_chip->pressure();
    Baro_temperature_cache = baro_chip->temperature();
    BaroPresTempTimeMarker = millis();
  }
}

void Baro_fini()
{
  if (baro_chip != NULL) baro_chip->fini();
}

float Baro_altitude()
{
  return Baro_altitude_cache;
}

float Baro_pressure()
{
  return Baro_pressure_cache;
}

float Baro_temperature()
{
  return Baro_temperature_cache;
}

#endif /* EXCLUDE_BMP180 && EXCLUDE_BMP280 EXCLUDE_BME680 EXCLUDE_MPL3115A2 */

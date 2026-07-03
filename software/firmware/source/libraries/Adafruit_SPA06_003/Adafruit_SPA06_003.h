/*!
 * @file Adafruit_SPA06_003.h
 *
 * This is part of Adafruit's SPA06_003 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit SPA06_003 breakout:
 * https://www.adafruit.com/products/xxxx
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 * for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef _ADAFRUIT_SPA06_003_H
#define _ADAFRUIT_SPA06_003_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

#if !(defined(ESP8266)              || defined(ESP32)                 || \
      defined(ENERGIA_ARCH_CC13XX)  || defined(ENERGIA_ARCH_CC13X2)   || \
      defined(ARDUINO_ARCH_STM32)   || defined(__ASR6501__)           || \
      defined(ARDUINO_ARCH_ASR650X) || defined(ARDUINO_ARCH_ASR6601)  || \
      defined(ARDUINO_ARCH_NRF52)   || defined(ARDUINO_ARCH_NRF52840) || \
      defined(ARDUINO_ARCH_SAMD)    || defined(ARDUINO_ARCH_AVR)      || \
      defined(ARDUINO_ARCH_RP2040)  || defined(HACKRF_ONE)            || \
      defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_SILABS)   || \
      defined(ARDUINO_ARCH_CH32)    || defined(ARDUINO_ARCH_RP2350)   || \
      defined(ARDUINO_ARCH_ZEPHYR)  || defined(RASPBERRY_PI)          || \
      defined(LUCKFOX_LYRA)         || defined(ARDUINO_ARCH_NRF54L15CLEAN))
#include <Adafruit_Sensor.h>
#endif

#include <SPI.h>
#include <Wire.h>

#include "Arduino.h"

// Forward declarations
class Adafruit_SPA06_003_Temp;
class Adafruit_SPA06_003_Pressure;

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
/**
 * @brief Default I2C address for SPA06_003
 */
#define SPA06_003_DEFAULT_ADDR 0x77

/*=========================================================================
    SPI SETTINGS
    -----------------------------------------------------------------------*/
/**
 * @brief Default SPI frequency for SPA06_003
 */
#define SPA06_003_DEFAULT_SPIFREQ 1000000  ///< 1 MHz

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

/**
 * @brief Pressure data registers
 */
#define SPA06_003_REG_PSR_B2 0x00  ///< Pressure data byte 2 (MSB)
#define SPA06_003_REG_PSR_B1 0x01  ///< Pressure data byte 1
#define SPA06_003_REG_PSR_B0 0x02  ///< Pressure data byte 0 (LSB)

/**
 * @brief Temperature data registers
 */
#define SPA06_003_REG_TMP_B2 0x03  ///< Temperature data byte 2 (MSB)
#define SPA06_003_REG_TMP_B1 0x04  ///< Temperature data byte 1
#define SPA06_003_REG_TMP_B0 0x05  ///< Temperature data byte 0 (LSB)

/**
 * @brief Configuration registers
 */
#define SPA06_003_REG_PRS_CFG 0x06   ///< Pressure configuration register
#define SPA06_003_REG_TMP_CFG 0x07   ///< Temperature configuration register
#define SPA06_003_REG_MEAS_CFG 0x08  ///< Measurement configuration register
#define SPA06_003_REG_CFG_REG 0x09   ///< General configuration register

/**
 * @brief Status registers
 */
#define SPA06_003_REG_INT_STS 0x0A   ///< Interrupt status register
#define SPA06_003_REG_FIFO_STS 0x0B  ///< FIFO status register

/**
 * @brief Control registers
 */
#define SPA06_003_REG_RESET 0x0C  ///< Reset register
#define SPA06_003_REG_ID 0x0D     ///< Chip ID register

/**
 * @brief Calibration coefficient registers
 */
#define SPA06_003_REG_COEF 0x10  ///< Calibration coefficients start address

/*=========================================================================
    INTERRUPT STATUS FLAGS (INT_STS register 0x0A)
    -----------------------------------------------------------------------*/
/**
 * @brief Interrupt status flag definitions
 */
#define SPA06_003_INT_FIFO_FULL 0x04  ///< FIFO full flag
#define SPA06_003_INT_TMP_RDY 0x02    ///< Temperature measurement ready flag
#define SPA06_003_INT_PRS_RDY 0x01    ///< Pressure measurement ready flag

/*=========================================================================*/

/**
 * @brief Measurement rate options (pressure and temperature)
 */
typedef enum {
  SPA06_003_RATE_1 = 0x00,      ///< 1 measurements per second
  SPA06_003_RATE_2 = 0x01,      ///< 2 measurements per second
  SPA06_003_RATE_4 = 0x02,      ///< 4 measurements per second
  SPA06_003_RATE_8 = 0x03,      ///< 8 measurements per second
  SPA06_003_RATE_16 = 0x04,     ///< 16 measurements per second
  SPA06_003_RATE_32 = 0x05,     ///< 32 measurements per second
  SPA06_003_RATE_64 = 0x06,     ///< 64 measurements per second
  SPA06_003_RATE_128 = 0x07,    ///< 128 measurements per second
  SPA06_003_RATE_25_16 = 0x08,  ///< 25/16 samples per second
  SPA06_003_RATE_25_8 = 0x09,   ///< 25/8 samples per second
  SPA06_003_RATE_25_4 = 0x0A,   ///< 25/4 samples per second
  SPA06_003_RATE_25_2 = 0x0B,   ///< 25/2 samples per second
  SPA06_003_RATE_25 = 0x0C,     ///< 25 samples per second
  SPA06_003_RATE_50 = 0x0D,     ///< 50 samples per second
  SPA06_003_RATE_100 = 0x0E,    ///< 100 samples per second
  SPA06_003_RATE_200 = 0x0F     ///< 200 samples per second
} spa06_003_rate_t;

/**
 * @brief Oversampling rate options (shared by pressure and temperature)
 */
typedef enum {
  SPA06_003_OVERSAMPLE_1 = 0x00,   ///< Single
  SPA06_003_OVERSAMPLE_2 = 0x01,   ///< 2 times
  SPA06_003_OVERSAMPLE_4 = 0x02,   ///< 4 times
  SPA06_003_OVERSAMPLE_8 = 0x03,   ///< 8 times
  SPA06_003_OVERSAMPLE_16 = 0x04,  ///< 16 times
  SPA06_003_OVERSAMPLE_32 = 0x05,  ///< 32 times
  SPA06_003_OVERSAMPLE_64 = 0x06,  ///< 64 times
  SPA06_003_OVERSAMPLE_128 = 0x07  ///< 128 times
} spa06_003_oversample_t;

/**
 * @brief Measurement mode options
 */
typedef enum {
  SPA06_003_MEAS_IDLE = 0x00,      ///< Idle / Stop background measurement
  SPA06_003_MEAS_PRESSURE = 0x01,  ///< Pressure measurement (Command Mode)
  SPA06_003_MEAS_TEMPERATURE =
      0x02,  ///< Temperature measurement (Command Mode)
  SPA06_003_MEAS_CONTINUOUS_PRESSURE =
      0x05,  ///< Continuous pressure measurement (Background Mode)
  SPA06_003_MEAS_CONTINUOUS_TEMPERATURE =
      0x06,  ///< Continuous temperature measurement (Background Mode)
  SPA06_003_MEAS_CONTINUOUS_BOTH =
      0x07  ///< Continuous pressure and temperature measurement (Background
            ///< Mode)
} spa06_003_meas_mode_t;

/**
 * @brief Interrupt polarity options
 */
typedef enum {
  SPA06_003_INT_ACTIVE_LOW = 0x00,  ///< Interrupt active low
  SPA06_003_INT_ACTIVE_HIGH = 0x01  ///< Interrupt active high
} spa06_003_int_polarity_t;

/*!
 * @brief Class that stores state and functions for interacting with SPA06_003
 */
class Adafruit_SPA06_003 {
 public:
  Adafruit_SPA06_003();
  ~Adafruit_SPA06_003();

  // I2C initialization
  bool begin(uint8_t i2c_addr = SPA06_003_DEFAULT_ADDR, TwoWire *wire = &Wire);

  // Hardware SPI initialization
  bool begin(int8_t cspin, SPIClass *theSPI);

  // Software SPI initialization
  bool begin(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  uint32_t getPressureData();
  uint32_t getTemperatureData();
  spa06_003_rate_t getPressureMeasureRate();
  bool setPressureMeasureRate(spa06_003_rate_t rate);
  spa06_003_rate_t getTemperatureMeasureRate();
  bool setTemperatureMeasureRate(spa06_003_rate_t rate);
  spa06_003_oversample_t getPressureOversampling();
  bool setPressureOversampling(spa06_003_oversample_t prc);
  spa06_003_oversample_t getTemperatureOversampling();
  bool setTemperatureOversampling(spa06_003_oversample_t prc);
  bool isCoeffReady();
  bool isSensorReady();
  bool isTempDataReady();
  bool isPresDataReady();
  spa06_003_meas_mode_t getMeasurementMode();
  bool setMeasurementMode(spa06_003_meas_mode_t mode);
  bool setInterruptPolarity(spa06_003_int_polarity_t polarity);
  bool setInterruptSource(bool fifo, bool temp_ready, bool pres_ready);
  bool setTempShift(bool enable);
  bool setPresShift(bool enable);
  bool enableFIFO(bool enable);
  bool isFIFOEnabled();
  bool isFIFOEmpty();
  bool isFIFOFull();
  uint8_t getStatusFlags();
  bool flushFIFO();
  bool reset();
  float readTemperature();
  float readPressure();
  float calculateTemperature(uint32_t raw_temp);
  float calculatePressure(uint32_t raw_pres, uint32_t raw_temp);

#if 0
  Adafruit_Sensor *getTemperatureSensor();
  Adafruit_Sensor *getPressureSensor();
#endif

 private:
  Adafruit_I2CDevice *i2c_dev;
  Adafruit_SPIDevice *spi_dev;

#if 0
  // Adafruit Sensor objects
  Adafruit_SPA06_003_Temp *temp_sensor = NULL;
  Adafruit_SPA06_003_Pressure *pressure_sensor = NULL;
#endif

  // Calibration coefficients
  int16_t c0, c1;
  int32_t c00, c10;                 // 20-bit 2's complement
  int16_t c01, c11, c20, c21, c30;  // 16-bit 2's complement
  int16_t c31;                      // 12-bit 2's complement
  int16_t c40;                      // 12-bit 2's complement

  bool _init();
  bool readCoefficients();
  float getScalingFactor(spa06_003_oversample_t oversample);
};

#if 0
/*!
 * @brief Adafruit Unified Sensor interface for temperature component of
 * SPA06_003
 */
class Adafruit_SPA06_003_Temp : public Adafruit_Sensor {
 public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the SPA06_003 class */
  Adafruit_SPA06_003_Temp(Adafruit_SPA06_003 *parent) {
    _theSPA06003 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

 private:
  int _sensorID = 60003;
  Adafruit_SPA06_003 *_theSPA06003 = NULL;
};

/*!
 * @brief Adafruit Unified Sensor interface for pressure component of SPA06_003
 */
class Adafruit_SPA06_003_Pressure : public Adafruit_Sensor {
 public:
  /** @brief Create an Adafruit_Sensor compatible object for the pressure sensor
      @param parent A pointer to the SPA06_003 class */
  Adafruit_SPA06_003_Pressure(Adafruit_SPA06_003 *parent) {
    _theSPA06003 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

 private:
  int _sensorID = 60003;
  Adafruit_SPA06_003 *_theSPA06003 = NULL;
};
#endif

#endif /* _ADAFRUIT_SPA06_003_H */

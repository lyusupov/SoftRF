/*!
 *  @file Adafruit_BMP5xx.h
 *
 * 	I2C Driver for BMP5xx pressure and temperature sensor
 *
 * 	This is a library for the Adafruit BMP5xx breakout:
 * 	https://www.adafruit.com/products/xxxx
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @author   Limor "ladyada" Fried (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_BMP5XX_H
#define ADAFRUIT_BMP5XX_H

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

#include <Wire.h>

#include "Arduino.h"

extern "C" {
#include "bmp5.h"
}

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
/**! Default I2C address */
#define BMP5XX_DEFAULT_ADDRESS (0x46)
/**! Alternative I2C address */
#define BMP5XX_ALTERNATIVE_ADDRESS (0x47)

/**! Chip ID for BMP580 */
#define BMP580_CHIP_ID (0x50)
/**! Chip ID for BMP581 */
#define BMP581_CHIP_ID (0x51)

/*=========================================================================*/

/**
 * @brief Sampling rate enum
 */
typedef enum {
  BMP5XX_OVERSAMPLING_1X = BMP5_OVERSAMPLING_1X,     ///< 1x oversampling
  BMP5XX_OVERSAMPLING_2X = BMP5_OVERSAMPLING_2X,     ///< 2x oversampling
  BMP5XX_OVERSAMPLING_4X = BMP5_OVERSAMPLING_4X,     ///< 4x oversampling
  BMP5XX_OVERSAMPLING_8X = BMP5_OVERSAMPLING_8X,     ///< 8x oversampling
  BMP5XX_OVERSAMPLING_16X = BMP5_OVERSAMPLING_16X,   ///< 16x oversampling
  BMP5XX_OVERSAMPLING_32X = BMP5_OVERSAMPLING_32X,   ///< 32x oversampling
  BMP5XX_OVERSAMPLING_64X = BMP5_OVERSAMPLING_64X,   ///< 64x oversampling
  BMP5XX_OVERSAMPLING_128X = BMP5_OVERSAMPLING_128X, ///< 128x oversampling
} bmp5xx_oversampling_t;

/**
 * @brief IIR filter coefficients
 */
typedef enum {
  BMP5XX_IIR_FILTER_BYPASS = BMP5_IIR_FILTER_BYPASS,       ///< No filtering
  BMP5XX_IIR_FILTER_COEFF_1 = BMP5_IIR_FILTER_COEFF_1,     ///< Filter coeff 1
  BMP5XX_IIR_FILTER_COEFF_3 = BMP5_IIR_FILTER_COEFF_3,     ///< Filter coeff 3
  BMP5XX_IIR_FILTER_COEFF_7 = BMP5_IIR_FILTER_COEFF_7,     ///< Filter coeff 7
  BMP5XX_IIR_FILTER_COEFF_15 = BMP5_IIR_FILTER_COEFF_15,   ///< Filter coeff 15
  BMP5XX_IIR_FILTER_COEFF_31 = BMP5_IIR_FILTER_COEFF_31,   ///< Filter coeff 31
  BMP5XX_IIR_FILTER_COEFF_63 = BMP5_IIR_FILTER_COEFF_63,   ///< Filter coeff 63
  BMP5XX_IIR_FILTER_COEFF_127 = BMP5_IIR_FILTER_COEFF_127, ///< Filter coeff 127
} bmp5xx_iir_filter_t;

/**
 * @brief Output Data Rate settings
 */
typedef enum {
  BMP5XX_ODR_240_HZ = BMP5_ODR_240_HZ,     ///< 240 Hz
  BMP5XX_ODR_218_5_HZ = BMP5_ODR_218_5_HZ, ///< 218.5 Hz
  BMP5XX_ODR_199_1_HZ = BMP5_ODR_199_1_HZ, ///< 199.1 Hz
  BMP5XX_ODR_179_2_HZ = BMP5_ODR_179_2_HZ, ///< 179.2 Hz
  BMP5XX_ODR_160_HZ = BMP5_ODR_160_HZ,     ///< 160 Hz
  BMP5XX_ODR_149_3_HZ = BMP5_ODR_149_3_HZ, ///< 149.3 Hz
  BMP5XX_ODR_140_HZ = BMP5_ODR_140_HZ,     ///< 140 Hz
  BMP5XX_ODR_129_8_HZ = BMP5_ODR_129_8_HZ, ///< 129.8 Hz
  BMP5XX_ODR_120_HZ = BMP5_ODR_120_HZ,     ///< 120 Hz
  BMP5XX_ODR_110_1_HZ = BMP5_ODR_110_1_HZ, ///< 110.1 Hz
  BMP5XX_ODR_100_2_HZ = BMP5_ODR_100_2_HZ, ///< 100.2 Hz
  BMP5XX_ODR_89_6_HZ = BMP5_ODR_89_6_HZ,   ///< 89.6 Hz
  BMP5XX_ODR_80_HZ = BMP5_ODR_80_HZ,       ///< 80 Hz
  BMP5XX_ODR_70_HZ = BMP5_ODR_70_HZ,       ///< 70 Hz
  BMP5XX_ODR_60_HZ = BMP5_ODR_60_HZ,       ///< 60 Hz
  BMP5XX_ODR_50_HZ = BMP5_ODR_50_HZ,       ///< 50 Hz
  BMP5XX_ODR_45_HZ = BMP5_ODR_45_HZ,       ///< 45 Hz
  BMP5XX_ODR_40_HZ = BMP5_ODR_40_HZ,       ///< 40 Hz
  BMP5XX_ODR_35_HZ = BMP5_ODR_35_HZ,       ///< 35 Hz
  BMP5XX_ODR_30_HZ = BMP5_ODR_30_HZ,       ///< 30 Hz
  BMP5XX_ODR_25_HZ = BMP5_ODR_25_HZ,       ///< 25 Hz
  BMP5XX_ODR_20_HZ = BMP5_ODR_20_HZ,       ///< 20 Hz
  BMP5XX_ODR_15_HZ = BMP5_ODR_15_HZ,       ///< 15 Hz
  BMP5XX_ODR_10_HZ = BMP5_ODR_10_HZ,       ///< 10 Hz
  BMP5XX_ODR_05_HZ = BMP5_ODR_05_HZ,       ///< 5 Hz
  BMP5XX_ODR_04_HZ = BMP5_ODR_04_HZ,       ///< 4 Hz
  BMP5XX_ODR_03_HZ = BMP5_ODR_03_HZ,       ///< 3 Hz
  BMP5XX_ODR_02_HZ = BMP5_ODR_02_HZ,       ///< 2 Hz
  BMP5XX_ODR_01_HZ = BMP5_ODR_01_HZ,       ///< 1 Hz
  BMP5XX_ODR_0_5_HZ = BMP5_ODR_0_5_HZ,     ///< 0.5 Hz
  BMP5XX_ODR_0_250_HZ = BMP5_ODR_0_250_HZ, ///< 0.25 Hz
  BMP5XX_ODR_0_125_HZ = BMP5_ODR_0_125_HZ, ///< 0.125 Hz
} bmp5xx_odr_t;

/**
 * @brief Power mode settings
 */
typedef enum {
  BMP5XX_POWERMODE_STANDBY = BMP5_POWERMODE_STANDBY, ///< Standby mode
  BMP5XX_POWERMODE_NORMAL = BMP5_POWERMODE_NORMAL,   ///< Normal mode
  BMP5XX_POWERMODE_FORCED =
      BMP5_POWERMODE_FORCED, ///< Forced mode: single shot, then standby
  BMP5XX_POWERMODE_CONTINUOUS = BMP5_POWERMODE_CONTINOUS, ///< Continuous mode
  BMP5XX_POWERMODE_CONTINOUS =
      BMP5_POWERMODE_CONTINOUS, ///< @deprecated Use BMP5XX_POWERMODE_CONTINUOUS
  BMP5XX_POWERMODE_DEEP_STANDBY =
      BMP5_POWERMODE_DEEP_STANDBY, ///< Deep standby mode
} bmp5xx_powermode_t;

/**
 * @brief Interrupt polarity settings
 */
typedef enum {
  BMP5XX_INTERRUPT_ACTIVE_LOW = BMP5_ACTIVE_LOW,  ///< Interrupt active low
  BMP5XX_INTERRUPT_ACTIVE_HIGH = BMP5_ACTIVE_HIGH ///< Interrupt active high
} bmp5xx_interrupt_polarity_t;

/**
 * @brief Interrupt drive settings
 */
typedef enum {
  BMP5XX_INTERRUPT_PUSH_PULL = BMP5_INTR_PUSH_PULL,  ///< Push-pull output
  BMP5XX_INTERRUPT_OPEN_DRAIN = BMP5_INTR_OPEN_DRAIN ///< Open-drain output
} bmp5xx_interrupt_drive_t;

/**
 * @brief Interrupt mode settings
 */
typedef enum {
  BMP5XX_INTERRUPT_PULSED = BMP5_PULSED,  ///< Pulsed interrupt
  BMP5XX_INTERRUPT_LATCHED = BMP5_LATCHED ///< Latched interrupt
} bmp5xx_interrupt_mode_t;

/**
 * @brief Interrupt source settings (can be combined with bitwise OR)
 */
typedef enum {
  BMP5XX_INTERRUPT_DATA_READY = 0x01,     ///< Data ready interrupt
  BMP5XX_INTERRUPT_FIFO_FULL = 0x02,      ///< FIFO full interrupt
  BMP5XX_INTERRUPT_FIFO_THRESHOLD = 0x04, ///< FIFO threshold interrupt
  BMP5XX_INTERRUPT_PRESSURE_OUT_OF_RANGE =
      0x08 ///< Pressure out of range interrupt
} bmp5xx_interrupt_source_t;

#if 0
/**
 * @brief Adafruit Unified Sensor interface for temperature component of BMP5xx
 */
class Adafruit_BMP5xx_Temp : public Adafruit_Sensor {
 public:
  /**
   * @brief Construct a new Adafruit_BMP5xx_Temp object
   *
   * @param parent A pointer to the BMP5xx class
   */
  Adafruit_BMP5xx_Temp(class Adafruit_BMP5xx* parent) {
    _theBMP5xx = parent;
  }

  bool getEvent(sensors_event_t* event);
  void getSensor(sensor_t* sensor);

 private:
  int _sensorID = 0x580; /**< ID number for temperature sensor */
  class Adafruit_BMP5xx* _theBMP5xx = NULL; /**< Pointer to BMP5xx instance */
};

/**
 * @brief Adafruit Unified Sensor interface for pressure component of BMP5xx
 */
class Adafruit_BMP5xx_Pressure : public Adafruit_Sensor {
 public:
  /**
   * @brief Construct a new Adafruit_BMP5xx_Pressure object
   *
   * @param parent A pointer to the BMP5xx class
   */
  Adafruit_BMP5xx_Pressure(class Adafruit_BMP5xx* parent) {
    _theBMP5xx = parent;
  }

  bool getEvent(sensors_event_t* event);
  void getSensor(sensor_t* sensor);

 private:
  int _sensorID = 0x581; /**< ID number for pressure sensor */
  class Adafruit_BMP5xx* _theBMP5xx = NULL; /**< Pointer to BMP5xx instance */
};
#endif

/**
 * Driver for the Adafruit BMP5xx barometric pressure sensor.
 */
class Adafruit_BMP5xx {
 public:
  Adafruit_BMP5xx();
  ~Adafruit_BMP5xx(void);

  bool begin(uint8_t addr = BMP5XX_DEFAULT_ADDRESS, TwoWire* theWire = &Wire);
  bool begin(int8_t cspin, SPIClass* theSPI = &SPI);

  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel = 1013.25);

  bool performReading(void);

  bool setTemperatureOversampling(bmp5xx_oversampling_t oversampling);
  bool setPressureOversampling(bmp5xx_oversampling_t oversampling);
  bool setIIRFilterCoeff(bmp5xx_iir_filter_t filtercoeff);
  bool setOutputDataRate(bmp5xx_odr_t odr);
  bool setPowerMode(bmp5xx_powermode_t powermode);

  bmp5xx_oversampling_t getTemperatureOversampling(void);
  bmp5xx_oversampling_t getPressureOversampling(void);
  bmp5xx_iir_filter_t getIIRFilterCoeff(void);
  bmp5xx_odr_t getOutputDataRate(void);
  bmp5xx_powermode_t getPowerMode(void);

#if 0
  Adafruit_Sensor* getTemperatureSensor(void);
  Adafruit_Sensor* getPressureSensor(void);
#endif

  bool enablePressure(bool enable = true);
  bool dataReady(void);

  bool configureInterrupt(bmp5xx_interrupt_mode_t mode,
                          bmp5xx_interrupt_polarity_t polarity,
                          bmp5xx_interrupt_drive_t drive,
                          uint8_t sources = BMP5XX_INTERRUPT_DATA_READY,
                          bool enable = true);

  /**! Temperature (Celsius) assigned after calling performReading() */
  float temperature;
  /**! Pressure (hPa) assigned after calling performReading() */
  float pressure;

 private:
  bool _init(void);

  /**! BMP5xx device struct from Bosch API */
  struct bmp5_dev _bmp5_dev;

  /**! Configuration struct for OSR/ODR settings */
  struct bmp5_osr_odr_press_config _osr_odr_config;

  /**! Configuration struct for IIR filter settings */
  struct bmp5_iir_config _iir_config;

  /**! I2C interface object */
  Adafruit_I2CDevice* _i2c_dev = NULL;
  /**! SPI interface object */
  Adafruit_SPIDevice* _spi_dev = NULL;

#if 0
  /**! Adafruit unified sensor interface for temperature */
  Adafruit_BMP5xx_Temp* _temp_sensor = NULL;
  /**! Adafruit unified sensor interface for pressure */
  Adafruit_BMP5xx_Pressure* _pressure_sensor = NULL;
#endif

  // Static callback functions for Bosch API
  static int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len,
                         void* intf_ptr);
  static int8_t i2c_write(uint8_t reg_addr, const uint8_t* reg_data,
                          uint32_t len, void* intf_ptr);
  static int8_t spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len,
                         void* intf_ptr);
  static int8_t spi_write(uint8_t reg_addr, const uint8_t* reg_data,
                          uint32_t len, void* intf_ptr);
  static void delay_usec(uint32_t us, void* intf_ptr);
};

#endif // ADAFRUIT_BMP5XX_H

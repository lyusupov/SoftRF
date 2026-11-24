/*!
 *  @file Adafruit_BMP5xx.cpp
 *
 *  @mainpage Adafruit BMP5xx pressure and temperature sensor library
 *
 *  @section intro_sec Introduction
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
 *  @section author Author
 *
 *  Limor "ladyada" Fried (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_BMP5xx.h"

/*!
 * @brief  BMP5xx constructor
 */
Adafruit_BMP5xx::Adafruit_BMP5xx(void) {
#if 0
  _temp_sensor = new Adafruit_BMP5xx_Temp(this);
  _pressure_sensor = new Adafruit_BMP5xx_Pressure(this);
#endif
  temperature = 0.0;
  pressure = 0.0;
}

/*!
 * @brief  BMP5xx destructor
 */
Adafruit_BMP5xx::~Adafruit_BMP5xx(void) {
  if (_i2c_dev) {
    delete _i2c_dev;
  }
  if (_spi_dev) {
    delete _spi_dev;
  }
#if 0
  if (_temp_sensor) {
    delete _temp_sensor;
  }
  if (_pressure_sensor) {
    delete _pressure_sensor;
  }
#endif
}

/*!
 * @brief Initializes the sensor
 * @param addr Optional I2C address the sensor can be found on. Default is 0x46
 * @param theWire Optional Wire interface the sensor is connected to. Default is
 * &Wire
 * @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BMP5xx::begin(uint8_t addr, TwoWire* theWire) {
  if (_i2c_dev) {
    delete _i2c_dev;
  }
  _i2c_dev = new Adafruit_I2CDevice(addr, theWire);

  if (!_i2c_dev->begin()) {
    return false;
  }

  // Setup Bosch API callbacks
  _bmp5_dev.intf_ptr = _i2c_dev;
  _bmp5_dev.intf = BMP5_I2C_INTF;
  _bmp5_dev.read = i2c_read;
  _bmp5_dev.write = i2c_write;
  _bmp5_dev.delay_us = delay_usec;

  return _init();
}

/*!
 * @brief Initializes the sensor over SPI
 * @param cspin The pin to use for CS/Chip Select
 * @param theSPI Optional SPI interface the sensor is connected to. Default is
 * &SPI
 * @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BMP5xx::begin(int8_t cspin, SPIClass* theSPI) {
  if (_spi_dev) {
    delete _spi_dev;
  }
  _spi_dev = new Adafruit_SPIDevice(cspin, 1000000, SPI_BITORDER_MSBFIRST,
                                    SPI_MODE0, theSPI);

  if (!_spi_dev->begin()) {
    return false;
  }

  // Setup Bosch API callbacks
  _bmp5_dev.intf_ptr = _spi_dev;
  _bmp5_dev.intf = BMP5_SPI_INTF;
  _bmp5_dev.read = spi_read;
  _bmp5_dev.write = spi_write;
  _bmp5_dev.delay_us = delay_usec;

  return _init();
}

/*!
 * @brief Performs the actual initialization using Bosch API
 * @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BMP5xx::_init(void) {
  // Reset the sensor first
  int8_t rslt = bmp5_soft_reset(&_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Now initialize the sensor
  rslt = bmp5_init(&_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Set default configuration
  _osr_odr_config.osr_t = BMP5_OVERSAMPLING_2X;
  _osr_odr_config.osr_p = BMP5_OVERSAMPLING_16X;
  _osr_odr_config.odr = BMP5_ODR_50_HZ;
  _osr_odr_config.press_en = BMP5_ENABLE;

  rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Set default IIR filter
  _iir_config.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
  _iir_config.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
  _iir_config.shdw_set_iir_t = BMP5_ENABLE;
  _iir_config.shdw_set_iir_p = BMP5_ENABLE;
  _iir_config.iir_flush_forced_en = BMP5_ENABLE;

  rslt = bmp5_set_iir_config(&_iir_config, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Set to normal mode
  rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Enable data ready interrupt for non-blocking data ready checking
  struct bmp5_int_source_select int_source_select = {0};
  int_source_select.drdy_en = BMP5_ENABLE;
  int_source_select.fifo_full_en = BMP5_DISABLE;
  int_source_select.fifo_thres_en = BMP5_DISABLE;
  int_source_select.oor_press_en = BMP5_DISABLE;

  rslt = bmp5_int_source_select(&int_source_select, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Configure interrupt pin as push-pull, active high, latched mode
  rslt = bmp5_configure_interrupt(BMP5_LATCHED, BMP5_ACTIVE_HIGH,
                                  BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE,
                                  &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Performs a reading of both temperature and pressure and stores values
 * in class instance variables
 * @return True if the reading was successful, otherwise false.
 */
bool Adafruit_BMP5xx::performReading(void) {
  struct bmp5_sensor_data sensor_data;

  int8_t rslt =
      bmp5_get_sensor_data(&sensor_data, &_osr_odr_config, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  temperature = sensor_data.temperature;
  pressure = sensor_data.pressure / 100.0; // Convert Pa to hPa

  return true;
}

/*!
 * @brief Returns the temperature from the last reading
 * @return Temperature in degrees Celsius
 */
float Adafruit_BMP5xx::readTemperature(void) {
  performReading();
  return temperature;
}

/*!
 * @brief Returns the pressure from the last reading
 * @return Pressure in hPa
 */
float Adafruit_BMP5xx::readPressure(void) {
  performReading();
  return pressure;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure
 * @param seaLevel Sea level pressure in hPa (default = 1013.25)
 * @return Altitude in meters
 */
float Adafruit_BMP5xx::readAltitude(float seaLevel) {
  float atmospheric = readPressure();
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 * @brief Set temperature oversampling
 * @param oversampling Oversampling setting
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::setTemperatureOversampling(
    bmp5xx_oversampling_t oversampling) {
  _osr_odr_config.osr_t = (uint8_t)oversampling;
  int8_t rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Set pressure oversampling
 * @param oversampling Oversampling setting
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::setPressureOversampling(
    bmp5xx_oversampling_t oversampling) {
  _osr_odr_config.osr_p = (uint8_t)oversampling;
  int8_t rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Set IIR filter coefficient
 * @param filtercoeff IIR filter coefficient
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::setIIRFilterCoeff(bmp5xx_iir_filter_t filtercoeff) {
  _iir_config.set_iir_t = (uint8_t)filtercoeff;
  _iir_config.set_iir_p = (uint8_t)filtercoeff;
  int8_t rslt = bmp5_set_iir_config(&_iir_config, &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Set output data rate
 * @param odr Output data rate setting
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::setOutputDataRate(bmp5xx_odr_t odr) {
  _osr_odr_config.odr = (uint8_t)odr;
  int8_t rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Set power mode
 * @param powermode Power mode setting. BMP5XX_POWERMODE_FORCED takes one
 *        measurement then returns to standby; set it again for each reading.
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::setPowerMode(bmp5xx_powermode_t powermode) {
  int8_t rslt = bmp5_set_power_mode((enum bmp5_powermode)powermode, &_bmp5_dev);
  return rslt == BMP5_OK;
}

/*!
 * @brief Get temperature oversampling setting
 * @return Current temperature oversampling
 */
bmp5xx_oversampling_t Adafruit_BMP5xx::getTemperatureOversampling(void) {
  struct bmp5_osr_odr_press_config config;
  bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_oversampling_t)config.osr_t;
}

/*!
 * @brief Get pressure oversampling setting
 * @return Current pressure oversampling
 */
bmp5xx_oversampling_t Adafruit_BMP5xx::getPressureOversampling(void) {
  struct bmp5_osr_odr_press_config config;
  bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_oversampling_t)config.osr_p;
}

/*!
 * @brief Get IIR filter coefficient
 * @return Current IIR filter coefficient
 */
bmp5xx_iir_filter_t Adafruit_BMP5xx::getIIRFilterCoeff(void) {
  struct bmp5_iir_config config;
  bmp5_get_iir_config(&config, &_bmp5_dev);
  return (bmp5xx_iir_filter_t)config.set_iir_p;
}

/*!
 * @brief Get output data rate
 * @return Current output data rate
 */
bmp5xx_odr_t Adafruit_BMP5xx::getOutputDataRate(void) {
  struct bmp5_osr_odr_press_config config;
  bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_odr_t)config.odr;
}

/*!
 * @brief Get power mode
 * @return Current power mode
 */
bmp5xx_powermode_t Adafruit_BMP5xx::getPowerMode(void) {
  enum bmp5_powermode powermode;
  bmp5_get_power_mode(&powermode, &_bmp5_dev);
  return (bmp5xx_powermode_t)powermode;
}

/*!
 * @brief Enable/disable pressure measurement
 * @param enable True to enable pressure, false to disable
 * @return True on success, False on failure
 */
bool Adafruit_BMP5xx::enablePressure(bool enable) {
  _osr_odr_config.press_en = enable ? BMP5_ENABLE : BMP5_DISABLE;
  int8_t rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return rslt == BMP5_OK;
}

#if 0
/*!
 * @brief Gets an Adafruit Unified Sensor object for the temp sensor component
 * @return Adafruit_Sensor pointer to temperature sensor
 */
Adafruit_Sensor* Adafruit_BMP5xx::getTemperatureSensor(void) {
  return _temp_sensor;
}

/*!
 * @brief Gets an Adafruit Unified Sensor object for the pressure sensor
 * component
 * @return Adafruit_Sensor pointer to pressure sensor
 */
Adafruit_Sensor* Adafruit_BMP5xx::getPressureSensor(void) {
  return _pressure_sensor;
}
#endif

/*!
 * @brief Check if new sensor data is ready
 * @return True if new data is available, false otherwise
 */
bool Adafruit_BMP5xx::dataReady(void) {
  uint8_t int_status = 0;
  int8_t rslt = bmp5_get_interrupt_status(&int_status, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }
  // Check if data ready interrupt is asserted
  return (int_status & BMP5_INT_ASSERTED_DRDY) != 0;
}

/*!
 * @brief Configure interrupt pin settings and sources
 * @param mode Interrupt mode (pulsed or latched)
 * @param polarity Interrupt polarity (active high or low)
 * @param drive Interrupt drive (push-pull or open-drain)
 * @param sources Interrupt sources (can be combined with bitwise OR)
 * @param enable Enable or disable interrupt pin
 * @return True if configuration was successful, false otherwise
 */
bool Adafruit_BMP5xx::configureInterrupt(bmp5xx_interrupt_mode_t mode,
                                         bmp5xx_interrupt_polarity_t polarity,
                                         bmp5xx_interrupt_drive_t drive,
                                         uint8_t sources, bool enable) {
  // Configure interrupt pin settings first
  enum bmp5_intr_en_dis int_enable =
      enable ? BMP5_INTR_ENABLE : BMP5_INTR_DISABLE;

  int8_t rslt = bmp5_configure_interrupt(
      (enum bmp5_intr_mode)mode, (enum bmp5_intr_polarity)polarity,
      (enum bmp5_intr_drive)drive, int_enable, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Configure interrupt sources after pin settings
  struct bmp5_int_source_select int_source_select = {0};
  int_source_select.drdy_en =
      (sources & BMP5XX_INTERRUPT_DATA_READY) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.fifo_full_en =
      (sources & BMP5XX_INTERRUPT_FIFO_FULL) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.fifo_thres_en =
      (sources & BMP5XX_INTERRUPT_FIFO_THRESHOLD) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.oor_press_en =
      (sources & BMP5XX_INTERRUPT_PRESSURE_OUT_OF_RANGE) ? BMP5_ENABLE
                                                         : BMP5_DISABLE;

  rslt = bmp5_int_source_select(&int_source_select, &_bmp5_dev);

  return rslt == BMP5_OK;
}

/**************************************************************************/
/*!
    @brief  I2C read callback for Bosch API
    @param  reg_addr Register address to read from
    @param  reg_data Buffer to store read data
    @param  len Number of bytes to read
    @param  intf_ptr Pointer to interface (I2C device)
    @return 0 on success, negative on error
*/
/**************************************************************************/
int8_t Adafruit_BMP5xx::i2c_read(uint8_t reg_addr, uint8_t* reg_data,
                                 uint32_t len, void* intf_ptr) {
  Adafruit_I2CDevice* i2c_dev = (Adafruit_I2CDevice*)intf_ptr;

  if (!i2c_dev->write_then_read(&reg_addr, 1, reg_data, len)) {
    return -1;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  I2C write callback for Bosch API
    @param  reg_addr Register address to write to
    @param  reg_data Buffer containing data to write
    @param  len Number of bytes to write
    @param  intf_ptr Pointer to interface (I2C device)
    @return 0 on success, negative on error
*/
/**************************************************************************/
int8_t Adafruit_BMP5xx::i2c_write(uint8_t reg_addr, const uint8_t* reg_data,
                                  uint32_t len, void* intf_ptr) {
  Adafruit_I2CDevice* i2c_dev = (Adafruit_I2CDevice*)intf_ptr;

  // Create buffer with register address + data
  uint8_t buffer[len + 1];
  buffer[0] = reg_addr;
  memcpy(&buffer[1], reg_data, len);

  if (!i2c_dev->write(buffer, len + 1)) {
    return -1;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  SPI read callback for Bosch API
    @param  reg_addr Register address to read from
    @param  reg_data Buffer to store read data
    @param  len Number of bytes to read
    @param  intf_ptr Pointer to interface (SPI device)
    @return 0 on success, negative on error
*/
/**************************************************************************/
int8_t Adafruit_BMP5xx::spi_read(uint8_t reg_addr, uint8_t* reg_data,
                                 uint32_t len, void* intf_ptr) {
  Adafruit_SPIDevice* spi_dev = (Adafruit_SPIDevice*)intf_ptr;

  // Set read bit for SPI
  reg_addr |= 0x80;

  if (!spi_dev->write_then_read(&reg_addr, 1, reg_data, len)) {
    return -1;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  SPI write callback for Bosch API
    @param  reg_addr Register address to write to
    @param  reg_data Buffer containing data to write
    @param  len Number of bytes to write
    @param  intf_ptr Pointer to interface (SPI device)
    @return 0 on success, negative on error
*/
/**************************************************************************/
int8_t Adafruit_BMP5xx::spi_write(uint8_t reg_addr, const uint8_t* reg_data,
                                  uint32_t len, void* intf_ptr) {
  Adafruit_SPIDevice* spi_dev = (Adafruit_SPIDevice*)intf_ptr;

  // Create buffer with register address + data
  uint8_t buffer[len + 1];
  buffer[0] = reg_addr & 0x7F; // Clear read bit for write
  memcpy(&buffer[1], reg_data, len);

  if (!spi_dev->write(buffer, len + 1)) {
    return -1;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  Delay callback for Bosch API
    @param  us Microseconds to delay
    @param  intf_ptr Pointer to interface (unused)
*/
/**************************************************************************/
void Adafruit_BMP5xx::delay_usec(uint32_t us, void* intf_ptr) {
  (void)intf_ptr; // Unused parameter
  delayMicroseconds(us);
}

/**************************************************************************/
/*         Adafruit_BMP5xx_Temp                                          */
/**************************************************************************/

#if 0
/*!
 *  @brief  Gets the temperature as a standard sensor event
 *  @param  event Sensor event object that will be populated
 *  @returns True
 */
bool Adafruit_BMP5xx_Temp::getEvent(sensors_event_t* event) {
  _theBMP5xx->readTemperature();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theBMP5xx->temperature;

  return true;
}

/*!
 *  @brief  Gets the sensor_t device data
 *  @param  sensor Sensor description that will be populated
 */
void Adafruit_BMP5xx_Temp::getSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "BMP5xx", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40.0; // Datasheet minimum
  sensor->max_value = 85.0;  // Datasheet maximum
  sensor->resolution = 0.01; // Datasheet resolution
}

/**************************************************************************/
/*         Adafruit_BMP5xx_Pressure                                      */
/**************************************************************************/

/*!
 *  @brief  Gets the pressure as a standard sensor event
 *  @param  event Sensor event object that will be populated
 *  @returns True
 */
bool Adafruit_BMP5xx_Pressure::getEvent(sensors_event_t* event) {
  _theBMP5xx->readPressure();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  event->pressure = _theBMP5xx->pressure;

  return true;
}

/*!
 *  @brief  Gets the sensor_t device data
 *  @param  sensor Sensor description that will be populated
 */
void Adafruit_BMP5xx_Pressure::getSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "BMP5xx", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 300.0;  // Datasheet minimum 30kPa
  sensor->max_value = 1250.0; // Datasheet maximum 125kPa
  sensor->resolution = 0.016; // Datasheet RMS noise
}
#endif

/*!
 * @file Adafruit_LIS3DH.cpp
 *
 *  @mainpage Adafruit LIS3DH breakout board
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Adafruit LIS3DH Accel breakout board
 *
 *  Designed specifically to work with the Adafruit LIS3DH Accel breakout board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2809
 *
 *  This sensor communicates over I2C or SPI (our library code supports both) so
 * you can share it with a bunch of other sensors on the same I2C bus.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K. Townsend / Limor Fried (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Arduino.h"

#include <Adafruit_LIS3DH.h>
#include <Wire.h>

/*!
 *  @brief  Instantiates a new LIS3DH class in I2C
 *  @param  Wi
 *          optional wire object
 */
Adafruit_LIS3DH::Adafruit_LIS3DH(TwoWire *Wi)
    : _cs(-1), _mosi(-1), _miso(-1), _sck(-1), _sensorID(-1) {
  I2Cinterface = Wi;
}

/*!
 *   @brief  Instantiates a new LIS3DH class using hardware SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  *theSPI
 *           optional parameter contains spi object
 *   @param  frequency
 *           frequency of the SPI interface
 */
Adafruit_LIS3DH::Adafruit_LIS3DH(int8_t cspin, SPIClass *theSPI,
                                 uint32_t frequency) {
  _cs = cspin;
  _mosi = -1;
  _miso = -1;
  _sck = -1;
  _sensorID = -1;
  SPIinterface = theSPI;
  _frequency = frequency;
}

/*!
 *   @brief  Instantiates a new LIS3DH class using software SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  mosipin
 *           number of pin used for MOSI (Master Out Slave In))
 *   @param  misopin
 *           number of pin used for MISO (Master In Slave Out)
 *   @param  sckpin
 *           number of pin used for CLK (clock pin)
 *   @param  frequency
 *           frequency of the SPI interface
 */
Adafruit_LIS3DH::Adafruit_LIS3DH(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin, uint32_t frequency) {
  _cs = cspin;
  _mosi = mosipin;
  _miso = misopin;
  _sck = sckpin;
  _sensorID = -1;
  _frequency = frequency;
}

/*!
 *  @brief  Setups the HW (reads coefficients values, etc.)
 *  @param  i2caddr
 *          i2c address (optional, fallback to default)
 *  @param  nWAI
 *          Who Am I register value - defaults to 0x33 (LIS3DH)
 *  @return true if successful
 */
bool Adafruit_LIS3DH::begin(uint8_t i2caddr, uint8_t nWAI) {
  _i2caddr = i2caddr;
  _wai = nWAI;
  if (I2Cinterface) {
    i2c_dev = new Adafruit_I2CDevice(_i2caddr, I2Cinterface);

    if (!i2c_dev->begin()) {
      return false;
    }
  } else if (_cs != -1) {

    // SPIinterface->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    if (_sck == -1) {
      spi_dev = new Adafruit_SPIDevice(_cs,
                                       _frequency,            // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE0,             // data mode
                                       SPIinterface);
    } else {
      spi_dev = new Adafruit_SPIDevice(_cs, _sck, _miso, _mosi,
                                       _frequency,            // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE0);            // data mode
    }

    if (!spi_dev->begin()) {
      return false;
    }
  }

  /* Check connection */
  if (getDeviceID() != _wai) {
    /* No LIS3DH detected ... return false */
    // Serial.println(deviceid, HEX);
    return false;
  }
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
  _ctrl1.write(0x07); // enable all axes, normal mode

  // 400Hz rate
  setDataRate(LIS3DH_DATARATE_400_HZ);

  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
  _ctrl4.write(0x88); // High res & BDU enabled

  enableDRDY(true, 1);

  // Turn on orientation config

  Adafruit_BusIO_Register _tmp_cfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TEMPCFG, 1);
  _tmp_cfg.write(0x80); // enable adcs

  return true;
}

/*!
 *  @brief  Get Device ID from LIS3DH_REG_WHOAMI
 *  @return WHO AM I value
 */
uint8_t Adafruit_LIS3DH::getDeviceID(void) {
  Adafruit_BusIO_Register _chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_WHOAMI, 1);

  return _chip_id.read();
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool Adafruit_LIS3DH::haveNewData(void) {
  Adafruit_BusIO_Register status_2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_STATUS2, 1);
  Adafruit_BusIO_RegisterBits zyx_data_available =
      Adafruit_BusIO_RegisterBits(&status_2, 1, 3);
  return zyx_data_available.read();
}

/*!
 *  @brief  Reads x y z values at once
 */
void Adafruit_LIS3DH::read(void) {

  uint8_t register_address = LIS3DH_REG_OUT_X_L;
  if (i2c_dev) {
    register_address |= 0x80; // set [7] for auto-increment
  } else {
    register_address |= 0x40; // set [6] for auto-increment
    register_address |= 0x80; // set [7] for read
  }

  Adafruit_BusIO_Register xl_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, register_address, 6);

  uint8_t buffer[6];
  xl_data.read(buffer, 6);

  x = buffer[0];
  x |= ((uint16_t)buffer[1]) << 8;
  y = buffer[2];
  y |= ((uint16_t)buffer[3]) << 8;
  z = buffer[4];
  z |= ((uint16_t)buffer[5]) << 8;

  uint8_t range = getRange();
  uint8_t mode = getPerformanceMode();

  // this scaling process accounts for the shift due to actually being 10 bits
  // (normal mode) as well as the lsb=> mg conversion and the mg=> g conversion
  // final value is raw_lsb => 10-bit lsb -> milli-gs -> gs

  // depending on the range, we'll always convert the value to 8/10/12 bits and
  // g's so we'll divide by LIS3DH_LSB16_TO_KILO_LSB10 (16000), _LSB8 or _LSB12:

  // then we can then multiply the resulting value by the lsb value to get the
  // value in g's

  uint8_t lsb_value = 1;
  if (range == LIS3DH_RANGE_2_G)
    lsb_value = 4;
  if (range == LIS3DH_RANGE_4_G)
    lsb_value = 8;
  if (range == LIS3DH_RANGE_8_G)
    lsb_value = 16;
  if (range == LIS3DH_RANGE_16_G)
    lsb_value = 48;

  float convert_from_LSB16 = 64000.0;
  if (mode == LIS3DH_MODE_HIGH_RESOLUTION) {
    lsb_value = lsb_value / 4; // 1 at 2G, 2 at 4G, 4 at 8G, 12 at 16G
    convert_from_LSB16 = LIS3DH_LSB16_TO_KILO_LSB12;
  } else if (mode == LIS3DH_MODE_NORMAL) {
    convert_from_LSB16 = LIS3DH_LSB16_TO_KILO_LSB10;
  } else if (mode == LIS3DH_MODE_LOW_POWER) {
    lsb_value = lsb_value * 4; // 16 at 2G, 32 at 4G, 64 at 8G, 192 at 16G
    convert_from_LSB16 = LIS3DH_LSB16_TO_KILO_LSB8;
  }
  x_g = lsb_value * ((float)x / convert_from_LSB16);
  y_g = lsb_value * ((float)y / convert_from_LSB16);
  z_g = lsb_value * ((float)z / convert_from_LSB16);
}

/*!
 *  @brief  Read the auxilary ADC
 *  @param  adc
 *          adc index. possible values (1, 2, 3).
 *  @return auxilary ADC value
 */
int16_t Adafruit_LIS3DH::readADC(uint8_t adc) {
  if ((adc < 1) || (adc > 3))
    return 0;
  adc--; // switch to 0 indexed

  uint16_t value;
  uint8_t reg = LIS3DH_REG_OUTADC1_L + (adc * 2);

  if (i2c_dev) {
    reg |= 0x80; // set [7] for auto-increment
  } else {
    reg |= 0x40; // set [6] for auto-increment
    reg |= 0x80; // set [7] for read
  }

  uint8_t buffer[2];
  Adafruit_BusIO_Register adc_data =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg, 2);

  adc_data.read(buffer, 2);

  value = buffer[0];
  value |= ((uint16_t)buffer[1]) << 8;

  return value;
}

/*!
 *   @brief  Set INT to output for single or double click
 *   @param  c
 *					 0 = turn off I1_CLICK
 *           1 = turn on all axes & singletap
 *					 2 = turn on all axes & doubletap
 *   @param  clickthresh
 *           CLICK threshold value
 *   @param  timelimit
 *           sets time limit (default 10)
 *   @param  timelatency
 *   				 sets time latency (default 20)
 *   @param  timewindow
 *   				 sets time window (default 255)
 */

void Adafruit_LIS3DH::setClick(uint8_t c, uint8_t clickthresh,
                               uint8_t timelimit, uint8_t timelatency,
                               uint8_t timewindow) {

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL3, 1);
  Adafruit_BusIO_RegisterBits i1_click =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);

  Adafruit_BusIO_Register click_cfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKCFG, 1);

  if (!c) {
    // disable int
    i1_click.write(0); // disable i1 click
    click_cfg.write(0);
    return;
  }
  // else...

  i1_click.write(1); // enable i1 click

  Adafruit_BusIO_Register ctrl5 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL5, 1);

  Adafruit_BusIO_RegisterBits int1_latch_bit =
      Adafruit_BusIO_RegisterBits(&ctrl5, 1, 3);
  int1_latch_bit.write(true);

  if (c == 1)
    click_cfg.write(0x15); // turn on all axes & singletap
  if (c == 2)
    click_cfg.write(0x2A); // turn on all axes & doubletap

  Adafruit_BusIO_Register click_ths = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKTHS, 1);
  click_ths.write(clickthresh); // arbitrary

  Adafruit_BusIO_Register time_limit = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMELIMIT, 1);
  time_limit.write(timelimit); // arbitrary

  Adafruit_BusIO_Register time_latency = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMELATENCY, 1);
  time_latency.write(timelatency); // arbitrary

  Adafruit_BusIO_Register time_window = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMEWINDOW, 1);
  time_window.write(timewindow); // arbitrary
}

/*!
 *   @brief  Get uint8_t for single or double click
 *   @return register LIS3DH_REG_CLICKSRC
 */
uint8_t Adafruit_LIS3DH::getClick(void) {
  Adafruit_BusIO_Register click_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKSRC, 1);

  return click_reg.read();
}

/*!
 *   @brief  Get uint8_t for INT1 source and clear interrupt
 *   @return register LIS3DH_REG_INT1SRC
 */
uint8_t Adafruit_LIS3DH::readAndClearInterrupt(void) {
  Adafruit_BusIO_Register int_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1SRC, 1);

  return int_reg.read();
}

/**
 * @brief Enable or disable the Data Ready interupt
 *
 * @param enable_drdy true to enable the given Data Ready interrupt on INT1,
 * false to disable it
 * @param int_pin which DRDY interrupt to enable; 1 for DRDY1, 2 for DRDY2
 * @return true: success false: failure
 */
bool Adafruit_LIS3DH::enableDRDY(bool enable_drdy, uint8_t int_pin) {
  Adafruit_BusIO_Register _ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL3, 1);
  Adafruit_BusIO_RegisterBits _drdy1_int_enable =
      Adafruit_BusIO_RegisterBits(&_ctrl3, 1, 4);
  Adafruit_BusIO_RegisterBits _drdy2_int_enable =
      Adafruit_BusIO_RegisterBits(&_ctrl3, 1, 3);

  if (int_pin == 1) {
    return _drdy1_int_enable.write(enable_drdy);
  } else if (int_pin == 2) {
    return _drdy2_int_enable.write(enable_drdy);
  } else {
    return false;
  }
}

/*!
 *   @brief  Sets the performance mode for the LIS3DH.
 *
 *   The turn-on time to transition to 12-bit mode (high resolution) is set at
 * 7ms, or swtch to 10-bit mode (normal) or to 8-bit mode (low power) is 1ms
 *
 *   @param  mode
 *          mode - low power, normal, high resolution e.g. LIS3DH_MODE_LOW_POWER
 */
void Adafruit_LIS3DH::setPerformanceMode(lis3dh_mode_t mode) {
  // low power bit is in CTRL1, 4th bit from right
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits ctrl1_mode_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 1, 3);
  // high res bit is in CTRL4, 4th bit from right
  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
  Adafruit_BusIO_RegisterBits ctrl4_mode_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 1, 3);
  switch (mode) {
  case LIS3DH_MODE_LOW_POWER:
    // set HR bit low (CTRL4) and LP bit high (CTRL1)
    ctrl4_mode_bits.write(0);
    ctrl1_mode_bits.write(1);
    delay(1); // turn-on transition time (worst case)
    break;
  case LIS3DH_MODE_NORMAL:
    // set HR bit low (CTRL4) and LP bit low (CTRL1)
    ctrl1_mode_bits.write(0);
    ctrl4_mode_bits.write(0);
    delay(1); // turn-on transition time (worst case)
    break;
  case LIS3DH_MODE_HIGH_RESOLUTION:
    // set HR bit high (CTRL4) and LP bit low (CTRL1)
    ctrl1_mode_bits.write(0);
    ctrl4_mode_bits.write(1);
    delay(7); // turn-on transition time (worst case)
    break;
  }
}

/*!
 *   @brief  Gets the performance mode for the LIS3DH
 *   @return Returns performance mode value
 */
lis3dh_mode_t Adafruit_LIS3DH::getPerformanceMode(void) {
  // low power bit is in CTRL1, 4th bit from right
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits ctrl1_mode_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 1, 3);
  // high res bit is in CTRL4, 4th bit from right
  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
  Adafruit_BusIO_RegisterBits ctrl4_mode_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 1, 3);

  bool lp = ctrl1_mode_bits.read() == 1;
  bool hr = ctrl4_mode_bits.read() == 1;
  if (!lp && !hr) {
    return LIS3DH_MODE_NORMAL;
  } else if (lp && !hr) {
    return LIS3DH_MODE_LOW_POWER;
  }
  return LIS3DH_MODE_HIGH_RESOLUTION;
}

/*!
 *   @brief  Sets the g range for the accelerometer
 *   @param  range
 *           range value
 */
void Adafruit_LIS3DH::setRange(lis3dh_range_t range) {

  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
  range_bits.write(range);
  delay(15); // delay to let new setting settle
}

/*!
 *  @brief  Gets the g range for the accelerometer
 *  @return Returns g range value
 */
lis3dh_range_t Adafruit_LIS3DH::getRange(void) {
  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
  return (lis3dh_range_t)range_bits.read();
}

/*!
 *  @brief  Sets the data rate for the LIS3DH (controls power consumption)
 *  @param  dataRate
 *          data rate value
 */
void Adafruit_LIS3DH::setDataRate(lis3dh_dataRate_t dataRate) {
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);

  data_rate_bits.write(dataRate);
}

/*!
 *   @brief  Gets the data rate for the LIS3DH (controls power consumption)
 *   @return Returns Data Rate value
 */
lis3dh_dataRate_t Adafruit_LIS3DH::getDataRate(void) {
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);

  return (lis3dh_dataRate_t)data_rate_bits.read();
}

#if 0
/*!
 *  @brief  Gets the most recent sensor event
 *  @param  *event
 *          sensor event that we want to read
 *  @return true if successful
 */
bool Adafruit_LIS3DH::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

  return true;
}

/*!
 *   @brief  Gets the sensor_t data
 *   @param  *sensor
 *           sensor that we want to write data into
 */
void Adafruit_LIS3DH::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 0;
  sensor->min_value = 0;
  sensor->resolution = 0;
}
#endif

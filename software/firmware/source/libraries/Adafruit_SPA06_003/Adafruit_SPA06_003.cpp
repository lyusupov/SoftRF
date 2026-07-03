/*!
 * @file Adafruit_SPA06_003.cpp
 *
 * @mainpage Adafruit SPA06_003 Digital Pressure Sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's SPA06_003 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit SPA06_003 breakout: https://www.adafruit.com/products/xxxx
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 * for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_SPA06_003.h"

/*!
 *  @brief  Instantiates a new SPA06_003 class
 */
Adafruit_SPA06_003::Adafruit_SPA06_003() {
  i2c_dev = NULL;
  spi_dev = NULL;
}

/*!
 *  @brief  Cleans up the SPA06_003
 */
Adafruit_SPA06_003::~Adafruit_SPA06_003() {
  if (i2c_dev) {
    delete i2c_dev;
  }
  if (spi_dev) {
    delete spi_dev;
  }
#if 0
  if (temp_sensor) {
    delete temp_sensor;
  }
  if (pressure_sensor) {
    delete pressure_sensor;
  }
#endif
}

/*!
 *  @brief  Sets up the hardware and initializes I2C
 *  @param  i2c_addr
 *          The I2C address to be used.
 *  @param  wire
 *          The Wire object to be used for I2C connections.
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_SPA06_003::begin(uint8_t i2c_addr, TwoWire *wire) {
  if (spi_dev) {
    delete spi_dev;
    spi_dev = NULL;
  }
  if (i2c_dev) {
    delete i2c_dev;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

/*!
 *  @brief  Sets up the hardware and initializes hardware SPI
 *  @param  cspin The pin to use for CS/chip select
 *  @param  theSPI The SPI object to use, defaults to &SPI
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_SPA06_003::begin(int8_t cspin, SPIClass *theSPI) {
  if (i2c_dev) {
    delete i2c_dev;
    i2c_dev = NULL;
  }
  if (spi_dev) {
    delete spi_dev;
  }

  spi_dev = new Adafruit_SPIDevice(cspin, SPA06_003_DEFAULT_SPIFREQ,
                                   SPI_BITORDER_MSBFIRST, SPI_MODE3, theSPI);

  if (!spi_dev->begin()) {
    return false;
  }

  return _init();
}

/*!
 *  @brief  Sets up the hardware and initializes software SPI
 *  @param  cspin The pin to use for CS/chip select
 *  @param  mosipin The pin to use for MOSI/data out
 *  @param  misopin The pin to use for MISO/data in
 *  @param  sckpin The pin to use for SCK/clock
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_SPA06_003::begin(int8_t cspin, int8_t mosipin, int8_t misopin,
                               int8_t sckpin) {
  if (i2c_dev) {
    delete i2c_dev;
    i2c_dev = NULL;
  }
  if (spi_dev) {
    delete spi_dev;
  }

  spi_dev = new Adafruit_SPIDevice(cspin, sckpin, misopin, mosipin,
                                   SPA06_003_DEFAULT_SPIFREQ,
                                   SPI_BITORDER_MSBFIRST, SPI_MODE3);

  if (!spi_dev->begin()) {
    return false;
  }

  return _init();
}

/*!
 *  @brief  Common initialization code for I2C and SPI interfaces
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_SPA06_003::_init() {
  // Check chip ID using the combined constructor
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_ID, 1);

  if (chip_id.read() != 0x11) {
    return false;
  }

  // Perform soft reset
  if (!reset()) {
    return false;
  }
  delay(10);

  // Wait for coefficients and sensor to be ready
  while (!isCoeffReady() || !isSensorReady()) {
    delay(10);
  }

  // Read calibration coefficients
  if (!readCoefficients()) {
    return false;
  }

  // Configure for highest precision and sample rate
  // Set pressure to highest oversampling (128x) and highest rate (200 Hz)
  setPressureOversampling(SPA06_003_OVERSAMPLE_128);
  setPressureMeasureRate(SPA06_003_RATE_200);

  // Set temperature to highest oversampling (128x) and highest rate (200 Hz)
  setTemperatureOversampling(SPA06_003_OVERSAMPLE_128);
  setTemperatureMeasureRate(SPA06_003_RATE_200);

  // Enable interrupts for temperature and pressure ready
  setInterruptSource(false, true,
                     true);  // FIFO=false, temp_ready=true, pres_ready=true

  // Set measurement mode to continuous both
  setMeasurementMode(SPA06_003_MEAS_CONTINUOUS_BOTH);

  return true;
}

/*!
 *  @brief  Reads the 24-bit raw pressure data from registers
 *  @return 24-bit pressure data as uint32_t
 */
uint32_t Adafruit_SPA06_003::getPressureData() {
  Adafruit_BusIO_Register psr_reg =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              SPA06_003_REG_PSR_B2, 3, MSBFIRST);

  uint32_t psr_data = psr_reg.read();

  if (psr_data & 0x800000) {
    psr_data |= 0xFF000000;
  }

  return psr_data;
}

/*!
 *  @brief  Reads the 24-bit raw temperature data from registers
 *  @return 24-bit temperature data as uint32_t
 */
uint32_t Adafruit_SPA06_003::getTemperatureData() {
  Adafruit_BusIO_Register tmp_reg =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              SPA06_003_REG_TMP_B2, 3, MSBFIRST);

  uint32_t tmp_data = tmp_reg.read();

  if (tmp_data & 0x800000) {
    tmp_data |= 0xFF000000;
  }

  return tmp_data;
}

/*!
 *  @brief  Gets the pressure measurement rate
 *  @return Current pressure measurement rate setting
 */
spa06_003_rate_t Adafruit_SPA06_003::getPressureMeasureRate() {
  Adafruit_BusIO_Register prs_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_PRS_CFG, 1);
  Adafruit_BusIO_RegisterBits pm_rate_bits =
      Adafruit_BusIO_RegisterBits(&prs_cfg_reg, 4, 4);

  return (spa06_003_rate_t)pm_rate_bits.read();
}

/*!
 *  @brief  Sets the pressure measurement rate
 *  @param  rate The pressure measurement rate to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setPressureMeasureRate(spa06_003_rate_t rate) {
  Adafruit_BusIO_Register prs_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_PRS_CFG, 1);
  Adafruit_BusIO_RegisterBits pm_rate_bits =
      Adafruit_BusIO_RegisterBits(&prs_cfg_reg, 4, 4);

  return pm_rate_bits.write(rate);
}

/*!
 *  @brief  Gets the temperature measurement rate
 *  @return Current temperature measurement rate setting
 */
spa06_003_rate_t Adafruit_SPA06_003::getTemperatureMeasureRate() {
  Adafruit_BusIO_Register tmp_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_TMP_CFG, 1);
  Adafruit_BusIO_RegisterBits tmp_rate_bits =
      Adafruit_BusIO_RegisterBits(&tmp_cfg_reg, 4, 4);

  return (spa06_003_rate_t)tmp_rate_bits.read();
}

/*!
 *  @brief  Sets the temperature measurement rate
 *  @param  rate The temperature measurement rate to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setTemperatureMeasureRate(spa06_003_rate_t rate) {
  Adafruit_BusIO_Register tmp_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_TMP_CFG, 1);
  Adafruit_BusIO_RegisterBits tmp_rate_bits =
      Adafruit_BusIO_RegisterBits(&tmp_cfg_reg, 4, 4);

  return tmp_rate_bits.write(rate);
}

/*!
 *  @brief  Gets the pressure oversampling rate
 *  @return Current pressure oversampling rate setting
 */
spa06_003_oversample_t Adafruit_SPA06_003::getPressureOversampling() {
  Adafruit_BusIO_Register prs_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_PRS_CFG, 1);
  Adafruit_BusIO_RegisterBits pm_prc_bits =
      Adafruit_BusIO_RegisterBits(&prs_cfg_reg, 4, 0);

  return (spa06_003_oversample_t)pm_prc_bits.read();
}

/*!
 *  @brief  Sets the pressure oversampling rate
 *  @param  prc The pressure oversampling rate to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setPressureOversampling(spa06_003_oversample_t prc) {
  Adafruit_BusIO_Register prs_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_PRS_CFG, 1);
  Adafruit_BusIO_RegisterBits pm_prc_bits =
      Adafruit_BusIO_RegisterBits(&prs_cfg_reg, 4, 0);

  if (!pm_prc_bits.write(prc)) {
    return false;
  }

  return setPresShift(prc > SPA06_003_OVERSAMPLE_8);
}

/*!
 *  @brief  Gets the temperature oversampling rate
 *  @return Current temperature oversampling rate setting
 */
spa06_003_oversample_t Adafruit_SPA06_003::getTemperatureOversampling() {
  Adafruit_BusIO_Register tmp_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_TMP_CFG, 1);
  Adafruit_BusIO_RegisterBits tmp_prc_bits =
      Adafruit_BusIO_RegisterBits(&tmp_cfg_reg, 4, 0);

  return (spa06_003_oversample_t)tmp_prc_bits.read();
}

/*!
 *  @brief  Sets the temperature oversampling rate
 *  @param  prc The temperature oversampling rate to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setTemperatureOversampling(
    spa06_003_oversample_t prc) {
  Adafruit_BusIO_Register tmp_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_TMP_CFG, 1);
  Adafruit_BusIO_RegisterBits tmp_prc_bits =
      Adafruit_BusIO_RegisterBits(&tmp_cfg_reg, 4, 0);

  if (!tmp_prc_bits.write(prc)) {
    return false;
  }

  return setTempShift(prc > SPA06_003_OVERSAMPLE_8);
}

/*!
 *  @brief  Checks if calibration coefficients are ready
 *  @return True if coefficients are ready, false otherwise
 */
bool Adafruit_SPA06_003::isCoeffReady() {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits coef_rdy_bit =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 1, 7);

  return coef_rdy_bit.read();
}

/*!
 *  @brief  Checks if sensor is ready
 *  @return True if sensor is ready, false otherwise
 */
bool Adafruit_SPA06_003::isSensorReady() {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits sensor_rdy_bit =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 1, 6);

  return sensor_rdy_bit.read();
}

/*!
 *  @brief  Checks if temperature data is ready
 *  @return True if temperature data is ready, false otherwise
 */
bool Adafruit_SPA06_003::isTempDataReady() {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits tmp_rdy_bit =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 1, 5);

  return tmp_rdy_bit.read();
}

/*!
 *  @brief  Checks if pressure data is ready
 *  @return True if pressure data is ready, false otherwise
 */
bool Adafruit_SPA06_003::isPresDataReady() {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits prs_rdy_bit =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 1, 4);

  return prs_rdy_bit.read();
}

/*!
 *  @brief  Gets the current measurement mode
 *  @return Current measurement mode setting
 */
spa06_003_meas_mode_t Adafruit_SPA06_003::getMeasurementMode() {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits meas_ctrl_bits =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 3, 0);

  return (spa06_003_meas_mode_t)meas_ctrl_bits.read();
}

/*!
 *  @brief  Sets the measurement mode
 *  @param  mode The measurement mode to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setMeasurementMode(spa06_003_meas_mode_t mode) {
  Adafruit_BusIO_Register meas_cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_MEAS_CFG, 1);
  Adafruit_BusIO_RegisterBits meas_ctrl_bits =
      Adafruit_BusIO_RegisterBits(&meas_cfg_reg, 3, 0);

  return meas_ctrl_bits.write(mode);
}

/*!
 *  @brief  Sets the interrupt polarity
 *  @param  polarity The interrupt polarity to set
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setInterruptPolarity(
    spa06_003_int_polarity_t polarity) {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);
  Adafruit_BusIO_RegisterBits int_hl_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 7);

  return int_hl_bit.write(polarity);
}

/*!
 *  @brief  Sets the interrupt sources
 *  @param  fifo Enable FIFO interrupt
 *  @param  temp_ready Enable temperature ready interrupt
 *  @param  pres_ready Enable pressure ready interrupt
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setInterruptSource(bool fifo, bool temp_ready,
                                            bool pres_ready) {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);

  Adafruit_BusIO_RegisterBits int_fifo_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 6);
  Adafruit_BusIO_RegisterBits int_tmp_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 5);
  Adafruit_BusIO_RegisterBits int_prs_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 4);

  return int_fifo_bit.write(fifo) && int_tmp_bit.write(temp_ready) &&
         int_prs_bit.write(pres_ready);
}

/*!
 *  @brief  Sets the temperature result bit shift
 *  @param  enable True to enable bit shift, false to disable
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setTempShift(bool enable) {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);
  Adafruit_BusIO_RegisterBits t_shift_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 3);

  return t_shift_bit.write(enable);
}

/*!
 *  @brief  Sets the pressure result bit shift
 *  @param  enable True to enable bit shift, false to disable
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::setPresShift(bool enable) {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);
  Adafruit_BusIO_RegisterBits p_shift_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 2);

  return p_shift_bit.write(enable);
}

/*!
 *  @brief  Enables or disables FIFO
 *  @param  enable True to enable FIFO, false to disable
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::enableFIFO(bool enable) {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);
  Adafruit_BusIO_RegisterBits fifo_en_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 1);

  return fifo_en_bit.write(enable);
}

/*!
 *  @brief  Checks if FIFO is enabled
 *  @return True if FIFO is enabled, false otherwise
 */
bool Adafruit_SPA06_003::isFIFOEnabled() {
  Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_CFG_REG, 1);
  Adafruit_BusIO_RegisterBits fifo_en_bit =
      Adafruit_BusIO_RegisterBits(&cfg_reg, 1, 1);

  return fifo_en_bit.read();
}

/*!
 *  @brief  Checks if FIFO is empty
 *  @return True if FIFO is empty, false otherwise
 */
bool Adafruit_SPA06_003::isFIFOEmpty() {
  Adafruit_BusIO_Register fifo_sts_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_FIFO_STS, 1);
  Adafruit_BusIO_RegisterBits fifo_empty_bit =
      Adafruit_BusIO_RegisterBits(&fifo_sts_reg, 1, 0);

  return fifo_empty_bit.read();
}

/*!
 *  @brief  Checks if FIFO is full
 *  @return True if FIFO is full, false otherwise
 */
bool Adafruit_SPA06_003::isFIFOFull() {
  Adafruit_BusIO_Register fifo_sts_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_FIFO_STS, 1);
  Adafruit_BusIO_RegisterBits fifo_full_bit =
      Adafruit_BusIO_RegisterBits(&fifo_sts_reg, 1, 1);

  return fifo_full_bit.read();
}

/*!
 *  @brief  Gets the interrupt status flags
 *  @return Interrupt status register value with flags
 */
uint8_t Adafruit_SPA06_003::getStatusFlags() {
  Adafruit_BusIO_Register int_sts_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_INT_STS, 1);

  return int_sts_reg.read() & 0x07;
}

/*!
 *  @brief  Flushes the FIFO buffer
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::flushFIFO() {
  Adafruit_BusIO_Register reset_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_RESET, 1);
  Adafruit_BusIO_RegisterBits fifo_flush_bit =
      Adafruit_BusIO_RegisterBits(&reset_reg, 1, 7);

  return fifo_flush_bit.write(1);
}

/*!
 *  @brief  Performs a soft reset of the sensor
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::reset() {
  Adafruit_BusIO_Register reset_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_RESET, 1);
  Adafruit_BusIO_RegisterBits soft_rst_bits =
      Adafruit_BusIO_RegisterBits(&reset_reg, 4, 0);

  return soft_rst_bits.write(0x09);
}

/*!
 *  @brief  Reads calibration coefficients from sensor registers
 *  @return True if successful, false otherwise
 */
bool Adafruit_SPA06_003::readCoefficients() {
  if (!isCoeffReady()) {
    return false;
  }

  // Read all coefficient data (21 bytes from 0x10 to 0x24)
  uint8_t coef_data[21];
  Adafruit_BusIO_Register coef_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, SPA06_003_REG_COEF, 21);

  if (!coef_reg.read(coef_data, 21)) {
    return false;
  }

  // Parse coefficients according to datasheet table
  // c0: 12-bit (0x10-0x11)
  c0 = (int16_t)((coef_data[0] << 4) | (coef_data[1] >> 4));
  if (c0 & 0x800) {
    c0 |= 0xF000;  // Sign extend
  }

  // c1: 12-bit (0x11-0x12)
  c1 = (int16_t)(((coef_data[1] & 0x0F) << 8) | coef_data[2]);
  if (c1 & 0x800) {
    c1 |= 0xF000;  // Sign extend
  }

  // c00: 20-bit (0x13-0x15)
  uint32_t c00_temp = (((uint32_t)coef_data[3] << 12) & 0xFF000) |
                      (((uint16_t)coef_data[4] << 4) & 0x00FF0) |
                      ((coef_data[5] >> 4) & 0x0000F);
  if (c00_temp & 0x80000) {
    c00 = (int32_t)(c00_temp | 0xFFF00000);  // Sign extend
  } else {
    c00 = (int32_t)c00_temp;
  }

  // c10: 20-bit (0x15-0x17)
  uint32_t c10_temp = (((uint32_t)coef_data[5] << 16) & 0xF0000) |
                      (((uint16_t)coef_data[6] << 8) & 0x0FF00) | coef_data[7];
  if (c10_temp & 0x80000) {
    c10 = (int32_t)(c10_temp | 0xFFF00000);  // Sign extend
  } else {
    c10 = (int32_t)c10_temp;
  }

  // c01: 16-bit (0x18-0x19)
  c01 = (int16_t)((coef_data[8] << 8) | coef_data[9]);

  // c11: 16-bit (0x1A-0x1B)
  c11 = (int16_t)((coef_data[10] << 8) | coef_data[11]);

  // c20: 16-bit (0x1C-0x1D)
  c20 = (int16_t)((coef_data[12] << 8) | coef_data[13]);

  // c21: 16-bit (0x1E-0x1F)
  c21 = (int16_t)((coef_data[14] << 8) | coef_data[15]);

  // c30: 16-bit (0x20-0x21)
  c30 = (int16_t)((coef_data[16] << 8) | coef_data[17]);

  // c31: 12-bit (0x22 + 0x23 bits 7:4)
  c31 = (int16_t)(((coef_data[18] << 4) & 0xFF0) |
                  ((coef_data[19] >> 4) & 0x00F));
  if (c31 & 0x800) {
    c31 |= 0xF000;  // Sign extend
  }

  // c40: 12-bit (0x23 bits 3:0 + 0x24)
  c40 = (int16_t)(((coef_data[19] & 0x0F) << 8) | coef_data[20]);
  if (c40 & 0x800) {
    c40 |= 0xF000;  // Sign extend
  }

  return true;
}

/*!
 *  @brief  Gets scaling factor based on oversampling setting
 *  @param  oversample The oversampling rate
 *  @return Scaling factor for compensation calculation
 */
float Adafruit_SPA06_003::getScalingFactor(spa06_003_oversample_t oversample) {
  switch (oversample) {
    case SPA06_003_OVERSAMPLE_1:
      return 524288;  // Single
    case SPA06_003_OVERSAMPLE_2:
      return 1572864;  // 2x
    case SPA06_003_OVERSAMPLE_4:
      return 3670016;  // 4x
    case SPA06_003_OVERSAMPLE_8:
      return 7864320;  // 8x
    case SPA06_003_OVERSAMPLE_16:
      return 253952;  // 16x
    case SPA06_003_OVERSAMPLE_32:
      return 516096;  // 32x
    case SPA06_003_OVERSAMPLE_64:
      return 1040384;  // 64x
    case SPA06_003_OVERSAMPLE_128:
      return 2088960;  // 128x
    default:
      return 524288;  // Default to single
  }
}

/*!
 *  @brief  Reads compensated temperature value in Celsius
 *  @return Temperature in degrees Celsius
 */
float Adafruit_SPA06_003::readTemperature() {
  // Read raw temperature data (24-bit 2's complement)
  uint32_t temp_raw = getTemperatureData();

  // Use the calculation function
  return calculateTemperature(temp_raw);
}

/*!
 *  @brief  Reads compensated pressure value in hectopascals
 *  @return Pressure in hectopascals (hPa)
 */
float Adafruit_SPA06_003::readPressure() {
  // Read raw pressure and temperature data (24-bit 2's complement)
  uint32_t pres_raw = getPressureData();
  uint32_t temp_raw = getTemperatureData();

  // Use the calculation function
  return calculatePressure(pres_raw, temp_raw);
}

/*!
 *  @brief  Calculates compensated temperature from raw data
 *  @param  raw_temp Raw 24-bit temperature data
 *  @return Temperature in degrees Celsius
 */
float Adafruit_SPA06_003::calculateTemperature(uint32_t raw_temp) {
  // Get temperature oversampling setting for scaling
  spa06_003_oversample_t oversample = getTemperatureOversampling();
  float kT = getScalingFactor(oversample);

  // Convert to signed 32-bit
  int32_t temp_raw_signed = (int32_t)raw_temp;

  // Calculate scaled measurement result
  float temp_raw_sc = (float)temp_raw_signed / kT;

  // Calculate compensated temperature: Tcomp = c0*0.5 + c1*Traw_sc
  float temp_comp = (float)c0 * 0.5f + (float)c1 * temp_raw_sc;

  return temp_comp;
}

/*!
 *  @brief  Calculates compensated pressure from raw data
 *  @param  raw_pres Raw 24-bit pressure data
 *  @param  raw_temp Raw 24-bit temperature data (for compensation)
 *  @return Pressure in hectopascals (hPa)
 */
float Adafruit_SPA06_003::calculatePressure(uint32_t raw_pres,
                                            uint32_t raw_temp) {
  // Get oversampling settings for scaling
  spa06_003_oversample_t pres_oversample = getPressureOversampling();
  spa06_003_oversample_t temp_oversample = getTemperatureOversampling();

  float kP = getScalingFactor(pres_oversample);
  float kT = getScalingFactor(temp_oversample);

  // Convert to signed 32-bit
  int32_t pres_raw_signed = (int32_t)raw_pres;
  int32_t temp_raw_signed = (int32_t)raw_temp;

  // Calculate scaled measurement results
  float pres_raw_sc = (float)pres_raw_signed / kP;
  float temp_raw_sc = (float)temp_raw_signed / kT;

  // Calculate powers of Praw_sc for the compensation formula
  float pres_raw_sc_2 = pres_raw_sc * pres_raw_sc;
  float pres_raw_sc_3 = pres_raw_sc_2 * pres_raw_sc;
  float pres_raw_sc_4 = pres_raw_sc_3 * pres_raw_sc;

  // Calculate compensated pressure using the formula:
  // Pcomp = c00 + c10*Praw_sc + c20*Praw_sc^2 + c30*Praw_sc^3 + c40*Praw_sc^4 +
  //         Traw_sc*(c01 + c11*Praw_sc + c21*Praw_sc^2 + c31*Praw_sc^3)
  float pres_comp =
      (float)c00 + (float)c10 * pres_raw_sc + (float)c20 * pres_raw_sc_2 +
      (float)c30 * pres_raw_sc_3 + (float)c40 * pres_raw_sc_4 +
      temp_raw_sc * ((float)c01 + (float)c11 * pres_raw_sc +
                     (float)c21 * pres_raw_sc_2 + (float)c31 * pres_raw_sc_3);

  // Convert from Pa to hPa (divide by 100)
  return pres_comp / 100.0f;
}

#if 0
/*!
 *  @brief  Gets the Adafruit_Sensor object for temperature readings
 *  @return Adafruit_Sensor pointer for temperature
 */
Adafruit_Sensor *Adafruit_SPA06_003::getTemperatureSensor() {
  if (!temp_sensor) {
    temp_sensor = new Adafruit_SPA06_003_Temp(this);
  }
  return temp_sensor;
}

/*!
 *  @brief  Gets the Adafruit_Sensor object for pressure readings
 *  @return Adafruit_Sensor pointer for pressure
 */
Adafruit_Sensor *Adafruit_SPA06_003::getPressureSensor() {
  if (!pressure_sensor) {
    pressure_sensor = new Adafruit_SPA06_003_Pressure(this);
  }
  return pressure_sensor;
}

/*!
 *  @brief  Gets the sensor_t device data for temperature sensor
 *  @param  sensor Pointer to sensor_t device info struct
 */
void Adafruit_SPA06_003_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "SPA06_003", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40.0;  // Datasheet minimum
  sensor->max_value = 85.0;   // Datasheet maximum
  sensor->resolution = 0.01;  // Datasheet resolution
}

/*!
 *  @brief  Gets the latest sensor event for temperature
 *  @param  event Pointer to sensors_event_t struct
 *  @return True on successful event generation
 */
bool Adafruit_SPA06_003_Temp::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theSPA06003->readTemperature();
  return true;
}

/*!
 *  @brief  Gets the sensor_t device data for pressure sensor
 *  @param  sensor Pointer to sensor_t device info struct
 */
void Adafruit_SPA06_003_Pressure::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "SPA06_003", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 300.0;   // Datasheet minimum in hPa
  sensor->max_value = 1100.0;  // Datasheet maximum in hPa
  sensor->resolution = 0.012;  // Datasheet resolution in hPa
}

/*!
 *  @brief  Gets the latest sensor event for pressure
 *  @param  event Pointer to sensors_event_t struct
 *  @return True on successful event generation
 */
bool Adafruit_SPA06_003_Pressure::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  event->pressure = _theSPA06003->readPressure();
  return true;
}
#endif

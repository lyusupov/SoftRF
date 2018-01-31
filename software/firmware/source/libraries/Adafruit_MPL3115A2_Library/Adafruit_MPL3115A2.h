/*!
 * @file Adafruit_MPL3115A2.h
 *
 * This is part of Adafruit's MPL3115A2 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit MPL3115A2 breakout:
 * https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include <TinyWireM.h>
#else
 #include <Wire.h>
#endif


#define MPL3115A2_ADDRESS                       (0x60)    ///< default I2C address 1100000

/**************************************************************************/
/*!
    @brief  MPL3115A2 registers
*/
/**************************************************************************/
enum {
    MPL3115A2_REGISTER_STATUS        =       (0x00),

    MPL3115A2_REGISTER_PRESSURE_MSB     =    (0x01),
    MPL3115A2_REGISTER_PRESSURE_CSB    =     (0x02),
    MPL3115A2_REGISTER_PRESSURE_LSB    =     (0x03),

    MPL3115A2_REGISTER_TEMP_MSB       =      (0x04),
    MPL3115A2_REGISTER_TEMP_LSB      =       (0x05),

    MPL3115A2_REGISTER_DR_STATUS     =       (0x06),

    MPL3115A2_OUT_P_DELTA_MSB        =       (0x07),
    MPL3115A2_OUT_P_DELTA_CSB        =       (0x08),
    MPL3115A2_OUT_P_DELTA_LSB        =       (0x09),

    MPL3115A2_OUT_T_DELTA_MSB        =       (0x0A),
    MPL3115A2_OUT_T_DELTA_LSB        =       (0x0B),

    MPL3115A2_WHOAMI            =            (0x0C),

    MPL3115A2_BAR_IN_MSB         =           (0x14),
    MPL3115A2_BAR_IN_LSB         =           (0x15),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 status register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_REGISTER_STATUS_TDR       = 0x02,
    MPL3115A2_REGISTER_STATUS_PDR       = 0x04,
    MPL3115A2_REGISTER_STATUS_PTDR      = 0x08,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 PT DATA register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_PT_DATA_CFG       = 0x13,
    MPL3115A2_PT_DATA_CFG_TDEFE     = 0x01, 
    MPL3115A2_PT_DATA_CFG_PDEFE     = 0x02,
    MPL3115A2_PT_DATA_CFG_DREM      = 0x04,
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control registers
*/
/**************************************************************************/
enum {

    MPL3115A2_CTRL_REG1     =    (0x26),
    MPL3115A2_CTRL_REG2         =   (0x27),
    MPL3115A2_CTRL_REG3         =   (0x28),
    MPL3115A2_CTRL_REG4         =   (0x29),
    MPL3115A2_CTRL_REG5         =    (0x2A),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_SBYB   = 0x01,
    MPL3115A2_CTRL_REG1_OST     = 0x02,
    MPL3115A2_CTRL_REG1_RST      = 0x04,
    MPL3115A2_CTRL_REG1_RAW     = 0x40,
    MPL3115A2_CTRL_REG1_ALT     = 0x80,
    MPL3115A2_CTRL_REG1_BAR     = 0x00,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 oversample values
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_OS1     = 0x00,
    MPL3115A2_CTRL_REG1_OS2     = 0x08,
    MPL3115A2_CTRL_REG1_OS4     = 0x10,
    MPL3115A2_CTRL_REG1_OS8     = 0x18,
    MPL3115A2_CTRL_REG1_OS16        = 0x20,
    MPL3115A2_CTRL_REG1_OS32        = 0x28,
    MPL3115A2_CTRL_REG1_OS64        = 0x30,
    MPL3115A2_CTRL_REG1_OS128       = 0x38,
};


#define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion
/*=========================================================================*/

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with MPL3115A2 altimeter
*/
/**************************************************************************/
class Adafruit_MPL3115A2{
 public:
  Adafruit_MPL3115A2();
  boolean begin(TwoWire *twoWire = &Wire);
  float getPressure(void);
  float getAltitude(void);
  float getTemperature(void);
  void setSeaPressure(float pascal);

  void write8(uint8_t a, uint8_t d);

 private:
  TwoWire *_i2c;
  uint8_t read8(uint8_t a);
  uint8_t mode;

  typedef union {
    struct {
        uint8_t SBYB:1;
        uint8_t OST:1;
        uint8_t RST:1;
        uint8_t OS:3;
        uint8_t RAW:1;
        uint8_t ALT:1;
    } bit;
    uint8_t reg;
  } ctrl_reg1;
  ctrl_reg1 _ctrl_reg1;

};

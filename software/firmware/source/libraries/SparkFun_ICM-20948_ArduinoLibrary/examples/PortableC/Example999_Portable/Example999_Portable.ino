/****************************************************************
 * Example999_Portable.ino
 * ICM 20948 Arduino Library Demo
 * Uses underlying portable C skeleton with user-defined read/write functions
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

//#define USE_SPI // uncomment to use SPI instead

#ifdef USE_SPI
  #define SPI_PORT SPI
  #define CS_PIN 2
  #define SPI_CLK 1000000
  SPISettings mySettings(SPI_CLK, MSBFIRST, SPI_MODE3);
#else
  #define WIRE_PORT Wire
  #define I2C_ADDR ICM_20948_I2C_ADDR_AD1
#endif

// These are the interface functions that you would define for your system. They can use either I2C or SPI,
// or really **any** protocol as long as it successfully reads / writes the desired data into the ICM in the end
#ifdef USE_SPI
  ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);
  ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
#else
  ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
  ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
#endif

// You declare a "Serial Interface" (serif) type and give it the pointers to your interface functions
#ifdef USE_SPI
const ICM_20948_Serif_t mySerif = {
    my_write_spi, // write
    my_read_spi,  // read
    NULL,         // this pointer is passed into your functions when they are called.
};
#else
const ICM_20948_Serif_t mySerif = {
    my_write_i2c, // write
    my_read_i2c,  // read
    NULL,
};
#endif

// Now declare the structure that represents the ICM.
ICM_20948_Device_t myICM;

void setup()
{

  // Perform platform initialization
  Serial.begin(115200);

#ifdef USE_SPI
  SPI_PORT.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  // Aha! The SPI initialization monster bytes again! Let's send one byte out to 'set' the pins into a good starting state
  SPI_PORT.beginTransaction(mySettings);
  SPI_PORT.transfer(0x00);
  SPI_PORT.endTransaction();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // Link the serif
  ICM_20948_link_serif(&myICM, &mySerif);

  while (ICM_20948_check_id(&myICM) != ICM_20948_Stat_Ok)
  {
    Serial.println("whoami does not match. Halting...");
    delay(1000);
  }

  ICM_20948_Status_e stat = ICM_20948_Stat_Err;
  uint8_t whoami = 0x00;
  while ((stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI))
  {
    whoami = 0x00;
    stat = ICM_20948_get_who_am_i(&myICM, &whoami);
    Serial.print("whoami does not match (0x");
    Serial.print(whoami, HEX);
    Serial.print("). Halting...");
    Serial.println();
    delay(1000);
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  ICM_20948_sw_reset(&myICM);
  delay(250);

  // Set Gyro and Accelerometer to a particular sample mode
  ICM_20948_set_sample_mode(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myfss;
  myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  ICM_20948_set_full_scale(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss);

  // Set up DLPF configuration
  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.a = acc_d473bw_n499bw;
  myDLPcfg.g = gyr_d361bw4_n376bw5;
  ICM_20948_set_dlpf_cfg(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

  // Choose whether or not to use DLPF
  ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Acc, false);
  ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Gyr, false);

  // Now wake the sensor up
  ICM_20948_sleep(&myICM, false);
  ICM_20948_low_power(&myICM, false);
}

void loop()
{
  delay(1000);

  ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
  if (ICM_20948_get_agmt(&myICM, &agmt) == ICM_20948_Stat_Ok)
  {
    printRawAGMT(agmt);
  }
  else
  {
    Serial.println("Uh oh");
  }
}

///////////////////////////////////////////////////////////////
/* Here's where you actually define your interface functions */
///////////////////////////////////////////////////////////////

#ifdef USE_SPI
ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  SPI_PORT.beginTransaction(mySettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI_PORT.transfer(((reg & 0x7F) | 0x00));
  for (uint32_t indi = 0; indi < len; indi++)
  {
    SPI_PORT.transfer(*(data + indi));
  }
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI_PORT.endTransaction();

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  SPI_PORT.beginTransaction(mySettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI_PORT.transfer(((reg & 0x7F) | 0x80));
  for (uint32_t indi = 0; indi < len; indi++)
  {
    *(buff + indi) = SPI_PORT.transfer(0x00);
  }
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI_PORT.endTransaction();

  return ICM_20948_Stat_Ok;
}

#else

ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  WIRE_PORT.beginTransmission(I2C_ADDR);
  WIRE_PORT.write(reg);
  WIRE_PORT.write(data, len);
  WIRE_PORT.endTransmission();

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  WIRE_PORT.beginTransmission(I2C_ADDR);
  WIRE_PORT.write(reg);
  WIRE_PORT.endTransmission(false); // Send repeated start

  uint32_t num_received = WIRE_PORT.requestFrom(I2C_ADDR, len);
  if (num_received == len)
  {
    for (uint32_t i = 0; i < len; i++)
    {
      buff[i] = WIRE_PORT.read();
    }
  }
  WIRE_PORT.endTransmission();

  return ICM_20948_Stat_Ok;
}

#endif

// Some helper functions

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

float getAccMG(int16_t raw, uint8_t fss)
{
  switch (fss)
  {
  case 0:
    return (((float)raw) / 16.384);
    break;
  case 1:
    return (((float)raw) / 8.192);
    break;
  case 2:
    return (((float)raw) / 4.096);
    break;
  case 3:
    return (((float)raw) / 2.048);
    break;
  default:
    return 0;
    break;
  }
}

float getGyrDPS(int16_t raw, uint8_t fss)
{
  switch (fss)
  {
  case 0:
    return (((float)raw) / 131);
    break;
  case 1:
    return (((float)raw) / 65.5);
    break;
  case 2:
    return (((float)raw) / 32.8);
    break;
  case 3:
    return (((float)raw) / 16.4);
    break;
  default:
    return 0;
    break;
  }
}

float getMagUT(int16_t raw)
{
  return (((float)raw) * 0.15);
}

float getTmpC(int16_t raw)
{
  return (((float)raw) / 333.87);
}

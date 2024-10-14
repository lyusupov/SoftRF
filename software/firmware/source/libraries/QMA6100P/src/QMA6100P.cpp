#include <Arduino.h>
#include "QMA6100P.h"

uint8_t QMA6100P::getUniqueID()
{
  uint8_t tempVal;
  if(!readRegisterRegion(SFE_QMA6100P_CHIP_ID, &tempVal, 1))
    return 0xFF;

  return tempVal;
}

//////////////////////////////////////////////////
// softwareReset()
//
// writing 0xB6 to 0x36, soft reset all of the registers. 
// After soft-reset, user should write 0x00 back
//
bool QMA6100P::softwareReset()
{
  if(!writeRegisterByte(SFE_QMA6100P_SR, static_cast<uint8_t>(0xb6)))
    return false;

  sfe_qma6100p_sr_bitfeild_t sr;

  for(int i = 0; i < 10; i ++){
    if(!readRegisterRegion(SFE_QMA6100P_SR, &sr.all, 1))
      return false;

    if(sr.all == 0xb6)
      break;
    delay(1);
  }

  if(!writeRegisterByte(SFE_QMA6100P_SR, 0x00))
    return false;

  return true;
}

//////////////////////////////////////////////////
// enableAccel()
//
// Enables accelerometer data. In addition
// some settings can only be set when the accelerometer is
// powered down
//
// Parameter:
// enable - enables or disables the accelerometer
//
//
bool QMA6100P::enableAccel(bool enable)
{

  uint8_t tempVal;

  if(!readRegisterRegion(SFE_QMA6100P_PM, &tempVal, 1))
    return false;

  sfe_qma6100p_pm_bitfield_t pm;
  pm.all = tempVal;
  pm.bits.mode_bit = enable; // sets QMA6100P to active mode
  tempVal = pm.all;

  if(!writeRegisterByte(SFE_QMA6100P_PM, tempVal))
    return false;

  return true;
}

//////////////////////////////////////////////////
// getOperatingMode()
//
// Retrieves the current operating mode - stanby/active mode
//
uint8_t QMA6100P::getOperatingMode()
{

  uint8_t tempVal;
  int retVal;

  if(!readRegisterRegion(SFE_QMA6100P_PM, &tempVal, 1))
    return false;

  sfe_qma6100p_pm_bitfield_t pm;
  pm.all = tempVal; // This is a long winded but definitive way of getting the operating mode bit

  return (pm.bits.mode_bit); // Return the operating mode bit
}

//////////////////////////////////////////////////
// setRange()
//
// Sets the operational g-range of the accelerometer.
//
// Parameter:
// range - sets the range of the accelerometer 2g - 32g depending
// on the version. 2g - 32g for the QMA6100P.
//
bool QMA6100P::setRange(uint8_t range)
{

  uint8_t tempVal;

  if (range > SFE_QMA6100P_RANGE32G)
    return false;

  // Read - Modify - Write                                                                      
  if(!readRegisterRegion(SFE_QMA6100P_FSR, &tempVal, 1))
    return false;

  sfe_qma6100p_fsr_bitfield_t fsr;
  fsr.all = tempVal;
  fsr.bits.range = range; // This is a long winded but definitive way of setting the range (g select)
  tempVal = fsr.all;

  if(!writeRegisterByte(SFE_QMA6100P_FSR, tempVal))
    return false;

  _range = range; // Update our local copy

  return true;
}

// return current setting for accelleration range
uint8_t QMA6100P::getRange(){

  uint8_t tempVal;
  uint8_t range;

  // Read - Modify - Write
  if(!readRegisterRegion(SFE_QMA6100P_FSR, &tempVal, 1))
    return false;

  sfe_qma6100p_fsr_bitfield_t fsr;
  fsr.all = tempVal;
  range = fsr.bits.range;
  
  return range;
}

//////////////////////////////////////////////////
// enableDataEngine()
//
// Enables the data ready bit. and maps it to INT1
//
// Parameter:
// enable - enable/disables the data ready bit.
bool QMA6100P::enableDataEngine(bool enable)
{
  uint8_t tempVal;

  if(!readRegisterRegion(SFE_QMA6100P_PM, &tempVal, 1))
    return false;

  sfe_qma6100p_int_map1_bitfield_t int_map1;
  int_map1.all = tempVal;
  int_map1.bits.int1_data = enable; // data ready interrupt to INT1
  tempVal = int_map1.all;

  if(!writeRegisterByte(SFE_QMA6100P_INT_MAP1, tempVal))
    return false;

  // enable data ready interrupt
  if(!readRegisterRegion(SFE_QMA6100P_INT_EN1, &tempVal, 1))
    return false;

  sfe_qma6100p_int_en1_bitfield_t int_en1;
  int_en1.all = tempVal;
  int_en1.bits.int_data_en = enable; // set data ready interrupt
  tempVal = int_en1.all;

  if(!writeRegisterByte(SFE_QMA6100P_INT_EN1, tempVal))
    return false;

  return true;
}

bool QMA6100P::setFifoMode(uint8_t fifo_mode){

  uint8_t tempVal;

  if(!readRegisterRegion(SFE_QMA6100P_FIFO_CFG0, &tempVal, 1))
    return false;

  sfe_qma6100p_fifo_cfg0_bitfield_t fifo_cfg0;
  fifo_cfg0.all = tempVal;
  fifo_cfg0.bits.fifo_mode = fifo_mode; // data ready interrupt to INT1
  fifo_cfg0.bits.fifo_en_xyz = 0b111;
  tempVal = fifo_cfg0.all;

  if(!writeRegisterByte(SFE_QMA6100P_FIFO_CFG0, tempVal))
    return false;

  return true;

}

//////////////////////////////////////////////////
// getRawAccelRegisterData()
//
// Retrieves the raw register values representing accelerometer data.
//
//
// Parameter:
// *rawAccelData - a pointer to the data struct that holds acceleromter X/Y/Z data.
//
bool QMA6100P::getRawAccelRegisterData(rawOutputData *rawAccelData)
{
  uint8_t tempRegData[6] = {0};
  int16_t tempData = 0;

  if(!readRegisterRegion(SFE_QMA6100P_DX_L, tempRegData, 6)) // Read 3 * 16-bit
    return false;

  // check newData_X
  if(tempRegData[0] & 0x1){
    tempData = (int16_t)(((uint16_t)(tempRegData[1] << 8)) | (tempRegData[0]));
    rawAccelData->xData = tempData >> 2;
  }
  // check newData_Y
  if(tempRegData[2] & 0x1){
    tempData = (int16_t)(((uint16_t)(tempRegData[3] << 8)) | (tempRegData[2]));
    rawAccelData->yData = tempData >> 2;
  } 
  // check newData_Z
  if(tempRegData[4] & 0x1){
    tempData = (int16_t)(((uint16_t)(tempRegData[5] << 8)) | (tempRegData[4]));
    rawAccelData->zData = tempData >> 2;
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////
// readRegisterRegion()

bool QMA6100P::readRegisterRegion(uint8_t registerAddress, uint8_t* sensorData, int len)
{
  Wire.beginTransmission(QMA6100P_ADDRESS_LOW);
  Wire.write(registerAddress); // Register address to read from
  uint8_t err = Wire.endTransmission(); // Send the request without stopping the transmission

  if (err > 0) {
    return false;
  }

  uint8_t bytesAvailable = Wire.requestFrom(static_cast<int>(QMA6100P_ADDRESS_LOW), static_cast<int>(len), static_cast<int>(true)); // Request len byte of data

  delay(10);

  if (bytesAvailable >= len) {
    for (uint8_t i = 0; i < len; i++) {
      sensorData[i] = Wire.read(); // Read the bytes from the sensor and store them in the array pointed to by sensorData
    }
    return true; // Return true if the read operation was successful
  } else {
    return false;
  }
}


//////////////////////////////////////////////////////////////////////////////////
// writeRegisterRegion()

bool QMA6100P::writeRegisterByte(uint8_t registerAddress, uint8_t data)
{
  Wire.beginTransmission(QMA6100P_ADDRESS_LOW);
  Wire.write(registerAddress); // Register address to write to
  Wire.write(data); // Data to write, dereferenced from the pointer
  uint8_t err = Wire.endTransmission(); // End the transmission

  if (err > 0) {
    return false; // Return false if there's a communication error
  }

  return true; // Return true if the write operation was successful
}


//***************************************** QMA6100P ******************************************************


bool QMA6100P::begin()
{
  if (getUniqueID() != QMA6100P_CHIP_ID)
    return false;

  return true;
}

bool QMA6100P::calibrateOffsets()
{
    outputData data;
    int numSamples = 100;
    float xSum = 0.0, ySum = 0.0, zSum = 0.0;


    // Take multiple samples to average out noise
    for (int i = 0; i < numSamples; i++)
    {
        if (!getAccelData(&data))
            return false;
        
        xSum += data.xData;
        ySum += data.yData;
        zSum += data.zData - 1;
        delay(10);
    }

    // Calculate average
    xOffset = xSum / numSamples;
    yOffset = ySum / numSamples;
    zOffset = zSum / numSamples;  // Assuming z-axis aligned with gravity

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
// getAccelData()
//
// Retrieves the raw accelerometer data and calls a conversion function to convert the raw values.
//
// Parameter:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
//
bool QMA6100P::getAccelData(outputData *userData)
{

  if(!getRawAccelRegisterData(&rawAccelData))
    return false;

  if(!convAccelData(userData, &rawAccelData))
    return false;

  return true;
}

//////////////////////////////////////////////////////////////////////////////////
// convAccelData()
//
// Converts raw acceleromter data with the current accelerometer's range settings.
//
// Parameter:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
// *rawAccelData - a pointer to the data struct that holds acceleromter X/Y/Z data.
//
bool QMA6100P::convAccelData(outputData *userAccel, rawOutputData *rawAccelData)
{
  if (_range < 0) // If the G-range is unknown, read it
  {
    uint8_t regVal;

    if(!readRegisterRegion(SFE_QMA6100P_FSR, &regVal, 1))
      return false;

    sfe_qma6100p_fsr_bitfield_t fsr;
    fsr.all = regVal;

    _range = fsr.bits.range; // Record the range
  }

  switch (_range)
  {
  case SFE_QMA6100P_RANGE2G:
    userAccel->xData = (float)rawAccelData->xData * convRange2G;
    userAccel->yData = (float)rawAccelData->yData * convRange2G;
    userAccel->zData = (float)rawAccelData->zData * convRange2G;
    break;
  case SFE_QMA6100P_RANGE4G:
    userAccel->xData = (float)rawAccelData->xData * convRange4G;
    userAccel->yData = (float)rawAccelData->yData * convRange4G;
    userAccel->zData = (float)rawAccelData->zData * convRange4G;
    break;
  case SFE_QMA6100P_RANGE8G:
    userAccel->xData = (float)rawAccelData->xData * convRange8G;
    userAccel->yData = (float)rawAccelData->yData * convRange8G;
    userAccel->zData = (float)rawAccelData->zData * convRange8G;
    break;
  case SFE_QMA6100P_RANGE16G:
    userAccel->xData = (float)rawAccelData->xData * convRange16G;
    userAccel->yData = (float)rawAccelData->yData * convRange16G;
    userAccel->zData = (float)rawAccelData->zData * convRange16G;
    break;
  case SFE_QMA6100P_RANGE32G:
    userAccel->xData = (float)rawAccelData->xData * convRange32G;
    userAccel->yData = (float)rawAccelData->yData * convRange32G;
    userAccel->zData = (float)rawAccelData->zData * convRange32G;
    break;
  default:
    return false;
  }

  return true;
}

void QMA6100P::setOffset(float x, float y, float z){
  xOffset = x;
  yOffset = y;
  zOffset = z;
}


void QMA6100P::offsetValues(float &x, float &y, float &z) {
  x = x - xOffset;
  y = y - yOffset;
  z = z - zOffset;
}
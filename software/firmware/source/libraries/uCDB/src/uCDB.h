/**
   @file uCDB.h
   uCDB API

   @author    Ioulianos Kakoulidis
   @date      2021
   @copyright Public Domain
*/

#ifndef uCDB_h
#define uCDB_h

#include "Arduino.h"

#if !defined(ARDUINO_ARCH_NRF52)
#include <SPI.h>
#include <SD.h>
#else
#include <Adafruit_SPIFlash.h>

extern FatFileSystem fatfs;

#define SD  fatfs
#endif

enum cdbResult {
  CDB_OK = 0,
  CDB_CLOSED, ///< Initial state
  CDB_NOT_FOUND,
  CDB_ERROR,  ///< CDB data integrity critical error
  FILE_ERROR, ///< File operation (open/seek/read) error
  KEY_FOUND,
  KEY_NOT_FOUND
};

unsigned long DJBHash(const void *key, unsigned long keyLen);

class uCDB
{
  public:
    uCDB();

    /**
        Open CDB file
        @param fileName  CDB filename
        @param userHashFunc  User provided hash function, default - DJBHash
    */
    cdbResult open(const char *fileName, unsigned long (*userHashFunc)(const void *key, unsigned long keyLen) = DJBHash);

    /**
        Find `key'
    */
    cdbResult findKey(const void *key, unsigned long keyLen);

    /**
        Find next `value' after successful finKey() call
    */
    cdbResult findNextValue();

    /**
        Read next available `value' byte
        after successful finKey() or findNextValue() call
    */
    int readValue();

    /**
        Read next available `value' byteNum bytes
        after successful finKey() or findNextValue() call
    */
    int readValue(void *buff, unsigned int byteNum);

    /**
        Close CDB
    */
    cdbResult close();

  private:
    File cdb;
    cdbResult state;

    const byte *key_;
    unsigned long keyLen_;
    unsigned long keyHash;

    unsigned long dataEndPos;
    unsigned long slotsNum;

    /// @name Hash table descriptor (HEADER section)
    /// @{
    unsigned long hashTabStartPos; ///< Hash table position
    unsigned long hashTabSlotsNum; ///< Hash table slot number
    /// @}
    unsigned long hashTabEndPos; ///< hashTabStartPos + 8 * hashTabSlotsNum

    /// @name Slot descriptor (HASH TABLE section)
    /// @{
    unsigned long slotHash;
    unsigned long dataPos;
    /// @}

    unsigned long slotsToScan;
    unsigned long nextSlotPos;

    /// @name Data (key, value) descriptor (DATA section)
    /// @{
    unsigned long dataKeyLen;   ///< Key length in bytes
    unsigned long dataValueLen; ///< Value length in bytes
    /// @}

    unsigned long valueBytesAvail;

    bool readDescriptor(byte *buff, unsigned long pos);
    cdbResult compareKey();
    unsigned long (*hashFunc)(const void *key, unsigned long keyLen);
    void zero();
};

#endif

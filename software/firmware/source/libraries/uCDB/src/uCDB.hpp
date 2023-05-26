/**
    @file uCDB.hpp
    uCDB template implementation

    @author    Ioulianos Kakoulidis
    @date      2021
    @copyright Public Domain


        A structure for constant databases
        19960914
        Copyright 1996
        D. J. Bernstein, djb@pobox.com

        A cdb is an associative array: it maps strings (`keys`) to strings
        (`data`).

        A cdb contains 256 pointers to linearly probed open hash tables. The
        hash tables contain pointers to (key,data) pairs. A cdb is stored in
        a single file on disk:

            +----------------+---------+-------+-------+-----+---------+
            | p0 p1 ... p255 | records | hash0 | hash1 | ... | hash255 |
            +----------------+---------+-------+-------+-----+---------+

        Each of the 256 initial pointers states a position and a length. The
        position is the starting byte position of the hash table. The length
        is the number of slots in the hash table.

        Records are stored sequentially, without special alignment. A record
        states a key length, a data length, the key, and the data.

        Each hash table slot states a hash value and a byte position. If the
        byte position is 0, the slot is empty. Otherwise, the slot points to
        a record whose key has that hash value.

        Positions, lengths, and hash values are 32-bit quantities, stored in
        little-endian form in 4 bytes. Thus a cdb must fit into 4 gigabytes.

        A record is located as follows. Compute the hash value of the key in
        the record. The hash value modulo 256 is the number of a hash table.
        The hash value divided by 256, modulo the length of that table, is a
        slot number. Probe that slot, the next higher slot, and so on, until
        you find the record or run into an empty slot.

        The cdb hash function is `h = ((h << 5) + h) ^ c`, with a starting
        hash of 5381.   

*/

#define UCDB_VERSION_MAJOR 0
#define UCDB_VERSION_MINOR 5
#define UCDB_VERSION_PATCH 5

#ifdef TRACE_CDB
#ifndef TracePrinter
#define TracePrinter Serial
#endif
#define RETURN(statement, var) \
  TracePrinter.print("[T]: "); \
  TracePrinter.print(__FUNCTION__); \
  TracePrinter.print(": "); \
  TracePrinter.print(__LINE__); \
  TracePrinter.print(", "); \
  TracePrinter.println(var); \
  return (statement);
#else
#define RETURN(statement, var) return (statement);
#endif

/// uCDB result codes and states
enum cdbResult {
  CDB_OK = 0,
  CDB_CLOSED, ///< Initial state
  CDB_NOT_FOUND, ///< open() result
  CDB_ERROR,  ///< CDB data integrity critical error
  FILE_ERROR, ///< File operation (open/seek/read) error
  KEY_FOUND,
  KEY_NOT_FOUND
};

unsigned long DJBHash(const void *key, unsigned long keyLen);

template <class TFileSystem, class TFile>
class uCDB
{
  public:

    /**
        uCDB constructor
    */
    uCDB(TFileSystem& fs) :
      fs_(fs),
      state_(CDB_CLOSED),
      slotsToScan_(0),
      nextSlotPos_(0) {}

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
        Total records number in CDB
    */
    unsigned long recordsNumber() const {
      switch (state_) {
        case CDB_CLOSED:
        case CDB_ERROR:
          return 0;
        default:
          return (slotsNum_ >> 1);
      }
    }

    /**
        The number of `value' bytes available for reading
    */
    unsigned long valueAvailable() const {
      return (state_ == KEY_FOUND ? valueBytesAvail_ : 0);
    }

    /**
        The UCDB state
    */
    cdbResult state() const {
      return state_;
    }

    /**
        Close CDB
    */
    cdbResult close();

    /**
        uCDB destructor
    */
    ~uCDB();

  private:
    TFileSystem& fs_;
    TFile cdb_;
    cdbResult state_;

    const byte *key_;
    unsigned long keyLen_;
    unsigned long keyHash_;

    unsigned long dataEndPos_; ///< Data end position
    unsigned long slotsNum_;   ///< Total slots number in CDB

    unsigned int hashTabID_; ///< Last accessed hash table
    /// @name Hash table descriptor (HEADER section)
    /// @{
    unsigned long hashTabStartPos_; ///< Hash table position
    unsigned long hashTabSlotsNum_; ///< Hash table slot number
    /// @}
    unsigned long hashTabEndPos_; ///< hashTabStartPos_ + 8 * hashTabSlotsNum_
    unsigned long slotsToScan_;
    unsigned long nextSlotPos_;

    /// @name Slot descriptor (HASH TABLE section)
    /// @{
    unsigned long slotHash_;
    unsigned long dataPos_;
    /// @}

    /// @name Data (key, value) descriptor (DATA section)
    /// @{
    unsigned long dataKeyLen_;   ///< Key length in bytes
    unsigned long dataValueLen_; ///< Value length in bytes
    /// @}
    unsigned long valueBytesAvail_;

    cdbResult compareKey();
    unsigned long (*hashFunc)(const void *key, unsigned long keyLen);
    void zero();
};

#define CDB_DESCRIPTOR_SIZE 2 * (sizeof (unsigned long))
#define CDB_HEADER_SIZE 256 * CDB_DESCRIPTOR_SIZE
#define CDB_BUFF_SIZE 64

static unsigned long unpack(const byte *buff);

template <class TFile>
static bool readDescriptor(TFile& file, byte *buff, unsigned long pos);

template <class TFileSystem, class TFile>
cdbResult uCDB<TFileSystem, TFile>::open(const char *fileName, unsigned long (*userHashFunc)(const void *key, unsigned long keyLen)) {
  unsigned long htPos;
  unsigned long htSlotsNum;

  unsigned long dend;
  unsigned long snum;

  byte buff[CDB_DESCRIPTOR_SIZE];

  // Close previously opened CDB file
  // State - CDB_CLOSED
  close();
  hashTabID_ = -1;

  if (!fs_.exists(fileName)) {
    return CDB_NOT_FOUND;
  }

  cdb_ = fs_.open(fileName);
  if (!cdb_) {
    return CDB_CLOSED;
  }

  // CDB hash tables position and slots number integrity check

  // CDB file size must be at least HEADER_SIZE bytes
  if (cdb_.size() < CDB_HEADER_SIZE) {
    RETURN(state_ = CDB_ERROR, CDB_ERROR);
  }

  dend = cdb_.size();
  snum = 0;

  for (unsigned long pos = 0; pos < CDB_HEADER_SIZE; pos += CDB_DESCRIPTOR_SIZE) {
    if (!readDescriptor<TFile>(cdb_, buff, pos)) {
      RETURN(state_ = CDB_ERROR, pos); // File read error is critical here.
    }

    htPos = unpack(buff);
    htSlotsNum = unpack(buff + 4);

    if ((htPos < CDB_HEADER_SIZE) || (htPos > cdb_.size())) {
      RETURN(state_ = CDB_ERROR, htPos); // Critical CDB format or data integrity error
    }
    if (((cdb_.size() - htPos) >> 3) < htSlotsNum) {
      RETURN(state_ = CDB_ERROR, htSlotsNum); // Critical CDB format or data integrity error
    }

    // Adjust data end position and total slots number
    if (htPos < dend) {
      dend = htPos;
    }
    snum += htSlotsNum;

    if (((cdb_.size() - dend) >> 3) < snum) {
      RETURN(state_ = CDB_ERROR, snum); // Critical CDB format or data integrity error
    }
  }
  // Check total
  if ((cdb_.size() - dend) != 8 * snum){
    RETURN(state_ = CDB_ERROR, 8 * snum); // Critical CDB format or data integrity error
  }

  dataEndPos_ = dend;
  slotsNum_ = snum;
  hashFunc = userHashFunc;

  return (state_ = CDB_OK);
}

template <class TFileSystem, class TFile>
cdbResult uCDB<TFileSystem, TFile>::findKey(const void *key, unsigned long keyLen) {
  byte buff[CDB_DESCRIPTOR_SIZE];
  unsigned int hashTabID;

  zero();
  // Check CDB state
  switch (state_) {
    case CDB_CLOSED:
    case CDB_ERROR:
      return state_;
    default:
      ;
  }

  key_ = static_cast<const byte *>(key);
  keyLen_ = keyLen;
  keyHash_ = hashFunc(key, keyLen);
  hashTabID = keyHash_ & 255;
  
  if (hashTabID != hashTabID_) {
    if (!readDescriptor<TFile>(cdb_, buff, hashTabID << 3)) {
      RETURN(state_ = FILE_ERROR, FILE_ERROR);
    }

    hashTabStartPos_ = unpack(buff);
    hashTabSlotsNum_ = unpack(buff + 4);
    hashTabEndPos_ = hashTabStartPos_ + hashTabSlotsNum_ * CDB_DESCRIPTOR_SIZE;
    hashTabID_ = hashTabID; 
  }
  slotsToScan_ = hashTabSlotsNum_;
  nextSlotPos_ = hashTabStartPos_ + ((keyHash_ >> 8) % hashTabSlotsNum_) * 8;

  return findNextValue();
}

template <class TFileSystem, class TFile>
cdbResult uCDB<TFileSystem, TFile>::findNextValue() {
  byte buff[CDB_BUFF_SIZE];

  // Check CDB state
  switch (state_) {
    case CDB_CLOSED:
    case CDB_ERROR:
      return state_;
    default:
      ;
  }

  while (slotsToScan_) {
    bool rd = readDescriptor<TFile>(cdb_, buff, nextSlotPos_);
    // Adjust slotsToScan_ and next slot position
    --slotsToScan_;
    nextSlotPos_ += CDB_DESCRIPTOR_SIZE;
    if (nextSlotPos_ == hashTabEndPos_) {
      nextSlotPos_ = hashTabStartPos_;
    }

    if (!rd) {
      RETURN(state_ = FILE_ERROR, FILE_ERROR);
    }

    slotHash_ = unpack(buff);
    dataPos_ = unpack(buff + 4);

    if (!dataPos_) {
      break;  
    }

    // Check data position
    if ((dataPos_ < CDB_HEADER_SIZE) || (dataPos_ > (dataEndPos_ - CDB_DESCRIPTOR_SIZE))) {
      RETURN(state_ = CDB_ERROR, CDB_ERROR); // Critical CDB format or data integrity error
    }

    if (slotHash_ == keyHash_) {
      if (!readDescriptor<TFile>(cdb_, buff, dataPos_)) {
        RETURN(state_ = FILE_ERROR, FILE_ERROR);
      }

      dataKeyLen_ = unpack(buff);
      dataValueLen_ = unpack(buff + 4);

      //> key, value length check
      unsigned long t = dataPos_ + CDB_DESCRIPTOR_SIZE;
      if ((dataEndPos_ - t) < dataKeyLen_) {
        RETURN(state_ = CDB_ERROR, CDB_ERROR); // Critical CDB format or data integrity error
      }
      t += dataKeyLen_;
      if ((dataEndPos_ - t) < dataValueLen_) {
        RETURN(state_ = CDB_ERROR, CDB_ERROR); // Critical CDB format or data integrity error
      }
      //< key, value length check

      if (keyLen_ == dataKeyLen_) {
        switch (compareKey()) {
          case KEY_FOUND:
            valueBytesAvail_ = dataValueLen_;
            return (state_ = KEY_FOUND);
          case FILE_ERROR:
            RETURN(state_ = FILE_ERROR, FILE_ERROR);
          default:
            ;
        }
      }
    }
  }

  zero();

  return (state_ = KEY_NOT_FOUND);
}

template <class TFileSystem, class TFile>
int uCDB<TFileSystem, TFile>::readValue() {
  if ((state_ == KEY_FOUND) && valueBytesAvail_) {
    int rt = cdb_.read();
    if (rt != -1) {
      --valueBytesAvail_;
    }
    return rt;
  }

  return -1;
}

template <class TFileSystem, class TFile>
int uCDB<TFileSystem, TFile>::readValue(void *buff, unsigned int byteNum) {
  if (state_ == KEY_FOUND) {
    if (byteNum > valueBytesAvail_) {
      byteNum = valueBytesAvail_;
    }
    int br = cdb_.read(buff, byteNum);
    if (br > 0) {
      valueBytesAvail_ -= br;
    }
    return br;
  }

  return -1;
}

template <class TFileSystem, class TFile>
cdbResult uCDB<TFileSystem, TFile>::close() {
  zero();
  if (cdb_) {
    cdb_.close();
  }
  return (state_ = CDB_CLOSED);
}

template <class TFileSystem, class TFile>
uCDB<TFileSystem, TFile>::~uCDB() {
  close();
}

// Private functions
template <class TFileSystem, class TFile>
cdbResult uCDB<TFileSystem, TFile>::compareKey() {
  const byte *key = key_;
  unsigned long keyLen = keyLen_;
  byte buff[CDB_BUFF_SIZE];

  while (keyLen >= CDB_BUFF_SIZE) {
    if (cdb_.read(buff, CDB_BUFF_SIZE) != CDB_BUFF_SIZE) {
      return FILE_ERROR;
    }
    if (memcmp(key, buff, CDB_BUFF_SIZE)) {
      return KEY_NOT_FOUND;
    }
    keyLen -= CDB_BUFF_SIZE;
    key += CDB_BUFF_SIZE;
  }

  // keyLen < CDB_BUFF_SIZE
  if (keyLen) {
    if (cdb_.read(buff, keyLen) != (int)keyLen) {
      return FILE_ERROR;
    }
    if (memcmp(key, buff, keyLen)) {
      return KEY_NOT_FOUND;
    }
  }

  return KEY_FOUND;
}

template <class TFileSystem, class TFile>
void uCDB<TFileSystem, TFile>::zero() {
  slotsToScan_ = 0;
  nextSlotPos_ = 0;
}

#define DJB_START_HASH 5381UL
unsigned long DJBHash(const void *key, unsigned long keyLen) {
  unsigned long h = DJB_START_HASH;
  const byte *curr = static_cast<const byte *>(key);
  const byte *end = curr + keyLen;

  while (curr < end) {
    h = ((h << 5) + h) ^ *curr;
    ++curr;
  }

  return h;
}
#undef DJB_START_HASH

// Static functions

unsigned long unpack(const byte *buff) {
  unsigned long v = buff[3];

  v = (v << 8) + buff[2];
  v = (v << 8) + buff[1];
  v = (v << 8) + buff[0];

  return v;
}

template <class TFile>
bool readDescriptor(TFile& file, byte *buff, unsigned long pos) {
  if (file.position() != pos) {
    file.seek(pos);
    if (file.position() != pos) {
      return false;
    }
  }

  return (file.read(buff, CDB_DESCRIPTOR_SIZE) == CDB_DESCRIPTOR_SIZE);
}

#undef CDB_HEADER_SIZE
#undef CDB_DESCRIPTOR_SIZE
#undef CDB_BUFF_SIZE

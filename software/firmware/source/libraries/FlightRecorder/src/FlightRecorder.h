/*
 * FlightRecorder.h
 *
 * Copyright (C) 2023 Linar Yusupov. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef FLIGHTRECORDER_h
#define FLIGHTRECORDER_h

#define PREFER_SDFAT_LIBRARY
#include <IniFile.h>
#include <TinyGPS++.h>
#include <MD5.h>
#include <SdFat.h>

// Config

/*
; IGC logger configuration file
[igcheader]
Pilot=Winnie Pooh
CoPilot=Mary Poppins
Glider=Duo Discus
Registration=RF-012345
CallSign=XXX
Class=Two Seater

[config]
liftoff_detection=true
liftoff_threshold=1.5
log_interval=5
*/

#if !defined(ARDUINO_ARCH_ESP32)
typedef struct __attribute__((__packed__))
#else
typedef struct
#endif /* ARDUINO_ARCH_ESP32 */
{
    char pilot[80];
    char copilot[80];
    char type[40];
    char reg[10];
    char cs[10];
    char cls[20];
    bool liftoff_detection;
    double liftoff_threshold;
    int log_interval;
} config_t;

class igc_file_writer final {

  igc_file_writer() = delete;
  igc_file_writer(const igc_file_writer &) = delete;
  igc_file_writer(igc_file_writer &&) = delete;

  igc_file_writer& operator=(const igc_file_writer &) = delete;
  igc_file_writer& operator=(igc_file_writer &&) = delete;

public:

  igc_file_writer(const char *file, bool grecord, SdFat *SD_ptr);

  template <size_t size> 
  bool append(const char (&data)[size]) {
    static_assert(size > 0, "invalid size");
    return append(data, size);
  }

private:
  bool append(const char *data, size_t size);

  const char *file_path; /** full path of target igc file */
  const bool add_grecord; /** true if G record must be added to file */

  long next_record_position = 0; /** position of G record */

  MD5::MD5_CTX md5_a; //= {0x63e54c01, 0x25adab89, 0x44baecfe, 0x60f25476};
  MD5::MD5_CTX md5_b; //= {0x41e24d03, 0x23b8ebea, 0x4a4bfc9e, 0x640ed89a};
  MD5::MD5_CTX md5_c; //= {0x61e54e01, 0x22cdab89, 0x48b20cfe, 0x62125476};
  MD5::MD5_CTX md5_d; //= {0xc1e84fe8, 0x21d1c28a, 0x438e1a12, 0x6c250aee};

  SdFat *_SD_ptr;
};

namespace IGC
{
    typedef union __attribute__((__packed__))
    {
        // IGC B record
        struct __attribute__((__packed__))
        {
            char b ;        // 'B'
            char time[6];   // YYMMDD
            char lat[8];    // DDMMmmmN/S
            char lng[9];    // DDDMMmmmE/W
            char a;         // 'A'
            char pAlt[5];   // pressure altitude
            char gAlt[5];   // GNSS altitude
            char fxa[3];    // 2 sigma FXA (Fix Accuracy)
            char siu[2];    // satellites in use
        };
        char raw[1+6+8+9+1+5+5+3+2+1]; // +1 for NULL terminator
    } igc_t;

    enum pos_type
    {
        LAT,
        LNG
    };

    bool initIGC(SdFat *SD_ptr);
    bool createIGCFileName(uint16_t y, uint16_t m, uint16_t d, SdFat *SD_ptr);
    void enableIGCWrite(bool enable=true);
    int  writeRecord(const char *, bool sign=true);
    int  writeIGCHeader(uint8_t y, uint8_t m, uint8_t d, config_t & config, SdFat *SD_ptr);
    int  writeARecord();
    int  writeBRecord(TinyGPSPlus *gps_ptr, float alt, config_t & config, SdFat *SD_ptr);
    void writeGRecord(const MD5::MD5_CTX &ctx);
    int  writeHRecord(const char *format, ...);
    void closeIGC();
}

class FlightRecorder {
public:
  FlightRecorder(void);

  bool begin(SdFat *, uint32_t, const char *);
  void loop(TinyGPSPlus *, float);
  void end();

protected:
  SdFat *_SD_ptr;
  float _alt;
};

#endif /* FLIGHTRECORDER_h */

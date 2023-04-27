/*
 * FlightRecorder.cpp
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

#include <FlightRecorder.h>
#include <TimeLib.h>

#define MANUFACTURER_NAME     "XSR"
#define INI_DIR               ""
#define FLIGHTS_DIR           "Flights"

static TinyGPSPlus *_gnss_ptr = NULL;
static uint32_t _id           = 0;
static const char *_gnss_type = "UNKNOWN";

// defaults
namespace CONFIG
{
  static const char* CONFIG_DEFAULT_PILOT = "Winnie Pooh";
  static const char* CONFIG_DEFAULT_COPILOT = "Mary Poppins";
  static const char* CONFIG_DEFAULT_TYPE = "Duo Discus";
  static const char* CONFIG_DEFAULT_REG = "RF-012345";
  static const char* CONFIG_DEFAULT_CS = "XXX";
  static const char* CONFIG_DEFAULT_CLASS = "Two Seater";
  static const float CONFIG_DEFAULT_LIFTOFF_THRESHOLD = 1.5;
  static const bool  CONFIG_DEFAULT_LIFTOFF_DETECT_ENABLE = true;
  static const int   CONFIG_DEFAULT_LOG_INTERVAL = 5;
}

void printErrorMessage(uint8_t e, bool eol = true)
{
  switch (e) 
  {
  case IniFile::errorNoError:
    Serial.print("no error");
    break;
  case IniFile::errorFileNotFound:
    Serial.print("file not found");
    break;
  case IniFile::errorFileNotOpen:
    Serial.print("file not open");
    break;
  case IniFile::errorBufferTooSmall:
    Serial.print("buffer too small");
    break;
  case IniFile::errorSeekError:
    Serial.print("seek error");
    break;
  case IniFile::errorSectionNotFound:
    Serial.print("section not found");
    break;
  case IniFile::errorKeyNotFound:
    Serial.print("key not found");
    break;
  case IniFile::errorEndOfFile:
    Serial.print("end of file");
    break;
  case IniFile::errorUnknownError:
    Serial.print("unknown error");
    break;
  default:
    Serial.print("unknown error value");
    break;
  }
  if (eol) 
  {
    Serial.println();
  }
}

static void getStringValue(IniFile &ini, const char *section, const char* key, char* result, const char* def_value)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  strcpy(result,def_value);
  if (ini.getValue(section, key, iniline, bufferLen)) 
  {
    strcpy(result,iniline);
  }
  else 
  {
    char err[200];
    snprintf(err,sizeof(err), "Could not read '%s' from section '%s', will use default '%s', error: ", 
      key,section, def_value);
    Serial.print(err);
    printErrorMessage(ini.getError());
  }
}

static void getULongValue(IniFile &ini, const char *section, const char* key, unsigned long &result, const unsigned long def_value)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  result = def_value;
  if (ini.getValue(section, key, iniline, bufferLen)) {
    result = atol(iniline);
  }
  else {
    char err[200];
    snprintf(err,sizeof(err),"Could not read '%s' from section '%s', will use default %ld, error: ",
        key,section,def_value);
    Serial.print(err);
    printErrorMessage(ini.getError());
  }
}

static void getIntValue(IniFile &ini, const char *section, const char* key, int &result, const int def_value)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  result = def_value;
  if (ini.getValue(section, key, iniline, bufferLen)) {
    result = atoi(iniline);
  }
  else {
    char err[200];
    snprintf(err,sizeof(err),"Could not read '%s' from section '%s', will use default %d, error: ",
        key,section,def_value);
    Serial.print(err);
    printErrorMessage(ini.getError());
  }
}

static void getBoolValue(IniFile &ini, const char *section, const char* key, bool &result, const bool def_value)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  result = def_value;
  if (ini.getValue(section, key, iniline, bufferLen)) {
    result = strcasecmp("true",iniline) == 0;
  }
  else {
    char err[200];
    snprintf(err,sizeof(err),"Could not read '%s' from section '%s', will use default %d, error: ",
        key,section,def_value);
    Serial.print(err);
    printErrorMessage(ini.getError());
  }
}

static void getDoubleValue(IniFile &ini, const char *section, const char* key, double &result, const double def_value)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  result = def_value;
  if (ini.getValue(section, key, iniline, bufferLen)) {
    result = atof(iniline);
  }
  else {
    char err[200];
    snprintf(err,sizeof(err),"Could not read '%s' from section '%s', will use default %f, error: ",
        key,section,def_value);
    Serial.print(err);
    printErrorMessage(ini.getError());
  }
}

/*
; IGC logger configuration file
[igcheader]
Pilot=Winnie Pooh
CoPilot=Mary Poppins
Type=Duo Discus
Registration=RF-012345
CallSign=XXX
Class=Two Seater

[config]
liftoff_detection=true
liftoff_threshold=1.5
log_interval=5
*/

static bool readConfig(const char* iniFilename, config_t &config, SdFat *SD_ptr = &SD)
{
  const size_t bufferLen = 80;
  char iniline[bufferLen];

  // init config with defaults
  strncpy(config.pilot,CONFIG::CONFIG_DEFAULT_PILOT, sizeof(config.pilot));
  strncpy(config.copilot, CONFIG::CONFIG_DEFAULT_COPILOT, sizeof(config.copilot));
  strncpy(config.type, CONFIG::CONFIG_DEFAULT_TYPE, sizeof(config.type));
  strncpy(config.reg, CONFIG::CONFIG_DEFAULT_REG, sizeof(config.reg));
  strncpy(config.cs, CONFIG::CONFIG_DEFAULT_CS, sizeof(config.cs));
  strncpy(config.cls, CONFIG::CONFIG_DEFAULT_CLASS, sizeof(config.cls));
  config.liftoff_detection = CONFIG::CONFIG_DEFAULT_LIFTOFF_DETECT_ENABLE;
  config.liftoff_threshold = CONFIG::CONFIG_DEFAULT_LIFTOFF_THRESHOLD;
  config.log_interval = CONFIG::CONFIG_DEFAULT_LOG_INTERVAL;

  IniFile ini(iniFilename);
  if (!ini.open(SD_ptr)) 
  {
    Serial.print("Ini file '");
    Serial.print(iniFilename);
    Serial.println("' does not exist");
    return false;
  }
  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(iniline, bufferLen)) 
  {
    Serial.print("ini file ");
    Serial.print(ini.getFilename());
    Serial.print(" not valid: ");
    printErrorMessage(ini.getError());
    return false;
  }
  // read settings
  getStringValue(ini,"igcheader","Pilot", config.pilot, CONFIG::CONFIG_DEFAULT_PILOT); 
  getStringValue(ini,"igcheader","CoPilot", config.copilot, CONFIG::CONFIG_DEFAULT_COPILOT); 
  getStringValue(ini,"igcheader","Type", config.type, CONFIG::CONFIG_DEFAULT_TYPE);
  getStringValue(ini,"igcheader","CallSign", config.cs, CONFIG::CONFIG_DEFAULT_CS);
  getStringValue(ini,"igcheader","Registration", config.reg, CONFIG::CONFIG_DEFAULT_REG);
  getStringValue(ini,"igcheader","Class", config.cls, CONFIG::CONFIG_DEFAULT_CLASS);
  getIntValue(   ini,"config", "log_interval", config.log_interval, CONFIG::CONFIG_DEFAULT_LOG_INTERVAL);
  getDoubleValue(ini,"config", "liftoff_threshold", config.liftoff_threshold, CONFIG::CONFIG_DEFAULT_LIFTOFF_THRESHOLD);
  getBoolValue(  ini,"config", "liftoff_detection", config.liftoff_detection, CONFIG::CONFIG_DEFAULT_LIFTOFF_DETECT_ENABLE);

  return true;
}

static void printConfig(const config_t &config)
{
    Serial.println(F("Config:"));
    Serial.println(F("======================================================"));
    Serial.print(F("Pilot            : "));
    Serial.println(config.pilot);
    Serial.print(F("Co-Pilot         : "));
    Serial.println(config.copilot);
    Serial.print(F("Type             : "));
    Serial.println(config.type);
    Serial.print(F("Registration     : "));
    Serial.println(config.reg);
    Serial.print(F("Class            : "));
    Serial.println(config.cls);
    Serial.print(F("Call Sign        : "));
    Serial.println(config.cs);
    Serial.print(F("Liftoff Detection: "));
    Serial.println(config.liftoff_detection);
    Serial.print(F("Liftoff Threshold: "));
    Serial.println(config.liftoff_threshold);
    Serial.print(F("Logger Interval  : "));
    Serial.println(config.log_interval);
    Serial.println(F("======================================================"));
}

namespace {
  // return c if valid char for IGC files
  // return space if not.
  char clean_igc_char(char c) {
    if (c >= 0x20 && c <= 0x7E && c != 0x24 &&
        c != 0x2A && c != 0x2C && c != 0x21 &&
        c != 0x5C && c != 0x5E && c != 0x7E) {
      return c;
    }
    return ' ';
  }

  void write_g_record(File32 &stream, const MD5::MD5_CTX &md5) {
    char part1[17];
    char part2[17];
    MD5::MD5_CTX md5_tmp;
    // we made copy to allow to continue to update hash after Final call.
  // make a copy, so we can continue with
  // next record in case flight not done
  memcpy(&md5_tmp,&md5, sizeof(MD5::MD5_CTX));
  unsigned char * hash = (unsigned char *) malloc(16);
  MD5::MD5::MD5Final(hash, &md5_tmp);
  char *md5str = MD5::MD5::make_digest(hash,16);
//  Serial.print("MD5 hash > ");
//  Serial.println(md5str);
  // split in two 16 char. strings
  memset(&part1,0,sizeof(part1));
  memset(&part2,0,sizeof(part2));
  memcpy(part1,md5str,16);
  memcpy(part2,(md5str+16),16);

  stream.print("G");
  stream.print(part1);
  stream.print("\r\nG");
  if (stream.getWriteError()) 
  {
    Serial.println(F("Error writing G-record!"));
  }
  stream.print(part2);
  stream.print("\r\n");
  if (stream.getWriteError()) 
  {
    Serial.println(F("Error writing G-record!"));
  }
  free(md5str);
  free(hash);
  }
} // namespace

igc_file_writer::igc_file_writer(const char *file, bool grecord, SdFat *SD_ptr = &SD)
    : file_path(file), add_grecord(grecord) {
        // LK8000, not yet working, for a test file OK though!
  MD5::MD5::MD5Initialize(&md5_a, (unsigned long) 0x63e54c01, (unsigned long) 0x25adab89, (unsigned long) 0x44baecfe, (unsigned long) 0x60f25476);
  MD5::MD5::MD5Initialize(&md5_b, (unsigned long) 0x41e24d03, (unsigned long) 0x23b8ebea, (unsigned long) 0x4a4bfc9e, (unsigned long) 0x640ed89a);
  MD5::MD5::MD5Initialize(&md5_c, (unsigned long) 0x61e54e01, (unsigned long) 0x22cdab89, (unsigned long) 0x48b20cfe, (unsigned long) 0x62125476);
  MD5::MD5::MD5Initialize(&md5_d, (unsigned long) 0xc1e84fe8, (unsigned long) 0x21d1c28a, (unsigned long) 0x438e1a12, (unsigned long) 0x6c250aee);

  _SD_ptr = SD_ptr;
}


bool igc_file_writer::append(const char *data, size_t size) {

  uint8_t mode = O_WRITE;
  if (next_record_position <= 0)
  {
    // if a G-record is written, we'll use seek
    // to reposition file write location
    // and do not want to use O_APPEND
    // since this breaks the seek function!!
    // (must be an arduino thing...)
    mode |= O_APPEND;
  }
  File32 igcFile = _SD_ptr->open(file_path, mode);
  if(igcFile) 
  {
      if (next_record_position > 0)
      {
        if (!igcFile.seek(next_record_position))
        {
          Serial.println(F("Seek failed!!"));
        }
      }

    char c;
    for (; *(data) && size > 1; ++data, --size) {
      if ((*data) != 0x0D && (*data) != 0x0A) {
        c = clean_igc_char(*data);

        if (add_grecord) {
          MD5::MD5::MD5Update(&md5_a,&c,1);
          MD5::MD5::MD5Update(&md5_b,&c,1);
          MD5::MD5::MD5Update(&md5_c,&c,1);
          MD5::MD5::MD5Update(&md5_d,&c,1);
        }
      } else {
        c = *data;
      }
      igcFile.print(c);
    }

    next_record_position = igcFile.position();

    if (add_grecord) {
      write_g_record(igcFile, md5_a);
      write_g_record(igcFile, md5_b);
      write_g_record(igcFile, md5_c);
      write_g_record(igcFile, md5_d);
    }
    igcFile.close();
    return true;
  }
  return false;
}

namespace IGC
{

static File32 igcFile;
static char igc_full_path[48];
static bool bIGCHeaderWritten = false;
static bool bIGCFileWrite = false;
static int BRecordCount = 0;
static const char* IGC_EOL = "\r\n";

static igc_file_writer* igc_writer_ptr = NULL;

template<size_t size>
bool IGCWriteRecord(const char(&szIn)[size]) {
    return igc_writer_ptr && igc_writer_ptr->append(szIn);
}

bool initIGC(SdFat *SD_ptr = &SD)
{
  bIGCHeaderWritten = false;
  BRecordCount = 0;

  // create Flights folder
  if (!SD_ptr->exists(FLIGHTS_DIR))
  {
    if (!SD_ptr->mkdir(FLIGHTS_DIR))
    {
       Serial.println(F("Error creating folder on SD!"));
       return false;
    }
  }
  return true;
}

void DumpIGCFile(const char* path, SdFat *SD_ptr = &SD)
{
  // re-open the file for reading:
  File32 myFile = SD_ptr->open(path);
  if (myFile) {
    Serial.print(F("Dumping "));
    Serial.println(path);

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      char c = myFile.read();
      Serial.write(c);
      if (c == 0x0a)
      {
        Serial.print('\r');
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.print(F("Error opening "));
    Serial.print(path);
    Serial.println(F(" for reading!"));
  }  
}

int writeIGCHeader(uint8_t y, uint8_t m, uint8_t d, config_t &config, SdFat *SD_ptr = &SD)
{
  int result = 0;
  if (!bIGCHeaderWritten)
  {
    //Serial.println(F("Writing IGC Header..."));
    // create empty file
    igcFile = SD_ptr->open(igc_full_path,O_WRITE | O_CREAT | O_TRUNC);
    if(igcFile)
    {
      igcFile.close();
      BRecordCount = 0;
      //construct IGC header
      result = writeARecord(); // MUST be 1st record!
      // put in UTC time stamp and config values
      if (result) result = writeHRecord("HFDTE%02d%02d%02d",d,m,y); // DDMMYY
      if (result) result = writeHRecord("HFFXA035");
      if (result) result = writeHRecord("HFPLTPILOTINCHARGE: %s",config.pilot);
      if (result) result = writeHRecord("HFCM2CREW2: %s",config.copilot);
      if (result) result = writeHRecord("HFGTYGLIDERTYPE: %s",config.type);
      if (result) result = writeHRecord("HFGIDGLIDERID: %s",config.reg);
      if (result) result = writeHRecord("HFDTM100GPSDATUM: WGS-1984");
      if (result) result = writeHRecord("HFRFWFIRMWAREVERSION: 1.4");
      if (result) result = writeHRecord("HFRHWHARDWAREVERSION: 1.0");
      if (result) result = writeHRecord("HFFTYFRTYPE: SoftRF,Prime Mark III");
      if (result) result = writeHRecord("HFGPSRECEIVER: %s",_gnss_type);
      if (result) result = writeHRecord("HFALGALTGPS:GEO");     // for non IGC loggers
      if (result) result = writeHRecord("HFALPALTPRESSURE:ISA");
      if (result) result = writeHRecord("HFPRSPRESSALTSENSOR: Bosch,BME280,9163m");
      if (result) result = writeHRecord("HFCIDCOMPETITIONID: %s",config.cs);
      if (result) result = writeHRecord("HFCCLCOMPETITIONCLASS: %s",config.cls);
      if (result) result = writeHRecord("I023638FXA3940SIU");  // FXA and SIU number
      if (result)
      {
        bIGCHeaderWritten = true;
        // for debug!
        //DumpIGCFile(igc_full_path, SD_ptr);
      }
    }
    else 
    {
        Serial.println(F("Error opening IGC file for header!"));
    }
  }
  return result;  
}

static char* convertDoubleToDDMMmmm(double f, const char* c, pos_type type)
{
  static char result[20];
  memset(&result,0,sizeof(result));
  char tmp[10];
  
  // Abs(f)
  double d = f;
  if (d<0) d = -d;
	// degrees
	int degr = (int) d;
	double dminu = (d - degr)*60;
	int minu = floor(dminu);
	int sec = (int) .5 + (dminu - minu)*1000;
	switch (type)
	{
	case LAT:
		// DDMMmmmN/S
		sprintf(tmp, "%02d", degr);
		strcpy(result, tmp);
		sprintf(tmp, "%02d", minu);
		strcat(result, tmp);
		sprintf(tmp, "%03d", sec);
		strncat(result, tmp, 3);
		break;
	case LNG:
		// DDDMMmmmE/W
		sprintf(tmp, "%03d", degr);
		strcpy(result, tmp);
		sprintf(tmp, "%02d", minu);
		strcat(result, tmp);
		sprintf(tmp, "%03d", sec);
		strncat(result, tmp, 3);
		break;
	}
	strcat(result, c);
//  Serial.print(f,6);
//  Serial.print(F(" -> "));
//  Serial.println(result);
  return result;
}

int writeBRecord(TinyGPSPlus *gps_ptr, float alt, config_t &config, SdFat *SD_ptr = &SD)
{
    const char* c;
    igc_t cur_igc;
    char igcbuffer[100];
    int result = 0;

    // IGC file write enabled but no header written yet?
    if (bIGCFileWrite && !bIGCHeaderWritten)
    {
        IGC::writeIGCHeader(gps_ptr->date.year()-2000,
                            gps_ptr->date.month(),
                            gps_ptr->date.day(),
                            config, SD_ptr);
    }
    // prepare IGC B-record
    memset(&cur_igc,0,sizeof(cur_igc));
    memset(&cur_igc,'0',sizeof(cur_igc) - 1);
    cur_igc.a = 'A';
    cur_igc.b = 'B';
    // HHMMSS
    sprintf(igcbuffer, "%02d%02d%02d",gps_ptr->time.hour(),
                                      gps_ptr->time.minute(),
                                      gps_ptr->time.second());
    memcpy(&cur_igc.time,igcbuffer,6);
    //      12345678
    // LAT: DDMMmmmN/S
    c = gps_ptr->location.rawLat().negative ? "S" : "N";
    sprintf(igcbuffer,"%s",convertDoubleToDDMMmmm(gps_ptr->location.lat(),c, IGC::LAT));
    memcpy(&cur_igc.lat,igcbuffer,8);
    //      123456789
    // LNG: DDDMMmmmE/W
    c = gps_ptr->location.rawLng().negative ? "W" : "E";
    sprintf(igcbuffer,"%s",convertDoubleToDDMMmmm(gps_ptr->location.lng(),c, IGC::LNG));
    memcpy(&cur_igc.lng,igcbuffer,9);
    // pressure altitude in meters
    sprintf(igcbuffer,"%05d",(int) alt);
    memcpy(&cur_igc.pAlt,igcbuffer,5);
    // GNSS altitude in meters
    sprintf(igcbuffer,"%05d",(int) gps_ptr->altitude.meters());
    memcpy(&cur_igc.gAlt,igcbuffer,5);

    // Using this formula to get a rough 2-sigma ehp value
    float fxa = (gps_ptr->hdop.value()/100.0) * 5.1 * 2.0;
//    Serial.print("HDOP = ");
//    Serial.print(gps_ptr->hdop.value()/100.0,6);
//    Serial.print(", FXA = ");
//    Serial.println(fxa, 6);

    sprintf(igcbuffer,"%03u", (unsigned int) fxa);
    memcpy(&cur_igc.fxa,igcbuffer,3);

    // Output the SIU (Satellites In Use) Information
    sprintf(igcbuffer,"%02u", (unsigned int) gps_ptr->satellites.value());
    memcpy(&cur_igc.siu,igcbuffer,2);

//    Serial.println(cur_igc.raw);
    result = writeRecord(cur_igc.raw, true);
    if (result)
    {
      BRecordCount++;
    }
    return result;
}

void closeIGC()
{
  igcFile.close();
}

bool createIGCFileName(uint16_t y,uint16_t m, uint16_t d, SdFat *SD_ptr = &SD)
{
    int i = 0;

    for (i=1; i<100; i++) {
      memset(&igc_full_path,0,sizeof(igc_full_path));

      strcpy(igc_full_path, FLIGHTS_DIR);
      strcat(igc_full_path,"/");

      size_t len = strlen(igc_full_path);
      char *short_name = igc_full_path + len;
      snprintf(short_name, sizeof(igc_full_path) - len,
               "%d-%d-%d-" MANUFACTURER_NAME "-%03X-%02d.IGC",
               y, m, d, _id, i);
      if (!SD_ptr->exists(igc_full_path)) {
        break;
      }
    }

    if (i >= 100) return false;

    //Serial.print(F("IGC path: "));
    //Serial.println(igc_full_path);

    if (IGC::igc_writer_ptr == NULL)
    {
      IGC::igc_writer_ptr = new igc_file_writer(igc_full_path, true, SD_ptr);
    }
    return true;
}

int writeRecord(const char *data, bool sign)
{
  int len = strlen(data);
  int result = 0;
  if (bIGCFileWrite)
  {
    char line[128];
    memset(&line,0,sizeof(line));
    strncpy(line,data, len);
    strcat(line, IGC_EOL);
    if (IGC::IGCWriteRecord(line))
    {
      result = 1;
    }
  }
  return result;
}

int writeARecord()
{
  // A RECORD - FR ID NUMBER. 
  // The A Record must be the first record in an FR Data File, 
  // and includes the three-character GNSS FR Serial Number (S/N) unique
  // to the manufacturer of the FR that recorded the flight.
  // Format of the A Record:
  //
  // A M M M N N N T E X T S T R I N G CR LF
  // +---------------------------------------------------------+
  // | A record Description | Size     | Element| Remarks      |
  // +---------------------------------------------------------+
  // | Manufacturer         | 3 bytes  | MMM    | Alphanumeric |
  // | Unique ID            | 3 bytes  | NNN    | Alphanumeric |
  // | ID extension         | Optional | STRING | Alphanumeric |
  // +---------------------------------------------------------+
  //
  // XXX is reserved for other manufacturers, other than
  // the offical assigned three-character codes
  //
  // X and XXX are general designations for IGC format files
  // from manufacturers who do not produce an IGC-approved recorder.
  // Such recorders will not have been tested and evaluated by
  // GFAC and may not fulfil certain aspects of the IGC Specification
  // such as security protections, recording of pressure altitude, 
  // ENL or other variables. There is no guarantee that the file will
  // conform exactly to the IGC format, although specimen files will
  // be checked for compliance with the IGC format if emailed to the
  // GFAC chairman for evaluation. Even after this procedure, 
  // no compliance guarantee can be made because the type of recorder
  // will not have completed a full GFAC evaluation. It should be noted 
  // that although the file name will not contain the information, 
  // the details of the manufacturer and the recorder model concerned
  // will be identifiable (if the file conforms to the IGC standard)
  // because they will be included in the H (Header) record, 
  // see below under H Record in the line: 
  // HFFTYFRTYPE:MANUFACTURERSNAME,FRMODELNUMBER CRLF

  char arec[8];
  memset(&arec,0,sizeof(arec));
  snprintf(arec, sizeof(arec),"A" MANUFACTURER_NAME "%03X", _id);

  return writeRecord(arec, true);
}

int writeHRecord(const char* format, ...)
{
  char line[80];
  va_list arg;
  va_start(arg, format);
  vsnprintf(line,sizeof(line),format,arg);
  va_end(arg);
  return writeRecord(line, true);
}

// NOTE: FILE IS EXPECTED TO BE STILL OPEN FOR WRITING!
void writeGRecord(const MD5::MD5_CTX &ctx)
{
  MD5::MD5_CTX md5_context_tmp;
  char line[80];
  char part1[17];
  char part2[17];
  memset(&line,0,sizeof(line));

  // make a copy, so we can continue with
  // next record in case flight not done
  memcpy(&md5_context_tmp,&ctx, sizeof(MD5::MD5_CTX));
  unsigned char * hash = (unsigned char *) malloc(16);
  MD5::MD5::MD5Final(hash, &md5_context_tmp);
  char *md5str = MD5::MD5::make_digest(hash,16);
  Serial.print("MD5 hash > ");
  Serial.println(md5str);
  // split in two 16 char. strings
  memset(&part1,0,sizeof(part1));
  memset(&part2,0,sizeof(part2));
  memcpy(part1,md5str,16);
  memcpy(part2,(md5str+16),16);

  snprintf(line,sizeof(line),"G%s",part1);
  igcFile.print(line);
  igcFile.print(IGC_EOL);
  if (igcFile.getWriteError()) 
  {
    Serial.println(F("Error writing G-record!"));
  }
  snprintf(line,sizeof(line),"G%s",part2);
  igcFile.print(line);
  igcFile.print(IGC_EOL);
  free(md5str);
  free(hash);
}

void enableIGCWrite(bool enable)
{
  bIGCFileWrite = enable;
}

} // IGC namespace

//------------------------------------------------------------------------------
// call back for file timestamps (UTC but that's okay!)
void dateTime(uint16_t* date, uint16_t* time) {
  if (_gnss_ptr) {
    // return GNSS date using FAT_DATE macro to format fields
    *date = FAT_DATE(_gnss_ptr->date.year(), _gnss_ptr->date.month(), _gnss_ptr->date.day());
  
    // return GNSS time using FAT_TIME macro to format fields
    *time = FAT_TIME(_gnss_ptr->time.hour(), _gnss_ptr->time.minute(), _gnss_ptr->time.second());
  }
}

FlightRecorder::FlightRecorder(void)
: _SD_ptr(NULL)
, _alt(0)
{
}

// configuration
config_t config;

static bool in_flight = false;

bool FlightRecorder::begin(SdFat *SD_ptr, uint32_t id, const char *gnss_specs) {
  bool rval = false;

  _SD_ptr    = SD_ptr;
  _id        = id & 0xFFF;
  _gnss_type = gnss_specs;

  Serial.println(F("Reading FlightRecorder.ini..."));
  if (!readConfig("FlightRecorder.ini", config, _SD_ptr))
  {
    Serial.println(F("WARNING: unable to read configuration, will run with defaults!"));
  }
  printConfig(config);

  in_flight = !config.liftoff_detection;

  if (IGC::initIGC(_SD_ptr)) {
    // set date time callback function
    SdFile::dateTimeCallback(dateTime);

    rval= true;
  }

  return rval;
}

//#define DEBUG Serial
#define DEBUG if (false) Serial

char buffer[80];
uint8_t loop_count = 0;
static bool inits = true;
static bool bIGCFileWrite = false;
static unsigned long last_igc_write = 0;

// number of satelites from GNSS
static int sats = 0;

void FlightRecorder::loop(TinyGPSPlus *gnss_ptr, float press_alt) {
    static unsigned long sec = 0;
    static uint8_t minu = 0;
    static uint8_t hours = 23;
    static unsigned long msec = 0;
    static unsigned long old_msec = 0;
    static float old_alt;
    static float raw_deriv,derivative;
    static uint8_t count_lcd = 0;
    static uint8_t count_sd = 0;
    static uint8_t count_gnss = 0;
    static float ground_speed = 0.0f;
    static int elapsed;
    static long val = 0;

    _gnss_ptr = gnss_ptr;
    _alt      = press_alt;

    // Update Running Time
    msec = millis();
    val   = now();
    hours = hour();
    minu  = minute();
    sec   = second();

    // 1st time here?
    if (inits) 
    {
        old_alt = _alt;
        old_msec = msec;
        last_igc_write = msec;
        inits = false;
    }

    // keep track of Vario
    elapsed = msec - old_msec;  
    // Process Derivative every ~0.1 s
    if (elapsed >= 100)
    {
        // ~ m/s
        // delta altitude divided by elapsed time
        raw_deriv = ((_alt - old_alt) * 1000) /elapsed;
        old_alt = _alt;
        old_msec = msec;
    }
    // 90% is old value and 10% is from raw result
    derivative = 0.9 * derivative + 0.1 * raw_deriv;
    // more than 1.5 m/s and valid GNSS?
    if(!in_flight && derivative > config.liftoff_threshold && _gnss_ptr->location.isValid())
    {
      DEBUG.print(F("Take off! Vario="));
      DEBUG.print(derivative);
      DEBUG.print(F("m/s, Raw="));
      DEBUG.println(raw_deriv);
      in_flight = true;
    }

    switch (loop_count)
    {
        case 1 : // Print data
#if 0
            count_lcd ++;
            if (count_lcd == 100) // limit printing
            {
                // Note: this is logger running time, not GNSS time!
                sprintf(buffer,"%02d:%02d:%02d ",hours,minu,(uint8_t) sec & 0xff);
                DEBUG.print(buffer);

                // Number of GNSS SATS
                DEBUG.print(F("SATS: "));
                if (_gnss_ptr->satellites.isValid())
                {
                  sats = _gnss_ptr->satellites.value();
                  sprintf(buffer,"%02d, ",sats);
                  DEBUG.print(buffer);
                }
                else
                {
                  DEBUG.print("**, ");
                }
 
                DEBUG.print(F("Altitude:      "));
                DEBUG.print(_alt);
                DEBUG.print(F(" m, "));

                if (_gnss_ptr->location.isValid())
                {
                  DEBUG.print(F(", GNSS Altitude:      "));
                  DEBUG.print(_gnss_ptr->altitude.meters());
                  DEBUG.print(F("m, Groundspeed: "));
                  DEBUG.print(ground_speed);
                  DEBUG.print(F(" km/h"));
                }
                DEBUG.println();
                if (msec < 5000)
                {
                  // at startup dump the GNSS stream to Serial for 5 sec.
                  DEBUG.println(F("GNSS stream dump:"));
                  while (millis()<5000) {
                    if (Serial1.available() > 0) { // any data coming in?
                      DEBUG.write(Serial1.read());
                    }
                  }
                  DEBUG.println();
                }
               count_lcd = 0;
            }
#endif
            break;
        case 2: // Read GNSS & Write data to SD Card
            if (_gnss_ptr->location.isValid())
            {
              // get ground speed
              if (_gnss_ptr->speed.isValid() && _gnss_ptr->speed.isUpdated())
              {
                ground_speed = _gnss_ptr->speed.kmph();
              }

              // in flight?
              if (in_flight)
              {
                // write a "B" record to file?
                if ((msec - last_igc_write) >= (unsigned long) (config.log_interval * 1000))
                {
                  last_igc_write = msec;
                  count_sd += IGC::writeBRecord(_gnss_ptr, _alt, config, _SD_ptr);
                }
              }
            }
            break;
        case 3:
            break;
       default:
            // reset state machine
            loop_count = 0;
            break;
    }

    // date and time known?
    if (!bIGCFileWrite && 
        _gnss_ptr->time.isValid() && _gnss_ptr->date.isValid() &&
        _gnss_ptr->date.month()>0 && _gnss_ptr->date.day()>0 &&
        _gnss_ptr->time.isUpdated()) 
    {
      // wait 5 loops longer
      count_gnss++;
      if (count_gnss > 5)
      {
        count_gnss = 0;
        if (!bIGCFileWrite)
        {
            DEBUG.println(F("*******************************************"));
            DEBUG.println(F("***** GNSS clock set, enable IGC write ****"));
            DEBUG.println(F("*******************************************"));
            bIGCFileWrite = IGC::createIGCFileName(_gnss_ptr->date.year(),
                                                   _gnss_ptr->date.month(),
                                                   _gnss_ptr->date.day(),
                                                   _SD_ptr);
            IGC::enableIGCWrite(bIGCFileWrite);
            if (!bIGCFileWrite)
            {
              DEBUG.println(F("***** ERROR enabling IGC write! *****"));
            }
            DEBUG.print(F("in_flight = "));
            DEBUG.println(in_flight);
        }
      }
    }
    loop_count++;
}

void FlightRecorder::end() {
    IGC::closeIGC();
}

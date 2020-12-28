/*
 * LogHelper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SoC.h"
#include "Log.h"

#if LOGGER_IS_ENABLED

#define LOGFILE "/Logfile.txt"

File LogFile;
FSInfo fs_info;
FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

void Logger_setup()
{
  char LogFilename[] = LOGFILE;


  if (SPIFFS.begin()) {
    Serial.println(F("SPIFFS volume is mounted successfully."));       

      SPIFFS.info(fs_info);
      
      Serial.println();
      Serial.print(F("Total bytes: "));
      Serial.println(fs_info.totalBytes);
      Serial.print(F("Used bytes: "));
      Serial.println(fs_info.usedBytes);
      Serial.print(F("Block size: "));
      Serial.println(fs_info.blockSize);
      Serial.print(F("Page size: "));
      Serial.println(fs_info.pageSize);

    //if (SPIFFS.exists(LogFilename)) SPIFFS.remove(LogFilename); 
  
    LogFile = SPIFFS.open(LogFilename, "a+");
  
    if (!LogFile) {
      Serial.print(F("Unable to open log file: "));
      Serial.println(LogFilename);
    } else {
      LogFile.println();
      LogFile.println(F("******* Logging is restarted *******"));
      LogFile.print(F("*** Storage free space: "));
      LogFile.print(fs_info.totalBytes - fs_info.usedBytes);    
      LogFile.println(F(" bytes ***"));
  
      //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
      ftpSrv.begin("softrf","softrf");    
    };
  } else {
    Serial.println(F("ERROR: Unable to mount SPIFFS volume."));      
  };

}

void Logger_loop()
{
  ftpSrv.handleFTP();
}

void Logger_fini()
{
  LogFile.close();
  SPIFFS.end();
}

#endif /* LOGGER_IS_ENABLED */
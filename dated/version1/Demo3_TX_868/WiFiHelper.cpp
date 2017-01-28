
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <TimeLib.h>

#include "WiFiHelper.h"
#include "OTAHelper.h"

String station_ssid = MY_ACCESSPOINT_SSID ;
String station_psk = MY_ACCESSPOINT_PSK ;

/**
 * @brief Default WiFi connection information.
 * @{
 */
const char* ap_default_ssid = "SoftRF"; ///< Default SSID.
const char* ap_default_psk = "123456123456"; ///< Default PSK.
/// @}

/// Uncomment the next line for verbose output over UART.
//#define SERIAL_VERBOSE

WiFiClient client;
#include <WiFiUDP.h>

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

#define TypeADSB  1

char UDPpacketBuffer[512]; //buffer to hold incoming and outgoing packets

/**
 * @brief Read WiFi connection information from file system.
 * @param ssid String pointer for storing SSID.
 * @param pass String pointer for storing PSK.
 * @return True or False.
 * 
 * The config file have to containt the WiFi SSID in the first line
 * and the WiFi PSK in the second line.
 * Line seperator can be \r\n (CR LF) \r or \n.
 */
bool loadConfig(String *ssid, String *pass)
{
  // open file for reading.
  File configFile = SPIFFS.open("/cl_conf.txt", "r");
  if (!configFile)
  {
    Serial.println("Failed to open cl_conf.txt.");

    return false;
  }

  // Read content from config file.
  String content = configFile.readString();
  configFile.close();
  
  content.trim();

  // Check if ther is a second line available.
  int8_t pos = content.indexOf("\r\n");
  uint8_t le = 2;
  // check for linux and mac line ending.
  if (pos == -1)
  {
    le = 1;
    pos = content.indexOf("\n");
    if (pos == -1)
    {
      pos = content.indexOf("\r");
    }
  }

  // If there is no second line: Some information is missing.
  if (pos == -1)
  {
    Serial.println("Infvalid content.");
    Serial.println(content);

    return false;
  }

  // Store SSID and PSK into string vars.
  *ssid = content.substring(0, pos);
  *pass = content.substring(pos + le);

  ssid->trim();
  pass->trim();

#ifdef SERIAL_VERBOSE
  Serial.println("----- file content -----");
  Serial.println(content);
  Serial.println("----- file content -----");
  Serial.println("ssid: " + *ssid);
  Serial.println("psk:  " + *pass);
#endif

  return true;
} // loadConfig


/**
 * @brief Save WiFi SSID and PSK to configuration file.
 * @param ssid SSID as string pointer.
 * @param pass PSK as string pointer,
 * @return True or False.
 */
bool saveConfig(String *ssid, String *pass)
{
  // Open config file for writing.
  File configFile = SPIFFS.open("/cl_conf.txt", "w");
  if (!configFile)
  {
    Serial.println("Failed to open cl_conf.txt for writing");

    return false;
  }

  // Save SSID and PSK.
  configFile.println(*ssid);
  configFile.println(*pass);

  configFile.close();
  
  return true;
} // saveConfig


/**
 * @brief Arduino setup function.
 */
void WiFi_setup()
{

  // Set Hostname.
  //hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(HOSTNAME);

  // Print hostname.
  //Serial.println("Hostname: " + hostname);
  Serial.println("Hostname: " + WiFi.hostname());


  // Initialize file system.
  if (!SPIFFS.begin())
  {
    Serial.println("Failed to mount file system");
    return;
  }

  // Load wifi connection information.
  if (! loadConfig(&station_ssid, &station_psk))
  {
    station_ssid = MY_ACCESSPOINT_SSID ;
    station_psk = MY_ACCESSPOINT_PSK ;

    Serial.println("No WiFi connection information available.");
  }

  // Check WiFi connection
  // ... check mode
  if (WiFi.getMode() != WIFI_STA)
  {
    WiFi.mode(WIFI_STA);
    delay(10);
  }

  // ... Compare file config with sdk config.
  if (WiFi.SSID() != station_ssid || WiFi.psk() != station_psk)
  {
    Serial.println("WiFi config changed.");

    // ... Try to connect to WiFi station.
    WiFi.begin(station_ssid.c_str(), station_psk.c_str());

    // ... Pritn new SSID
    Serial.print("new SSID: ");
    Serial.println(WiFi.SSID());

    // ... Uncomment this for debugging output.
    //WiFi.printDiag(Serial);
  }
  else
  {
    // ... Begin with sdk config.
    WiFi.begin();
  }

  Serial.println("Wait for WiFi connection.");

  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
  {
    Serial.write('.');
    //Serial.print(WiFi.status());
    delay(500);
  }
  Serial.println();

  // Check connection
  if(WiFi.status() == WL_CONNECTED)
  {
    // ... print IP Address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

#if CLOUD_MODE

    // close any connection before send a new request.
    // This will free the socket on the WiFi shield
    client.stop();

    delay(1000);

    Serial.print("Connection to "); Serial.print(CLOUD_HOSTNAME); Serial.print(":"); Serial.print(CLOUD_PORT); Serial.print(" ");
    // if there's a successful connection:
    if (client.connect(CLOUD_HOSTNAME, CLOUD_PORT)) {
      Serial.println("succeed");
    } else {
      // if you couldn't make a connection:
      Serial.println("failed");
    }
#endif

  }
  else
  {
    Serial.println("Can not connect to WiFi station. Go into AP mode.");
    
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);

    delay(10);

    WiFi.softAP(ap_default_ssid, ap_default_psk);

    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  }

}

void WiFi_forward_to_argus()
{
    char str_lat[16];
    char str_lon[16];

    dtostrf(fo.latitude, 8, 4, str_lat);
    dtostrf(fo.longtitude, 8, 4, str_lon);

    Udp.beginPacket(ARGUS_HOSTNAME, ARGUS_PORT);

    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "%u %X %s %s %d %s %s %u", \
      fo.timestamp * 1000, fo.addr, str_lat, str_lon, fo.altitude, "0" , "0" , TypeADSB );

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}

//convert degrees to radians
double dtor(double fdegrees)
{
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtod(double fradians)
{
  return(fradians * 180.0 / PI);
}

/*
 * arbarnhart
 * 
 * http://forum.arduino.cc/index.php?topic=45760.msg332012#msg332012
 */
//Calculate bearing from lat1/lon1 to lat2/lon2
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns bearing in degrees
int CalcBearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  lat2 = dtor(lat2);
  lon2 = dtor(lon2);
  
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  //bearing = fmod((bearing + 360.0), 360);
  //return (int) bearing + 0.5;
  return ((int) bearing + 360) % 360;
}

void WiFi_forward_to_xcsoar()
{
    int bearing = CalcBearing(fo.latitude, fo.longtitude, LATITUDE, LONGTITUDE);
    int alt_diff = fo.altitude - ALTITUDE;
    char *csum_ptr;
    unsigned char cs = 0; //clear any old checksum

    Udp.beginPacket(ARGUS_HOSTNAME, XCSOAR_PORT);
    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "$PFLAU,0,1,1,1,%d,2,%d,%u,%X*", \
      bearing, alt_diff, (int) fo.distance, fo.addr );

    //calculate the checksum
    for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++) {
      cs ^= UDPpacketBuffer[n]; //calculates the checksum
    }

    csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
    snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\n", cs);

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();

    Udp.beginPacket(XCSOAR_HOSTNAME, XCSOAR_PORT);
    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "$PFLAA,2,%d,%d,%d,2,%X,,,,,1*",   \
      (int) (fo.distance * cos(dtor(bearing))), (int) (fo.distance * sin(dtor(bearing))), \
      alt_diff, fo.addr );

    cs = 0; //clear any old checksum
    for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++) {
      cs ^= UDPpacketBuffer[n]; //calculates the checksum
    }

    csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
    snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\n", cs);

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}

#define take_degrees(x) ( (int) x )
#define take_minutes(x) ( fabs(x - (float) take_degrees(x)) * 60.00)

char * dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
  char * rval = dtostrf(number, width, prec, s);
  if (number < 10.0) {
    s[0] = '0';
  }
  return rval;
}

// FLRDDE626>APRS,qAS,EGHL:/074548h5111.32N/00102.04W'086/007/A=000607 id0ADDE626 -019fpm +0.0rot 5.5dB 3e -4.3kHz
void WiFi_forward_to_cloud() {
  tmElements_t tm;
  char str_lat[8];
  char str_lon[8];

  breakTime(fo.timestamp, tm);

  dtostrf_workaround(take_minutes(fo.latitude), 5, 2, str_lat);
  dtostrf_workaround(take_minutes(fo.longtitude), 5, 2, str_lon);

  //Serial.print(fo.latitude); Serial.print(" "); Serial.println(fo.longtitude); 

  snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), \
    "FLR%X>APRS,qAS,%s:/%02d%02d%02dh%02d%s%s/%03d%s%s/A=%05u TS:%d RAW:%s", \
    fo.addr, STATION_ID, tm.Hour, tm.Minute, tm.Second, \
    abs(take_degrees(fo.latitude)), str_lat, (fo.latitude < 0 ? "S" : "N"), \
    abs(take_degrees(fo.longtitude)), str_lon, (fo.longtitude < 0 ? "W" : "E"), \
    fo.altitude, fo.timestamp, fo.raw.c_str() );

#ifdef SERIAL_VERBOSE
  Serial.println(UDPpacketBuffer);
#endif

  client.println(UDPpacketBuffer);
}


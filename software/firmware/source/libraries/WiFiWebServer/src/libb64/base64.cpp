/****************************************************************************************************************************
  base64.cpp - cpp source to a base64 encoding algorithm implementation

  For any WiFi shields, such as WiFiNINA W101, W102, W13x, or custom, such as ESP8266/ESP32-AT, Ethernet, etc

  WiFiWebServer is a library for the ESP32-based WiFi shields to run WebServer
  Forked and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Forked and modified from Arduino WiFiNINA library https://www.arduino.cc/en/Reference/WiFiNINA
  Built by Khoi Hoang https://github.com/khoih-prog/WiFiWebServer
  Licensed under MIT license

  Original author:
  @file       Esp8266WebServer.h
  @author     Ivan Grokhotkov
 *****************************************************************************************************************************/

#include "base64.h"

/* Simple test program
  #include <stdio.h>
  void main()
  {
    char* in = "amcewen";
    char out[22];

    b64_encode(in, 15, out, 22);
    out[21] = '\0';

    printf(out);
  }
*/

int base64_encode(const unsigned char* aInput, int aInputLen, unsigned char* aOutput, int aOutputLen)
{
  // Work out if we've got enough space to encode the input
  // Every 6 bits of input becomes a byte of output
  if (aOutputLen < (aInputLen * 8) / 6)
  {
    // FIXME Should we return an error here, or just the length
    return (aInputLen * 8) / 6;
  }

  // If we get here we've got enough space to do the encoding

  const char* b64_dictionary = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  if (aInputLen == 3)
  {
    aOutput[0] = b64_dictionary[aInput[0] >> 2];
    aOutput[1] = b64_dictionary[(aInput[0] & 0x3) << 4 | (aInput[1] >> 4)];
    aOutput[2] = b64_dictionary[(aInput[1] & 0x0F) << 2 | (aInput[2] >> 6)];
    aOutput[3] = b64_dictionary[aInput[2] & 0x3F];
  }
  else if (aInputLen == 2)
  {
    aOutput[0] = b64_dictionary[aInput[0] >> 2];
    aOutput[1] = b64_dictionary[(aInput[0] & 0x3) << 4 | (aInput[1] >> 4)];
    aOutput[2] = b64_dictionary[(aInput[1] & 0x0F) << 2];
    aOutput[3] = '=';
  }
  else if (aInputLen == 1)
  {
    aOutput[0] = b64_dictionary[aInput[0] >> 2];
    aOutput[1] = b64_dictionary[(aInput[0] & 0x3) << 4];
    aOutput[2] = '=';
    aOutput[3] = '=';
  }
  else
  {
    // Break the input into 3-byte chunks and process each of them
    int i;

    for (i = 0; i < aInputLen / 3; i++)
    {
      base64_encode(&aInput[i * 3], 3, &aOutput[i * 4], 4);
    }

    if (aInputLen % 3 > 0)
    {
      // It doesn't fit neatly into a 3-byte chunk, so process what's left
      base64_encode(&aInput[i * 3], aInputLen % 3, &aOutput[i * 4], aOutputLen - (i * 4));
    }
  }

  return ((aInputLen + 2) / 3) * 4;
}


/*
 Stream.cpp - adds parsing methods to Stream class
 Copyright (c) 2008 David A. Mellis.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 Created July 2011
 parsing functions based on TextFinder library by Michael Margolis
 */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)

#if defined(ARDUINO)
#include "Arduino.h"
#include "Stream.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include "raspi/Stream.h"
#endif /* RASPBERRY_PI */


#define PARSE_TIMEOUT 1000  // default number of milli-seconds to wait
#define NO_SKIP_CHAR  1  // a magic char not found in a valid ASCII numeric field

// private method to read stream with timeout
int Stream::timedRead()
{
  int c;
  _startMillis = millis();
  do {
    c = read();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);
  return -1;     // -1 indicates timeout
}

// private method to peek stream with timeout
int Stream::timedPeek()
{
  int c;
  _startMillis = millis();
  do {
    c = peek();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);
  return -1;     // -1 indicates timeout
}

// returns peek of the next digit in the stream or -1 if timeout
// discards non-numeric characters
int Stream::peekNextDigit(LookaheadMode lookahead, bool detectDecimal)
{
  int c;
  while (1) {
    c = timedPeek();

    if( c < 0 ||
        c == '-' ||
        (c >= '0' && c <= '9') ||
        (detectDecimal && c == '.')) return c;

    switch( lookahead ){
        case SKIP_NONE: return -1; // Fail code.
        case SKIP_WHITESPACE:
            switch( c ){
                case ' ':
                case '\t':
                case '\r':
                case '\n': break;
                default: return -1; // Fail code.
            }
        case SKIP_ALL:
            break;
    }
    read();  // discard non-numeric
  }
}

// Public Methods
//////////////////////////////////////////////////////////////

void Stream::setTimeout(unsigned long timeout)  // sets the maximum number of milliseconds to wait
{
  _timeout = timeout;
}

 // find returns true if the target string is found
bool  Stream::find(char *target)
{
  return findUntil(target, (char *)"");
}

// reads data from the stream until the target string of given length is found
// returns true if target string is found, false if timed out
bool Stream::find(char *target, size_t length)
{
  return findUntil(target, length, NULL, 0);
}

// as find but search ends if the terminator string is found
bool  Stream::findUntil(char *target, char *terminator)
{
  return findUntil(target, strlen(target), terminator, strlen(terminator));
}

// reads data from the stream until the target string of the given length is found
// search terminated if the terminator string is found
// returns true if target string is found, false if terminated or timed out
bool Stream::findUntil(char *target, size_t targetLen, char *terminator, size_t termLen)
{
  size_t index = 0;  // maximum target string length is 64k bytes!
  size_t termIndex = 0;
  int c;
  
  if( *target == 0)
    return true;   // return true if target is a null string
  while( (c = timedRead()) > 0){
    
    if(c != target[index])
      index = 0; // reset index if any char does not match
    
    if( c == target[index]){
      //////Serial.print("found "); Serial.write(c); Serial.print("index now"); Serial.println(index+1);
      if(++index >= targetLen){ // return true if all chars in the target match
        return true;
      }
    }
    
    if(termLen > 0 && c == terminator[termIndex]){
      if(++termIndex >= termLen)
        return false;       // return false if terminate string found before target string
    }
    else
      termIndex = 0;
  }
  return false;
}

// returns the first valid (long) integer value from the current position.
// lookahead determines how parseInt looks ahead in the stream.
// See LookaheadMode enumeration at the top of the file.
// Lookahead is terminated by the first character that is not a valid part of an integer.
// Once parsing commences, 'ignore' will be skipped in the stream.
long Stream::parseInt(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  long value = 0;
  int c;

  c = peekNextDigit(lookahead, false);
  // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout

  do{
    if(c == ignore)
      ; // ignore this character
    else if(c == '-')
      isNegative = true;
    else if(c >= '0' && c <= '9')        // is c a digit?
      value = value * 10 + c - '0';
    read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9') || c == ignore );

  if(isNegative)
    value = -value;
  return value;
}

// as parseInt but returns a floating point value
float Stream::parseFloat(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  bool isFraction = false;
  long value = 0;
  int c;
  float fraction = 1.0;

  c = peekNextDigit(lookahead, true);
    // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout

  do{
    if(c == ignore)
      ; // ignore
    else if(c == '-')
      isNegative = true;
    else if (c == '.')
      isFraction = true;
    else if(c >= '0' && c <= '9')  {      // is c a digit?
      value = value * 10 + c - '0';
      if(isFraction)
         fraction *= 0.1;
    }
    read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9')  || (c == '.' && !isFraction) || c == ignore );

  if(isNegative)
    value = -value;
  if(isFraction)
    return value * fraction;
  else
    return value;
}

// read characters from stream into buffer
// terminates if length characters have been read, or timeout (see setTimeout)
// returns the number of characters placed in the buffer
// the buffer is NOT null terminated.
//
size_t Stream::readBytes(char *buffer, size_t length)
{
  size_t count = 0;
  while (count < length) {
    int c = timedRead();
    if (c < 0) break;
    *buffer++ = (char)c;
    count++;
  }
  return count;
}


// as readBytes with terminator character
// terminates if length characters have been read, timeout, or if the terminator character  detected
// returns the number of characters placed in the buffer (0 means no valid data found)

size_t Stream::readBytesUntil(char terminator, char *buffer, size_t length)
{
  if (length < 1) return 0;
  size_t index = 0;
  while (index < length) {
    int c = timedRead();
    if (c < 0 || c == terminator) break;
    *buffer++ = (char)c;
    index++;
  }
  return index; // return number of characters, not including null terminator
}

String Stream::readString()
{
  String ret;
  int c = timedRead();
  while (c >= 0)
  {
    ret += (char)c;
    c = timedRead();
  }
  return ret;
}

String Stream::readStringUntil(char terminator)
{
  String ret;
  int c = timedRead();
  while (c >= 0 && c != terminator)
  {
    ret += (char)c;
    c = timedRead();
  }
  return ret;
}

#endif // RASPBERRY_PI

#include "arduino_compat.h"

#include <iostream>
using namespace std;

#include "File.h"
#include "IniFile.h"

enum {
  errorFileNotOpen = -1,
  errorBufferTooShort = -2,
  errorSeekError = -3,
};


int main(void)
{
  const int len = 80;
  char buffer[len] = {'\0'};
  uint32_t pos = 0;
  
  IniFile ini("test.ini");
  IniFileState state;
  ini.open();
 
  state = IniFileState();
  while (ini.getValue("network", "mac", buffer, len, state) == 0)
    ;
  cout << "Found: " << buffer << endl;
  
  return 0;

  // *****************************************************************
  const char filename[] = "test.ini";
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    cout << "Cannot open " << filename << endl;
    return 1;
  }
  while (1) {
    int done = IniFile::readLine(file, buffer, len, pos);
    //cout << "---\nREAD: " << buffer << endl;
    cout << buffer << endl;
    //cout << "pos: " << pos << endl;
    if (done) {
      //cout << "done: " << done << endl;
      break;
    }
  }
  return 0;
}

#include <iostream>

#include "IniFile.h"

using namespace std;


const char noError[] = "no error";
const char fileNotFound[] = "file not found";
const char fileNotOpen[] = "file not open";
const char bufferTooSmall[] = "buffer too small";
const char seekError[] = "seek error";
const char sectionNotFound[] = "section not found";
const char keyNotFound[] = "key not found";
const char unknownError[] = "unknown error";
const char unknownErrorValue[] = "unknown error value";

const char* getErrorMessage(int e)
{
  const char *cp = unknownError;

  switch (e) {
  case IniFile::errorNoError:
    cp = noError;
    break;
  case IniFile::errorFileNotFound:
    cp = fileNotFound;
    break;
  case IniFile::errorFileNotOpen:
    cp = fileNotOpen;
    break;
  case IniFile::errorBufferTooSmall:
    cp = bufferTooSmall;
    break;
  case IniFile::errorSeekError:
    cp = seekError;
    break;
  case IniFile::errorSectionNotFound:
    cp = sectionNotFound;
    break;
  case IniFile::errorKeyNotFound:
    cp = keyNotFound;
    break;
  default:
    cp = unknownErrorValue;
    break;
  };
  return cp;
}

void testForKey(IniFile &ini, const char *key, const char *section = NULL)
{
  cout << "    Looking for key \"" << key << '"';
  if (section)
    cout << " in section \"" << section << "\"";
  cout << endl;

  const int len = 80;
  char buffer[len];

  bool b = ini.getValue(section, key, buffer, len);
  if (b == false) {
    int e = ini.getError();
    cout << "      Error: " << getErrorMessage(e) << " (" << int(e) << ")"
	 << endl;
    if (ini.getError() == IniFile::errorBufferTooSmall)
      cout << "Buffer too small for line \"" << buffer << "...\"" << endl;
  }
  else
    cout << "      Value of " << key << " is \"" << buffer << '"' << endl;

}

void runTest(IniFile &ini)
{
  cout << "Using file " << ini.getFilename() << endl;
  cout << "  File open? " << (ini.isOpen() ? "true" : "false") << endl;

  testForKey(ini, "mac");
  testForKey(ini, "mac", "network");
  testForKey(ini, "mac", "network2");
  testForKey(ini, "mac", "fake");
  testForKey(ini, "ip");
  testForKey(ini, "gateway");
  testForKey(ini, "hosts allow", "network");
  testForKey(ini, "hosts allow", "network");
  testForKey(ini, "hosts allow", "network2");
  testForKey(ini, "hosts allow", "network2");
  testForKey(ini, "string", "misc");
  testForKey(ini, "string2", "misc");
  testForKey(ini, "pi", "misc");
  float iniPi = 0.0;
  char buffer[80];
  if (ini.getValue("misc", "pi", buffer, sizeof(buffer), iniPi)) {
    cout << "    Pi: " << iniPi << endl;
    if (iniPi <= 3.1410 || iniPi >= 3.1416)
      cout << "    Pi out of range" << endl;
  }
  else {
    int e = ini.getError();
    cout << "    Could not read \"pi\" from section \"misc\" in " << ini.getFilename()
	 << endl
	 << "      Error: " << getErrorMessage(e) << " (" << int(e) << ")"
	 << endl;
  }
  cout << "----" << endl;


}

// Test the browseSections code contributed by kaixxx. This is based on the
// IniBrowseExample.ino by kaixxx but adapted to work under a standard C++
// environment.
void browseTest(IniFile &ini)
{
  cout << "Using file " << ini.getFilename() << endl;
  cout << "  File open? " << (ini.isOpen() ? "true" : "false") << endl;

  const int bufferLen = 100;
  char buffer[bufferLen];
  IniFileState state;
  char sectName[bufferLen];

  while (ini.browseSections(sectName, bufferLen, state)) {
    cout << sectName;

    if (ini.getValue(sectName, "meal", buffer, bufferLen)) {
      cout << " eats " << buffer;
    } else
      cout << " eats nothing";

    if (ini.getValue(sectName, "drinks", buffer, bufferLen)) {
      cout << ", drinks " << buffer;
    } else
      cout << ", drinks nothing";

    if (ini.getValue(sectName, "dessert", buffer, bufferLen)) {
      cout << ", and has " << buffer << " for dessert." << endl;
    } else
      cout << ", and has no dessert." << endl;
  }

}

int main(void)
{

  // Cannot cleanly pass string constants to IniFile constructor
  // because is doesn't take const char*, but that is because
  // SD.open() isn't const char*.
  char missingIniFilename[] = "missing.ini";
  char testIniFilename[] = "test.ini";
  char browseTestIniFilename[] = "browsetest.ini";

  // Try the construtor which opens a file
  IniFile missingIni(missingIniFilename);
  IniFile testIni(testIniFilename);
  IniFile browseTestIni(browseTestIniFilename);

  cout << "*** Testing IniFile(char*) ***" << endl;
  missingIni.open();
  runTest(missingIni);
  testIni.open();
  runTest(testIni);
  browseTestIni.open();
  browseTest(browseTestIni);
  cout << "Done" << endl;

}



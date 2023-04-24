#define PREFER_SDFAT_LIBRARY
#include "IniFile.h"

#include <string.h>

const uint8_t IniFile::maxFilenameLen = INI_FILE_MAX_FILENAME_LEN;

IniFile::IniFile(const char* filename, mode_t mode,
				 bool caseSensitive)
{
	if (strlen(filename) <= maxFilenameLen)
		strcpy(_filename, filename);
	else
		_filename[0] = '\0';
	_mode = mode;
	_caseSensitive = caseSensitive;
}

IniFile::~IniFile()
{
	//if (_file)
	//  _file.close();
}


bool IniFile::validate(char* buffer, size_t len) const
{
	uint32_t pos = 0;
	error_t err;
	while ((err = readLine(_file, buffer, len, pos)) == errorNoError)
		;
	if (err == errorEndOfFile) {
		_error = errorNoError;
		return true;
	}
	else {
		_error = err;
		return false;
	}
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, IniFileState &state) const
{
	char *cp = nullptr;
	bool done = false;
	if (!_file) {
		_error = errorFileNotOpen;
		return true;
	}

	switch (state.getValueState) {
	case IniFileState::funcUnset:
		state.getValueState = (section == NULL ? IniFileState::funcFindKey
							   : IniFileState::funcFindSection);
		state.readLinePosition = 0;
		break;

	case IniFileState::funcFindSection:
		if (findSection(section, buffer, len, state)) {
			if (_error != errorNoError)
				return true;
			state.getValueState = IniFileState::funcFindKey;
		}
		break;

	case IniFileState::funcFindKey:
		if (findKey(section, key, buffer, len, &cp, state)) {
			if (_error != errorNoError)
				return true;
			// Found key line in correct section
			cp = skipWhiteSpace(cp);
			removeTrailingWhiteSpace(cp);

			// Copy from cp to buffer, but the strings overlap so strcpy is out
			while (*cp != '\0')
				*buffer++ = *cp++;
			*buffer = '\0';
			return true;
		}
		break;

	default:
		// How did this happen?
		_error = errorUnknownError;
		done = true;
		break;
	}

	return done;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len) const
{
	IniFileState state;
	while (!getValue(section, key, buffer, len, state))
		;
	return _error == errorNoError;
}


bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, char *value, size_t vlen) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error
	if (strlen(buffer) >= vlen)
		return false;
	strcpy(value, buffer);
	return true;
}


// For true accept: true, yes, 1
// For false accept: false, no, 0
bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, bool& val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	if (strcasecmp(buffer, "true") == 0 ||
		strcasecmp(buffer, "yes") == 0 ||
		strcasecmp(buffer, "1") == 0) {
		val = true;
		return true;
	}
	if (strcasecmp(buffer, "false") == 0 ||
		strcasecmp(buffer, "no") == 0 ||
		strcasecmp(buffer, "0") == 0) {
		val = false;
		return true;
	}
	return false; // does not match any known strings
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, int& val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	val = atoi(buffer);
	return true;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, double& val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	val = atof(buffer);
	return true;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, uint8_t& val) const
{
	long longval;
	bool r = getValue(section, key, buffer, len, longval);
	if (r)
		val = uint8_t(longval);
	return r;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, uint16_t& val) const
{
	long longval;
	bool r = getValue(section, key, buffer, len, longval);
	if (r)
		val = uint16_t(longval);
	return r;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, long& val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	val = atol(buffer);
	return true;
}

bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, unsigned long& val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	char *endptr;
	unsigned long tmp = strtoul(buffer, &endptr, 10);
	if (endptr == buffer)
		return false; // no conversion
	if (*endptr == '\0') {
		val = tmp;
		return true; // valid conversion
	}
	// buffer has trailing non-numeric characters, and since the buffer
	// already had whitespace removed discard the entire results
	return false;
}


bool IniFile::getValue(const char* section, const char* key,
					   char* buffer, size_t len, float & val) const
{
	if (!getValue(section, key, buffer, len))
		return false; // error

	char *endptr;
	float tmp = strtod(buffer, &endptr);
	if (endptr == buffer)
		return false; // no conversion
	if (*endptr == '\0') {
		val = tmp;
		return true; // valid conversion
	}
	// buffer has trailing non-numeric characters, and since the buffer
	// already had whitespace removed discard the entire results
	return false;
}


bool IniFile::getIPAddress(const char* section, const char* key,
						   char* buffer, size_t len, uint8_t* ip) const
{
	// Need 16 chars minimum: 4 * 3 digits, 3 dots and a null character
	if (len < 16)
		return false;

	if (!getValue(section, key, buffer, len))
		return false; // error

	int i = 0;
	char* cp = buffer;
	ip[0] = ip[1] = ip[2] = ip[3] = 0;
	while (*cp != '\0' && i < 4) {
		if (*cp == '.') {
			++i;
			++cp;
			continue;
		}
		if (isdigit(*cp)) {
			ip[i] *= 10;
			ip[i] += (*cp - '0');
		}
		else {
			ip[0] = ip[1] = ip[2] = ip[3] = 0;
			return false;
		}
		++cp;
	}
	return true;
}


#if defined(ARDUINO) && ARDUINO >= 100
bool IniFile::getIPAddress(const char* section, const char* key,
						   char* buffer, size_t len, IPAddress& ip) const
{
	// Need 16 chars minimum: 4 * 3 digits, 3 dots and a null character
	if (len < 16)
		return false;

	if (!getValue(section, key, buffer, len))
		return false; // error

	int i = 0;
	char* cp = buffer;
	ip = IPAddress(0, 0, 0, 0);
	while (*cp != '\0' && i < 4) {
		if (*cp == '.') {
			++i;
			++cp;
			continue;
		}
		if (isdigit(*cp)) {
			ip[i] *= 10;
			ip[i] += (*cp - '0');
		}
		else {
			ip = IPAddress(0, 0, 0, 0);
			return false;
		}
		++cp;
	}
	return true;
}
#endif

bool IniFile::getMACAddress(const char* section, const char* key,
							char* buffer, size_t len, uint8_t mac[6]) const
{
	// Need 18 chars: 6 * 2 hex digits, 5 : or - and a null char
	if (len < 18)
		return false;

	if (!getValue(section, key, buffer, len))
		return false; // error

	int i = 0;
	char* cp = buffer;
	memset(mac, 0, 6);

	while (*cp != '\0' && i < 6) {
		if (*cp == ':' || *cp == '-') {
			++i;
			++cp;
			continue;
		}
		if (isdigit(*cp)) {
			mac[i] *= 16; // working in hex!
			mac[i] += (*cp - '0');
		}
		else {
			if (isxdigit(*cp)) {
				mac[i] *= 16; // working in hex!
				mac[i] += (toupper(*cp) - 55); // convert A to 0xA, F to 0xF
			}
			else {
				memset(mac, 0, 6);
				return false;
			}
		}
		++cp;
	}
	return true;
}

// From the file location saved in 'state' look for the next section and read its name.
// The name will be in the buffer. Returns false if no section found. 
bool IniFile::browseSections(char* buffer, size_t len, IniFileState &state) const
{
	error_t err = errorNoError;
	
	do {
		err = IniFile::readLine(_file, buffer, len, state.readLinePosition);
		
		if (err != errorNoError) {
			// end of file or other error
			_error = err;
			return false;
		} else { 
			char *cp = skipWhiteSpace(buffer);

			if (*cp == '[') {
				// Found a section, read the name
				++cp;
				cp = skipWhiteSpace(cp);
				char *ep = strchr(cp, ']');
				if (ep != NULL) {
					*ep = '\0'; // make ] be end of string
					removeTrailingWhiteSpace(cp);
					// Copy from cp to buffer, but the strings overlap so strcpy is out
					while (*cp != '\0')
						*buffer++ = *cp++;
					*buffer = '\0';
					_error = errorNoError;
					return true;
				}
			}
		}
		// continue searching
	} while (err == errorNoError);
	
	// we should never get here...
	_error = err;
	return false;
}

//int8_t IniFile::readLine(File &file, char *buffer, size_t len, uint32_t &pos)
IniFile::error_t IniFile::readLine(File32 &file, char *buffer, size_t len, uint32_t &pos)
{
	if (!file)
		return errorFileNotOpen;

	if (len < 3)
		return errorBufferTooSmall;

	if (!file.seek(pos))
		return errorSeekError;

#if defined(ARDUINO_ARCH_ESP32) && !defined(PREFER_SDFAT_LIBRARY)
	size_t bytesRead = file.readBytes(buffer, len);
#else
	size_t bytesRead = file.read(buffer, len);
#endif
	if (!bytesRead) {
		buffer[0] = '\0';
		//return 1; // done
		return errorEndOfFile;
	}

	for (size_t i = 0; i < bytesRead && i < len-1; ++i) {
		// Test for '\n' with optional '\r' too
		// if (endOfLineTest(buffer, len, i, '\n', '\r')

		if (buffer[i] == '\n' || buffer[i] == '\r') {
			char match = buffer[i];
			char otherNewline = (match == '\n' ? '\r' : '\n');
			// end of line, discard any trailing character of the other sort
			// of newline
			buffer[i] = '\0';

			if (buffer[i+1] == otherNewline)
				++i;
			pos += (i + 1); // skip past newline(s)
			//return (i+1 == bytesRead && !file.available());
			return errorNoError;
		}
	}
	if (!file.available()) {
		// end of file without a newline
		buffer[bytesRead] = '\0';
		// return 1; //done
		return errorEndOfFile;
	}

	buffer[len-1] = '\0'; // terminate the string
	return errorBufferTooSmall;
}

bool IniFile::isCommentChar(char c)
{
	return (c == ';' || c == '#');
}

char* IniFile::skipWhiteSpace(char* str)
{
	char *cp = str;
	if (cp)
		while (isspace(*cp))
			++cp;
	return cp;
}

void IniFile::removeTrailingWhiteSpace(char* str)
{
	if (str == nullptr)
		return;
	char *cp = str + strlen(str) - 1;
	while (cp >= str && isspace(*cp))
		*cp-- = '\0';
}

bool IniFile::findSection(const char* section, char* buffer, size_t len,
						  IniFileState &state) const
{
	if (section == NULL) {
		_error = errorSectionNotFound;
		return true;
	}

	error_t err = IniFile::readLine(_file, buffer, len, state.readLinePosition);

	if (err != errorNoError && err != errorEndOfFile) {
		// Signal to caller to stop looking and any error value
		_error = err;
		return true;
	}

	char *cp = skipWhiteSpace(buffer);
	//if (isCommentChar(*cp))
	//return (done ? errorSectionNotFound : 0);
	if (isCommentChar(*cp)) {
		// return (err == errorEndOfFile ? errorSectionNotFound : errorNoError);
		if (err == errorEndOfFile) {
			_error = errorSectionNotFound;
			return true;
		}
		else
			return false; // Continue searching
	}

	if (*cp == '[') {
		// Start of section
		++cp;
		cp = skipWhiteSpace(cp);
		char *ep = strchr(cp, ']');
		if (ep != NULL) {
			*ep = '\0'; // make ] be end of string
			removeTrailingWhiteSpace(cp);
			if (_caseSensitive) {
				if (strcmp(cp, section) == 0) {
					_error = errorNoError;
					return true;
				}
			}
			else {
				if (strcasecmp(cp, section) == 0) {
					_error = errorNoError;
					return true;
				}
			}
		}
	}

	// Not a valid section line
	//return (done ? errorSectionNotFound : 0);
	if (err == errorEndOfFile) {
		_error = errorSectionNotFound;
		return true;
	}

	return false;
}

// From the current file location look for the matching key. If
// section is non-NULL don't look in the next section
bool IniFile::findKey(const char* section, const char* key,
					  char* buffer, size_t len, char** keyptr,
					  IniFileState &state) const
{
	if (key == NULL || *key == '\0') {
		_error = errorKeyNotFound;
		return true;
	}

	error_t err = IniFile::readLine(_file, buffer, len, state.readLinePosition);
	if (err != errorNoError && err != errorEndOfFile) {
		_error = err;
		return true;
	}

	char *cp = skipWhiteSpace(buffer);
	// if (isCommentChar(*cp))
	//   return (done ? errorKeyNotFound : 0);
	if (isCommentChar(*cp)) {
		if (err == errorEndOfFile) {
			_error = errorKeyNotFound;
			return true;
		}
		else
			return false; // Continue searching
	}

	if (section && *cp == '[') {
		// Start of a new section
		_error = errorKeyNotFound;
		return true;
	}

	// Find '='
	char *ep = strchr(cp, '=');
	if (ep != NULL) {
		*ep = '\0'; // make = be the end of string
		removeTrailingWhiteSpace(cp);
		if (_caseSensitive) {
			if (strcmp(cp, key) == 0) {
				*keyptr = ep + 1;
				_error = errorNoError;
				return true;
			}
		}
		else {
			if (strcasecmp(cp, key) == 0) {
				*keyptr = ep + 1;
				_error = errorNoError;
				return true;
			}
		}
	}

	// Not the valid key line
	if (err == errorEndOfFile) {
		_error = errorKeyNotFound;
		return true;
	}
	return false;
}

bool IniFile::getCaseSensitive(void) const
{
	return _caseSensitive;
}

void IniFile::setCaseSensitive(bool cs)
{
	_caseSensitive = cs;
}

IniFileState::IniFileState()
{
	readLinePosition = 0;
	getValueState = funcUnset;
}

[![arduino-lint](https://github.com/JulStrat/uCDB/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/JulStrat/uCDB/actions/workflows/arduino-lint.yml)
[![GitHub license](https://img.shields.io/github/license/JulStrat/uCDB)](https://github.com/JulStrat/uCDB/blob/master/LICENSE.md)

# uCDB

Arduino library for querying [Constant DataBase](https://en.wikipedia.org/wiki/Cdb_(software)) (key, value) store.
Simple, :cyclone: fast and portable CDB file format was developed by D. J. Bernstein.

## Features

> **Fast lookups**: A successful lookup in a large database normally takes just two disk accesses. An unsuccessful lookup takes only one.
>
> **Low overhead**: A database uses 2048 bytes, plus 24 bytes per record, plus the space for keys and data.
>
> **No random limits**: cdb can handle any database up to 4 gigabytes. There are no other restrictions; records don't even have to fit into memory. Databases are stored in a machine-independent format.

Compatible storage libraries:
- official [Arduino SD](https://github.com/arduino-libraries/SD)
- [Greiman SdFat](https://github.com/greiman/SdFat)
- [SdFat - Adafruit fork](https://github.com/adafruit/SdFat)
- [Adafruit SPIFlash](https://github.com/adafruit/Adafruit_SPIFlash)

Simple tracing for CDB format/integrity and run time file operation errors.
```C++
#define TRACE_CDB
#include "uCDB.hpp"
```

## API

```C++
cdbResult open(const char *fileName, unsigned long (*userHashFunc)(const void *key, unsigned long keyLen) = DJBHash);

cdbResult findKey(const void *key, unsigned long keyLen);

cdbResult findNextValue();

int readValue();

int readValue(void *buff, unsigned int byteNum);

unsigned long recordsNumber() const;

unsigned long valueAvailable() const;

cdbResult close();
```

### States transitions

<img src="https://github.com/JulStrat/uCDB/blob/master/docs/uCDB_state.png">

## Usage examples

`examples` folder contains `airports.ino` Arduino IDE sketch, Python converter `airports.py` script and data files.

<img src="https://github.com/JulStrat/uCDB/blob/master/examples/airports/airports.png">

`satcat` sketch

<img src="https://github.com/JulStrat/uCDB/blob/master/examples/satcat/satcat.png">

`benchmark` sketch

<img src="https://github.com/JulStrat/uCDB/blob/master/examples/benchmark/benchmark.png">

## License

`uCDB` source code released into the public domain.

## Links

- [CDB](https://cr.yp.to/cdb.html)

## Data sets

- [Airports](https://ourairports.com/data/)
- [Satellite Catalog](https://celestrak.com/satcat/search.php)

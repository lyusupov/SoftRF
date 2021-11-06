# libmodes

[![Build status](https://travis-ci.org/watson/libmodes.svg?branch=master)](https://travis-ci.org/watson/libmodes)

This is a C library for decoding Mode S messages from aviation
aircrafts. It supports both standard Mode S Acquisition Squitter
messages (56 bits) and Mode S Extended Squitter messages (112 bits) that
also carry ADS-B information.

This project is a refactoring of the popular
[dump1090](https://github.com/antirez/dump1090) project by Salvatore
Sanfilippo. It modularizes the code into separate functions and removes
all non-essentials, so that only the decoding logic is left.

## Usage

```c
#include "mode-s.h"
#include <stdio.h>

void on_msg(mode_s_t *self, struct mode_s_msg *mm) {
  printf("Got message from flight %s at altitude %d\n", mm->flight, mm->altitude);
}

int main(int argc, char **argv) {
  mode_s_t state;
  uint32_t data_len = 262620;
  unsigned char data[data_len];
  uint16_t mag[data_len / 2];

  // initialize the decoder state
  mode_s_init(&state);

  // get some raw IQ data somehow
  get_samples(&data);

  // compute the magnitude of the signal
  mode_s_compute_magnitude_vector(&data, &mag, data_len);

  // detect Mode S messages in the signal and call on_msg with each message
  mode_s_detect(&state, &mag, data_len/2, on_msg);
}
```

Check out
[`tests/test.c`](https://github.com/watson/libmodes/blob/master/tests/test.c)
for a complete example.

## Message Format

The provided callback to `mode_s_detect` will be called with a
`mode_s_msg` struct. This struct contain the following fields:

### Generic fields

- `unsigned char msg[14]` - Binary message
- `int msgbits` - Number of bits in message
- `int msgtype` - Downlink format #
- `int crcok` - True if CRC was valid
- `uint32_t crc` - Message CRC
- `int errorbit` - Bit corrected. `-1` if no bit corrected
- `int aa1` - ICAO Address byte 1
- `int aa2` - ICAO Address byte 2
- `int aa3` - ICAO Address byte 3
- `int phase_corrected` - True if phase correction was applied

### DF 11

- `int ca` - Responder capabilities

### DF 17

- `int metype` - Extended squitter message type
- `int mesub` - Extended squitter message subtype
- `int heading_is_valid`
- `int heading`
- `int aircraft_type`
- `int fflag` - `1` = Odd, `0` = Even CPR message
- `int tflag` - UTC synchronized?
- `int raw_latitude` - Non decoded latitude
- `int raw_longitude` - Non decoded longitude
- `char flight[9]` - 8 chars flight number
- `int ew_dir` - `0` = East, `1` = West
- `int ew_velocity` - E/W velocity
- `int ns_dir` - `0` = North, `1` = South
- `int ns_velocity` - N/S velocity.
- `int vert_rate_source` - Vertical rate source
- `int vert_rate_sign` - Vertical rate sign
- `int vert_rate` - Vertical rate
- `int velocity` - Computed from EW and NS velocity

### DF4, DF5, DF20, DF21

- `int fs` - Flight status for DF4, 5, 20, 21
- `int dr` - Request extraction of downlink request
- `int um` - Request extraction of downlink request
- `int identity` - 13 bits identity (Squawk)

### Fields used by multiple message types

- `int altitude`
- `int unit`

## Testing

To test the library, simply run:

```
make && make test
```

Note that the first time you run `make test`, a large (ca. 50MB) test
fixture will be downloaded to `tests/fixtures`. You can delete this
folder at any time if you wish.

## License

BSD-2-Clause

/*
 * This file is part of nmealib.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Extended descriptions of sentences are taken from
 *   http://www.gpsinformation.org/dale/nmea.htm
 */

#ifndef __NMEALIB_GPGSV_H__
#define __NMEALIB_GPGSV_H__

#include <nmealib/info.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/** The NMEA prefix */
#define NMEALIB_GPGSV_PREFIX "GPGSV"

/** The maximum number of satellites per sentence (must be a power of 2) */
#define NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE (4u)

/** The maximum number of satellites per sentence, expressed as shift */
#define NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT (2u)

/** The maximum number of satellites per sentence modulo mask */
#define NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_MOD_MASK (NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE - 1)

/** The maximum number of GPGSV sentences (depends on the maximum number of satellites tracked) */
#define NMEALIB_GPGSV_MAX_SENTENCES (NMEALIB_MAX_SATELLITES >> NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT)

/**
 * GPGSV packet information structure (Satellites in view)
 *
 * <pre>
 * $GPGSV,sentences,sentence,satellites,prn1,elevation1,azimuth1,snr1,prn2,elevation2,azimuth2,snr2,prn3,elevation3,azimuth3,snr3,prn4,elevation4,azimuth4,snr4*checksum
 * </pre>
 *
 * | Field       | Description                                      | present        |
 * | :---------: | ------------------------------------------------ | :------------: |
 * | $GPGSV      | NMEA prefix                                      | -              |
 * | sentences   | The number of sentences for full data            | -              |
 * | sentence    | The current sentence number                      | -              |
 * | satellites  | The number of satellites in view                 | SATINVIEWCOUNT |
 * | prn1        | Satellite PRN number                             | SATINVIEW      |
 * | elevation1  | Elevation, in degrees                            | SATINVIEW      |
 * | azimuth1    | Azimuth, in degrees                              | SATINVIEW      |
 * | snr1        | Signal-Noise-Ratio, in dB                        | SATINVIEW      |
 * | prn2        | Satellite PRN number                             | SATINVIEW      |
 * | elevation2  | Elevation, in degrees                            | SATINVIEW      |
 * | azimuth2    | Azimuth, in degrees                              | SATINVIEW      |
 * | snr2        | Signal-Noise-Ratio, in dB                        | SATINVIEW      |
 * | prn3        | Satellite PRN number                             | SATINVIEW      |
 * | elevation3  | Elevation, in degrees                            | SATINVIEW      |
 * | azimuth3    | Azimuth, in degrees                              | SATINVIEW      |
 * | snr3        | Signal-Noise-Ratio, in dB                        | SATINVIEW      |
 * | prn4        | Satellite PRN number                             | SATINVIEW      |
 * | elevation4  | Elevation, in degrees                            | SATINVIEW      |
 * | azimuth4    | Azimuth, in degrees                              | SATINVIEW      |
 * | snr4        | Signal-Noise-Ratio, in dB                        | SATINVIEW      |
 * | checksum    | NMEA checksum                                    | -              |
 *
 * Shows data about the satellites that the unit might be able to find based on
 * its viewing mask and almanac data. It also shows current ability to track
 * this data. Note that one GPGSV sentence only can provide data for up to 4
 * satellites and thus there may need to be multiple sentences for the full
 * information. It is reasonable for the GPGSV sentence to contain more satellites
 * than GPGGA might indicate since GPGSV may include satellites that are not used as
 * part of the solution. It is not a requirement that the GPGSV sentences all
 * appear in sequence. To avoid overloading the data bandwidth some receivers
 * may place the various sentences in totally different samples since each
 * sentence identifies which one it is.
 *
 * The field called SNR (Signal to Noise Ratio) in the NMEA standard is often
 * referred to as signal strength. SNR is an indirect but more useful value than
 * raw signal strength. It can range from 0 to 99 and has units of dB according
 * to the NMEA standard, but the various manufacturers send different ranges of
 * numbers with different starting numbers so the values themselves cannot
 * necessarily be used to evaluate different units. The range of working values
 * in a given gps will usually show a difference of about 25 to 35 between the
 * lowest and highest values, however 0 is a special case and may be shown on
 * satellites that are in view but not being tracked.
 *
 * Example:
 *
 * <pre>
 * $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
 * </pre>
 */
typedef struct _NmeaGPGSV {
  uint32_t      present;
  unsigned int  sentenceCount;
  unsigned int  sentence;
  unsigned int  inViewCount;
  NmeaSatellite inView[NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE];
} NmeaGPGSV;

/**
 * Determine the number of GPGSV sentences needed for the specified number of
 * satellites
 *
 * @param satellites The number of satellites
 * @return The number of GPGSV sentences needed (at least 1)
 */
size_t nmeaGPGSVsatellitesToSentencesCount(const size_t satellites);

/**
 * Parse a GPGSV sentence
 *
 * @param s The sentence
 * @param sz The length of the sentence
 * @param pack Where the result should be stored
 * @return True on success
 */
bool nmeaGPGSVParse(const char *s, const size_t sz, NmeaGPGSV *pack);

/**
 * Update an unsanitised NmeaInfo structure from a GPGSV packet structure
 *
 * @param pack The GPGSV packet structure
 * @param info The unsanitised NmeaInfo structure
 */
void nmeaGPGSVToInfo(const NmeaGPGSV *pack, NmeaInfo *info);

/**
 * Convert a sanitised NmeaInfo structure into a NmeaGPGSV structure
 *
 * @param info The sanitised NmeaInfo structure
 * @param pack The NmeaGPGSV structure
 * @param sentence The sentence index of the NmeaGPGSV structure (zero based)
 */
void nmeaGPGSVFromInfo(const NmeaInfo *info, NmeaGPGSV *pack, size_t sentence);

/**
 * Generate a GPGSV sentence
 *
 * @param s The buffer to generate the sentence in
 * @param sz The size of the buffer
 * @param pack The NmeaGPGSV structure
 * @return The length of the generated sentence; less than zero on failure,
 * larger than sz when the size of the buffer is too small to generate the
 * sentence in
 */
size_t nmeaGPGSVGenerate(char *s, const size_t sz, const NmeaGPGSV *pack);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_GPGSV_H__ */

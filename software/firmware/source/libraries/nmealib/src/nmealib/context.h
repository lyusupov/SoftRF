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

#ifndef __NMEALIB_CONTEXT_H__
#define __NMEALIB_CONTEXT_H__

#include <stddef.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Function type definition for tracing and error logging functions
 *
 * @param s The string to trace or log an error with
 * @param sz The length of the string
 */
typedef void (*NmeaContextPrintFunction)(const char *s, size_t sz);

/**
 * Set the trace function
 *
 * Note that only 1 trace function is accepted, it will overwrite
 * any trace function that was previously set, so use the return value
 * for function chaining.
 *
 * Setting the function to NULL disables tracing.
 *
 * The function can be set at any time.
 *
 * @param function The trace function
 * @return The overwritten trace function
 */
NmeaContextPrintFunction nmeaContextSetTraceFunction(NmeaContextPrintFunction function);

/**
 * Set the error logging function
 *
 * Note that only 1 error logging function is accepted, it will overwrite
 * any error logging function that was previously set, so use the return value
 * for function chaining.
 *
 * Setting the function to NULL disables error logging.
 *
 * The function can be set at any time.
 *
 * @param function The error logging function
 * @return The overwritten error logging function
 */
NmeaContextPrintFunction nmeaContextSetErrorFunction(NmeaContextPrintFunction function);

/**
 * Trace a buffer (a sized string)
 *
 * @param s The buffer (sized string)
 * @param sz The size of the buffer (length of the size string)
 */
void nmeaContextTraceBuffer(const char *s, size_t sz);

/**
 * Trace a formatted string
 *
 * @param s The formatted string to trace
 */
void nmeaContextTrace(const char *s, ...) __attribute__ ((format(printf, 1, 2)));

/**
 * Log a formatted string as an error
 *
 * @param s The formatted string to log as an error
 */
void nmeaContextError(const char *s, ...) __attribute__ ((format(printf, 1, 2)));

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_CONTEXT_H__ */

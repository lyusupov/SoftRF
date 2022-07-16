// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090.h: main program header
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __COMMON_H
#define __COMMON_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>

#define MODES_RTL_BUF_SIZE         (16*16384)                 // 256k
#define MODES_LEGACY_AUTO_GAIN     -10                        // old gain value for "use automatic gain"
#define MODES_DEFAULT_GAIN         999999                     // Use default SDR gain

#define MODES_NOTUSED(V)           ((void) V)

#include "sdr/util.h"
#include "sdr/convert.h"
#include "sdr/sdr.h"
#include "sdr/fifo.h"
#include "sdr/starch.h"

//======================== structure declarations =========================

typedef enum {
    SDR_NONE, SDR_IFILE, SDR_RTLSDR, SDR_BLADERF, SDR_HACKRF, SDR_LIMESDR, SDR_MIRI
} sdr_type_t;

// Program global state
struct _Modes {                             // Internal state
#if 0
    pthread_t       reader_thread;

    pthread_mutex_t reader_cpu_mutex;                     // mutex protecting reader_cpu_accumulator
    struct timespec reader_cpu_accumulator;               // accumulated CPU time used by the reader thread
    struct timespec reader_cpu_start;                     // start time for the last reader thread CPU measurement

    unsigned        trailing_samples;                     // extra trailing samples in magnitude buffers
#endif
    double          sample_rate;                          // actual sample rate in use (in hz)

#if 0
    uint16_t       *log10lut;        // Magnitude -> log10 lookup table
#endif
    atomic_int      exit;            // Exit from the main loop when true (2 = unclean exit)

    // Sample conversion
    int            dc_filter;        // should we apply a DC filter?

    // RTLSDR and some other SDRs
    char *        dev_name;
    float         gain;              // value in dB, or MODES_AUTO_GAIN, or MODES_MAX_GAIN
    int           freq;

#if 0
    // Networking
    char           aneterr[ANET_ERR_LEN];
    struct net_service *services;    // Active services
    struct client *clients;          // Our clients

    struct net_service *beast_verbatim_service;  // Beast-format output service, verbatim mode
    struct net_service *beast_cooked_service;    // Beast-format output service, "cooked" mode

    struct net_writer raw_out;                   // AVR-format output
    struct net_writer beast_verbatim_out;        // Beast-format output, verbatim mode
    struct net_writer beast_cooked_out;          // Beast-format output, "cooked" mode
    struct net_writer sbs_out;                   // SBS-format output
    struct net_writer stratux_out;               // Stratux-format output
    struct net_writer fatsv_out;                 // FATSV-format output

#ifdef _WIN32
    WSADATA        wsaData;          // Windows socket initialisation
#endif

    // Configuration
    sdr_type_t sdr_type;             // where are we getting data from?
    int   nfix_crc;                  // Number of crc bit error(s) to correct
    int   check_crc;                 // Only display messages with good CRC
    int   fix_df;                    // Try to correct damage to the DF field, as well as the main message body
    int   enable_df24;               // Enable decoding of DF24..DF31 (Comm-D ELM)
    int   raw;                       // Raw output format
    int   mode_ac;                   // Enable decoding of SSR Modes A & C
    int   mode_ac_auto;              // allow toggling of A/C by Beast commands
    int   net;                       // Enable networking
    int   net_only;                  // Enable just networking
    uint64_t net_heartbeat_interval; // TCP heartbeat interval (milliseconds)
    int   net_output_flush_size;     // Minimum Size of output data
    uint64_t net_output_flush_interval; // Maximum interval (in milliseconds) between outputwrites
    char *net_output_raw_ports;      // List of raw output TCP ports
    char *net_input_raw_ports;       // List of raw input TCP ports
    char *net_output_sbs_ports;      // List of SBS output TCP ports
    char *net_output_stratux_ports;  // List of Stratux output TCP ports
    char *net_input_beast_ports;     // List of Beast input TCP ports
    char *net_output_beast_ports;    // List of Beast output TCP ports
    char *net_bind_address;          // Bind address
    int   net_sndbuf_size;           // TCP output buffer size (64Kb * 2^n)
    int   net_verbatim;              // if true, Beast output connections default to verbatim mode
    int   forward_mlat;              // allow forwarding of mlat messages to output ports
    int   quiet;                     // Suppress stdout
    uint32_t show_only;              // Only show messages from this ICAO
    int   interactive;               // Interactive mode
    uint64_t interactive_display_ttl;// Interactive mode: TTL display
    int interactive_display_size;    // Size of TTL display
    int   interactive_show_distance; // Show aircraft distance and bearing instead of lat/lon
    interactive_distance_unit_t interactive_distance_units; // Units for interactive distance display
    char *interactive_callsign_filter; // Filter for interactive display callsigns
    uint64_t stats;                  // Interval (millis) between stats dumps,
    int   stats_range_histo;         // Collect/show a range histogram?
    int   onlyaddr;                  // Print only ICAO addresses
    int   metric;                    // Use metric units
    int   use_gnss;                  // Use GNSS altitudes with H suffix ("HAE", though it isn't always) when available
    int   mlat;                      // Use Beast ascii format for raw data output, i.e. @...; iso *...;
    char *json_dir;                  // Path to json base directory, or NULL not to write json.
    uint64_t json_interval;          // Interval between rewriting the json aircraft file, in milliseconds; also the advertised map refresh interval
    uint64_t json_stats_interval;    // Interval between rewriting the json stats file, in milliseconds
    int   json_location_accuracy;    // Accuracy of location metadata: 0=none, 1=approx, 2=exact
    double faup_rate_multiplier;     // Multiplier to adjust rate of faup1090 messages emitted
    bool faup_upload_unknown_commb;  // faup1090: should we upload Comm-B messages that weren't in a recognized format?

    int   json_aircraft_history_next;
    struct {
        char *content;
        int clen;
    } json_aircraft_history[HISTORY_SIZE];

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    int    bUserFlags;              // Flags relating to the user details
    double maxRange;                // Absolute maximum decoding range, in *metres*

    // State tracking
    struct aircraft *aircrafts;

    // Statistics
    struct stats stats_current;     // Currently accumulating stats, this is where all stats are initially collected
    struct stats stats_alltime;     // Accumulated stats since the start of the process
    struct stats stats_periodic;    // Accumulated stats since the last periodic stats display (--stats-every)
    struct stats stats_latest;      // Accumulated stats since the end of the last 1-minute period
    struct stats stats_1min[15];    // Accumulated stats for a full 1-minute window; this is a ring buffer maintaining a history of 15 minutes
    int stats_newest_1min;          // Index into stats_1min of the most recent 1-minute window
    struct stats stats_5min;        // Accumulated stats from the last 5 complete 1-minute windows
    struct stats stats_15min;       // Accumulated stats from the last 15 complete 1-minute windows

    // Adaptive gain config
    float adaptive_min_gain_db;
    float adaptive_max_gain_db;

    float adaptive_duty_cycle;

    bool adaptive_burst_control;
    float adaptive_burst_alpha;
    unsigned adaptive_burst_change_delay;
    float adaptive_burst_loud_rate;
    unsigned adaptive_burst_loud_runlength;
    float adaptive_burst_quiet_rate;
    unsigned adaptive_burst_quiet_runlength;

    bool adaptive_range_control;
    float adaptive_range_alpha;
    unsigned adaptive_range_percentile;
#endif

    float adaptive_range_target;

#if 0
    unsigned adaptive_range_change_delay;
    unsigned adaptive_range_scan_delay;
    unsigned adaptive_range_rescan_delay;
#endif
};

extern struct _Modes Modes;

#endif // __COMMON_H

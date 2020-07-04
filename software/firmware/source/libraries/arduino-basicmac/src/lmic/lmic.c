// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//! \file
#include "lmic.h"

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS_BCN BCN_PREAMBLE_LEN
#define PAMBL_SYMS     STD_PREAMBLE_LEN
#define PAMBL_FSK  5
#define PRERX_FSK  2
#define RXLEN_FSK  (PRERX_FSK+5+3) // rx preamble and sync word
#define BCN_100PPM_ms 13   // 13ms = 128sec*100ppm which is roughly +/-100ppm

#define BCN_INTV_osticks       sec2osticks(BCN_INTV_sec)
#define TXRX_GUARD_osticks     ms2osticks(TXRX_GUARD_ms)
#define JOIN_GUARD_osticks     ms2osticks(JOIN_GUARD_ms)
#define DELAY_DNW1_osticks     sec2osticks(DELAY_DNW1)
#define DELAY_DNW2_osticks     sec2osticks(DELAY_DNW2)
#define DELAY_JACC1_osticks    sec2osticks(DELAY_JACC1)
#define DELAY_JACC2_osticks    sec2osticks(DELAY_JACC2)
#define DELAY_EXTDNW2_osticks  sec2osticks(DELAY_EXTDNW2)
#define BCN_RESERVE_osticks    ms2osticks(BCN_RESERVE_ms)
#define BCN_GUARD_osticks      ms2osticks(BCN_GUARD_ms)
#define BCN_WINDOW_osticks     ms2osticks(BCN_WINDOW_ms)
#define AIRTIME_BCN_osticks    us2osticksRound(AIRTIME_BCN)

#define DNW2_SAFETY_ZONE_ETSI  ms2osticks(3000)
#define DNW2_SAFETY_ZONE_FCC   ms2osticks(750)

// Special APIs - for development or testing
#if defined(CFG_extapi)
#define isTESTMODE() (LMIC.opmode & OP_TESTMODE)
#else
#define isTESTMODE() 0
#endif

DEFINE_LMIC;

// Fwd decls.
static void engineUpdate(void);
static void startScan (void);


// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(HAS_os_calls)

#if !defined(os_rlsbf2)
u2_t os_rlsbf2 (const u1_t* buf) {
    return (u2_t)((u2_t)buf[0] | ((u2_t)buf[1]<<8));
}
#endif

#if !defined(os_rlsbf4)
u4_t os_rlsbf4 (const u1_t* buf) {
    return (u4_t)((u4_t)buf[0] | ((u4_t)buf[1]<<8) | ((u4_t)buf[2]<<16) | ((u4_t)buf[3]<<24));
}
#endif


#if !defined(os_rmsbf4)
u4_t os_rmsbf4 (const u1_t* buf) {
    return (u4_t)((u4_t)buf[3] | ((u4_t)buf[2]<<8) | ((u4_t)buf[1]<<16) | ((u4_t)buf[0]<<24));
}
#endif


#if !defined(os_wlsbf2)
void os_wlsbf2 (u1_t* buf, u2_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}
#endif

#if !defined(os_wlsbf3)
void os_wlsbf3 (u1_t* buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
}
#endif

#if !defined(os_wlsbf4)
void os_wlsbf4 (u1_t* buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}
#endif

#if !defined(os_wmsbf4)
void os_wmsbf4 (u1_t* buf, u4_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}
#endif

#if !defined(os_crc16)
// New CRC-16 CCITT(XMODEM) checksum for beacons:
u2_t os_crc16 (u1_t* data, uint len) {
    u2_t remainder = 0;
    u2_t polynomial = 0x1021;
    uint i;
    for(i = 0; i < len; i++ ) {
        remainder ^= data[i] << 8;
        u1_t bit;
        for(bit = 8; bit > 0; bit--) {
            if( (remainder & 0x8000) )
                remainder = (remainder << 1) ^ polynomial;
            else
                remainder <<= 1;
        }
    }
    return remainder;
}
#endif

#endif // !HAS_os_calls


// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG NEW REGION STUFF

// data rate tables
#ifdef REG_DRTABLE_EU
static const u1_t DR2RPS_EU[16] = {
    LWUPDR(SF12, BW125), LWUPDR(SF11, BW125), LWUPDR(SF10, BW125), LWUPDR(SF9,  BW125),
    LWUPDR(SF8,  BW125), LWUPDR(SF7,  BW125), LWUPDR(SF7,  BW250), LWUPDR(FSK,  BW125),
    ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,
    ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,
};
#endif
#ifdef REG_DRTABLE_125kHz
static const u1_t DR2RPS_125kHz[16] = {
    LWUPDR(SF12, BW125), LWUPDR(SF11, BW125), LWUPDR(SF10, BW125), LWUPDR(SF9,  BW125),
    LWUPDR(SF8,  BW125), LWUPDR(SF7,  BW125), LWUPDR(SF7,  BW250), ILLEGAL_RPS,
    ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,
    ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,
};
#endif
#ifdef REG_DRTABLE_US
static const u1_t DR2RPS_US[16] = {
    LWUPDR(SF10, BW125), LWUPDR(SF9,  BW125), LWUPDR(SF8,  BW125), LWUPDR(SF7,  BW125),
    LWUPDR(SF8,  BW500), ILLEGAL_RPS,         ILLEGAL_RPS,         ILLEGAL_RPS,
    LWDNDR(SF12, BW500), LWDNDR(SF11, BW500), LWDNDR(SF10, BW500), LWDNDR(SF9,  BW500),
    LWDNDR(SF8,  BW500), LWDNDR(SF7,  BW500), ILLEGAL_RPS,         ILLEGAL_RPS,
};
#endif
#ifdef REG_DRTABLE_AU
static const u1_t DR2RPS_AU[16] = {
    LWUPDR(SF12, BW125), LWUPDR(SF11, BW125), LWUPDR(SF10, BW125), LWUPDR(SF9,  BW125),
    LWUPDR(SF8,  BW125), LWUPDR(SF7,  BW125), LWUPDR(SF7,  BW500), ILLEGAL_RPS,
    LWDNDR(SF12, BW500), LWDNDR(SF11, BW500), LWDNDR(SF10, BW500), LWDNDR(SF9,  BW500),
    LWDNDR(SF8,  BW500), LWDNDR(SF7,  BW500), ILLEGAL_RPS,         ILLEGAL_RPS,
};
#endif
#ifdef REG_DYN
static const rfuncs_t RFUNCS_DYN;  // fwd decl
#endif
#ifdef REG_FIX
static const rfuncs_t RFUNCS_FIX;  // fwd decl
#endif
#define ILLEGAL_RX1DRoff (-128)
#define __RX1DRval(dnoff,di) ((di)==ILLEGAL_RX1DRoff ? ILLEGAL_RX1DRoff : (dnoff)-(di))
#define RX1DR_OFFSETS(dnoff,d0,d1,d2,d3,d4,d5,d6,d7) {          \
        __RX1DRval(dnoff,d0), __RX1DRval(dnoff,d1),             \
            __RX1DRval(dnoff,d2), __RX1DRval(dnoff,d3),         \
            __RX1DRval(dnoff,d4), __RX1DRval(dnoff,d5),         \
            __RX1DRval(dnoff,d6), __RX1DRval(dnoff,d7) }

static const region_t REGIONS[REGIONS_COUNT] = {
#ifdef CFG_eu868
    [REGION_EU868] = {
	.regcode        = REGCODE_EU868,
        .flags          = REG_PSA,
        .minFreq        = 863000000,
        .maxFreq        = 870000000,
        .defaultCh      = { 868100000, 868300000, 868500000 },
        .chTxCap        = 36, // PSA: 100s/1h max cumulative on time
        .ccaThreshold   = (-80 + RSSI_OFF), // -80dBm XXX
        .ccaTime        = us2osticks(160),
        .bands          = {
            { 869400000, 869650000, CAP_DECI,  29 }, // h1.6
            { 868000000, 868600000, CAP_CENTI, 16 }, // h1.4
            { 869700000, 870000000, CAP_CENTI, 16 }, // h1.7
            { 863000000, 869650000, CAP_MILLI, 16 }, // h1.3
        },
        .beaconFreq     = 869525000,
        .rx2Freq        = 869525000,
        .pingFreq       = 869525000,
        .pingDr         = 0,
        .rx2Dr          = 0,
        .beaconDr       = 3,
        .beaconOffInfo  = 8,
        .beaconLen      = 17,
        .beaconAirtime  = us2osticksRound(152576),
        .maxEirp        = 16,
        .rx1DrOff       = RX1DR_OFFSETS(0,  0, 1, 2, 3, 4, 5,
                                        ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff),
        .dr2rps         = DR2RPS_EU,
        .rfuncs         = &RFUNCS_DYN,
    },
#endif
#ifdef CFG_as923
    [REGION_AS923] = {
	.regcode        = REGCODE_AS923,
        .flags          = 0,
        .minFreq        = 920000000,
        .maxFreq        = 928000000,
        .defaultCh      = { 923200000, 923400000 },
        .chTxCap        = 10, // 10%
        .bands          = {
            { 0, 0, CAP_NONE,  16 }, // ==> no bands (XXX:bit of a hack)
        },
        .beaconFreq     = 923400000,
        .rx2Freq        = 923200000,
        .pingFreq       = 923400000,
        .pingDr         = 2,
        .rx2Dr          = 2,
        .beaconDr       = 3,
        .beaconOffInfo  = 8,
        .beaconLen      = 17,
        .beaconAirtime  = us2osticksRound(152576),
        .maxEirp        = 16,
        .rx1DrOff       = RX1DR_OFFSETS(0,  0, 1, 2, 3, 4, 5, -1, -2),
        .dr2rps         = DR2RPS_EU,
        .rfuncs         = &RFUNCS_DYN,
    },
#endif
#ifdef CFG_us915
    [REGION_US915] = {
	.regcode        = REGCODE_US915,
        .flags          = REG_FIXED,
        .minFreq        = 902000000,
        .maxFreq        = 928000000,
        .baseFreq125    = 902300000,
        .baseFreqFix    = 903000000,
        .baseFreqDn     = 923300000,
        .numChBlocks    = 8,
        .numChDnBlocks  = 1,
        .rx2Freq        = 923300000,
        .pingDr         = 8,
        .rx2Dr          = 8,
        .beaconDr       = 8,
        .beaconOffInfo  = 11,
        .beaconLen      = 23,
        .beaconAirtime  = us2osticksRound(305152),
        .maxEirp        = 30,
        .fixDr          = 4,
        .rx1DrOff       = RX1DR_OFFSETS(10,  0, 1, 2, 3,
                                        ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff,
                                        ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff),
        .dr2rps         = DR2RPS_US,
        .rfuncs         = &RFUNCS_FIX,
    },
#endif
#ifdef CFG_au915
    [REGION_AU915] = {
	.regcode        = REGCODE_AU915,
        .flags          = REG_FIXED,
        .minFreq        = 915000000,
        .maxFreq        = 928000000,
        .baseFreq125    = 915200000,
        .baseFreqFix    = 915900000,
        .baseFreqDn     = 923300000,
        .numChBlocks    = 8,
        .numChDnBlocks  = 1,
        .rx2Freq        = 923300000,
        .pingDr         = 10,
        .rx2Dr          = 8,
        .beaconDr       = 10,
        .beaconOffInfo  = 9,
        .beaconLen      = 19,
        .beaconAirtime  = us2osticksRound(76288),
        .maxEirp        = 30,
        .fixDr          = 6,
        .rx1DrOff       = RX1DR_OFFSETS(8,  0, 1, 2, 3, 4, 5,
                                        ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff),
        .dr2rps         = DR2RPS_AU,
        .rfuncs         = &RFUNCS_FIX,
    },
#endif
#ifdef CFG_cn470
    [REGION_CN470] = {
	.regcode        = REGCODE_CN470,
        .flags          = REG_FIXED,
        .minFreq        = 470000000,
        .maxFreq        = 510000000,
        .baseFreq125    = 470200000,
        .baseFreqFix    = 0,
        .baseFreqDn     = 500300000,
        .numChBlocks    = 12,
        .numChDnBlocks  = 6,
        .rx2Freq        = 505300000,
        .pingDr         = 0,
        .rx2Dr          = 0,
        .beaconDr       = 2,
        .beaconOffInfo  = 9,
        .beaconLen      = 19,
        .beaconAirtime  = us2osticksRound(305152),
        .maxEirp        = 30,
        .fixDr          = ILLEGAL_DR,
        .rx1DrOff       = RX1DR_OFFSETS(0,  0, 1, 2, 3, 4,
                                        ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff, ILLEGAL_RX1DRoff),
        .dr2rps         = DR2RPS_125kHz,
        .rfuncs         = &RFUNCS_FIX,
    },
#endif
};

#define REGION          (*LMIC.region)
#define isREGION(reg)   (&REGION == &REGIONS[REGION_##reg])

#if defined(REG_FIX) && defined(REG_DYN)
#define REG_IS_FIX()    ((REGION.flags & REG_FIXED) != 0)
#elif defined(REG_FIX)
#define REG_IS_FIX()    (1)
#else
#define REG_IS_FIX()    (0)
#endif

#define _call_rfunc(fn,...)     (REGION.rfuncs->fn(__VA_ARGS__))
#define disableChannel(...)     _call_rfunc( disableChannel, __VA_ARGS__)
#define initDefaultChannels()   _call_rfunc( initDefaultChannels)
#define prepareDn()             _call_rfunc( prepareDn)
#define applyChannelMap(...)    _call_rfunc( applyChannelMap, __VA_ARGS__)
#define checkChannelMap(...)    _call_rfunc( checkChannelMap, __VA_ARGS__)
#define syncDatarate()          _call_rfunc( syncDatarate)
#define updateTx(...)           _call_rfunc( updateTx, __VA_ARGS__)
#define nextTx(...)             _call_rfunc( nextTx, __VA_ARGS__)
#define setBcnRxParams()        _call_rfunc( setBcnRxParams)


static void initRegion (u1_t regionCode) {
    int regionIdx = LMIC_regionIdx(regionCode);
    ASSERT(regionIdx >= 0);
    LMIC.region = &REGIONS[regionIdx];
}

// encode dwell/duty cycle/quiet time availability in 8bit
// 0-64 (0-64 * 1s), then 65-255 (2-192 * 64s), i.e.:
// 0s, 1s, 2s, .. 62s, 63s, 64s, 128s, 192s, .. 12224s, 12288s (~3.4h)

static int avail_dec (u1_t avail) {
    if (avail <= 64) {
        return avail;
    } else {
        return (avail - 63) << 6;
    }
}

static int avail_enc (int sec) {
    if (sec <= 64) {
        return sec;
    } else {
	int e = 63 + (sec >> 6);
	return (e <= 255) ? e : -1; // signal error or truncate to max (255) if cast to u1_t
    }
}

static osxtime_t getAvail (u1_t avail) {
    return LMIC.baseAvail + sec2osticks(avail_dec(avail));
}

static void adjAvail (u1_t* pavail, osxtime_t base) {
    osxtime_t t = getAvail(*pavail);
    *pavail = (t > base) ? avail_enc(osticks2sec(t - base)) : 0;
}

static void setAvail (u1_t* pavail, osxtime_t t) {
    osxtime_t base = LMIC.baseAvail;
    int v;
    while (1) {
        if (base > t) {
            t = base; // make sure t is not in the past
        }
        if ((v = avail_enc(osticks2sec(t - base))) >= 0) {
            break;
        }
        // need to fix up baseAvail
        base = os_getXTime();
        adjAvail(&LMIC.globalAvail, base);
#ifdef REG_DYN
        if (!REG_IS_FIX()) {
            int i;
            for (i = 0; i < MAX_DYN_CHNLS; i++) {
                adjAvail(&LMIC.dyn.chAvail[i], base);
            }
            for(i=0; i < MAX_BANDS && REGION.bands[i].lo; i++) {
                adjAvail(&LMIC.dyn.bandAvail[i], base);
            }
        }
#endif
        LMIC.baseAvail = base;
    }
    *pavail = v;
}

static dr_t decDR (dr_t dr) {  // move to slower DR
    return dr == 0 ? 0 : dr-1;
}

static dr_t lowerDR (dr_t dr, u1_t n) {
    return dr <= n ? 0 : dr-n;
}

static drmap_t all125up () {
    drmap_t map = 0;
    u1_t dr;
    for(dr=0; dr < 16; dr++ ) {
        rps_t rps = REGION.dr2rps[dr];
        if( rps != ILLEGAL_RPS && getSf(rps) != FSK
            && getBw(rps) == BW125 && !getNocrc(rps) ) // not DN only DR
            map |= 1<<dr;
    }
    return map;
}

static dr_t fastest125 () {
    dr_t dr = 1;  // assuming DR=0 is always 125kHz
    for(; dr < 16; dr++ ) {
        rps_t rps = REGION.dr2rps[dr];
        if( rps == ILLEGAL_RPS || getBw(rps) != BW125 || getNocrc(rps) ) // DN only DR
            break;
    }
    return dr-1;
}

static inline bit_t validDR (dr_t dr) {
    return REGION.dr2rps[dr] != ILLEGAL_RPS;
}

static rps_t updr2rps (dr_t dr) {
    return REGION.dr2rps[dr];
}

static rps_t dndr2rps (dr_t dr) {
    return setNocrc(updr2rps(dr), 1);
}

static u1_t numBcnChannels() {
    return REG_IS_FIX() ? 8 : 1;
}

// END NEW REGION STUFF
// ================================================================================


// ================================================================================
// BEG LORA

static const u1_t SENSITIVITY[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    { 141-109,  141-109, 141-109 },  // FSK
    { 141-127,  141-124, 141-121 },  // SF7
    { 141-129,  141-126, 141-123 },  // SF8
    { 141-132,  141-129, 141-126 },  // SF9
    { 141-135,  141-132, 141-129 },  // SF10
    { 141-138,  141-135, 141-132 },  // SF11
    { 141-141,  141-138, 141-135 }   // SF12
};

int getSensitivity (rps_t rps) {
    return -141 + SENSITIVITY[getSf(rps)][getBw(rps)];
}

ostime_t calcAirTime (rps_t rps, u1_t plen) {
    u1_t bw = getBw(rps);  // 0,1,2 = 125,250,500kHz
    u1_t sf = getSf(rps);  // 0=FSK, 1..6 = SF7..12
    if( sf == FSK ) {
        return (plen+/*preamble*/5+/*syncword*/3+/*len*/1+/*crc*/2) * /*bits/byte*/8
            * (s4_t)OSTICKS_PER_SEC / /*kbit/s*/50000;
    }
    u1_t sfx = 4*(sf+(7-SF7));
    u1_t q = sfx - 8*enDro(rps);
    int tmp = 8*plen - sfx + 28 + (getNocrc(rps)?0:16) - (getIh(rps)?20:0);
    if( tmp > 0 ) {
        tmp = (tmp + q - 1) / q;
        tmp *= getCr(rps)+5;
        tmp += 8;
    } else {
        tmp = 8;
    }
    tmp = (tmp<<2) + /*preamble*/49 /* 4 * (8 + 4.25) */;
    // bw = 125000 = 15625 * 2^3
    //      250000 = 15625 * 2^4
    //      500000 = 15625 * 2^5
    // sf = 7..12
    //
    // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
    //
    // 3 => counter reduced divisor 125000/8 => 15625
    // 2 => counter 2 shift on tmp
    sfx = sf+(7-SF7) - (3+2) - bw;
    int div = 15625;
    if( sfx > 4 ) {
        // prevent 32bit signed int overflow in last step
        div >>= sfx-4;
        sfx = 4;
    }
    // Need 32bit arithmetic for this last step
    return (((ostime_t)tmp << sfx) * OSTICKS_PER_SEC + div/2) / div;
}

extern inline sf_t  getSf    (rps_t params);
extern inline rps_t setSf    (rps_t params, sf_t sf);
extern inline bw_t  getBw    (rps_t params);
extern inline rps_t setBw    (rps_t params, bw_t cr);
extern inline cr_t  getCr    (rps_t params);
extern inline rps_t setCr    (rps_t params, cr_t cr);
extern inline int   getNocrc (rps_t params);
extern inline rps_t setNocrc (rps_t params, int nocrc);
extern inline int   getIh    (rps_t params);
extern inline rps_t setIh    (rps_t params, int ih);
extern inline rps_t makeRps  (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int   sameSfBw (rps_t r1, rps_t r2);
extern inline int   enDro    (rps_t params);


// END LORA
// ================================================================================


#if defined(CFG_lorawan11)
static void incPollcnt (void) {
    u1_t c = LMIC.pollcnt;
    if( c < 0xFF )
        LMIC.pollcnt = c+1;
}

static void decPollcnt (void) {
    u1_t c = LMIC.pollcnt;
    if( c > 0 )
        LMIC.pollcnt = c-1;
}
#endif

// Table below defines the size of one symbol as
//   symtime = 2 ^ T(sf,bw) us
//                 SF:
//      BW:      |__7___8___9__10__11__12
//      125kHz   | 10  11  12  13  14  15
//      250kHz   |  9  10  11  12  13  14
//      500kHz   |  8   9  10  11  12  13
//
static ostime_t dr2hsym (dr_t dr, s1_t num) {
    rps_t rps = REGION.dr2rps[dr];
    u1_t sf = getSf(rps);
    u1_t bw = getBw(rps);
    ASSERT(sf >= SF7 && sf <=SF12 && bw >= BW125 && bw <= BW500);
    s4_t us = num << (9 + sf - SF7 - bw);  // 10 => 9 half symbol time
    return us2osticks(us);
}

#if !defined(DISABLE_CLASSB)
static ostime_t calcRxWindow (u1_t secs, dr_t dr) {
    ostime_t rxoff, err;

    // assume max wobble for missed bcn periods
    err = (ostime_t)LMIC.maxDriftDiff * LMIC.missedBcns;
    if( secs==0 ) {
        // aka 128 secs (next becaon)
        rxoff  = dr2hsym(dr, PAMBL_SYMS_BCN);
        rxoff += LMIC.drift;
        err   += LMIC.lastDriftDiff;
    } else {
        // scheduled RX window within secs into current beacon period
        rxoff  = dr2hsym(dr, PAMBL_SYMS);
        rxoff += (LMIC.drift * (ostime_t)secs) >> BCN_INTV_exp;
        err   += (LMIC.lastDriftDiff * (ostime_t)secs) >> BCN_INTV_exp;
    }
    // std RX window, enlarged by drift wobble
    ostime_t hsym = dr2hsym(dr,1);   // 1 symbol in ticks
    u4_t rxsyms = MINRX_SYMS + (err+hsym-1) / hsym;  // ceil syms
    // rxoff is the center of the beacon preamble adjusted by drift
    // rxsyms is the width of the rx window
    // limit for dr2hsym/rxsym: s1_t
    return rxoff - dr2hsym(dr, rxsyms>127 ? 127 : rxsyms);
}


// Setup beacon RX parameters assuming we need a tolerance of 'ms' (aka +/-ms)
static void calcBcnRxWindowFromMillis (u1_t ms, bit_t ini) {
    if( ini ) {
        LMIC.drift = 0;
        LMIC.maxDriftDiff = 0;
        LMIC.missedBcns = 0;
        LMIC.bcninfo.flags |= BCN_NODRIFT|BCN_NODDIFF;
    }
    ostime_t hsym = dr2hsym(REGION.beaconDr, 1);
    ostime_t cpre = dr2hsym(REGION.beaconDr,
            PAMBL_SYMS_BCN);                             // offset: center preamble
    int wsyms = (ms2osticksCeil(ms) + hsym - 1) / hsym;  // len RX span (2*ms) in syms (ceil)
    if( wsyms < MINRX_SYMS ) wsyms = MINRX_SYMS;         // no smaller than min
    ostime_t whspan = dr2hsym(REGION.beaconDr, wsyms);  // half RX span in osticks
    LMIC.bcnRxsyms = wsyms;
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks + cpre - whspan;
}
#endif

static void iniRxdErr () {
    // Avg(rxdErrs) == 0
    // Min(rxdErrs) == -RXDERR_INI/2
    // Min(rxdErrs) == +RXDERR_INI/2
    LMIC.rxdErrs[0] = 0;
    u1_t u;
    for(u=RXDERR_NUM&0; u<RXDERR_NUM; u++ )
        LMIC.rxdErrs[u] = (us2osticksCeil(RXDERR_INI) << (RXDERR_SHIFT-1)) * (u&1?-1:1);
    LMIC.rxdErrIdx = 0;
}

static void addRxdErr (u1_t rxdelay) {
    s4_t err = (((LMIC.rxtime0 - LMIC.txend) - sec2osticks(rxdelay)) << RXDERR_SHIFT) / rxdelay;
    if( (u4_t)((err>>20)+1) > 1 )  // overflow?
        return;
    LMIC.rxdErrs[LMIC.rxdErrIdx] = err;
    LMIC.rxdErrIdx = (LMIC.rxdErrIdx + 1) % RXDERR_NUM;
}

static s4_t evalRxdErr (u4_t* span) {
    s4_t min = 0x7FFFFFFF, min2=0x7FFFFFFF;
    s4_t max = 0x80000000, max2=0x80000000;
    s4_t sum = 0;
    u1_t u;
    for(u=0; u<RXDERR_NUM; u++ ) {
        s4_t v = LMIC.rxdErrs[u];
        /**/ if( v <= min ) { min2=min; min=v; }
        else if( v <= min2) { min2=v; }
        /**/ if( v >= max ) { max2=max; max=v; }
        else if( v >= max2) { max2=v; }
        sum += v;
    }
    //*span = max-min;
    //return (sum + (RXDERR_NUM/2)) / RXDERR_NUM;
    *span = max2-min2;
    return (sum - max - min + ((RXDERR_NUM-2)/2)) / (RXDERR_NUM-2);
}

static void adjustByRxdErr (u1_t rxdelay, u1_t dr) {
#ifdef CFG_testpin
    // XXX: for now only in HW regr tests - not yet ready for prime time
    u4_t span;
    s4_t skew = evalRxdErr(&span);
    LMIC.rxtime += (skew * rxdelay + (1<<(RXDERR_SHIFT-1))) >> RXDERR_SHIFT;
    ostime_t hsym = dr2hsym(dr,1);
    span /= dr2hsym(dr,1); // additional half symbols
    LMIC.rxsyms += (span + 1) >> 1;
    LMIC.rxtime -= span*hsym;
#endif // CFG_testpin
}


#if !defined(DISABLE_CLASSB)
// Setup scheduled RX window (ping/multicast slot)
static void rxschedInit (rxsched_t* rxsched) {
    // Relates to the standard in the following way:
    //   pingNb = 2^(7-intvExp)
    //   pingOffset = Rand % pingPeriod
    //   pingPeriod = 2^12 / pingNb = 2^12 / 2^(7-intvExp) = 2^5/2^-intvExp = 32<<intvExp
    ASSERT((LMIC.opmode & OP_PINGABLE) && rxsched->intvExp <= 7);
    u1_t intvExp = rxsched->intvExp;
    os_clearMem(LMIC.frame+8,8);
    os_wlsbf4(LMIC.frame, LMIC.bcninfo.time);
    os_wlsbf4(LMIC.frame+4, LMIC.devaddr);
    lce_encKey0(LMIC.frame);
    ostime_t off = os_rlsbf2(LMIC.frame) & ((32<<intvExp)-1); // random offset (slot units)
    rxsched->rxbase = (LMIC.bcninfo.txtime +
                       BCN_RESERVE_osticks +
                       ms2osticks(BCN_SLOT_SPAN_ms * off)); // random offset osticks
    rxsched->slot   = 0;
    rxsched->rxtime = rxsched->rxbase + calcRxWindow(/*secs BCN_RESERVE*/2+(1<<intvExp),rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
}


static bit_t rxschedNext (rxsched_t* rxsched, ostime_t cando) {
  again:
    if( rxsched->rxtime - cando >= 0 )
        return 1;
    u1_t slot;
    if( (slot=rxsched->slot) >= 128 )
        return 0;
    u1_t intv = 1<<(rxsched->intvExp & 0x7);
    if( (rxsched->slot = (slot += (intv))) >= 128 )
        return 0;
    rxsched->rxtime = rxsched->rxbase
        + ((BCN_WINDOW_osticks * (ostime_t)slot) >> BCN_INTV_exp)
        + calcRxWindow(/*secs BCN_RESERVE*/2+slot+intv,rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
    goto again;
}
#endif


static ostime_t rndDelay (u1_t secSpan) {
    u2_t r = os_getRndU2();
    ostime_t delay = r;
    if( delay > OSTICKS_PER_SEC )
        delay = r % (u2_t)OSTICKS_PER_SEC;
    if( secSpan > 0 )
        delay += ((u1_t)r % secSpan) * OSTICKS_PER_SEC;
    return delay;
}


static void txDelay (ostime_t reftime, u1_t secSpan) {
    reftime += rndDelay(secSpan);
    if( LMIC.globalDutyRate == 0  ||  (reftime - LMIC.globalDutyAvail) > 0 ) {
        LMIC.globalDutyAvail = reftime;
        LMIC.opmode |= OP_RNDTX;
    }
}


static void setDrJoin (u1_t reason, dr_t dr) {
    LMIC.datarate = dr;
}


static void setDrTxpow (u1_t reason, dr_t dr, s1_t powadj) {
    if( powadj != KEEP_TXPOWADJ )
        LMIC.txPowAdj = powadj;
    if( LMIC.datarate != dr ) {
        LMIC.datarate = dr;
        LMIC.opmode |= OP_NEXTCHNL;
    }
}


#if !defined(DISABLE_CLASSB)
void LMIC_stopPingable (void) {
    LMIC.opmode &= ~(OP_PINGABLE|OP_PINGINI);
}


u1_t LMIC_setPingable (u1_t intvExp) {
    ASSERT(intvExp <= 7);
    // Change setting
    if( LMIC.ping.intvExp == intvExp ) {
        LMIC.opmode |= OP_PINGABLE;
        return 0;  // no change
    }
    // Change of interval requires to disable class B until we got this ACKed
    LMIC.ping.intvExp = 0x80 | intvExp;
    LMIC.opmode &= ~OP_PINGABLE;
    if( (LMIC.opmode & OP_TRACK) != 0 )
        return 1;   // already tracking a beacon - communicating change to NWKS
    // Start tracking a beacon)
    LMIC_enableTracking(3);
    return 2;
}
#endif


static u1_t selectRandomChnl (u2_t map, u1_t nbits) {
    u1_t k;
 again:
    // Note: we have a small negligible bias of 2^16 % nbits (nbits <= 16 => bias < 0.025%)
    k = os_getRndU2() % nbits;
    u1_t chnl;
    for(chnl=0; chnl<16; chnl++ ) {
        if( (map & (1<<chnl)) == 0 )
            continue;
        if( k==0 ) {
            if( LMIC.refChnl == chnl && nbits > 1 )
                goto again; // don't use same channel twice
            LMIC.refChnl = chnl;
            return chnl;
        }
        k--;
    }
    ASSERT(0);
    return 0;
}

static freq_t rdFreq (u1_t* p) {
    freq_t freq = ((p[2] << 16) | (p[1] << 8) | p[0]) * 100;
    if( freq != 0 && (freq < REGION.minFreq || freq > REGION.maxFreq) )
        return -1;
    return freq;
}


// Shard between REG_DYN/REG_FIX
static dr_t prepareDnDr (dr_t updr) {
    s1_t dndr = (s1_t) updr + REGION.rx1DrOff[LMIC.dn1DrOffIdx];
    dr_t dr8 = REGION.dr2rps[8];
    s1_t mindr = (dr8 != ILLEGAL_RPS && getNocrc(dr8)) ? 8 : 0;
    if( dndr < mindr )
        dndr = mindr;
    else if( dndr >= 16 )
        dndr = 15;
    while( REGION.dr2rps[dndr] == ILLEGAL_RPS )
        dndr -= 1;
    return dndr;
}

// ================================================================================
// BEG DYNAMIC CHANNEL PLAN REGIONS

#ifdef REG_DYN

static void prepareDn_dyn () {
    LMIC.dndr = prepareDnDr(LMIC.dndr);
    // Check reconfigured DN link freq
    freq_t dnfreq = LMIC.dyn.chDnFreq[LMIC.txChnl];
    if( dnfreq ) {
        LMIC.freq = dnfreq;
    }
}

static void disableChannel_dyn (u1_t chidx) {
    LMIC.dyn.chUpFreq[chidx] = 0;
    LMIC.dyn.chDnFreq[chidx] = 0;
    LMIC.dyn.chDrMap [chidx] = 0;
    LMIC.dyn.channelMap &= ~(1 << chidx);
    if (LMIC.dyn.channelMap == 0) {
        LMIC.dyn.channelMap = (1 << MIN_DYN_CHNLS) - 1; // safety net
        // XXX - won't the default channels have 0 as their freqency
        //       if they have been disabled in the past?
        //       I suspect the safety net is not needed anymore...
    }
}

static bit_t setupChannel_dyn (u1_t chidx, freq_t freq, drmap_t drmap) {
    if (chidx >= MAX_DYN_CHNLS) {
        return 0;
    }
    if (freq == 0) {
        disableChannel_dyn(chidx);
        return 1;
    }
    // clear lowest bits for band index
    freq &= ~BAND_MASK;
    if( REGION.bands[0].lo ) {
        // Region has bands - freq must fall within one (currently EU868 only)
        u1_t i;
        for (i = 0; i < MAX_BANDS; i++) {
            const band_t* b = &REGION.bands[i];
            // XXX:TODO: take bandwidth into account when checking frequencies
            if (freq >= b->lo && freq <= b->hi) {
                freq |= i;
                goto ok;
            }
        }
        return 0;
    }
  ok:
    LMIC.dyn.chUpFreq[chidx] = freq;
    LMIC.dyn.chDnFreq[chidx] = 0;               // reset DN freq if channel is setup/modified
    LMIC.dyn.chDrMap[chidx] = drmap ?: all125up();
    setAvail(&LMIC.dyn.chAvail[chidx], 0);      // available right away
    LMIC.dyn.channelMap |= 1 << chidx;          // enabled right away
    return 1;
}

static void initDefaultChannels_dyn (void) {
    os_clearMem(&LMIC.dyn, sizeof(LMIC.dyn));
    drmap_t defaultDrMap = all125up();
    u1_t ch;
    for (ch = 0; ch < MIN_DYN_CHNLS && REGION.defaultCh[ch]; ch++) {
        setupChannel_dyn(ch, REGION.defaultCh[ch], defaultDrMap);
    }
}

static u1_t applyChannelMap_dyn (u1_t chpage, u2_t chmap, u2_t* dest) {
    if( chpage == MCMD_LADR_CHP_ALLON ) {
        chmap = 0;
        u1_t ci;
        for(ci=0; ci<MAX_DYN_CHNLS; ci++ ) {
            if( LMIC.dyn.chUpFreq[ci] != 0 ) {
                chmap |= (1<<ci);
            }
        }
    } else if( chpage != 0 ) {
        return 0;  // illegal input
    }
    *dest = chmap;
    return 1;
}

static u1_t checkChannelMap_dyn (u2_t* map) {
    if( *map == 0) {
        return 0; // no channel is enabled
    }
    u1_t ci;
    for(ci=0; ci<MAX_DYN_CHNLS; ci++ ) {
        if( (*map & (1<<ci)) != 0 && LMIC.dyn.chUpFreq[ci] == 0 ) {
            return 0; // channel is not defined
        }
    }
    return 1;
}

static void syncDatarate_dyn (void) {
    drmap_t endrs = 0;  // enabled data rates
    u1_t ci;
    for (ci = 0; ci < MAX_DYN_CHNLS; ci++) {
        if (LMIC.dyn.channelMap & (1 << ci)) {
            endrs |= LMIC.dyn.chDrMap[ci];
        }
    }
    ASSERT(endrs != 0);
    drmap_t drbit = 1 << LMIC.datarate;
    if ((drbit & endrs) == 0) {
        // Find closest DR
        dr_t dr;
        u1_t n = 1;
        while (1) {
            if ((drbit >> n) & endrs) {
                dr = LMIC.datarate - n;
                break;
            }
            if ((drbit << n) & endrs) {
                dr = LMIC.datarate + n;
                break;
            }
            n += 1;
        }
        setDrTxpow(DRCHG_SET, dr, KEEP_TXPOWADJ);
    }
}

static void updateTx_dyn (ostime_t txbeg) {
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    freq_t freq = LMIC.dyn.chUpFreq[LMIC.txChnl];
    u1_t b = freq & BAND_MASK;
    // set frequency/power
    LMIC.freq  = freq & ~BAND_MASK;
    LMIC.txpow = LMIC.txPowAdj + REGION.bands[b].txpow;
    // Update band duty cycle stats
    osxtime_t xnow = os_getXTime();
    //XXX:TBD: osxtime_t xtxbeg = os_time2XTime(txbeg, os_getXTime());
    setAvail(&LMIC.dyn.bandAvail[b], os_time2XTime(txbeg +
                airtime * REGION.bands[b].txcap,
                xnow));
    // Update channel duty cycle stats
    setAvail(&LMIC.dyn.chAvail[LMIC.txChnl], os_time2XTime(txbeg +
                airtime * REGION.chTxCap,
                xnow));
    // Update global duty cycle stats
    if (LMIC.globalDutyRate != 0) {
        LMIC.globalDutyAvail = txbeg + (airtime << LMIC.globalDutyRate);
    }
    debug_verbose_printf("Updating info for TX at %F, airtime will be %F, frequency %.2F. Setting available time for band %d to %u\r\n", txbeg, 0, airtime, 0, LMIC.freq, 6, b, LMIC.dyn.bandAvail[b]);
    if( LMIC.globalDutyRate != 0 )
        debug_verbose_printf("Updating global duty avail to %F\r\n", LMIC.globalDutyAvail, 0);
}

// select channel, perform LBT if required
// returns now if ready to send, or time
// when to try again (no free channel or DC).
// will block while doing LBT
static ostime_t nextTx_dyn (ostime_t now) {
    drmap_t drbit = 1 << LMIC.datarate;
    osxtime_t xnow = os_getXTime();
    osxtime_t txavail = OSXTIME_MAX;
    u1_t cccnt = 0; // number of candidate channels
    u2_t ccmap = 0; // candidate channel mask
    u2_t pcmap = 0; // probe channel mask
    u1_t chnl;
again:
    for (chnl = 0; chnl < MAX_DYN_CHNLS; chnl++) {
        u2_t chnlbit = 1 << chnl;
        if ((LMIC.dyn.channelMap & chnlbit) == 0 ||             // channel disabled
                (LMIC.dyn.chDrMap[chnl] & drbit) == 0) {        // or not enabled for current datarate
            continue;
        }
        // check channel DC availability
        osxtime_t avail = getAvail(LMIC.dyn.chAvail[chnl]);
        // check band DC availability
        osxtime_t bavail = getAvail(LMIC.dyn.bandAvail[LMIC.dyn.chUpFreq[chnl] & BAND_MASK]);
        debug_verbose_printf("Considering channel %d, available at %F (in band %d available at %F)\r\n", chnl, (ostime_t)avail, 0, LMIC.dyn.chUpFreq[chnl] & BAND_MASK, (ostime_t)bavail, 0);
        if( LMIC.noDC )
            goto addch;
        if (REGION.flags & REG_PSA ) {
            // PSA: channel can be used if band DC (unconditional)
            // or channel DC (PSA) are available
            if( bavail <= xnow ) {
                avail = bavail;  // just use the channel without probe
            } else {
                pcmap |= chnlbit; // if PSA DC permits then probe this channel
            }
        } else {
            // do not use channel unless both band+channel DC are available
            if (bavail > avail) {
                avail = bavail;
            }
        }
        if( avail <= xnow ) {
          addch:
            cccnt += 1;
            ccmap |= chnlbit;
        }
        if( txavail > avail ) {
            txavail = avail;
        }
    }
    if (txavail == OSXTIME_MAX) {
        debug_verbose_printf("No suitable channel found, trying different datarate\r\n");
        // No suitable channel found - maybe there's no channel which includes current datarate
        syncDatarate();
        drmap_t drbit2 = 1 << LMIC.datarate;
        ASSERT(drbit != drbit2);
        drbit = drbit2;
        goto again;
    }

    if (cccnt) {
        debug_verbose_printf("%d channels are available now\r\n", cccnt);
        while( cccnt ) {
            u1_t chnl = selectRandomChnl(ccmap, cccnt);

            if (REGION.ccaThreshold
                    && (((REGION.flags & REG_PSA) == 0) || pcmap & (1 << chnl))) {
                // perform CCA
                LMIC.rps = updr2rps(LMIC.datarate);
                LMIC.freq = LMIC.dyn.chUpFreq[chnl] & ~BAND_MASK;
                LMIC.rxtime = REGION.ccaTime;
                LMIC.rssi = REGION.ccaThreshold;
                os_radio(RADIO_CCA);
                if (LMIC.rssi >= REGION.ccaThreshold) { // channel is not available
                    debug_verbose_printf("Channel %d not available due to CCA\r\n", chnl);
                    goto unavailable;
                }
            } else {
                // channel decision is stable (i.e. won't change even if not used immediately)
                LMIC.opmode &= ~OP_NEXTCHNL; // XXX - not sure if that's necessary, since we only consider channels that can be used NOW
            }
            debug_verbose_printf("Selected channel %d (%.2F)\r\n", chnl, LMIC.dyn.chUpFreq[chnl] & ~BAND_MASK, 6);
            // good to go!
            LMIC.txChnl = chnl;
            return now;
          unavailable:
            ccmap &= ~(1 << chnl);
            cccnt -= 1;
        }
        // Avoid being bombarded...
        txavail = os_getXTime() + ms2osticks(100);
    }
    // Earliest duty cycle expiry or earliest time a channel might be tested again
    debug_verbose_printf("Channel(s) will become available at %F\r\n", (ostime_t)txavail, 0);
    return (ostime_t) txavail;
}

#if !defined(DISABLE_CLASSB)
static void setBcnRxParams_dyn (void) {
    LMIC.dataLen = 0;
    LMIC.freq = LMIC.bcnFreq ? LMIC.bcnFreq : REGION.beaconFreq;
    LMIC.rps  = setIh(setNocrc(dndr2rps(REGION.beaconDr), 1), REGION.beaconLen);
}
#endif

#endif // REG_DYN

// END DYNAMIC CHANNEL PLAN REGIONS
// ================================================================================



static void initJoinLoop (void) {
    initDefaultChannels();
    LMIC.txChnl = 0; // XXX - join should use nextTx!
    LMIC.txPowAdj = 0;
    LMIC.nbTrans = 0;
    setDrJoin(DRCHG_SET, fastest125());
    ASSERT((LMIC.opmode & OP_NEXTCHNL) == 0);
    LMIC.txend = os_getTime() + rndDelay(8); // random delay before first join req
}

static ostime_t nextJoinState (void) {
    u1_t failed = 0;

    // use next channel // XXX - join should use nextTx!
    if( ++LMIC.txChnl == MIN_DYN_CHNLS ) {
        LMIC.txChnl = 0;
    }
    // lower DR every 2nd try
    if( (++LMIC.txCnt & 1) == 0 ) {
        if( LMIC.datarate == 0 ) {
            failed = 1; // we have tried all DR - signal EV_JOIN_FAILED
        }
        else {
            setDrJoin(DRCHG_NOJACC, decDR(LMIC.datarate));
        }
    }
    // Clear NEXTCHNL because join state engine controls channel hopping
    LMIC.opmode &= ~OP_NEXTCHNL;
#if 0 // XXX:TODO
    // Move txend to randomize synchronized concurrent joins.
    // Duty cycle is based on txend.
    ostime_t time = os_getTime();
    if( time - LMIC.bands[BAND_JOIN/*XXX*/].avail < 0 )
        time = LMIC.bands[BAND_JOIN/*XXX*/].avail;
    LMIC.txend = time +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT @ SF12 being sent by GW (but we missed it)
         ? DNW2_SAFETY_ZONE
         // Otherwise: randomize join (street lamp case):
         // SF12:255, SF11:127, .., SF7:8secs
         : DNW2_SAFETY_ZONE+rndDelay(255>>LMIC.datarate));
#endif
    if (failed)
        debug_verbose_printf("Join failed\r\n");
    else
        debug_verbose_printf("Scheduling next join at %F\r\n", LMIC.txend, 0);
    // 1 - triggers EV_JOIN_FAILED event
    return failed;
}


// ================================================================================
// BEG FIXED CHANNEL PLAN REGIONS

#ifdef REG_FIX

// Pseudo-random permutation of array p[0]..p[n-1]
// Caller must fill in array elements p[i]
static void perm (unsigned char* p, int n, u4_t* prng) {
    u1_t buf[16]; // use "hoplist-" || DevEUI as key
    int i;
    for(i=0; i < n; i++ ) {
        if( i%16 == 0 ) {
            os_wlsbf4(buf,   (i/16)  + 0x6c706f68); // hopl
            os_wlsbf4(buf+4, prng[0] + 0x2d747369); // ist-
            os_getDevEui(buf + 8);
            lce_encKey0(buf);
            prng[0] += 1;
        }
        int j = buf[i%16] % n; // note: small bias if 256%n != 0
        u1_t temp = p[j];
        p[j] = p[i];
        p[i] = temp;
    }
}

// Generate a pseudo-random hoplist
// All available channels are grouped into 8 channel blocks
// - channels block are permutated
// - channels within each block are permutated
// => the hoplist for all channels can be easily trimmed down
// for smaller channel sets (e.g. 8/16 etc.)
static void generateHopList (u1_t* hoplist, int nch) {
    ASSERT(nch%8 == 0);
    int bn = nch/8;   // # of 8 ch blocks
    int i;
    for(i=0; i<bn; i++ ) {
        hoplist[i] = i*8;
    }
    u4_t prng = 0;      // prng state (counter)
    // Keep (randomized) block0 channels as 1st 8
    // Join works faster on networks that only support 8 lowest channels.
    perm(&hoplist[1], bn-1, &prng);
    int bi;
    for(bi=bn-1; bi>=0; bi-- ) {
        u1_t off = hoplist[bi];
        int i;
        for(i=0; i<8; i++ ) {
            hoplist[bi*8+i] = off+i;
        }
        perm(&hoplist[bi*8], 8, &prng);
    }
}

static int numChannels (void) {
    //  CN470        baseFreqFix==0
    //  US915/AU915  baseFreqFix!=0
    return (REGION.baseFreqFix ? 9 : 8) * REGION.numChBlocks;
}

// return: 1 - some channels were disabled, 0 - all channels were already enabled
static u1_t enableAllChannels_fix (void) {
    int nch = numChannels();
    u1_t rv = 0;
    u1_t i;
    for (i = 0; i < (nch >> 4); i++) {
        if (LMIC.fix.channelMap[i] != 0xffff) {
            LMIC.fix.channelMap[i] = 0xffff;
            rv = 1;
        }
    }
    if (nch & 0xf) {
        if (LMIC.fix.channelMap[nch >> 4] != (1 << (nch & 0xf)) - 1) {
            LMIC.fix.channelMap[nch >> 4]  = (1 << (nch & 0xf)) - 1;
            rv = 1;
        }
    }
    return rv;
}

static void disableChannel_fix (u1_t chidx) {
    if (chidx < numChannels()) {
        LMIC.fix.channelMap[chidx >> 4] &= ~(1 << (chidx & 0xF));
    }
    // safety net - all channels disabled -> turn all on -- XXX maybe we shouldn't?
    u1_t i;
    for (i = 0; i < sizeof(LMIC.fix.channelMap) / sizeof(u2_t); i++) {
        if (LMIC.fix.channelMap[i]) {
            return;
        }
    }
    enableAllChannels_fix();
}

static void initDefaultChannels_fix (void) {
    generateHopList(LMIC.fix.hoplist, REGION.numChBlocks * 8);
    os_clearMem(LMIC.fix.channelMap, sizeof(LMIC.fix.channelMap));
    enableAllChannels_fix();
}

static void prepareDn_fix () {
    LMIC.dndr = prepareDnDr(LMIC.dndr);
    LMIC.freq = REGION.baseFreqDn
        + (LMIC.txChnl % (REGION.numChDnBlocks*8))
        * (REGION.baseFreqFix ? DNCHSPACING_500kHz : DNCHSPACING_125kHz);
}

static u1_t checkChannelMap_fix (u2_t* map) {
    u2_t anyon = 0;
    u1_t i;
    for(i=0; i<CHMAP_SZ; i++ ) {
        anyon |= map[i];
    }
    return !!anyon;
}

static u1_t applyChannelMap_fix (u1_t chpage, u2_t chmap, u2_t* dest) {
    if (chpage == MCMD_LADR_CHP_125ON || chpage == MCMD_LADR_CHP_125OFF) {
        u2_t en125 = (chpage == MCMD_LADR_CHP_125ON) ? 0xFFFF : 0x0000;
        u1_t u;
        for (u = 0; u < (REGION.numChBlocks >> 1); u++) {
            dest[u] = en125;
        }
        dest[REGION.numChBlocks >> 1] = chmap;
    } else if( chpage == MCMD_LADR_CHP_BLK8 )  {
        dest[REGION.numChBlocks >> 1] = chmap & 0xFF;
        u1_t u;
        for (u = 0; u < (REGION.numChBlocks >> 1); u++) {
            dest[u] = ((chmap & 1) ? 0x00ff : 0) | ((chmap & 2) ? 0xff00 : 0);
            chmap >>= 2;
        }
    } else {
        int nch = numChannels();
        chpage >>= MCMD_LADR_CHPAGE_SHIFT;
        if (chpage >= ((nch+15) >> 4)) {
            return 0;
        }
        if ((nch & 15) && chpage == (REGION.numChBlocks >> 1)) { // partial map in last 16bit word
            chmap &= ~(~0 << (nch & 15));
        }
        dest[chpage] = chmap;
    }
    return 1;
}

static int activeChannelCount_fix (void) {
    int i, cc = 0;
    for (i = 0; i < CHMAP_SZ; i++) {
        cc += __builtin_popcount(LMIC.fix.channelMap[i]);
    }
    return cc;
}

static void updateTx_fix (ostime_t txbeg) {  //XXX:BUG: this is US915/AU915 centric - won't work for CN470
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    u1_t chnl = LMIC.txChnl;
    if( chnl < REGION.numChBlocks*8 ) {
        LMIC.freq = REGION.baseFreq125 + chnl*UPCHSPACING_125kHz;
#if CFG_cn470
        if( isREGION(CN470) )
            goto updateGDC;
#endif
        // US915 - XXX:and AU915 too?
        int chcnt = activeChannelCount_fix();

        LMIC.txpow =
            //XXX:? why not always??  #if defined(CFG_high_txpow)
            (chcnt >= 50) ? 30 :   // XXX:US915 centric, AU915/CN470 are different...
            //XXX:? #endif
            21;

        // update channel dwell time availability
#if 0 // XXX:TBD - do we still need that?
        airtime
        // add 6.25% margin to airtime        airtime += (airtime >> 4);
        LMIC.chDwellAvail[chnl] = os_time2XTime(txbeg
                + ((chcnt >= 50) ? sec2osticks(20) : chcnt * ms2osticks(400))
                - (ms2osticks(400) - airtime),
                os_getXTime());
#endif
        goto updateGDC;
    }

    LMIC.txpow = 26;
    LMIC.freq = REGION.baseFreqFix + (chnl-REGION.numChBlocks*8)*UPCHSPACING_500kHz;

  updateGDC:
    // Update global duty cycle stats XXX this is skipped for 125kHz channels right now!
    if( LMIC.globalDutyRate != 0 ) {
        LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
    }

    debug_verbose_printf("Updating info for TX at %F, airtime will be %F, frequency %.2F.\r\n", txbeg, 0 airtime, 0, LMIC.freq, 6);
    if( LMIC.globalDutyRate != 0 )
        debug_verbose_printf("Updating global duty avail to %F\r\n", LMIC.globalDutyAvail, 0);
}

static void syncDatarate_fix () {
    u1_t beg8, end8;
    if( LMIC.datarate == REGION.fixDr ) {
        beg8 = REGION.numChBlocks;
        end8 = CHMAP_SZ;
    } else {
        beg8 = 0;
        end8 = REGION.numChBlocks;
    }
    for( ;beg8 < end8; beg8++ ) {
        if( LMIC.fix.channelMap[beg8>>1] >> ((beg8 & 1)<<3) )
            return;
    }
    setDrTxpow(DRCHG_SET,
               LMIC.datarate == REGION.fixDr ? fastest125() : REGION.fixDr,
               KEEP_TXPOWADJ);
}

#if !defined(DISABLE_CLASSB)
static void setBcnRxParams_fix (void) {
    LMIC.dataLen = 0;
    LMIC.rps  = setIh(setNocrc(dndr2rps(REGION.beaconDr),1),REGION.beaconLen);
#if CFG_cn470
    if( isREGION(CN470) ) {
        LMIC.freq = (LMIC.bcnFreq ? LMIC.bcnFreq : 508300000 + LMIC.bcnChnl * DNCHSPACING_125kHz);
        return;
    }
#endif
    // US915/AU915
    LMIC.freq = (LMIC.bcnFreq ? LMIC.bcnFreq : REGION.baseFreqDn + LMIC.bcnChnl * DNCHSPACING_500kHz);
}
#endif

static ostime_t nextTx_fix (ostime_t now) {
    if( LMIC.datarate == REGION.fixDr ) {
        u1_t off = REGION.numChBlocks*8;
        for( u1_t i=0, e=REGION.numChBlocks; i<e; i++ ) {  // assuming for every 8ch 125 is one fix DR ch (iff fixDr != ILLEGAL_DR)
            u1_t chnl = ++LMIC.refChnl % e + off;
            if( (LMIC.fix.channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
                LMIC.txChnl = chnl;
                break;
            }
        }
        return now;
    }
    // 125kHz
    for( u1_t i=0, e=REGION.numChBlocks*8; i<e; i++ ) {
        u1_t chnl = LMIC.fix.hoplist[++LMIC.refChnl % e];
        if( (LMIC.fix.channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
            LMIC.txChnl = chnl;
            break;
        }
    }
    LMIC.opmode &= ~OP_NEXTCHNL;  // channel decision is stable
    osxtime_t xnow = os_time2XTime(now, os_getXTime());
#if 0 // XXX:TBD
    osxtime_t avail = LMIC.chDwellAvail[LMIC.txChnl];
    return (ostime_t) ((xnow >= avail) ? xnow : avail);
#else
    return (ostime_t)xnow;
#endif
}

#endif // REG_FIX


static void runEngineUpdate (osjob_t* osjob) {
    engineUpdate();
}

static void opmodePoll (void) {
    if ((LMIC.opmode & OP_POLL) == 0) {
        LMIC.opmode |= OP_POLL;
        LMIC.polltime = os_getTime();
        os_setApproxTimedCallback(&LMIC.osjob, LMIC.polltime + LMIC.polltimeout, runEngineUpdate);
    }
}

static void reportEvent (ev_t ev) {
    TRACE_EV(ev);
    ON_LMIC_EVENT(ev);
    engineUpdate();
}


static void runReset (osjob_t* osjob) {
    // Disable session
    LMIC_reset();
    LMIC_startJoining();
    reportEvent(EV_RESET);
}

static void stateJustJoined (void) {
    LMIC.opmode     &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI) | OP_NEXTCHNL;
    LMIC.opmode     |= OP_NEXTCHNL;
    LMIC.txCnt       = 0;
    LMIC.seqnoDn     = LMIC.seqnoUp = 0;
#if defined(CFG_lorawan11)
    LMIC.seqnoADn    = 0;
#endif
    LMIC.rejoinCnt   = 0;
    LMIC.foptsUpLen  = 0;
    LMIC.dnConf      = LMIC.devsAns = 0;
    LMIC.dnfqAns     = LMIC.dnfqAnsPend = LMIC.dnfqAcks = 0;
    LMIC.moreData    = LMIC.dn2Ans = LMIC.dn1DlyAns = LMIC.dutyCapAns = 0;
    LMIC.dn2Freq     = REGION.rx2Freq;
    LMIC.gwmargin    = 0;
    LMIC.gwcnt       = 0;
#if !defined(DISABLE_CLASSB)
    LMIC.bcnfAns     = 0;
    LMIC.bcnChnl     = 0;
    LMIC.bcnFreq     = 0;
    LMIC.ping.freq   = REGION.pingFreq;
    LMIC.ping.dr     = REGION.pingDr;
#endif
#if defined(CFG_lorawan11)
    if( (LMIC.opts &= OPT_LORAWAN11) ) {
        LMIC.opts |= OPT_OPTNEG;
    }
#endif
}


// ================================================================================
// Decoding frames


#if !defined(DISABLE_CLASSB)
// Decode beacon  - do not overwrite bcninfo unless we have a match!
static int decodeBeacon (void) {
    u1_t* d = LMIC.frame;
    if( LMIC.dataLen != REGION.beaconLen || os_rlsbf2(&d[REGION.beaconOffInfo-2]) != os_crc16(d,REGION.beaconOffInfo-2) )
        return 0;   // first (common) part fails CRC check

    LMIC.bcninfo.flags &= ~(BCN_PARTIAL|BCN_FULL);
    // Match - update bcninfo structure
    LMIC.bcninfo.snr    = LMIC.snr;
    LMIC.bcninfo.rssi   = LMIC.rssi;
    LMIC.bcninfo.txtime = LMIC.rxtime - REGION.beaconAirtime;
    LMIC.bcninfo.time   = os_rlsbf4(&d[REGION.beaconOffInfo-2-4]);
    LMIC.bcninfo.flags |= BCN_PARTIAL;

    // Check 2nd set
    if( os_rlsbf2(&d[LMIC.dataLen-2]) != os_crc16(d,LMIC.dataLen-2) )
        return 1;
    // Second set of fields is ok
    LMIC.bcninfo.lat    = (s4_t)os_rlsbf4(&d[REGION.beaconOffInfo]) >> 8;   // read as signed 24-bit
    LMIC.bcninfo.lon    = (s4_t)os_rlsbf4(&d[REGION.beaconOffInfo+3]) >> 8; // ditto
    LMIC.bcninfo.info   = d[REGION.beaconOffInfo];
    LMIC.bcninfo.flags |= BCN_FULL;
    return 2;
}
#endif


static bit_t decodeFrame (void) {
    u1_t* d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    const char *window = (LMIC.txrxFlags & TXRX_DNW1) ? "RX1" : ((LMIC.txrxFlags & TXRX_DNW2) ? "RX2" : "Other");
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
        (ftype != HDR_FTYPE_DADN  &&  ftype != HDR_FTYPE_DCDN) ) {
        // Basic sanity checks failed
      norx:
        debug_printf("Invalid downlink[window=%s]\r\n", window);
        LMIC.dataLen = 0;
        return 0;
    }
    // Validate exact frame length
    // Note: device address was already read+evaluated in order to arrive here.
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  olen  = fct & FCT_OPTLEN;
    int  ackup = (fct & FCT_ACK) != 0 ? 1 : 0;   // ACK last up frame
    int  poff  = OFF_DAT_OPTS+olen;
    int  pend  = dlen-4;  // MIC

    if( addr != LMIC.devaddr ) {
        goto norx;
    }
    if( poff > pend ) {
        goto norx;
    }

    int port = -1;
    int replayConf = 0;

    if( pend > poff )
        port = d[poff++];

    if( port == 0 && olen > 0 )
        goto norx;

    u4_t* pseqnoDn;
#if defined(CFG_lorawan11)
    pseqnoDn = (port > 0 && (LMIC.opts & OPT_LORAWAN11))
        ? &LMIC.seqnoADn : &LMIC.seqnoDn;
#else
    pseqnoDn = &LMIC.seqnoDn;
#endif
    seqno = *pseqnoDn + (s2_t)(seqno - *pseqnoDn);

    if( !lce_verifyMic(LCE_NWKSKEY, LMIC.devaddr, seqno, d, pend) ) {
        goto norx;
    }
    if( seqno < *pseqnoDn ) {
        if( (s4_t)seqno > (s4_t)*pseqnoDn ) {
            goto norx;
        }
        if( seqno != *pseqnoDn-1 || ftype != HDR_FTYPE_DCDN ) {
            goto norx;
        }
        // Replay of previous sequence number allowed only if
        // previous frame and repeated both requested confirmation
        replayConf = 1;
    }
    else {
        *pseqnoDn = seqno+1;  // next number to be expected
    }
    // DN frame requested confirmation - provide ACK once with next UP frame
    LMIC.dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);

    if( LMIC.dnConf || (fct & FCT_MORE) )
        opmodePoll();

    // We heard from network
    LMIC.rejoinCnt = 0;
    if( LMIC.adrAckReq != LINK_CHECK_OFF )
        LMIC.adrAckReq = LINK_CHECK_INIT;

    // Process OPTS
    u1_t* opts = &d[OFF_DAT_OPTS];
    int oidx = 0;

    if( !replayConf ) {
        // Handle payload only if not a replay
        // Decrypt payload - if any
        if( port >= 0 && pend-poff > 0 )
            lce_cipher(port <= 0 ? LCE_NWKSKEY : LCE_APPSKEY,
                       LMIC.devaddr, seqno, /*dn*/1, d+poff, pend-poff);
    } else {
        // treat replayed frame as empty
        pend = poff = OFF_DAT_OPTS;
    }

    if( port == 0 && pend-poff > 0 ) {
        // We process both FOpts and FRMPayload - FPort=0 inbetween is ignored
        olen = pend-OFF_DAT_OPTS;
    }

    if( (LMIC.dn2Ans || LMIC.dn1DlyAns || LMIC.dnfqAns || LMIC.dnfqAnsPend )
        && !(LMIC.txrxFlags & TXRX_PING) && !((LMIC.clmode & CLASS_C) && (LMIC.txrxFlags & TXRX_DNW2)) ) {
        // Ack of RXParamSetup is very delicate since it might lead to loosing shared state between NWKS/device.
        // The server proves it has seen the RXParamSetupAns from the device by responding in a RX window anchored
        // to the frame that carried the RXParamSetupAns! This means RX from ping slots **and** any class C using RX2
        // is not prove of reception! Thus, keep sending the RXParamSetupAns.
        // Ditto: RXTimingSetup
        LMIC.dn2Ans = LMIC.dn1DlyAns = LMIC.dnfqAns = LMIC.dnfqAnsPend = LMIC.dnfqAcks = 0;
    }
    LMIC.foptsUpLen = 0;
    while( oidx < olen ) {
        switch( opts[oidx] ) {
        case 0:  // FPort=0 if we had MAC commands in payload
            oidx += 1;
            continue;

        case MCMD_LCHK_ANS: {
            LMIC.gwmargin = opts[oidx+1];
            LMIC.gwcnt = opts[oidx+2];
            oidx += 3;
            continue;
        }
        case MCMD_LADR_REQ: {
            u1_t p1, chpage, nbtrans, cnt = 0;
            u2_t chmap;
            u2_t dmap[8] = { 0 }; // XXX
            u1_t ans = 0x80 |     // Include an answer into next frame up
                MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
            do {
                p1      = opts[oidx+1];                         // txpow + DR
                chmap   = os_rlsbf2(&opts[oidx+2]);             // list of enabled channels
                chpage  = opts[oidx+4] & MCMD_LADR_CHPAGE_MASK; // channel page
                nbtrans = opts[oidx+4] & MCMD_LADR_REPEAT_MASK; // up repeat count
                oidx += 5;
                cnt  += 1;
                if( !applyChannelMap(chpage, chmap, dmap) ) {
                    ans &= ~MCMD_LADR_ANS_CHACK;
                }
            } while (oidx < olen && opts[oidx] == MCMD_LADR_REQ);

            if( (ans & MCMD_LADR_ANS_CHACK) && !checkChannelMap(dmap) ) {
                ans &= ~MCMD_LADR_ANS_CHACK;
            }
            if( nbtrans == 0 ) {
                nbtrans = LMIC.nbTrans;  // keep unchanged
            }
            dr_t dr = (dr_t)(((p1 & MCMD_LADR_DR_MASK) >> MCMD_LADR_DR_SHIFT));
            s1_t powadj = (s1_t)((p1 & MCMD_LADR_POW_MASK) >> MCMD_LADR_POW_SHIFT);
#if 0
            debug_printf("ADR: p1=%02x,dr=%d,powadj=%d,chmap=%04x,chpage=%d,nbtrans=%d\r\n",
                    p1, dr, powadj, chmap, chpage, nbtrans);
#endif
            if( dr == 15 ) {
                dr = LMIC.datarate; // Do not change DR
            } else if( !validDR(dr) ) {
                ans &= ~MCMD_LADR_ANS_DRACK;
            }
            if( (ans & 0x7F) == (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK) ) {
                // Nothing went wrong - use settings
                if (REG_IS_FIX()) {
#ifdef REG_FIX
                    os_copyMem(LMIC.fix.channelMap, dmap, sizeof(LMIC.fix.channelMap));
#endif
                } else {
#ifdef REG_DYN
                    LMIC.dyn.channelMap = *dmap;
#endif
                }
                LMIC.nbTrans = nbtrans;
                // XXX: so far all regions define a power reduction table in the form:
                // XXX:   MaxEIRP - 2*powadj dB with 0 <= powadj <= REGMAX
                // XXX: where REGMAX is a region specific maximum.
                // XXX: We currently do not check if the max value is exceeded (a field in LMIC.region->maxPowAdjIdx?)
                setDrTxpow(DRCHG_NWKCMD, dr, powadj==15 ? KEEP_TXPOWADJ : -2*powadj);
                reportEvent(EV_DATARATE);
            }
            while( cnt-- > 0 ) {
                LMIC.foptsUp[LMIC.foptsUpLen++] = MCMD_LADR_ANS;
                LMIC.foptsUp[LMIC.foptsUpLen++] = ans & ~MCMD_LADR_ANS_RFU;
            }
            syncDatarate();              // fix DR if no more channel allowing this datarate
            LMIC.opmode |= OP_NEXTCHNL;  // DR might have changed, channel might no longer be available
            continue;
        }
        case MCMD_DEVS_REQ: {
            LMIC.margin = (LMIC.snr >> 2) & 0x3f;
            LMIC.devsAns = 1;
            oidx += 1;
            opmodePoll();
            continue;
        }
        case MCMD_DN2P_SET: {
            u1_t dr = opts[oidx+1] & 0xF;
            u1_t off = (opts[oidx+1] >> 4) & 7;
            freq_t freq = rdFreq(&opts[oidx+2]);
            oidx += 5;
            u1_t ans = 0;
            if( validDR(dr) )  //XXX:BUG validDNDR
                ans |= MCMD_DN2P_ANS_DRACK;
            if( freq > 0 )
                ans |= MCMD_DN2P_ANS_CHACK;
            if( REGION.rx1DrOff[off] != ILLEGAL_RX1DRoff )
                ans |= MCMD_DN2P_ANS_OFFACK;
            if( ans == (MCMD_DN2P_ANS_OFFACK|MCMD_DN2P_ANS_DRACK|MCMD_DN2P_ANS_CHACK) ) {
                LMIC.dn1DrOffIdx = off;
                LMIC.dn2Dr = dr;
                LMIC.dn2Freq = freq;
            }
            u1_t i = LMIC.foptsUpLen;
            LMIC.foptsUpLen = i+2;
            LMIC.foptsUp[i+0] = MCMD_DN2P_ANS;
            LMIC.foptsUp[i+1] = ans;
            LMIC.dn2Ans = MCMD_DN2P_ANS_REPLY | ans;   // answer pending
            opmodePoll();
            continue;
        }
        case MCMD_DCAP_REQ: {
            u1_t cap = opts[oidx+1];
            oidx += 2;
            // A value cap=0xFF means device is OFF unless enabled again manually.
            if( cap==0xFF )
                LMIC.opmode |= OP_SHUTDOWN;  // stop any sending
            LMIC.globalDutyRate  = cap & 0xF;
            LMIC.globalDutyAvail = os_getTime();
            LMIC.dutyCapAns = 1;
            continue;
        }
        case MCMD_SNCH_REQ: {
            u1_t ans = MCMD_SNCH_ANS_PEND;
#ifdef REG_DYN
            if (!REG_IS_FIX()) { // only available in dynamic regions
                u1_t chidx = opts[oidx+1];
                freq_t freq  = rdFreq(&opts[oidx+2]);
                if( chidx < MIN_DYN_CHNLS && REGION.defaultCh[chidx] ) {
                    // don't allow modification of default channels
                } else if( freq == 0 && chidx < MAX_DYN_CHNLS ) {
                    disableChannel_dyn(chidx);
                    ans = MCMD_SNCH_ANS_PEND|MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK;
                } else {
                    u1_t mindr = opts[oidx+5] & 0xF;
                    u1_t maxdr = opts[oidx+5] >> 4;

                    if( validDR(mindr) && validDR(maxdr) && mindr <= maxdr &&  //XXX:BUG use a validUPDR?
                        (chidx >= MIN_DYN_CHNLS || (mindr == 0 && maxdr >= fastest125())) )
                        ans |= MCMD_SNCH_ANS_DRACK;
                    if( chidx <= MAX_DYN_CHNLS && freq >= 0 )
                        ans |= MCMD_SNCH_ANS_FQACK;
                    if( ans == (MCMD_SNCH_ANS_PEND|MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK) )
                        setupChannel_dyn(chidx, freq, DR_RANGE_MAP(mindr,maxdr));
                }
            }
#endif
            oidx += 6;
            opmodePoll();
            u1_t i = LMIC.foptsUpLen;
            LMIC.foptsUpLen = i+2;
            LMIC.foptsUp[i+0] = MCMD_SNCH_ANS;
            LMIC.foptsUp[i+1] = ans & ~MCMD_SNCH_ANS_RFU;
            continue;
        }
        case MCMD_DNFQ_REQ: {
#ifdef REG_DYN
            if (!REG_IS_FIX()) { // only available in dynamic regions
                u1_t ans = MCMD_DNFQ_ANS_PEND;
                u1_t chidx = opts[oidx+1];
                freq_t freq  = rdFreq(&opts[oidx+2]);
                if( chidx <= MAX_DYN_CHNLS && LMIC.dyn.chUpFreq[chidx] != 0 )
                    ans |= MCMD_DNFQ_ANS_CHACK;
                if( freq > 0 )
                    ans |= MCMD_DNFQ_ANS_FQACK;
                if( ans == (MCMD_DNFQ_ANS_PEND|MCMD_DNFQ_ANS_CHACK|MCMD_DNFQ_ANS_FQACK) )
                    LMIC.dyn.chDnFreq[chidx] = freq;
                if( LMIC.dnfqAns + LMIC.dnfqAnsPend < 16 )
                    LMIC.dnfqAcks |= ans << (2*(LMIC.dnfqAns + LMIC.dnfqAnsPend));
                LMIC.dnfqAns += 1;
                u1_t i = LMIC.foptsUpLen;
                LMIC.foptsUpLen = i+2;
                LMIC.foptsUp[i+0] = MCMD_DNFQ_ANS;
                LMIC.foptsUp[i+1] = ans & ~MCMD_DNFQ_ANS_RFU;
            }
#endif
            oidx += 5;
            opmodePoll();
            continue;
        }
        case MCMD_RXTM_REQ: {
            LMIC.dn1Dly = opts[oidx+1] & 0xF;
            if( LMIC.dn1Dly == 0 )
                LMIC.dn1Dly = 1;
            LMIC.dn1DlyAns = 0x80;
            opmodePoll();
            oidx += 2;
            continue;
        }
#if !defined(DISABLE_CLASSB)
        case MCMD_PITV_ANS: {
            if( (LMIC.ping.intvExp & 0x80) ) {
                LMIC.ping.intvExp &= 0x7F;   // clear pending bit
                LMIC.opmode |= OP_PINGABLE;
            } // else: ignore if we weren't waiting for it
            oidx += 1;
            continue;
        }
        case MCMD_PNGC_REQ: {
            freq_t freq = rdFreq(&opts[oidx+1]);
            u1_t dr = opts[oidx+4] & 0xF;
            oidx += 5;
            u1_t ans = 0;
            if( validDR(dr) )  //XXX:BUG: validDNDR
                ans |= MCMD_PNGC_ANS_DRACK;
            if( freq >= 0)
                ans |= MCMD_PNGC_ANS_FQACK;
            if( ans == (MCMD_PNGC_ANS_FQACK|MCMD_PNGC_ANS_DRACK) ) {
                LMIC.ping.freq = freq ?: REGION.pingFreq;
                LMIC.ping.dr = dr;
            }
            u1_t i = LMIC.foptsUpLen;
            LMIC.foptsUpLen = i+2;
            LMIC.foptsUp[i+0] = MCMD_PNGC_ANS;
            LMIC.foptsUp[i+1] = ans;
            continue;
        }
        case MCMD_TIME_ANS: {
            u4_t secs = os_rlsbf4(&opts[oidx+1]);
            u1_t frac = opts[oidx+5];
            osxtime_t ref = os_time2XTime(LMIC.txend, os_getXTime());
            LMIC.gpsEpochOff = secs * OSTICKS_PER_SEC + (((frac * OSTICKS_PER_SEC) >> 8) + 128) - ref;
            LMIC.askForTime = 0;   // stop asking for time
            oidx += 6;
            // Currently, we only ask for time when we want to track a beacon
            // If there are other reasons for getting MCMD_TIME_ANS we have to discern them here
            // Set up tracking of next beacon based on the obtained time:
            // Accuracy error: 1/512 sec = ~2ms - spec promises +/-100ms
            LMIC.bcninfo.txtime = LMIC.txend - (secs & 0x7F) * OSTICKS_PER_SEC - (((frac * OSTICKS_PER_SEC) + 128) >> 8);
            LMIC.bcninfo.flags = 0;  // no previous beacon as reference (BCN_PARTIAL|BCN_FULL cleared)
            calcBcnRxWindowFromMillis(100,1);
            LMIC.bcnChnl = (1+(secs >> 7)) % numBcnChannels();
            LMIC.opmode = (LMIC.opmode & ~OP_SCAN) | OP_TRACK;
            continue;
        }
        case MCMD_BCNI_ANS: {
            // Ignore if tracking already enabled
            if( (LMIC.opmode & OP_TRACK) == 0 ) {
                LMIC.bcnChnl = opts[oidx+3];
                // Disable tracking
                LMIC.opmode |= OP_TRACK;
                // Cleared later in txComplete handling - triggers EV_BEACON_FOUND
                ASSERT(LMIC.askForTime!=0);
                // Setup RX parameters
                LMIC.bcninfo.txtime = (LMIC.rxtime
                                       + ms2osticks(os_rlsbf2(&opts[oidx+1]) * MCMD_BCNI_TUNIT)
                                       + ms2osticksCeil(MCMD_BCNI_TUNIT/2)
                                       - BCN_INTV_osticks);
                LMIC.bcninfo.flags = 0;  // txtime above cannot be used as reference (BCN_PARTIAL|BCN_FULL cleared)
                calcBcnRxWindowFromMillis(MCMD_BCNI_TUNIT,1);  // error of +/-N ms
            }
            oidx += 4;
            continue;
        }
        case MCMD_BCNF_REQ: {
            freq_t freq = rdFreq(&opts[oidx+1]);
            u1_t ans = MCMD_BCNF_ANS_PEND;
            if( freq >= 0 )
                ans |= MCMD_BCNF_ANS_FQACK;
            LMIC.bcnfAns = ans;
            if( ans == (MCMD_BCNF_ANS_PEND | MCMD_BCNF_ANS_FQACK) )
                LMIC.bcnFreq = freq;
            oidx += 4;
            continue;
        }
#endif
        case MCMD_ADRP_REQ: {
            LMIC_setLinkCheck(1 << (opts[oidx+1] >> 4), 1 << (opts[oidx+1] & 0xf));
            LMIC.foptsUp[LMIC.foptsUpLen++] = MCMD_ADRP_ANS;
            oidx += 2;
            continue;
        }
#if defined(CFG_lorawan11)
        case MCMD_RKEY_CNF: {
            // Ignore if we did not ask for options negotiation
            if( LMIC.opts & OPT_OPTNEG ) {
                LMIC.opts = OPT_LORAWAN11;
            }
            oidx += 2 + (opts[oidx+1] >> 4);
            continue;
        }
        case MCMD_DEVMD_CONF: {
            if( (LMIC.clmode & PEND_CLASS_C) && opts[oidx+1] == ((LMIC.clmode & CLASS_C) ? 0 : 2) ) {
                decPollcnt();
                LMIC.clmode &= ~PEND_CLASS_C;
                LMIC.clmode ^= CLASS_C;
            } // else: unexpected confirm or unexpected class -- ignore
            oidx += 2;
            continue;
        }
#endif
        }
        break;
    }

#if defined(CFG_lorawan11)
    if( LMIC.opts & OPT_OPTNEG ) {
        // Don't keep asking for options negotiation if the LNS does not want to answer us
        LMIC.opts &= ~OPT_OPTNEG;
    }
#endif

    if( // NWK acks but we don't have a frame pending
        (ackup && LMIC.pendTxConf == 0) ||
        // We sent up confirmed and we got a response in DNW1/DNW2
        // BUT it did not carry an ACK - this should never happen
        // Do not resend and assume frame was not ACKed.
        (!ackup && LMIC.pendTxConf != 0) ) {
    }

    if( LMIC.pendTxConf != 0 ) { // we requested an ACK
        LMIC.txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;
        LMIC.pendTxConf = 0;
    }

    if( port < 0 ) {
        LMIC.txrxFlags |= TXRX_NOPORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = 0;
    } else {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = pend-poff;
    }
    debug_printf("Received downlink[window=%s,port=%d,ack=%d]\r\n", window, port, ackup);
    return 1;
}

#if !defined(DISABLE_CLASSB)
static int decodeMultiCastFrame (void) {
    u1_t* d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
        ftype != HDR_FTYPE_DADN ) {
        // Basic sanity checks failed
      norx:
        LMIC.dataLen = 0;
        return 0;
    }
    // Validate exact frame length
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  poff  = OFF_DAT_OPTS;
    int  pend  = dlen-4;  // MIC

    // check for multicast session with this address
    session_t* s;
    for(s = LMIC.sessions; s<LMIC.sessions+MAX_MULTICAST_SESSIONS && s->grpaddr!=addr; s++);
    if( s == LMIC.sessions+MAX_MULTICAST_SESSIONS ) {
        goto norx;
    }
    // check for short frame
    if( poff > pend ) {
        goto norx;
    }
    // check for port
    int port = -1;
    if( pend > poff ) {
        port = d[poff++];
    }
    if( port == 0 ) {
        goto norx;
    }

    // check for bad flags or options (only FPending allowed, no options)
    if( fct & ~FCT_MORE ) {
        goto norx;
    }

    seqno = s->seqnoADn + (u2_t)(seqno - s->seqnoADn);

    // verify MIC
    if( !lce_verifyMic(LCE_MCGRP_0 + (s-LMIC.sessions), s->grpaddr, seqno, d, pend) ) {
        goto norx;
    }
    // check down frame counter
    if( seqno < s->seqnoADn ) {
        goto norx;
    }
    s->seqnoADn = seqno+1;  // next number to be expected

    // We heard from network
    LMIC.rejoinCnt = 0;
    if( LMIC.adrAckReq != LINK_CHECK_OFF )
        LMIC.adrAckReq = LINK_CHECK_INIT;

    // Decrypt payload - if any
    if( pend-poff > 0 ) {
        lce_cipher(LCE_MCGRP_0 + (s-LMIC.sessions), s->grpaddr, seqno, /*dn*/1, d+poff, pend-poff);
    }

    if( port < 0 ) {
        LMIC.txrxFlags |= TXRX_NOPORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = 0;
    } else {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = pend-poff;
    }
    return 1;
}
#endif


// ================================================================================
// TX/RX transaction support


static void setupRx2 (void) {
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    os_radio(RADIO_RX);
}


static void schedRx2 (u1_t delay, osjobcb_t func) {
#if defined(CFG_eu868) && !defined(CFG_kr920)
    if( getSf(dndr2rps(LMIC.dn2Dr)) == FSK ) {
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) - PRERX_FSK*us2osticksRound(160); // (8bit/50kbps=160us)
        LMIC.rxsyms = RXLEN_FSK;
    }
    else
#endif
    {
        // Add 1.5 symbols we need 5 out of 8. Try to sync 1.5 symbols into the preamble.
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) + dr2hsym(LMIC.dn2Dr, PAMBL_SYMS-MINRX_SYMS);
        adjustByRxdErr(delay, LMIC.dn2Dr);
    }
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}

static void setupRx1 (osjobcb_t func) {
    LMIC.txrxFlags = TXRX_DNW1;
    // Turn LMIC.rps from TX over to RX
    LMIC.rps = setNocrc(LMIC.rps,1);
    LMIC.dataLen = 0;
    LMIC.osjob.func = func;
    os_radio(RADIO_RX);
}


// Called by HAL once TX complete and delivers exact end of TX time stamp in LMIC.rxtime
static void txDone (u1_t delay, osjobcb_t func) {
#if !defined(DISABLE_CLASSB)
    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE|OP_PINGINI)) == (OP_TRACK|OP_PINGABLE) ) {
        rxschedInit(&LMIC.ping);    // note: reuses LMIC.frame buffer!
        LMIC.opmode |= OP_PINGINI;
    }
#endif
    prepareDn();
    LMIC.rps  = dndr2rps(LMIC.dndr);

    if( getSf(LMIC.rps) == FSK ) {
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) - PRERX_FSK*us2osticksRound(160); // (8bit/50kbps=160us)
        LMIC.rxsyms = RXLEN_FSK;
    } else {
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) + dr2hsym(LMIC.dndr, PAMBL_SYMS-MINRX_SYMS);
        LMIC.rxsyms = MINRX_SYMS;
        adjustByRxdErr(delay, LMIC.dndr);
    }
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}


// ======================================== Join frames


static void onJoinFailed (osjob_t* osjob) {
    // Notify app - must call LMIC_reset() to stop joining
    // otherwise join procedure continues.
    reportEvent(EV_JOIN_FAILED);
}


static bit_t processJoinAccept (void) {
    ASSERT(LMIC.txrxFlags != TXRX_DNW1 || LMIC.dataLen != 0);
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      nojoinframe:
        if( (LMIC.opmode & OP_JOINING) == 0 ) {
            ASSERT((LMIC.opmode & OP_REJOIN) != 0);
            // REJOIN attempt for roaming
            LMIC.opmode &= ~(OP_REJOIN|OP_TXRXPEND);
            if( LMIC.rejoinCnt < 10 )
                LMIC.rejoinCnt++;
            reportEvent(EV_REJOIN_FAILED);
            return 1;
        }
        LMIC.opmode &= ~OP_TXRXPEND;
        ostime_t delay = nextJoinState();
        // Build next JOIN REQUEST with next engineUpdate call
        // Optionally, report join failed.
        // Both after a random/chosen amount of ticks.
        os_setApproxTimedCallback(&LMIC.osjob, os_getTime()+delay,
                            (delay&1) != 0
                            ? FUNC_ADDR(onJoinFailed)      // one JOIN iteration done and failed
                            : FUNC_ADDR(runEngineUpdate)); // next step to be delayed
        return 1;
    }
    u1_t hdr  = LMIC.frame[0];
    u1_t dlen = LMIC.dataLen;
    if( (dlen != LEN_JA && dlen != LEN_JAEXT)
        || (hdr & (HDR_FTYPE|HDR_MAJOR)) != (HDR_FTYPE_JACC|HDR_MAJOR_V1) ) {
      badframe:
        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 )
            return 0;
        goto nojoinframe;
    }
    if( !lce_processJoinAccept(LMIC.frame, dlen, LMIC.devNonce) ) {
        goto badframe;
    }

    u4_t addr = os_rlsbf4(LMIC.frame+OFF_JA_DEVADDR);
    LMIC.devaddr  = addr;
    LMIC.netid    = os_rlsbf4(&LMIC.frame[OFF_JA_NETID]) & 0xFFFFFF;
    LMIC.dn1Dly   = os_minmax(1, LMIC.frame[OFF_JA_RXDLY] & 0xF, 15);
    LMIC.dn2Dr    = LMIC.frame[OFF_JA_DLSET] & JA_DLS_RX2DR;
    LMIC.dn1DrOffIdx = (LMIC.frame[OFF_JA_DLSET] & JA_DLS_RX1DROFF) >> 4;
    if( REGION.rx1DrOff[LMIC.dn1DrOffIdx] == ILLEGAL_RX1DRoff )
        goto badframe;
#if defined(CFG_lorawan11)
    LMIC.opts     = (LMIC.frame[OFF_JA_DLSET] & JA_DLS_OPTNEG) ? OPT_LORAWAN11 : 0;
#endif

    if( dlen > LEN_JA ) {
        dlen = OFF_CFLIST;
#ifdef REG_FIX
        if (REG_IS_FIX()) {
            if (LMIC.frame[OFF_CFLIST + 15] != 1) { // must be CFList type 1
                goto badframe;
            }
            LMIC.frame[OFF_CFLIST + 15] = 0; // so we can read the last byte with os_rlsbf2()
            for (u1_t i=0; i < 8 && i < CHMAP_SZ; i++, dlen += 2) {
                LMIC.fix.channelMap[i] = os_rlsbf2(&LMIC.frame[dlen]);
            }
        } else
#endif // REG_FIX
        {
#ifdef REG_DYN
            if (LMIC.frame[OFF_CFLIST + 15] != 0) { // must be CFList type 0
                goto badframe;
            }
            u1_t chidx;
            for(chidx=3; chidx<8; chidx++, dlen+=3 ) {
                s4_t freq = rdFreq(&LMIC.frame[dlen]);
                if( freq > 0 ) {
                    setupChannel_dyn(chidx, freq, 0);
                    debug_printf("Setup channel[idx=%d,freq=%.1F]\r\n", chidx, freq, 6);
                }
            }
#endif // REG_DYN
        }
    }

    ASSERT((LMIC.opmode & (OP_JOINING|OP_REJOIN))!=0);
    if( (LMIC.opmode & OP_REJOIN) != 0 ) {
        // Lower DR every try below current UP DR
        LMIC.datarate = lowerDR(LMIC.datarate, LMIC.rejoinCnt);
    }
    addRxdErr(DELAY_JACC1 + (LMIC.txrxFlags & TXRX_DNW2 ? DELAY_EXTDNW2 : 0));
    stateJustJoined();
    reportEvent(EV_JOINED);
    return 1;
}


static void processRx2Jacc (osjob_t* osjob) {
    if( LMIC.dataLen == 0 )
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processJoinAccept();
}


static void setupRx2Jacc (osjob_t* osjob) {
    LMIC.osjob.func = FUNC_ADDR(processRx2Jacc);
    setupRx2();
}


static void processRx1Jacc (osjob_t* osjob) {
    if( LMIC.dataLen == 0 || !processJoinAccept() )
        schedRx2(DELAY_JACC2, FUNC_ADDR(setupRx2Jacc));
}


static void setupRx1Jacc (osjob_t* osjob) {
    setupRx1(FUNC_ADDR(processRx1Jacc));
}


static void jreqDone (osjob_t* osjob) {
    txDone(DELAY_JACC1, FUNC_ADDR(setupRx1Jacc));
    reportEvent(EV_TXDONE);
}

// ======================================== Data frames

// Fwd decl.
static bit_t processDnData(void);

static void processRx2DnDataDelay (osjob_t* osjob) {
    processDnData();
}

static void processRx2DnData (osjob_t* osjob) {
    if( LMIC.dataLen == 0 ) {
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
        // Delay callback processing to avoid up TX while gateway is txing our missed frame!
        // Since DNW2 uses SF12 by default we wait 3 secs.
        os_setTimedCallback(&LMIC.osjob,
                            (os_getTime() + /* XXX DNW2_SAFETY_ZONE */ + rndDelay(2)),
                            FUNC_ADDR(processRx2DnDataDelay));
        return;
    }
    processDnData();
}


static void setupRx2DnData (osjob_t* osjob) {
    LMIC.osjob.func = FUNC_ADDR(processRx2DnData);
    setupRx2();
}


static void processRx1DnData (osjob_t* osjob) {
    if( LMIC.dataLen == 0 || !processDnData() ) {
        schedRx2(LMIC.dn1Dly+DELAY_EXTDNW2, FUNC_ADDR(setupRx2DnData));
    }
}

static void processRx2ClassC (osjob_t* osjob) {
    if( LMIC.dataLen != 0 ) {
        LMIC.txrxFlags = TXRX_DNW2;
        if( decodeFrame() ) {
            reportEvent(EV_RXCOMPLETE);
            return;
        }
    }
    engineUpdate();
}

static void setupRx2ClassC () {
    LMIC.osjob.func = FUNC_ADDR(processRx2ClassC);
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    os_radio(RADIO_RXON);
}

static void processRx1ClassC (osjob_t* osjob) {
    if( !processDnData() ) {
        setupRx2ClassC();
    }
}

static void setupRx1DnData (osjob_t* osjob) {
    setupRx1(FUNC_ADDR(processRx1DnData));
}

static void setupRx1ClassC (osjob_t* osjob) {
    setupRx1(FUNC_ADDR(processRx1ClassC));
}


static void updataDone (osjob_t* osjob) {
    reportEvent(EV_TXDONE);
    // check if rx window is to be scheduled
    // (dataLen is reset by radio if tx didn't complete regularly and txend is unknown)
    if( LMIC.pendTxNoRx || LMIC.dataLen == 0 ) {
        // transaction done
        LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND);
        LMIC.txrxFlags = TXRX_NOPORT;
        LMIC.dataBeg = 0;
        LMIC.dataLen = 0;
        LMIC.pendTxNoRx = 0;
        reportEvent(EV_TXCOMPLETE);
    } else {
        // schedule down window1 reception
        txDone(LMIC.dn1Dly,
               (LMIC.clmode & CLASS_C) == 0
               ? FUNC_ADDR(setupRx1DnData)
               : FUNC_ADDR(setupRx1ClassC));
    }
}

// ========================================


static void buildDataFrame (void) {
    bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    u1_t dlen = txdata ? LMIC.pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
#if defined(CFG_lorawan11)
    if( LMIC.opts & OPT_OPTNEG ) {
        LMIC.frame[end+0] = MCMD_RKEY_IND;
        LMIC.frame[end+1] = MCMD_RKEY_VERSION_1_1;
        end += 2;
    }
    if( (LMIC.clmode & PEND_CLASS_C) ) {
        LMIC.frame[end+0] = MCMD_DEVMD_IND;
        LMIC.frame[end+1] = (LMIC.clmode & CLASS_C) ? 0 : 2;
        end += 2;
    }
#endif
    if( LMIC.foptsUpLen ) {
        u1_t n = LMIC.foptsUpLen;
        if( end <= OFF_DAT_OPTS+15-n ) {
            os_copyMem(&LMIC.frame[end], LMIC.foptsUp, n);
            end += n;
        }
    }
#if !defined(DISABLE_CLASSB)
    if( LMIC.ping.intvExp & 0x80 ) {
        // Announce ping interval - LNS hasn't acked it yet
        LMIC.frame[end] = MCMD_PITV_REQ;
        LMIC.frame[end+1] = LMIC.ping.intvExp & 0x7;
        end += 2;
    }
#endif
    if( LMIC.dutyCapAns ) {
        if( end <= OFF_DAT_OPTS + 15 - 1 )
            LMIC.frame[end] = MCMD_DCAP_ANS;
        LMIC.dutyCapAns = 0;
        end += 1;
    }
    if( LMIC.dn2Ans ) {
        // Note: this is cleared with reception of a frame in a class A RX1/RX2 window
        if( (LMIC.dn2Ans & MCMD_DN2P_ANS_PEND) && end <= OFF_DAT_OPTS + 15 - 2 ) {
            LMIC.frame[end+0] = MCMD_DN2P_ANS;
            LMIC.frame[end+1] = LMIC.dn2Ans & ~MCMD_DN2P_ANS_RFU;
            end += 2;
        } else {
            LMIC.dn2Ans ^= MCMD_DN2P_ANS_REPLY|MCMD_DN2P_ANS_PEND;
        }
    }
    if( LMIC.dn1DlyAns && end <= OFF_DAT_OPTS + 15 - 1 ) {
        // Note: this is cleared with reception of a frame in a class A RX1/RX2 window
        LMIC.frame[end+0] = MCMD_RXTM_ANS;
        end += 1;
    }
    if( LMIC.dnfqAns || LMIC.dnfqAnsPend ) {
        // Note: this is cleared with reception of a frame in a class A RX1/RX2 window
        u1_t i;
        for(i=0; i < LMIC.dnfqAnsPend && end <= OFF_DAT_OPTS + 15 - 2; i++ ) {
            LMIC.frame[end+0] = MCMD_DNFQ_ANS;
            LMIC.frame[end+1] = (LMIC.dnfqAcks >> (2*i)) & 3;
            end += 2;
        }
        LMIC.dnfqAnsPend += LMIC.dnfqAns;
        LMIC.dnfqAns = 0;
    }
    if( LMIC.devsAns ) {  // answer to device status
        if( end <= OFF_DAT_OPTS + 15 - 3 ) {
            LMIC.frame[end+0] = MCMD_DEVS_ANS;
            LMIC.frame[end+1] = os_getBattLevel();
            LMIC.frame[end+2] = LMIC.margin;
        }
        LMIC.devsAns = 0;
        end += 3;
    }
#if !defined(DISABLE_CLASSB)
    if( LMIC.askForTime > 0 ) {
        LMIC.frame[end] = MCMD_TIME_REQ;
        end += 1;
    }
    if( LMIC.bcnfAns ) {
        LMIC.frame[end+0] = MCMD_BCNF_ANS;
        LMIC.frame[end+1] = LMIC.bcnfAns;
        LMIC.bcnfAns = 0;
        end += 2;
    }
#endif
    if( LMIC.gwmargin == 255 ) {
        LMIC.frame[end] = MCMD_LCHK_REQ;
        end += 1;
    }
    ASSERT(end-OFF_DAT_OPTS <= 15);

    // XXX: if( end - OFF_DAT_OPTS > 15 ) ... send as MAC frame with port=0
    u1_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
        // Options and payload too big - delay payload
        txdata = 0;
        flen = end+4;
    }
    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | (LMIC.pendTxNoRx ? 0 : LMIC.adrEnabled
                               | ((LMIC.adrAckReq >= 0 && (LMIC.opmode & OP_LINKDEAD) == 0) ? FCT_ADRARQ : 0))
                               | ((LMIC.opmode & (OP_TRACK|OP_PINGABLE)) == (OP_TRACK|OP_PINGABLE) ? FCT_CLASSB : 0)
                               | (end-OFF_DAT_OPTS));
    os_wlsbf4(LMIC.frame+OFF_DAT_ADDR,  LMIC.devaddr);

    if( LMIC.txCnt == 0 || (LMIC.opmode & (OP_TXDATA|OP_POLL)) == OP_POLL ) {
        LMIC.txCnt = 0;
        LMIC.seqnoUp += 1;
    }
    os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, LMIC.seqnoUp-1);

    // Clear pending DN confirmation
    LMIC.dnConf = 0;

    if( txdata ) {
        if( LMIC.pendTxConf ) {
            // Confirmed only makes sense if we have a payload (or at least a port)
            LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
        }
        LMIC.frame[end] = LMIC.pendTxPort;
        os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
        lce_cipher(LMIC.pendTxPort==0 ? LCE_NWKSKEY : LCE_APPSKEY,
                   LMIC.devaddr, LMIC.seqnoUp-1, /*up*/0, LMIC.frame+end+1, dlen);
    }
    lce_addMic(LCE_NWKSKEY, LMIC.devaddr, LMIC.seqnoUp-1, LMIC.frame, flen-4);
    LMIC.dataLen = flen;
}


#if !defined(DISABLE_CLASSB)
// Callback from HAL during scan mode or when job timer expires.
static void onBcnScanRx (osjob_t* job) {
    // stop radio and its job
    os_radio(RADIO_RST);
    if( decodeBeacon() ) {
        // Found our 1st beacon
        // We don't have a previous beacon to calc some drift - assume some max drift
        calcBcnRxWindowFromMillis(BCN_100PPM_ms,1);
        LMIC.opmode = (LMIC.opmode & ~OP_SCAN) | OP_TRACK;
        reportEvent(EV_BEACON_FOUND);    // can be disabled in callback
        return;
    }
    if( (os_getTime() - LMIC.bcninfo.txtime) >= 0 ) {
        LMIC.opmode &= ~(OP_SCAN | OP_TRACK);
        reportEvent(EV_SCAN_TIMEOUT);
        return;
    }
    engineUpdate();
}


// Enable receiver to listen to incoming beacons
// This mode ends with events: EV_SCAN_TIMEOUT/EV_SCAN_BEACON
// Implicitely cancels any pending TX/RX transaction.
// Also cancels an onpoing joining procedure.
static void startScan (void) {
    if( (LMIC.opmode & (OP_TRACK|OP_SHUTDOWN)) != 0 )
        return;  // already tracking a beacon or shutting down
    // Set scan timeout - one sec longer than beacon period
    LMIC.bcninfo.txtime = os_getTime() + sec2osticks(BCN_INTV_sec+1);
    LMIC.opmode |= OP_SCAN;
    LMIC.askForTime = 0;
}


u1_t LMIC_enableTracking (u1_t tryAskForTime) {
    if( (LMIC.opmode & (OP_SCAN|OP_SHUTDOWN)) != 0 )
        return 0;  // already scanning or shutdown in progress - ignore
    if( (LMIC.opmode & OP_TRACK) != 0 )
        return 2;  // already tracking a beacon - we're done
    // If BCN info requested from NWK then app has to take care
    // of sending data up so that MCMD_BCNI_REQ can be attached.
    LMIC.missedBcns = 0;
    LMIC.askForTime = tryAskForTime;
    if( tryAskForTime == 0 ) {
        startScan();
        reportEvent(EV_START_SCAN);
        return 1;
    }
    opmodePoll();
    engineUpdate();
    return 1;  // enabled
}


void LMIC_disableTracking (void) {
    LMIC.opmode &= ~(OP_SCAN|OP_TRACK);
    LMIC.askForTime = 0;
    engineUpdate();
}


// called by radio when data arrived or by timeout job
static void scan_done (osjob_t* job) {
    if( LMIC.dataLen ) { // frame received (scan_done scheduled by irqjob after rxdone irq, radio stopped)
        if( decodeMultiCastFrame() ) {
            // good frame
            LMIC.opmode &= ~OP_NOENGINE;
            reportEvent(EV_SCAN_FOUND);
        } else {
            // bad frame
            BACKTRACE();
            LMIC.dataLen = 0;
            // continue scanning until timeout
            os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, scan_done); // job/func also used for radio callback!
            os_radio(RADIO_RXON);
        }
    } else { // timeout
        // stop radio and its job
        os_radio(RADIO_RST);
        LMIC.opmode &= ~OP_NOENGINE;
        reportEvent(EV_SCAN_TIMEOUT);
    }
}


// start scanning for multi-cast data frame until timeout (LMIC.freq and LMIC.dndr must be set)
// will generate EV_SCAN_FOUND or EV_SCAN_TIMEOUT events
int LMIC_scan (ostime_t timeout) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 ) {
        return 0;  // already in progress or failed to enable
    }

    // cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = 0;
    LMIC.opmode = (LMIC.opmode | OP_NOENGINE) & ~(OP_TXRXPEND);
    LMIC.dataLen = 0;

    // start scanning until timeout
    LMIC.rps = dndr2rps(LMIC.dndr);
    LMIC.bcninfo.txtime = os_getTime() + timeout; // save timeout
    os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, scan_done); // job/func also used for radio callback!
    os_radio(RADIO_RXON);

    return 1;  // enabled
}


// called by radio when data arrived or on symbol timeout
static void track_done (osjob_t* job) {
    LMIC.opmode &= ~OP_NOENGINE;
    if( LMIC.dataLen && decodeMultiCastFrame() ) {
        reportEvent(EV_BEACON_TRACKED);
    } else {
        LMIC.dataLen = 0;
        reportEvent(EV_BEACON_MISSED);
    }
}


static void track_start (osjob_t* job) {
    LMIC.rxsyms = MINRX_SYMS;
    LMIC.osjob.func = track_done;
    os_radio(RADIO_RX);
}


// try to receive single multi-cast data frame at specified time (LMIC.freq and LMIC.dndr must be set)
// will generate EV_BEACON_TRACKED or EV_BEACON_MISSED events
int LMIC_track (ostime_t when) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 ) {
        return 0;  // already in progress or failed to enable
    }

    // cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = 0;
    LMIC.opmode = (LMIC.opmode | OP_NOENGINE) & ~(OP_TXRXPEND);
    LMIC.dataLen = 0;

    // schedule single rx at given time considering ramp-up time
    LMIC.rps = dndr2rps(LMIC.dndr);
    LMIC.rxtime = when + dr2hsym(LMIC.dndr, PAMBL_SYMS-MINRX_SYMS);
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, track_start);
    return 1;
}
#endif


// ================================================================================
//
// Join stuff
//
// ================================================================================

static void buildJoinRequest (u1_t ftype) {
    // Do not use pendTxData since we might have a pending
    // user level frame in there. Use RX holding area instead.
    LMIC.devNonce = hal_dnonce_next();
    u1_t* d = LMIC.frame;
    d[OFF_JR_HDR] = ftype;
    os_getJoinEui(d + OFF_JR_JOINEUI);
    os_getDevEui(d + OFF_JR_DEVEUI);
    os_wlsbf2(d + OFF_JR_DEVNONCE, LMIC.devNonce);
    lce_addMicJoinReq(d, OFF_JR_MIC);
    LMIC.dataLen = LEN_JR;
}

static void startJoining (osjob_t* osjob) {
    reportEvent(EV_JOINING);
}

// Start join procedure if not already joined.
bit_t LMIC_startJoining (void) {
    if( LMIC.devaddr == 0 ) {
        // There should be no TX/RX going on
        ASSERT((LMIC.opmode & (OP_POLL|OP_TXRXPEND)) == 0);
        // Lift any previous duty limitation
        LMIC.globalDutyRate = 0;
        // Cancel scanning
        LMIC.opmode &= ~(OP_SCAN|OP_REJOIN|OP_LINKDEAD|OP_NEXTCHNL);
        // Setup state
        LMIC.rejoinCnt = LMIC.txCnt = LMIC.pendTxConf = 0;
        initJoinLoop();
        LMIC.opmode |= OP_JOINING;
        LMIC.clmode = 0;
        LMIC.pollcnt = 0;
        // reportEvent will call engineUpdate which then starts sending JOIN REQUESTS
        os_setCallback(&LMIC.osjob, FUNC_ADDR(startJoining));
        return 1;
    }
    return 0; // already joined
}


// ================================================================================
//
//
//
// ================================================================================

#if !defined(DISABLE_CLASSB)
static void processPingRx (osjob_t* osjob) {
    if( LMIC.dataLen != 0 ) {
        LMIC.txrxFlags = TXRX_PING;
        if( decodeFrame() ) {
            reportEvent(EV_RXCOMPLETE);
            return;
        }
    }
    // Pick next ping slot
    engineUpdate();
}
#endif


static bit_t processDnData (void) {
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      norx:
        // Nothing received - implies no port
        LMIC.txrxFlags = TXRX_NOPORT;
        if( (LMIC.opmode & OP_TXDATA) ) {
            LMIC.txCnt += 1;
            if( LMIC.txCnt < LMIC.nbTrans ) {
                // Schedule another retransmission
                txDelay(LMIC.rxtime, RETRY_PERIOD_secs);
                LMIC.opmode |= OP_NEXTCHNL;
                os_setCallback(&LMIC.osjob, FUNC_ADDR(runEngineUpdate));
                goto txcontinue;
            } else {
                LMIC.opmode &= ~OP_TXDATA;
                LMIC.foptsUpLen = 0;    // clear fopts
            }
            if( LMIC.pendTxConf != 0 )  // we requested an ACK
                LMIC.txrxFlags |= TXRX_NACK;
        }
        if( LMIC.adrAckReq != LINK_CHECK_OFF )
            LMIC.adrAckReq += 1;
        LMIC.dataBeg = LMIC.dataLen = 0;
      txcomplete:
        if( (LMIC.txrxFlags & (TXRX_DNW1|TXRX_DNW2|TXRX_PING)) != 0  &&  (LMIC.opmode & OP_LINKDEAD) != 0 ) {
            LMIC.opmode &= ~OP_LINKDEAD;
            reportEvent(EV_LINK_ALIVE);
        }
        reportEvent(EV_TXCOMPLETE);
        // If we haven't heard from NWK in a while although we asked for a sign
        // assume link is dead - notify application and keep going
        if( LMIC.adrAckReq >= (s4_t) LINK_CHECK_DEAD ) {
            // We haven't heard from NWK for some time although we
            // asked for a response for some time - assume we're disconnected. Lower DR one notch.
            if (LMIC.txPowAdj) {
                setDrTxpow(DRCHG_NOADRACK, LMIC.datarate, 0);
            } else if (decDR(LMIC.datarate) != LMIC.datarate) {
                setDrTxpow(DRCHG_NOADRACK, decDR(LMIC.datarate), KEEP_TXPOWADJ);
            } else if (REG_IS_FIX()
#ifdef REG_FIX
                    && enableAllChannels_fix()
#endif
                    ) {
                // some channels were disabled
                // NOTE: we now enter a network/device state mismatch!!
            } else {
                // nothing we can do anymore
                LMIC.opmode |= /* XXX OP_REJOIN| */ OP_LINKDEAD;
            }
            LMIC.adrAckReq = 0;
            reportEvent((LMIC.opmode & OP_LINKDEAD) ? EV_LINK_DEAD : EV_ADR_BACKOFF);
        }
#if !defined(DISABLE_CLASSB)
        // If this falls to zero the NWK did not answer our MCMD_TIME_REQ commands - try full scan
        if( LMIC.askForTime > 0 ) {
            if( --LMIC.askForTime == 0 ) {
                startScan();   // NWK did not answer - try scan
                reportEvent(EV_START_SCAN);
            }
            else {
                opmodePoll();
            }
        }
#endif
      txcontinue:
        LMIC.opmode &= ~OP_TXRXPEND;
        return 1;
    }
    if( !decodeFrame() ) {
        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 )
            return 0;
        goto norx;
    }
    addRxdErr(LMIC.dn1Dly + (LMIC.txrxFlags & TXRX_DNW2 ? DELAY_EXTDNW2 : 0));
    if( (LMIC.opmode & OP_TXDATA) )
        LMIC.txCnt += 1;
    LMIC.opmode &= ~OP_TXDATA;
    LMIC.pendTxConf = 0;
    goto txcomplete;
}


#if !defined(DISABLE_CLASSB)
static void processBeacon (osjob_t* osjob) {
    ostime_t lasttx = LMIC.bcninfo.txtime;   // save previous - decodeBeacon overwrites
    u1_t flags = LMIC.bcninfo.flags;
    ev_t ev;

    if( decodeBeacon() >= 1 ) {
        ev = EV_BEACON_TRACKED;
        if( (flags & (BCN_PARTIAL|BCN_FULL)) == 0 ) {
            // We don't have a previous beacon to calc some drift - assume some max value
            calcBcnRxWindowFromMillis(BCN_100PPM_ms,0);
            goto rev;
        }
        // We have a previous BEACON to calculate some drift
        s2_t drift = (LMIC.bcninfo.txtime - lasttx) - BCN_INTV_osticks;
        if( LMIC.missedBcns > 0 ) {
            drift = LMIC.drift + (drift - LMIC.drift) / (LMIC.missedBcns+1);
        }
        if( (LMIC.bcninfo.flags & BCN_NODRIFT) == 0 ) {
            s2_t diff = LMIC.drift - drift;
            if( diff < 0 ) diff = -diff;
            LMIC.lastDriftDiff = diff;
            if( LMIC.maxDriftDiff < diff )
                LMIC.maxDriftDiff = diff;
            LMIC.bcninfo.flags &= ~BCN_NODDIFF;
        }
        LMIC.drift = drift;
        LMIC.missedBcns = LMIC.rejoinCnt = 0;
        LMIC.bcninfo.flags &= ~BCN_NODRIFT;
        ASSERT((LMIC.bcninfo.flags & (BCN_PARTIAL|BCN_FULL)) != 0);
    } else {
        ev = EV_BEACON_MISSED;
        LMIC.bcninfo.txtime += BCN_INTV_osticks + LMIC.drift;
        LMIC.bcninfo.time   += BCN_INTV_sec;
        LMIC.missedBcns++;
        // Delay any possible TX after surmised beacon - it's there although we missed it
        txDelay(LMIC.bcninfo.txtime + BCN_RESERVE_osticks, 4);
        if( LMIC.missedBcns > MAX_MISSED_BCNS )
            LMIC.opmode |= OP_REJOIN;  // try if we can roam to another network
        if( LMIC.bcnRxsyms > MAX_RXSYMS ) {
            LMIC.opmode &= ~(OP_TRACK|OP_PINGABLE|OP_PINGINI|OP_REJOIN);
            reportEvent(EV_LOST_TSYNC);
            return;
        }
    }
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks - calcRxWindow(0, REGION.beaconDr);
    LMIC.bcnRxsyms = LMIC.rxsyms;
  rev:
    LMIC.bcnChnl = (LMIC.bcnChnl+1) % numBcnChannels();
    if( (LMIC.opmode & OP_PINGINI) != 0 )
        rxschedInit(&LMIC.ping);  // note: reuses LMIC.frame buffer!
    reportEvent(ev);
}

static void startRxBcn (osjob_t* osjob) {
    LMIC.osjob.func = FUNC_ADDR(processBeacon);
    os_radio(RADIO_RX);
}


static void startRxPing (osjob_t* osjob) {
    LMIC.osjob.func = FUNC_ADDR(processPingRx);
    os_radio(RADIO_RX);
}
#endif



// Decide what to do next for the MAC layer of a device
static void engineUpdate (void) {
    debug_printf("engineUpdate[opmode=0x%x]\r\n", LMIC.opmode);
    // Check for ongoing state: scan or TX/RX transaction
    if( (LMIC.opmode & (OP_NOENGINE|OP_TXRXPEND|OP_SHUTDOWN)) != 0 ) {
        return;
    }

#ifdef CFG_autojoin
    if( LMIC.devaddr == 0 && (LMIC.opmode & OP_JOINING) == 0 ) {
        LMIC_startJoining();
        return;
    }
#endif // CFG_autojoin

    ostime_t now    = os_getTime();
    ostime_t rxtime = 0;
    ostime_t txbeg  = 0;

#if !defined(DISABLE_CLASSB)
    if( (LMIC.opmode & OP_SCAN) != 0 ) {
        // Looking for a beacon - LMIC.bcninfo.txtime is timeout for scan
        // Cancel onging TX/RX transaction
        LMIC.dataLen = 0;
        LMIC.txCnt = LMIC.dnConf = LMIC.bcninfo.flags = 0;
        LMIC.opmode &= ~OP_TXRXPEND;
        LMIC.bcnChnl = 0;
        setBcnRxParams();
        os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, FUNC_ADDR(onBcnScanRx));
        os_radio(RADIO_RXON);
        return;
    }
    if( (LMIC.opmode & OP_TRACK) != 0 ) {
        // We are tracking a beacon
        rxtime = LMIC.bcnRxtime - RX_RAMPUP;
#if CFG_simul
        // Simulation is sometimes late - don't die here but keep going.
        // Results in a missed beacon. On the HW this spells a more serious problem.
        if( (ostime_t)(rxtime-now) < 0 ) {
            fprintf(stderr, "ERROR: engineUpdate/OP_TRACK: delta=%d now=0x%X rxtime=0x%X LMIC.bcnRxtime=0x%X RX_RAMPUP=%d\n",
                    (ostime_t)(rxtime-now),now,rxtime,LMIC.bcnRxtime,RX_RAMPUP);
        }
#else
        ASSERT( (ostime_t)(rxtime-now) >= 0 );
#endif
    }
#endif
    if( LMIC.pollcnt )
        opmodePoll();

    if( (LMIC.opmode & (OP_JOINING|OP_REJOIN|OP_TXDATA)) != 0 ||
        ((LMIC.opmode & OP_POLL) && now - LMIC.polltime >= LMIC.polltimeout)) {
        // Need to TX some data...
        // Assuming txChnl points to channel which first becomes available again.
        bit_t jacc = ((LMIC.opmode & (OP_JOINING|OP_REJOIN)) != 0 ? 1 : 0);
        if (jacc)
            debug_verbose_printf("Uplink join pending\r\n", os_getTime());
        else
            debug_verbose_printf("Uplink data pending\r\n", os_getTime());
        // Find next suitable channel and return availability time
        if( (LMIC.opmode & OP_NEXTCHNL) != 0 ) {
            txbeg = LMIC.txend = nextTx(now);
            debug_verbose_printf("Airtime available at %F (channel duty limit)\r\n", txbeg, 0);
        } else {
            txbeg = LMIC.txend;
            debug_verbose_printf("Airtime available at %F (previously determined)\r\n", txbeg, 0);
        }
        // Delayed TX or waiting for duty cycle?
        if( (LMIC.globalDutyRate != 0 || (LMIC.opmode & OP_RNDTX) != 0)  &&  (txbeg - LMIC.globalDutyAvail) < 0 ) {
            txbeg = LMIC.globalDutyAvail;
            debug_verbose_printf("Airtime available at %F (global duty limit)\r\n", txbeg, 0);
        }
#if !defined(DISABLE_CLASSB)
        // If we're tracking a beacon...
        // then make sure TX-RX transaction is complete before beacon
        if( (LMIC.opmode & OP_TRACK) != 0 &&
            txbeg + (jacc ? JOIN_GUARD_osticks : TXRX_GUARD_osticks) - rxtime > 0 ) {

            debug_verbose_printf("Awaiting beacon before uplink\r\n");

            // Not enough time to complete TX-RX before beacon - postpone after beacon.
            // In order to avoid clustering of postponed TX right after beacon randomize start!
            txDelay(rxtime + BCN_RESERVE_osticks, 16);
            txbeg = 0;
            goto checkrx;
        }
#endif
        // Earliest possible time vs overhead to setup radio
        if( txbeg - (now + TX_RAMPUP) <= 0 ) {
        debug_verbose_printf("Ready for uplink\r\n");
            // We could send right now!
            txbeg = now;
            dr_t txdr = LMIC.datarate;
            if( jacc ) {
                u1_t ftype;
                if( (LMIC.opmode & OP_REJOIN) != 0 ) {
                    txdr = lowerDR(txdr, LMIC.rejoinCnt);
                    ftype = HDR_FTYPE_REJOIN;
                } else {
                    ftype = HDR_FTYPE_JREQ;
                }
                buildJoinRequest(ftype);
                LMIC.osjob.func = FUNC_ADDR(jreqDone);
            } else {
                // XXX:TODO - also handle LMIC.seqnoADn rollover
                if( LMIC.seqnoDn >= 0xFFFFFF80 ) {
                    // Imminent roll over - proactively reset MAC
                    // Device has to react! NWK will not roll over and just stop sending.
                    // Thus, we have N frames to detect a possible lock up.
                  reset:
                    os_setCallback(&LMIC.osjob, FUNC_ADDR(runReset));
                    return;
                }
                if( (LMIC.txCnt==0 && LMIC.seqnoUp == 0xFFFFFFFF) ) {
                    // Roll over of up seq counter
                    // Do not run RESET event callback from here!
                    // App code might do some stuff after send unaware of RESET.
                    goto reset;
                }
                buildDataFrame();
                LMIC.osjob.func = FUNC_ADDR(updataDone);
            }
            LMIC.rps    = setCr(updr2rps(txdr), (cr_t)LMIC.errcr);
            LMIC.dndr   = txdr;  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
            LMIC.opmode = (LMIC.opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
            updateTx(txbeg);
            reportEvent(EV_TXSTART);
            os_radio(RADIO_TX);
            return;
        }
        debug_verbose_printf("Uplink delayed until %F\r\n", txbeg, 0);
        // Cannot yet TX
        if( (LMIC.opmode & OP_TRACK) == 0 )
            goto txdelay; // We don't track the beacon - nothing else to do - so wait for the time to TX
        // Consider RX tasks
        if( txbeg == 0 ) // zero indicates no TX pending
            txbeg += 1;  // TX delayed by one tick (insignificant amount of time)
    } else {
        // No TX pending - no scheduled RX
        if( (LMIC.opmode & OP_TRACK) == 0 ) {
            if( (LMIC.clmode & CLASS_C) ) {
                setupRx2ClassC();
            }
            return;
        }
    }

#if !defined(DISABLE_CLASSB)
    // Are we pingable?
  checkrx:
    if( (LMIC.opmode & OP_PINGINI) != 0 ) {
        // One more RX slot in this beacon period?
        if( rxschedNext(&LMIC.ping, now+RX_RAMPUP) ) {
            if( txbeg != 0  &&  (txbeg - LMIC.ping.rxtime) < 0 )
                goto txdelay;
            LMIC.rxsyms  = LMIC.ping.rxsyms;
            LMIC.rxtime  = LMIC.ping.rxtime;
            LMIC.freq    = LMIC.ping.freq;          // XXX:US like => calc based on beacon time!
            LMIC.rps     = dndr2rps(LMIC.ping.dr);
            LMIC.dataLen = 0;
            ASSERT(LMIC.rxtime - now+RX_RAMPUP >= 0 );
            os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, FUNC_ADDR(startRxPing));
            return;
        }
        // no - just wait for the beacon
    }

    if( txbeg != 0  &&  (txbeg - rxtime) < 0 )
        goto txdelay;

    setBcnRxParams();
    LMIC.rxsyms = LMIC.bcnRxsyms;
    LMIC.rxtime = LMIC.bcnRxtime;
    if( now - rxtime >= 0 ) {
        LMIC.osjob.func = FUNC_ADDR(processBeacon);
        os_radio(RADIO_RX);
        return;
    }
    os_setTimedCallback(&LMIC.osjob, rxtime, FUNC_ADDR(startRxBcn));
    return;
#endif

  txdelay:
    if( (LMIC.clmode & CLASS_C) ) {
        setupRx2ClassC();
    }
    os_setTimedCallback(&LMIC.osjob, txbeg-TX_RAMPUP, FUNC_ADDR(runEngineUpdate));
}


void LMIC_setAdrMode (bit_t enabled) {
    LMIC.adrEnabled = enabled ? FCT_ADREN : 0;
}


//  Should we have/need an ext. API like this?
void LMIC_setDrTxpow (dr_t dr, s1_t txpowadj) {
    setDrTxpow(DRCHG_SET, dr, txpowadj);
    syncDatarate();
}


void LMIC_shutdown (void) {
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    LMIC.opmode |= OP_SHUTDOWN;
}


int LMIC_regionIdx (u1_t regionCode) {
    if( regionCode == REGCODE_UNDEF ) {
        return 0;
    }
    int idx;
    for (idx = 0; idx < sizeof(REGIONS) / sizeof(region_t); idx++) {
	if (REGIONS[idx].regcode == regionCode) {
	    return idx;
	}
    }
    return -1;
}

void LMIC_reset_ex (u1_t regionCode) {
    os_radio(RADIO_RST);
    os_clearCallback(&LMIC.osjob);


    os_clearMem((u1_t*) &LMIC, sizeof(LMIC));
    initRegion(regionCode);
    LMIC.devNonce     = os_getRndU2();
    LMIC.opmode       = OP_NONE;
    LMIC.errcr        = CR_4_5;
    LMIC.adrEnabled   = FCT_ADREN;
    LMIC.datarate     = fastest125();
    LMIC.dn1Dly       = 1;
    LMIC.dn2Dr        = REGION.rx2Dr;    // we need this for 2nd DN window of join accept
    LMIC.dn2Freq      = REGION.rx2Freq;  // ditto
#if !defined(DISABLE_CLASSB)
    LMIC.ping.freq    = REGION.pingFreq; // defaults for ping
    LMIC.ping.dr      = REGION.pingDr;   // ditto
    LMIC.ping.intvExp = 8;               // no ping interval ever sent up
#endif
    LMIC.refChnl      = REG_IS_FIX() ? 0 : os_getRndU1();  // fix uses randomized hoplist starting with block0
    LMIC.adrAckLimit  = ADR_ACK_LIMIT;
    LMIC.adrAckDelay  = ADR_ACK_DELAY;
    LMIC.adrAckReq    = LINK_CHECK_INIT;
    LMIC.syncword     = 0x34; // LORA_MAC sync word

    iniRxdErr();
}

void LMIC_reset (void) {
    LMIC_reset_ex(os_getRegion());
}

void LMIC_init (void) {
    LMIC.opmode = OP_SHUTDOWN;
}

void LMIC_clrTxData (void) {
    LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND|OP_POLL);
    LMIC.pendTxLen = 0;
    if( (LMIC.opmode & (OP_JOINING|OP_SCAN)) != 0 ) // do not interfere with JOINING/SCANNING
        return;
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    engineUpdate();
}


void LMIC_setTxData (void) {
    ASSERT((LMIC.opmode & OP_JOINING) == 0);
    LMIC.opmode |= OP_TXDATA;
    LMIC.txCnt = 0;             // reset nbTrans counter
    engineUpdate();
}


//
int LMIC_setTxData2 (u1_t port, u1_t* data, u1_t dlen, u1_t confirmed) {
    if( dlen > sizeof(LMIC.pendTxData) )
        return -2;
    if( data != (u1_t*)0 )
        os_copyMem(LMIC.pendTxData, data, dlen);
    LMIC.pendTxConf = confirmed;
    LMIC.pendTxPort = port;
    LMIC.pendTxLen  = dlen;
    LMIC_setTxData();
    return 0;
}


// Send a payload-less message to signal device is alive
void LMIC_sendAlive (void) {
    opmodePoll();
    LMIC.pendTxConf = 0;
    LMIC.pendTxLen = 0;
    LMIC.txCnt = 0;             // reset nbTrans counter
    engineUpdate();
}


// Check if other networks are around.
void LMIC_tryRejoin (void) {
    LMIC.opmode |= OP_REJOIN;
    engineUpdate();
}


// Check if other networks are around.
void LMIC_setClassC (u1_t enabled) {
    if( (LMIC.clmode & CLASS_C) == (enabled?CLASS_C:0) )
        return;  // already in that mode
    if( (LMIC.clmode & PEND_CLASS_C) )
        return; // change is already pending
#if !defined(DISABLE_CLASSB)
    if( enabled )
        LMIC_stopPingable();  // stop class B
#endif
#if defined(CFG_lorawan11)
    if( enabled != UNILATERAL_CLASS_C ) {
        LMIC.clmode |= PEND_CLASS_C;
        opmodePoll();
        incPollcnt();
        engineUpdate();
        return;
    }
#endif
    // LoRaWAN 1.0.2 - switch mode unilaterally
    // Device is provisioned as class C
    LMIC.clmode = CLASS_C;
    engineUpdate();
}

//! \brief Setup given session keys
//! and put the MAC in a state as if
//! a join request/accept would have negotiated just these keys.
//! It is crucial that the combinations `devaddr/nwkkey` and `devaddr/artkey`
//! are unique within the network identified by `netid`.
//! NOTE: on Harvard architectures when session keys are in flash:
//!  Caller has to fill in LMIC.{nwk,art}Key  before and pass {nwk,art}Key are NULL
//! \param netid a 24 bit number describing the network id this device is using
//! \param devaddr the 32 bit session address of the device. It is strongly recommended
//!    to ensure that different devices use different numbers with high probability.
//! \param nwkKey  the 16 byte network session key used for message integrity.
//!     If NULL the caller has copied the key into `LMIC.nwkKey` before.
#if defined(CFG_lorawan11)
//! \param nwkKeyDn  the 16 byte network session key used for down-link message integrity.
//!     If NULL the caller has copied the key into `LMIC.nwkKeyDn` before.
#endif
//! \param appKey  the 16 byte application router session key used for message confidentiality.
//!     If NULL the caller has copied the key into `LMIC.appKey` before.
void LMIC_setSession (u4_t netid, devaddr_t devaddr, const u1_t* nwkKey,
#if defined(CFG_lorawan11)
        const u1_t* nwkKeyDn,
#endif
        const u1_t* appKey) {
    LMIC.netid = netid;
    LMIC.devaddr = devaddr;
    lce_loadSessionKeys(nwkKey,
#if defined(CFG_lorawan11)
                        nwkKeyDn,
#endif
                        appKey);

    initDefaultChannels();

    stateJustJoined();
    setDrTxpow(DRCHG_SET, fastest125(), 0);
    LMIC.dn2Dr = REGION.rx2Dr;
    LMIC.dn1Dly = 1;
    LMIC.dn1DrOffIdx = 0;
}

void LMIC_setMultiCastSession (devaddr_t grpaddr, const u1_t* nwkKeyDn, const u1_t* appKey, u4_t seqnoADn) {
    session_t* s;
    for(s = LMIC.sessions; s<LMIC.sessions+MAX_MULTICAST_SESSIONS && s->grpaddr!=0 && s->grpaddr!=grpaddr; s++);
    ASSERT(s < LMIC.sessions+MAX_MULTICAST_SESSIONS);

    s->grpaddr  = grpaddr;
    s->seqnoADn = seqnoADn;

    if( nwkKeyDn != (u1_t*)0 ) {
        os_copyMem(s->nwkKeyDn, nwkKeyDn, 16);
    }

    if( appKey != (u1_t*)0 ) {
        os_copyMem(s->appKey, appKey, 16);
    }
}

// Enable/disable link check validation.
// LMIC sets the ADRACKREQ bit in UP frames if there were no DN frames
// for a while. It expects the network to provide a DN message to prove
// connectivity with a span of UP frames. If this no such prove is coming
// then the datarate is lowered and a LINK_DEAD event is generated.
// This mode can be disabled and no connectivity prove (ADRACKREQ) is requested
// nor is the datarate changed.
// This must be called only if a session is established (e.g. after EV_JOINED)
void LMIC_setLinkCheckMode (bit_t enabled) {
    LMIC.adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

void LMIC_setLinkCheck (u4_t limit, u4_t delay) {
    LMIC.adrAckLimit = limit;
    LMIC.adrAckDelay = delay;
    LMIC.adrAckReq = LINK_CHECK_INIT;
}

bit_t LMIC_setupChannel (u1_t chidx, freq_t freq, u2_t drmap) {
    if (REG_IS_FIX()) {
        return 0;
    } else {
#ifdef REG_DYN
        return setupChannel_dyn(chidx, freq, drmap);
#endif
    }
}

void LMIC_disableChannel (u1_t chidx) {
    disableChannel(chidx);
}

void LMIC_askForLinkCheck (void) {
    LMIC.gwmargin = 255;
    LMIC.gwcnt = 0;
}


// Fastest UP datarate
dr_t LMIC_fastestDr () {
    dr_t dr = 1;  // assuming DR=0 is always 125kHz
    for(; dr < 16; dr++ ) {
        rps_t rps = REGION.dr2rps[dr];
        if( rps == ILLEGAL_RPS || getNocrc(rps) ) // DN only DR
            break;
    }
    return dr-1;
}


// Slowest UP datarate
dr_t LMIC_slowestDr () {
    return 0; //XXX:AS923:TBD must be 2 under certain conditions
}


rps_t LMIC_updr2rps (u1_t dr) {
    return updr2rps(dr);
}

rps_t LMIC_dndr2rps (u1_t dr) {
    return dndr2rps(dr);
}

ostime_t LMIC_calcAirTime (rps_t rps, u1_t plen) {
    return calcAirTime(rps, plen);
}


#if defined(CFG_simul)
#include "addr2func.h"
#include "arr2len.h"
#endif

#if defined(CFG_extapi)

// Enable fast join (for testing only)
// Removes duty cycle limitations.
// Call directly after LMIC_startJoining() returns non-zero.
void LMIC_enableFastJoin (void) {
    // XXX deprecated
}

// Remove duty cycle limitations
void LMIC_disableDC (void) {
    LMIC.noDC = 1;
}

/// Used for regression testing
ostime_t LMIC_dr2hsym (dr_t dr, s1_t num) {
    return dr2hsym(dr,num);
}

ostime_t LMIC_nextTx (ostime_t now) {
    return nextTx(now);
}

void LMIC_updateTx (ostime_t now) {
    updateTx(now);
}

// Return scaled clock skew (RXDERR_SHIFT) and variation span in osticks.
void LMIC_getRxdErrInfo (s4_t* skew, u4_t* span) {
    *skew = evalRxdErr(span);
}

#endif

#define __i(func,suffix) .func = func ## suffix
#ifdef REG_DYN
#define __dyn(func) __i(func,_dyn)
static const rfuncs_t RFUNCS_DYN = {
    __dyn(disableChannel),
    __dyn(initDefaultChannels),
    __dyn(prepareDn),
    __dyn(applyChannelMap),
    __dyn(checkChannelMap),
    __dyn(syncDatarate),
    __dyn(updateTx),
    __dyn(nextTx),
#if !defined(DISABLE_CLASSB)
    __dyn(setBcnRxParams),
#endif
};
#undef __dyn
#endif // REG_DYN
#ifdef REG_FIX
#define __fix(func) __i(func,_fix)
static const rfuncs_t RFUNCS_FIX = {
    __fix(disableChannel),
    __fix(initDefaultChannels),
    __fix(prepareDn),
    __fix(applyChannelMap),
    __fix(checkChannelMap),
    __fix(syncDatarate),
    __fix(updateTx),
    __fix(nextTx),
#if !defined(DISABLE_CLASSB)
    __fix(setBcnRxParams),
#endif
};
#undef __fix
#endif // REG_FIX
#undef __i

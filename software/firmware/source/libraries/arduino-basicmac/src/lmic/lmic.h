// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//! @file
//! @brief LMIC API

#ifndef _lmic_h_
#define _lmic_h_

#include "region.h"
#include "oslmic.h"
#include "lorabase.h"
#include "lce.h"

#ifdef __cplusplus
extern "C"{
#endif

#ifdef RASPBERRY_PI
#include "raspi/raspi.h"
#endif

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <cc13xx/cc13xx.h>
#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */

#include <protocol.h>

// LMIC version
#define LMIC_VERSION_MAJOR 2
#define LMIC_VERSION_MINOR 1


// ------------------------------------------------
// BEGIN -- MULTI REGION

// band
typedef struct {
    freq_t      lo, hi;
    u2_t        txcap;
    s1_t        txpow;
} band_t;

#define MAX_BANDS       4
#define BAND_MASK       (MAX_BANDS-1)
#if (MAX_BANDS & BAND_MASK) != 0
#error "MAX_BANDS must be a power of 2"
#endif

enum {
    CAP_NONE  = 1,
    CAP_DECI  = 10,
    CAP_CENTI = 100,
    CAP_MILLI = 1000,
};

// region IDs
enum {
#ifdef CFG_eu868
    REGION_EU868,
#endif
#ifdef CFG_as923
    REGION_AS923,
#endif
#ifdef CFG_us915
    REGION_US915,
#endif
#ifdef CFG_au915
    REGION_AU915,
#endif
#ifdef CFG_cn470
    REGION_CN470,
#endif
    REGIONS_COUNT
};

// region flags
enum {
    REG_FIXED        = (1 << 0),     // fixed channel plan
    REG_PSA          = (1 << 1),     // ETSI-style polite spectrum access
};

enum { UPCHSPACING_125kHz =  200000 };  //XXX:hack
enum { UPCHSPACING_500kHz = 1600000 };  //XXX:hack
enum { DNCHSPACING_500kHz =  600000 };  //XXX:hack
enum { DNCHSPACING_125kHz =  200000 };  //XXX:hack

typedef struct {
    void     (*disableChannel) (u1_t chidx);
    void     (*initDefaultChannels) (void);
    void     (*prepareDn) ();
    u1_t     (*applyChannelMap) (u1_t chpage, u2_t chmap, u2_t* dest);
    u1_t     (*checkChannelMap) (u2_t* map);
    void     (*syncDatarate) (void);
    void     (*updateTx) (ostime_t txbeg);
    ostime_t (*nextTx) (ostime_t now);
    void     (*setBcnRxParams) (void);
} rfuncs_t;

// Immutable region definition
// TODO: reorder struct members to optimize padding
typedef struct {
    u4_t flags;
    union {
        // dynamic channels
        struct {
            freq_t      defaultCh[MIN_DYN_CHNLS];       // default channel frequencies
            freq_t      beaconFreq;                     // beacon frequency
            u2_t        chTxCap;                        // per-channel DC
            u2_t        ccaTime;                        // CCA time (ticks)
            s1_t        ccaThreshold;                   // CCA threshold
            band_t      bands[MAX_BANDS];               // band definitions
        };

        // fixed channels
        struct {
            freq_t      baseFreq125;                    // base frequency for 125kHz channels
            freq_t      baseFreqFix;                    // base frequency for fixed-DR channels
            freq_t      baseFreqDn;                     // base frequency downlink channels
            u1_t        numChBlocks;                    // number of 8-channel blocks
            u1_t        numChDnBlocks;                  // number of 8-channel blocks for downlink
            dr_t        fixDr;                          // fixed-DR channel data rate
        };
    };

    // Common
    const rfuncs_t*     rfuncs;
    const u1_t*         dr2rps;                 // pointer to DR table
    freq_t              minFreq, maxFreq;       // legal frequency range
    freq_t              rx2Freq;                // RX2 frequency
    freq_t              pingFreq;               // ping frequency
    dr_t                rx2Dr;                  // RX2 data rate
    dr_t                pingDr;                 // ping data rate
    dr_t                beaconDr;               // beacon data rate
    u1_t                beaconOffInfo;          // offset beacon info field
    u1_t                beaconLen;              // beacon length
    ostime_t            beaconAirtime;          // beacon air time
    eirp_t              maxEirp;                // max. EIRP (initial value)
    u1_t                rx1DrOff[8];            // RX1 data rate offsets
    u1_t                regcode;                // external region code
    // TODO: max frame lengths

} region_t;


//  END  -- MULTI REGION
// ------------------------------------------------

enum { TXCONF_ATTEMPTS    =   8 };   //!< Transmit attempts for confirmed frames
enum { MAX_MISSED_BCNS    =  20 };   // threshold for triggering rejoin requests
enum { MAX_RXSYMS         = 100 };   // stop tracking beacon beyond this

#define RXDERR_NUM 5
#define RXDERR_SHIFT 4
#define RXDERR_SCALE (1<<RXERR_SHIFT)
#ifndef RXDERR_INI
#define RXDERR_INI 50  // ppm
#endif

#define LINK_CHECK_OFF  (0x80000000)
#define LINK_CHECK_INIT (-LMIC.adrAckLimit)
#define LINK_CHECK_DEAD (LMIC.adrAckDelay)

enum { TIME_RESYNC        = 6*128 }; // secs
enum { TXRX_GUARD_ms      =  6000 };  // msecs - don't start TX-RX transaction before beacon
enum { JOIN_GUARD_ms      =  9000 };  // msecs - don't start Join Req/Acc transaction before beacon
enum { TXRX_BCNEXT_secs   =     2 };  // secs - earliest start after beacon time
enum { RETRY_PERIOD_secs  =     3 };  // secs - random period for retrying a confirmed send

// Keep in sync with evdefs.hpp::drChange
enum { DRCHG_SET, DRCHG_NOJACC, DRCHG_NOACK, DRCHG_NOADRACK, DRCHG_NWKCMD };
enum { KEEP_TXPOWADJ = -128 };


#if !defined(DISABLE_CLASSB)
//! \internal
typedef struct {
    u1_t     dr;
    u1_t     intvExp;   // bits: 7:pend, 3:illegal intv, 2-0:intv
    u1_t     slot;      // runs from 0 to 128
    u1_t     rxsyms;
    ostime_t rxbase;
    ostime_t rxtime;    // start of next spot
    u4_t     freq;
} rxsched_t;

//! Parsing and tracking states of beacons.
enum { BCN_NONE    = 0x00,   //!< No beacon received
       BCN_PARTIAL = 0x01,   //!< Only first (common) part could be decoded (info,lat,lon invalid/previous)
       BCN_FULL    = 0x02,   //!< Full beacon decoded
       BCN_NODRIFT = 0x04,   //!< No drift value measured yet
       BCN_NODDIFF = 0x08 }; //!< No differential drift measured yet
//! Information about the last and previous beacons.
typedef struct {
    ostime_t txtime;  //!< Time when the beacon was sent
    s1_t     rssi;    //!< Adjusted RSSI value of last received beacon
    s1_t     snr;     //!< Scaled SNR value of last received beacon
    u1_t     flags;   //!< Last beacon reception and tracking states. See BCN_* values.
    u4_t     time;    //!< GPS time in seconds of last beacon (received or surrogate)
    //
    u1_t     info;    //!< Info field of last beacon (valid only if BCN_FULL set)
    s4_t     lat;     //!< Lat field of last beacon (valid only if BCN_FULL set)
    s4_t     lon;     //!< Lon field of last beacon (valid only if BCN_FULL set)
} bcninfo_t;
#endif


// purpose of receive window - lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX=1, RADIO_RX=2, RADIO_RXON=3, RADIO_TXCW, RADIO_CCA };
// Netid values /  lmic_t.netid
enum { NETID_NONE=(int)~0U, NETID_MASK=(int)0xFFFFFF };
// MAC operation modes (lmic_t.opmode).
enum { OP_NONE     = 0x0000,
       OP_SCAN     = 0x0001, // radio scan to find a beacon
       OP_TRACK    = 0x0002, // track my networks beacon (netid)
       OP_JOINING  = 0x0004, // device joining in progress (blocks other activities)
       OP_TXDATA   = 0x0008, // TX user data (buffered in pendTxData)
       OP_POLL     = 0x0010, // send empty UP frame to ACK confirmed DN/fetch more DN data
       OP_REJOIN   = 0x0020, // occasionally send JOIN REQUEST
       OP_SHUTDOWN = 0x0040, // prevent MAC from doing anything
       OP_TXRXPEND = 0x0080, // TX/RX transaction pending
       OP_RNDTX    = 0x0100, // prevent TX lining up after a beacon
       OP_PINGINI  = 0x0200, // pingable is initialized and scheduling active
       OP_PINGABLE = 0x0400, // we're pingable - aka class B
       OP_NEXTCHNL = 0x0800, // find a new channel
       OP_LINKDEAD = 0x1000, // link was reported as dead
       OP_TESTMODE = 0x2000, // developer test mode
       OP_NOENGINE = 0x4000, // bypass engine update
       OP_NOCRYPT  = 0x8000, // do not encrypt uplinks
};
// TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_PING   = 0x04,   // received in a scheduled RX slot
};
// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET, EV_RXCOMPLETE,
             EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND, EV_TXSTART,
             EV_TXDONE, EV_DATARATE, EV_START_SCAN, EV_ADR_BACKOFF };
typedef enum _ev_t ev_t;


// Internal use values in lmic_t.opts, uses the unused upper nibble
// of option bitmap 1 (0xf0).
enum {
    OPT_LORAWAN11 = 0x80,  // Running LoRaWAN 1.1
    OPT_OPTNEG    = 0x40,  // Send ResetInd mac command
};

#ifndef __cplusplus
// data stored in clmode
enum {
    CLASS_C      = 0x01,  // 0=class A, 1=class C
    PEND_CLASS_C = 0x02,  // 1=MCMD_DEVMD_IND sent (with !CLASS_C, waiting for MCMD_DEVMD_CONF)
};
#endif /* __cplusplus */
// parameters for LMIC_setClassC(..)
enum {
    DISABLE_CLASS_C = 0,  // disable class C - aka enable class A
    ENABLE_CLASS_C  = 1,  // enable class C - stop class A
    // do not notify network and do not wait for confirmation
    // (device provisioned as class C)
    UNILATERAL_CLASS_C = 2,
};

typedef struct {
    devaddr_t   grpaddr;      // multicast group address
    u1_t        nwkKeyDn[16]; // network session key for down-link
    u1_t        appKey[16];   // application session key
    u4_t        seqnoADn;     // down stream seqno (AFCntDown)
} session_t;

// Compactified duty cycle/dwell time relative to baseAvail
// To avoid roll over this needs to be updated.
typedef u1_t avail_t;

#define MAX_MULTICAST_SESSIONS LCE_MCGRP_MAX

#define CHMAP_SZ (MAX_FIX_CHNLS+15)/16

struct lmic_t {
    // Radio settings TX/RX (also accessed by HAL)
    ostime_t    txend;
    ostime_t    rxtime;  // timestamp when frame was fully received
    ostime_t    rxtime0; // timestamp when preamble of frame was received (computed)
    u4_t        freq;
    s1_t        rssi;
    s1_t        snr;
    rps_t       rps;
    u1_t        rxsyms;
    u1_t        dndr;
    s1_t        txpow;     // dBm -- needs to be combined with txPowAdj

    osjob_t     osjob;

    osxtime_t   baseAvail;                      // base time for availability
    u1_t        globalAvail;                    // next available DC (global)
    u1_t        noDC;                           // disable all duty cycle

    const region_t* region;
    union {
#ifdef REG_DYN
        // ETSI-like (dynamic channels)
        struct {
            avail_t     bandAvail[MAX_BANDS];   // next available DC (per band)
            avail_t     chAvail[MAX_DYN_CHNLS]; // next available DC (per channel)
            freq_t      chUpFreq[MAX_DYN_CHNLS];// uplink frequency
            freq_t      chDnFreq[MAX_DYN_CHNLS];// downlink frequency
            drmap_t     chDrMap[MAX_DYN_CHNLS]; // enabled data rates

            u2_t        channelMap;             // active channels
        } dyn;
#endif
#ifdef REG_FIX
        // FCC-like (fixed channels)
        struct {
            u2_t        channelMap[CHMAP_SZ];           // enabled bits
            u1_t        hoplist[MAX_FIX_CHNLS_125];     // hoplist
        } fix;
#endif
    };

    u1_t        refChnl;         // channel randomizer - search relative to this indicator
    u1_t        txChnl;          // channel for next TX
    u1_t        globalDutyRate;  // max rate: 1/2^k
    ostime_t    globalDutyAvail; // time device can send again  -- XXX:PROBLEM if no TX for ~18h we have a rollover here!! --> avail_t??

    u4_t        netid;        // current network id (~0 - none)
    u2_t        opmode;
    u1_t        clmode;       // current/pending class A/B/C
    u1_t        pollcnt;      // >0 waiting for an answer from network
    u1_t        nbTrans;      // ADR controlled frame repetition
    s1_t        txPowAdj;     // adjustment for txpow (ADR controlled)
    dr_t        datarate;     // current data rate
    u1_t        errcr;        // error coding rate (used for TX only)
    u1_t        rejoinCnt;    // adjustment for rejoin datarate
    s2_t        drift;        // last measured drift
    s2_t        lastDriftDiff;
    s2_t        maxDriftDiff;
    osxtime_t   gpsEpochOff;  // gpstime = gpsEpochOff+getXTime(), 0=undefined
    s4_t        rxdErrs[RXDERR_NUM];
    u1_t        rxdErrIdx;

    u1_t        pendTxPort;
    u1_t        pendTxConf;   // confirmed data
    u1_t        pendTxLen;    // +0x80 = confirmed
    u1_t        pendTxData[MAX_LEN_PAYLOAD];
    u1_t        pendTxNoRx;   // don't listen for down data after tx

    u2_t        devNonce;     // last generated nonce
    lce_ctx_t   lceCtx;
    devaddr_t   devaddr;
    u4_t        seqnoDn;      // device level down stream seqno
#if defined(CFG_lorawan11)
    u4_t        seqnoADn;     // device level down stream seqno (AFCntDown)
#endif
    u4_t        seqnoUp;

    u1_t        dnConf;       // dn frame confirm pending: LORA::FCT_ACK or 0
    s4_t        adrAckReq;    // counter until we reset data rate (0x80000000=off)
    u4_t        adrAckLimit;  // ADR_ACK_LIMIT
    u4_t        adrAckDelay;  // ADR_ACK_DELAY

    u1_t        margin;       // bits 7/6:RFU, 0-5: SNR of last DevStatusReq frame, reported by DevStatusAns to network
    u1_t        gwmargin;     // last reported by network via LinkCheckAns
    u1_t        gwcnt;        //  - ditto -
    u1_t        foptsUpLen;
    u1_t        foptsUp[64];  // pending FOpts in up direction - cleared after next send
    bit_t       devsAns;      // device status answer pending
    u1_t        adrEnabled;
    u1_t        moreData;     // NWK has more data pending
    bit_t       dutyCapAns;   // have to ACK duty cycle settings
    //XXX:old: u1_t        snchAns;      // answer set new channel
    u1_t        dn1Dly;       // delay in secs to DNW1
    s1_t        dn1DrOffIdx;  // index into DR offset table (can be negative in some regions!)
    // 2nd RX window (after up stream)
    u1_t        dn2Dr;
    u4_t        dn2Freq;
    u1_t        dn2Ans;       // 0=no answer pend, 0x80+ACKs
    u1_t        dn1DlyAns;    // 0=no answer pend, 0x80 send MCMD_RXTM_ANS
    u1_t        dnfqAns;      // # of DNFQ in this down frame
    u1_t        dnfqAnsPend;  // pending ACK bits (2 each)
    u4_t        dnfqAcks;     // ack bit pending

    // multicast sessions
    session_t  sessions[MAX_MULTICAST_SESSIONS];

#if defined(CFG_lorawan11)
    u1_t        opts;         // negotiated protocol options
#endif

#if !defined(DISABLE_CLASSB)
    // Class B state
    u1_t        missedBcns;   // unable to track last N beacons
    s1_t        askForTime;   // how often to ask for time
    //XXX:old: u1_t        pingSetAns;   // answer set cmd and ACK bits
    rxsched_t   ping;         // pingable setup
#endif

    // Public part of MAC state
    u1_t        txCnt;
    u1_t        txrxFlags;  // transaction flags (TX-RX combo)
    u1_t        dataBeg;    // 0 or start of data (dataBeg-1 is port)
    u1_t        dataLen;    // 0 no data or zero length data, >0 byte count of data
    u1_t        frame[MAX_LEN_FRAME];

#if !defined(DISABLE_CLASSB)
    u1_t        bcnfAns;      // mcmd beacon freq: bit7:pending, bit0:ACK/NACK
    u1_t        bcnChnl;
    u4_t        bcnFreq;      // 0=default, !=0: specific BCN freq/no hopping
    u1_t        bcnRxsyms;    //
    ostime_t    bcnRxtime;
    bcninfo_t   bcninfo;      // Last received beacon info
#endif

    const rf_proto_desc_t  *protocol;
    u1_t        syncword;

    u1_t        noRXIQinversion;

    // automatic sending of MAC uplinks without payload
    ostime_t    polltime;     // time when OP_POLL flag was set
    ostime_t    polltimeout;  // timeout when frame will be sent even without payload (default 0)

#ifdef CFG_testpin
    // Signal specific event via a GPIO pin.
    // Test pin is routed to PPS pin of SX1301 to record time.
    // Current events:
    //  1=txdone, 2=rxend, 3=rxstart
    u1_t        testpinMode;
#endif
};
//! \var struct lmic_t LMIC
//! The state of LMIC MAC layer is encapsulated in this variable.
DECLARE_LMIC; //!< \internal

//! Construct a bit map of allowed datarates from drlo to drhi (both included).
#define DR_RANGE_MAP(drlo,drhi) ((drmap_t) ((0xFFFF<<(drlo)) & (0xFFFF>>(15-(drhi)))))

bit_t LMIC_setupChannel (u1_t channel, freq_t freq, u2_t drmap);
void  LMIC_disableChannel (u1_t channel);

void  LMIC_setDrTxpow   (dr_t dr, s1_t txpow);  // set default/start DR/txpow
void  LMIC_setAdrMode   (bit_t enabled);        // set ADR mode (if mobile turn off)
bit_t LMIC_startJoining (void);

void  LMIC_shutdown     (void);
void  LMIC_init         (void);
void  LMIC_reset        (void);
void  LMIC_reset_ex     (u1_t regionIdx);
int   LMIC_regionIdx    (u1_t regionCode);
void  LMIC_clrTxData    (void);
void  LMIC_setTxData    (void);
int   LMIC_setTxData2   (u1_t port, u1_t* data, u1_t dlen, u1_t confirmed);
void  LMIC_sendAlive    (void);

#if !defined(DISABLE_CLASSB)
u1_t  LMIC_enableTracking  (u1_t tryBcnInfo);
void  LMIC_disableTracking (void);
#endif

void  LMIC_setClassC     (u1_t enabled);
#if !defined(DISABLE_CLASSB)
void  LMIC_stopPingable  (void);
u1_t  LMIC_setPingable   (u1_t intvExp);
#endif
void  LMIC_tryRejoin     (void);

#if !defined(DISABLE_CLASSB)
int  LMIC_scan (ostime_t timeout);
int  LMIC_track (ostime_t when);
#endif
void LMIC_setMultiCastSession (devaddr_t grpaddr, const u1_t* nwkKeyDn, const u1_t* appKey, u4_t seqnoAdn);

void LMIC_setSession (u4_t netid, devaddr_t devaddr, const u1_t* nwkKey,
#if defined(CFG_lorawan11)
        const u1_t* nwkKeyDn,
#endif
        const u1_t* appKey);
void LMIC_setLinkCheckMode (bit_t enabled);
void LMIC_setLinkCheck (u4_t limit, u4_t delay);
void LMIC_askForLinkCheck (void);

dr_t     LMIC_fastestDr (); // fastest UP datarate
dr_t     LMIC_slowestDr (); // slowest UP datarate
rps_t    LMIC_updr2rps (u1_t dr);
rps_t    LMIC_dndr2rps (u1_t dr);
ostime_t LMIC_calcAirTime (rps_t rps, u1_t plen);

// Simulation only APIs
#if defined(CFG_simul)
const char* LMIC_addr2func (void* addr);
int LMIC_arr2len (const char* name);
#endif

// Declare onEvent() function, to make sure any definition will have the
// C conventions, even when in a C++ file.
DECL_ON_LMIC_EVENT;

// Special APIs - for development or testing
// !!!See implementation for caveats!!!
#if defined(CFG_extapi)
void     LMIC_enableFastJoin (void);
void     LMIC_disableDC (void);
ostime_t LMIC_dr2hsym (dr_t dr, s1_t num);
ostime_t LMIC_nextTx (ostime_t now);
void     LMIC_updateTx (ostime_t now);
void     LMIC_getRxdErrInfo (s4_t* skew, u4_t* span);
#endif


// Backtrace service support
#ifdef SVC_backtrace
#include "backtrace/backtrace.h"
#else
#define BACKTRACE()
#define TRACE_VAL(v)
#define TRACE_EV(e)
#define TRACE_ADDR(a)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _lmic_h_

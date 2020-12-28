/*
 * TTNHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../SoftRF.h"
#if defined(ENABLE_TTN)

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "TTNHelper.h"
#include "RFHelper.h"

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x03FF0001 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint8_t coords[9] = {0,0,0,0,0,0,0,0,0};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TTN_TX_INTERVAL = 120000; // TTN fair access policy
unsigned long TTN_TimeMarker = 0;

#define isTimeToTTN() (millis() - TTN_TimeMarker > TTN_TX_INTERVAL)

osjob_t ttn_txjob;
static void ttn_tx_func (osjob_t* job);
static bool ttn_transmit_complete = false;

extern bool sx1276_receive_active;

u1_t saved_datarate = DR_SF7B;
s1_t saved_txpow = 14;
u4_t saved_freq = 868200000UL;
u1_t saved_syncword = 0xF1; // FANET+

/* ------------------ BEGIN OF LMIC CODE ------------------------------------- */

/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

int TTN_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed) {
    if( dlen > SIZEOFEXPR(LMIC.pendTxData) )
        return -2;
    if( data != (xref2u1_t)0 )
        os_copyMem(LMIC.pendTxData, data, dlen);
    LMIC.pendTxConf = confirmed;
    LMIC.pendTxPort = port;
    LMIC.pendTxLen  = dlen;
    LMIC.opmode |= OP_TXDATA;
    if( (LMIC.opmode & OP_JOINING) == 0 )
        LMIC.txCnt = 0;             // cancel any ongoing TX/RX retries
    return 0;
}

static void updateTx (ostime_t txbeg) {
    u4_t freq = LMIC.channelFreq[LMIC.txChnl];
    // Update global/band specific duty cycle stats
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    // Update channel/global duty cycle stats
    xref2band_t band = &LMIC.bands[freq & 0x3];
    LMIC.freq  = freq & ~(u4_t)3;
    LMIC.txpow = band->txpow;
    band->avail = txbeg + airtime * band->txcap;
    if( LMIC.globalDutyRate != 0 )
        LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
}

// ================================================================================
// BEG AES

static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}

static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}

static void aes_cipher (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len) {
    if( len <= 0 )
        return;
    os_clearMem(AESaux, 16);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}

static void buildDataFrame (void) {
    bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    u1_t dlen = txdata ? LMIC.pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
#if !defined(DISABLE_PING)
    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE)) == (OP_TRACK|OP_PINGABLE) ) {
        // Indicate pingability in every UP frame
        LMIC.frame[end] = MCMD_PING_IND;
        LMIC.frame[end+1] = LMIC.ping.dr | (LMIC.ping.intvExp<<4);
        end += 2;
    }
#endif // !DISABLE_PING
#if !defined(DISABLE_MCMD_DCAP_REQ)
    if( LMIC.dutyCapAns ) {
        LMIC.frame[end] = MCMD_DCAP_ANS;
        end += 1;
        LMIC.dutyCapAns = 0;
    }
#endif // !DISABLE_MCMD_DCAP_REQ
#if !defined(DISABLE_MCMD_DN2P_SET)
    if( LMIC.dn2Ans ) {
        LMIC.frame[end+0] = MCMD_DN2P_ANS;
        LMIC.frame[end+1] = LMIC.dn2Ans & ~MCMD_DN2P_ANS_RFU;
        end += 2;
        LMIC.dn2Ans = 0;
    }
#endif // !DISABLE_MCMD_DN2P_SET
    if( LMIC.devsAns ) {  // answer to device status
        LMIC.frame[end+0] = MCMD_DEVS_ANS;
        LMIC.frame[end+1] = os_getBattLevel();
        LMIC.frame[end+2] = LMIC.margin;
        end += 3;
        LMIC.devsAns = 0;
    }
    if( LMIC.ladrAns ) {  // answer to ADR change
        LMIC.frame[end+0] = MCMD_LADR_ANS;
        LMIC.frame[end+1] = LMIC.ladrAns & ~MCMD_LADR_ANS_RFU;
        end += 2;
        LMIC.ladrAns = 0;
    }
#if !defined(DISABLE_BEACONS)
    if( LMIC.bcninfoTries > 0 ) {
        LMIC.frame[end] = MCMD_BCNI_REQ;
        end += 1;
    }
#endif // !DISABLE_BEACONS
    if( LMIC.adrChanged ) {
        if( LMIC.adrAckReq < 0 )
            LMIC.adrAckReq = 0;
        LMIC.adrChanged = 0;
    }
#if !defined(DISABLE_MCMD_PING_SET) && !defined(DISABLE_PING)
    if( LMIC.pingSetAns != 0 ) {
        LMIC.frame[end+0] = MCMD_PING_ANS;
        LMIC.frame[end+1] = LMIC.pingSetAns & ~MCMD_PING_ANS_RFU;
        end += 2;
        LMIC.pingSetAns = 0;
    }
#endif // !DISABLE_MCMD_PING_SET && !DISABLE_PING
#if !defined(DISABLE_MCMD_SNCH_REQ)
    if( LMIC.snchAns ) {
        LMIC.frame[end+0] = MCMD_SNCH_ANS;
        LMIC.frame[end+1] = LMIC.snchAns & ~MCMD_SNCH_ANS_RFU;
        end += 2;
        LMIC.snchAns = 0;
    }
#endif // !DISABLE_MCMD_SNCH_REQ
    ASSERT(end <= OFF_DAT_OPTS+16);

    u1_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
        // Options and payload too big - delay payload
        txdata = 0;
        flen = end+4;
    }
    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | LMIC.adrEnabled
                              | (LMIC.adrAckReq >= 0 ? FCT_ADRARQ : 0)
                              | (end-OFF_DAT_OPTS));
    os_wlsbf4(LMIC.frame+OFF_DAT_ADDR,  LMIC.devaddr);

    if( LMIC.txCnt == 0 ) {
        LMIC.seqnoUp += 1;
        DO_DEVDB(LMIC.seqnoUp,seqnoUp);
    } else {
        EV(devCond, INFO, (e_.reason = EV::devCond_t::RE_TX,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = LMIC.seqnoUp-1,
                           e_.info2  = ((LMIC.txCnt+1) |
                                        (TABLE_GET_U1(DRADJUST, LMIC.txCnt+1) << 8) |
                                        ((LMIC.datarate|DR_PAGE)<<16))));
    }
    os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, LMIC.seqnoUp-1);

    // Clear pending DN confirmation
    LMIC.dnConf = 0;

    if( txdata ) {
        if( LMIC.pendTxConf ) {
            // Confirmed only makes sense if we have a payload (or at least a port)
            LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
            if( LMIC.txCnt == 0 ) LMIC.txCnt = 1;
        }
        LMIC.frame[end] = LMIC.pendTxPort;
        os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
        aes_cipher(LMIC.pendTxPort==0 ? LMIC.nwkKey : LMIC.artKey,
                   LMIC.devaddr, LMIC.seqnoUp-1,
                   /*up*/0, LMIC.frame+end+1, dlen);
    }
    aes_appendMic(LMIC.nwkKey, LMIC.devaddr, LMIC.seqnoUp-1, /*up*/0, LMIC.frame, flen-4);

    EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                       e_.devaddr = LMIC.devaddr,
                       e_.seqno   = LMIC.seqnoUp-1,
                       e_.flags   = (LMIC.pendTxPort < 0 ? EV::dfinfo_t::NOPORT : EV::dfinfo_t::NOP),
                       e_.mic     = Base::lsbf4(&LMIC.frame[flen-4]),
                       e_.hdr     = LMIC.frame[LORA::OFF_DAT_HDR],
                       e_.fct     = LMIC.frame[LORA::OFF_DAT_FCT],
                       e_.port    = LMIC.pendTxPort,
                       e_.plen    = txdata ? dlen : 0,
                       e_.opts.length = end-LORA::OFF_DAT_OPTS,
                       memcpy(&e_.opts[0], LMIC.frame+LORA::OFF_DAT_OPTS, end-LORA::OFF_DAT_OPTS)));
    LMIC.dataLen = flen;
}

/* ------------------ END OF LMIC CODE ------------------------------------- */
  
void ttn_transmit()
{
    ttn_transmit_complete = false;
    sx1276_receive_active = false;

    os_setCallback(&ttn_txjob, ttn_tx_func);

    while (ttn_transmit_complete == false) {
      // execute scheduled jobs and events
      os_runloop_once();
      yield();
    };
}

void ttn_tx(osjobcb_t func) {

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        os_radio(RADIO_RST); // Stop RX first
        delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

        // Prepare upstream data transmission at the next possible time.
        TTN_setTxData2(2, (uint8_t*) coords, sizeof(coords), 0);

        // Check for ongoing state: scan or TX/RX transaction
        if( (LMIC.opmode & (OP_SCAN|OP_TXRXPEND|OP_SHUTDOWN)) != 0 )
            return;

        ostime_t now    = os_getTime();
        ostime_t rxtime = 0;
        ostime_t txbeg  = 0;

        saved_datarate  = LMIC.datarate;
        saved_txpow     = LMIC.txpow;
        saved_freq      = LMIC.freq;
        saved_syncword  = LMIC.syncword;
  
        LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  
        // Disable link check validation
        LMIC_setLinkCheckMode(0);
    
        // TTN uses SF9 for its RX2 window.
        LMIC.dn2Dr = DR_SF9;
   
        // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
        LMIC_setDrTxpow(DR_SF7,14);

        LMIC.syncword = 0x34;

        // We could send right now!
        txbeg = now;
        dr_t txdr = (dr_t)LMIC.datarate;

        buildDataFrame();
        LMIC.osjob.func = func;

        LMIC.rps    = setCr(updr2rps(txdr), (cr_t)LMIC.errcr);
        LMIC.dndr   = txdr;  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
        LMIC.opmode = (LMIC.opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
        updateTx(txbeg);

        os_radio(RADIO_TX);

        //Serial.println(F("TX"));
    }
}

static void ttn_txdone_func (osjob_t* job) {

  LMIC.datarate         = saved_datarate;
  LMIC.txpow            = saved_txpow;
  LMIC.freq             = saved_freq;
  LMIC.syncword         = saved_syncword;

  LMIC.opmode           &= ~(OP_TXDATA|OP_TXRXPEND);

  ttn_transmit_complete = true;
}

static void ttn_tx_func (osjob_t* job) {
  ttn_tx(ttn_txdone_func);
}

void TTN_setup()
{
  if (rf_chip && (rf_chip->type == RF_IC_SX1276)) {

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
  
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  }
}

void TTN_loop()
{
  if (rf_chip && (rf_chip->type == RF_IC_SX1276)) {

    if (isTimeToTTN() && (ThisAircraft.latitude || ThisAircraft.longitude) ) {

      int32_t lat         = ThisAircraft.latitude * 10000;
      int32_t lon         = ThisAircraft.longitude * 10000;
      int16_t altitudeGPS = ThisAircraft.altitude;
      int8_t hdopGPS      = ThisAircraft.hdop; 
    
      // Pad 2 int32_t to 6 8uint_t, big endian (24 bit each, having 11 meter precision)
      coords[0] = lat;
      coords[1] = lat >> 8;
      coords[2] = lat >> 16;
    
      coords[3] = lon;
      coords[4] = lon >> 8;
      coords[5] = lon >> 16;
    
      coords[6] = altitudeGPS;
      coords[7] = altitudeGPS >> 8;
    
      coords[8] = hdopGPS;

      ttn_transmit();

      TTN_TimeMarker = millis();
    }
  }
}

#endif /* ENABLE_TTN */

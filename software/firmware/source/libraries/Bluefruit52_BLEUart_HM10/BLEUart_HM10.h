/**************************************************************************/
/*!
    @file       BLEUart_HM10.h
    @author     hathach (tinyusb.org)
    @co-author  Linar Yusupov ( made it HM-10 compatible )

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef BLEUART_HM10_H_
#define BLEUART_HM10_H_

#include "bluefruit_common.h"
#include "utility/adafruit_fifo.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

#define BLE_UART_HM10_DEFAULT_RX_FIFO_DEPTH   256
#define BLE_UART_HM10_DEFAULT_TX_FIFO_DEPTH   1024

#define BLE_MAX_WRITE_CHUNK_SIZE              20

extern const uint8_t BLEUART_HM10_UUID_SERVICE[];
extern const uint8_t BLEUART_HM10_UUID_CHR_RW[];

class BLEUart_HM10 : public BLEService, public Stream
{
  public:
    typedef void (*rx_callback_t) (uint16_t conn_hdl);
    typedef void (*notify_callback_t)(uint16_t conn_hdl, bool enabled);
    typedef void (*rx_overflow_callback_t) (uint16_t conn_hdl, uint16_t leftover);

    BLEUart_HM10(uint16_t rx_fifo_depth = BLE_UART_HM10_DEFAULT_RX_FIFO_DEPTH,
                 uint16_t tx_fifo_depth = BLE_UART_HM10_DEFAULT_TX_FIFO_DEPTH);
    virtual ~BLEUart_HM10();

    virtual err_t begin(void);

    bool notifyEnabled(void);
    bool notifyEnabled(uint16_t conn_hdl);

    void setRxCallback (rx_callback_t fp, bool deferred = true);
    void setRxOverflowCallback(rx_overflow_callback_t fp);
    void setNotifyCallback(notify_callback_t fp);

    bool flushTXD (void);
    bool flushTXD (uint16_t conn_hdl);

    // Read helper
    uint8_t  read8 (void);

    // Stream API
    virtual int       read       ( void );
    virtual int       read       ( uint8_t * buf, size_t size );
            int       read       ( char    * buf, size_t size ) { return read( (uint8_t*) buf, size); }

    virtual size_t    write      ( uint8_t b );
    virtual size_t    write      ( const uint8_t *content, size_t len);

    virtual size_t    write      (uint16_t conn_hdl, uint8_t b);
    virtual size_t    write      (uint16_t conn_hdl, const uint8_t *content, size_t len);

    virtual int       available  ( void );
    virtual int       peek       ( void );
    virtual void      flush      ( void );

    // pull in write(str) and write(buf, size) from Print
    using Print::write;

  protected:
    BLECharacteristic _rtxd;

    // RXD
    Adafruit_FIFO* _rx_fifo;
    uint16_t       _rx_fifo_depth;

    // TXD
    Adafruit_FIFO* _tx_fifo;
    uint16_t       _tx_fifo_depth;

    // Callbacks
    rx_callback_t           _rx_cb;
    notify_callback_t       _notify_cb;
    rx_overflow_callback_t  _overflow_cb;

    // Static Method for callbacks
    static void bleuart_rxd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
    static void bleuart_txd_cccd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value);
};

#endif /* BLEUART_HM10_H_ */

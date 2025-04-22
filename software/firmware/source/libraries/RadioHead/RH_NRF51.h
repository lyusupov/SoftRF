// RH_NRF51.h
// Author: Mike McCauley
// Copyright (C) 2015 Mike McCauley
// $Id: RH_NRF51.h,v 1.5 2017/07/25 05:26:50 mikem Exp $
//

#ifndef RH_NRF51_h
#define RH_NRF51_h

#include <RHGenericDriver.h>

// This is the maximum number of bytes that can be carried by the nRF51.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_NRF51_MAX_PAYLOAD_LEN 254

// The length of the headers we add.
// The headers are inside the nRF51 payload
// We add:
// S0 (not used)
// LEN
// S1 (not used)
// to
// from
// id
// flags
#define RH_NRF51_HEADER_LEN 7

// This is the maximum RadioHead user message length that can be supported by this library. Limited by
// the supported message lengths in the nRF51
#define RH_NRF51_MAX_MESSAGE_LEN (RH_NRF51_MAX_PAYLOAD_LEN-RH_NRF51_HEADER_LEN)

// Define to be 1 if you want to support AES CCA encryption using the built-in
// encryption engine.
#define RH_NRF51_HAVE_ENCRYPTION 1

// When encryption is enabled, have a much shorter max message length
#define RH_NRF51_MAX_ENCRYPTED_MESSAGE_LEN (27-4)

// The required length of the AES encryption key
#define RH_NRF51_ENCRYPTION_KEY_LENGTH 16

// This is the size of the CCM data structure for AES encryption
// REVISIT: use a struct?
#define RH_NRF51_AES_CCM_CNF_SIZE 33

/////////////////////////////////////////////////////////////////////
/// \class RH_NRF51 RH_NRF51.h <RH_NRF51.h>
/// \brief Send and receive unaddressed, unreliable datagrams by nRF51 and nRF52 compatible transceivers.
///
/// Supported transceivers include:
/// - Nordic nRF51 based 2.4GHz radio modules, such as nRF51822 
/// and other compatible chips, such as used in RedBearLabs devices like:
/// http://store.redbearlab.com/products/redbearlab-nrf51822
/// http://store.redbearlab.com/products/blenano
/// and
/// Sparkfun nRF52832 breakout board, with Arduino 1.6.13 and
/// Sparkfun nRF52 boards manager 0.2.3
///
/// This base class provides basic functions for sending and receiving unaddressed, unreliable datagrams
/// of arbitrary length to 254 octets per packet. Use one of the Manager classes to get addressing and 
/// acknowledgement reliability, routing, meshes etc.
///
/// The nRF51822 (https://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF51822)
/// and nRF52832 (https://learn.sparkfun.com/tutorials/nrf52832-breakout-board-hookup-guide)
/// is a complete SoC (system on a chip) with ARM microprocessor and 2.4 GHz radio, which supports a range of channels 
/// and transmission bit rates. Chip antenna is on-board.
///
/// This library provides functions for sending and receiving messages of up to 254 octets on any 
/// frequency supported by the nRF51822/nRF52832, at a selected data rate.
///
/// The nRF51 transceiver is configured to use Enhanced Shockburst with no acknowledgement and no retransmits.
/// TXADDRESS and RXADDRESSES:RXADDR0 (ie pipe 0) are the logical address used. The on-air network address
/// is set in BASE0 and PREFIX0. SHORTS is used to automatically transition the radio between Ready, Start and Disable.
/// No interrupts are used.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// data rate, and with identical network addresses.
///
/// Example programs are included to show the main modes of use.
///
/// \par Packet Format
///
/// All messages sent and received by this class conform to this packet format. It is NOT compatible
/// with the one used by RH_NRF24 and the nRF24L01 product specification, mainly because the nRF24 only supports
/// 6 bits of message length.
///
/// - 1 octets PREAMBLE
/// - 3 to 5 octets NETWORK ADDRESS
/// - 1 octet S0 (not used, required if encryption used)
/// - 8 bits PAYLOAD LENGTH
/// - 1 octet S1 (not used, required if encryption used)
/// - 0 to 251 octets PAYLOAD (possibly encrypted), consisting of:
///   - 1 octet TO header
///   - 1 octet FROM header
///   - 1 octet ID header
///   - 1 octet FLAGS header
///   - 0 to 247 octets of user message
/// - 2 octets CRC (Algorithm x^16+x^12^x^5+1 with initial value 0xFFFF).
///
/// \par Example programs
///
/// Several example programs are provided.
///
/// The sample programs are designed to be built using Arduino 1.6.4 or later using the procedures outlined
/// in http://redbearlab.com/getting-started-nrf51822/
/// or with Sparkfun nRF52832 breakout board, with Arduino 1.6.13 and
/// Sparkfun nRF52 boards manager 0.2.3 using the procedures outlined in
/// https://learn.sparkfun.com/tutorials/nrf52832-breakout-board-hookup-guide
///
/// \par Radio Performance
///
/// At DataRate2Mbps (2Mb/s), payload length vs airtime:
/// 0 bytes takes about 70us, 128 bytes takes 520us, 254 bytes take 1020us.
/// You can extrapolate linearly to slower data rates.
///
/// The RF powers claimed by the chip manufacturer have not been independently verified here.
///
/// \par Memory
///
/// The compiled client and server sketches are about 42k bytes on Arduino. 
/// The reliable client and server sketches compile to about 43k bytes on Arduino. Unfortunately the 
/// Arduino build environmnet does not drop unused clsses and code, so the resulting programs include
/// all the unused classes ad code. This needs to be revisited.
/// RAM requirements are minimal.
///
class RH_NRF51 : public RHGenericDriver
{
public:

    /// \brief Defines convenient values for setting data rates in setRF()
    typedef enum
    {
	DataRate1Mbps = 0,   ///< 1 Mbps
	DataRate2Mbps,       ///< 2 Mbps
	DataRate250kbps      ///< 250 kbps
    } DataRate;

    /// \brief Convenient values for setting transmitter power in setRF()
    typedef enum
    {
	// Add 20dBm for nRF24L01p with PA and LNA modules
	TransmitPower4dBm = 0,        ///<  4 dBm
	TransmitPower0dBm,            ///<  0 dBm
	TransmitPowerm4dBm,           ///< -4 dBm
	TransmitPowerm8dBm,           ///< -8 dBm
	TransmitPowerm12dBm,          ///< -12 dBm
	TransmitPowerm16dBm,          ///< -16 dBm
	TransmitPowerm20dBm,          ///< -20 dBm
	TransmitPowerm30dBm,          ///< -30 dBm
    } TransmitPower;

    /// Constructor.
    /// After constructing, you must call init() to initialise the interface
    /// and the radio module
    RH_NRF51();
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:
    /// - Start the processors High Frequency clock  DC/DC converter and 
    /// - Disable and reset the radio
    /// - Set the logical channel to 0 for transmit and receive (only pipe 0 is used)
    /// - Configure the CRC (2 octets, algorithm x^16+x^12^x^5+1 with initial value 0xffff)
    /// - Set the default network address of 0xE7E7E7E7E7
    /// - Set channel to 2
    /// - Set data rate to DataRate2Mbps
    /// - Set TX power to TransmitPower0dBm
    /// \return  true if everything was successful
    bool        init();

    /// Sets the transmit and receive channel number.
    /// The frequency used is (2400 + channel) MHz
    /// \return true on success
    bool setChannel(uint8_t channel);

    /// Sets the Network address.
    /// Only nodes with the same network address can communicate with each other. You 
    /// can set different network addresses in different sets of nodes to isolate them from each other.
    /// Internally, this sets the nRF51 BASE0 and PREFIX0 to be the given network address.
    /// The first octet of the address is used for PREFIX0 and the rest is used for BASE0. BALEN is
    /// set to the approprtae base length.
    /// The default network address is 0xE7E7E7E7E7.
    /// \param[in] address The new network address. Must match the network address of any receiving node(s).
    /// \param[in] len Number of bytes of address to set (3 to 5).
    /// \return true on success, false if len is not in the range 3-5 inclusive.
    bool setNetworkAddress(uint8_t* address, uint8_t len);

    /// Sets the data rate and transmitter power to use.
    /// \param [in] data_rate The data rate to use for all packets transmitted and received. One of RH_NRF51::DataRate.
    /// \param [in] power Transmitter power. One of RH_NRF51::TransmitPower.
    /// \return true on success
    bool setRF(DataRate data_rate, TransmitPower power);

    /// Sets the radio in power down mode, with the configuration set to the
    /// last value from setOpMode().
    /// Sets chip enable to LOW.
    void setModeIdle();

    /// Sets the radio in RX mode.
    void setModeRx();

    /// Sets the radio in TX mode.
    void setModeTx();

    /// Sends data to the address set by setTransmitAddress()
    /// Sets the radio to TX mode.
    /// Caution: when encryption is enabled, the maximum message length is reduced to 23 octets.
    /// \param [in] data Data bytes to send.
    /// \param [in] len Number of data bytes to send
    /// \return true on success (which does not necessarily mean the receiver got the message, only that the message was
    /// successfully transmitted). False if the message is too long or was otherwise not transmitted.
    bool send(const uint8_t* data, uint8_t len);

    /// Blocks until the current message (if any) 
    /// has been transmitted
    /// \return true on success, false if the chip is not in transmit mode or other transmit failure
    virtual bool waitPacketSent();

    /// Indicates if the chip is in transmit mode and 
    /// there is a packet currently being transmitted
    /// \return true if the chip is in transmit mode and there is a transmission in progress
    bool isSending();

    /// Prints the value of all NRF_RADIO registers.
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// Caution: there are 1024 of them (many reserved and set to 0).
    /// \return true on success
    bool printRegisters();

    /// Checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool        available();

    /// Turns the receiver on if it not already on.
    /// Once a message with CRC correct is received, the receiver will be returned to Idle mode.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool        recv(uint8_t* buf, uint8_t* len);

    /// Enables AES encryption and sets the AES encryption key, used
    /// to encrypt and decrypt all messages using the on-chip AES CCM mode encryption engine. 
    /// The default is disabled.
    /// In the AES configuration, the message counter and IV is always set to 0, which
    /// means the same keystream is used for every message with a given key.
    /// Caution: when encryption is enabled, the maximum message length is reduced to 23 octets.
    /// \param[in] key The key to use. Must be 16 bytes long. The same key must be installed
    /// in other instances of RH_RF51, otherwise communications will not work correctly. If key is NULL,
    /// encryption is disabled, which is the default.
    void           setEncryptionKey(uint8_t* key = NULL);

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

    /// Reeads the current die temperature using the built in TEMP peripheral.
    /// Blocks while the temperature is measured, which takes about 30 microseconds.
    // \return the current die temperature in degrees C.
    float get_temperature();

protected:
    /// Examine the receive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

private:
    /// The receiver/transmitter buffer
    /// First octet is the payload length, remainder is the payload
    uint8_t             _buf[RH_NRF51_MAX_PAYLOAD_LEN+1];

    /// True when there is a valid message in the buffer
    bool                _rxBufValid;

#if RH_NRF51_HAVE_ENCRYPTION
    /// True if an AES key has been specified and that we are therfore encrypting
    /// and decrypting messages on the fly
    bool                _encrypting;

    /// Scratch area for AES encryption
    uint8_t             _scratch[RH_NRF51_MAX_PAYLOAD_LEN+1+16];

    /// Where the AES encryption key and IV are stored
    uint8_t             _encryption_cnf[RH_NRF51_AES_CCM_CNF_SIZE];
#endif
};

/// @example nrf51_client.ino
/// @example nrf51_server.ino
/// @example nrf51_reliable_datagram_client.ino
/// @example nrf51_reliable_datagram_server.ino
/// @example nrf51_audio_tx.ino
/// @example nrf51_audio_rx.ino
#endif 

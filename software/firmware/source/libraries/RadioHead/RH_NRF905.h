// RH_NRF905.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_NRF905.h,v 1.11 2017/07/25 05:26:50 mikem Exp $
//

#ifndef RH_NRF905_h
#define RH_NRF905_h

#include <RHGenericSPI.h>
#include <RHNRFSPIDriver.h>

// This is the maximum (and only) number of bytes that can be carried by the nRF905.
// We use some for headers, leaving fewer for RadioHead messages
#define RH_NRF905_MAX_PAYLOAD_LEN 32

// The length of the headers we add.
// The headers are inside the nRF905 payload
// As well as the usual TO, FROM, ID, FLAGS, we also need LEN, since
// nRF905 only has fixed width messages.
// REVISIT: could we have put the LEN into the FLAGS field?
#define RH_NRF905_HEADER_LEN 5

// This is the maximum RadioHead user message length that can be supported by this library. Limited by
// the supported message lengths in the nRF905
#define RH_NRF905_MAX_MESSAGE_LEN (RH_NRF905_MAX_PAYLOAD_LEN-RH_NRF905_HEADER_LEN)

// Register names
#define RH_NRF905_REG_MASK                   0x0f
#define RH_NRF905_REG_W_CONFIG               0x00
#define RH_NRF905_REG_R_CONFIG               0x10
#define RH_NRF905_REG_W_TX_PAYLOAD           0x20
#define RH_NRF905_REG_R_TX_PAYLOAD           0x21
#define RH_NRF905_REG_W_TX_ADDRESS           0x22
#define RH_NRF905_REG_R_TX_ADDRESS           0x23
#define RH_NRF905_REG_R_RX_PAYLOAD           0x24
#define RH_NRF905_REG_CHANNEL_CONFIG         0x80

// Configuration register
#define RH_NRF905_CONFIG_0                    0x00
#define RH_NRF905_CONFIG_0_CH_NO              0xff

#define RH_NRF905_CONFIG_1                    0x01
#define RH_NRF905_CONFIG_1_AUTO_RETRAN        0x20
#define RH_NRF905_CONFIG_1_RX_RED_PWR         0x10
#define RH_NRF905_CONFIG_1_PA_PWR             0x0c
#define RH_NRF905_CONFIG_1_PA_PWR_N10DBM      0x00
#define RH_NRF905_CONFIG_1_PA_PWR_N2DBM       0x04
#define RH_NRF905_CONFIG_1_PA_PWR_6DBM        0x08
#define RH_NRF905_CONFIG_1_PA_PWR_10DBM       0x0c
#define RH_NRF905_CONFIG_1_HFREQ_PLL          0x02
#define RH_NRF905_CONFIG_1_CH_NO              0x01

#define RH_NRF905_CONFIG_2                    0x02
#define RH_NRF905_CONFIG_2_TX_AFW             0x70
#define RH_NRF905_CONFIG_2_RX_AFW             0x07

#define RH_NRF905_CONFIG_3                    0x03
#define RH_NRF905_CONFIG_3_RX_PW              0x3f

#define RH_NRF905_CONFIG_4                    0x04
#define RH_NRF905_CONFIG_4_TX_PW              0x3f

#define RH_NRF905_CONFIG_5                    0x05
#define RH_NRF905_CONFIG_5_RX_ADDRESS         0xff

#define RH_NRF905_CONFIG_6                    0x06
#define RH_NRF905_CONFIG_6_RX_ADDRESS         0xff

#define RH_NRF905_CONFIG_7                    0x07
#define RH_NRF905_CONFIG_7_RX_ADDRESS         0xff

#define RH_NRF905_CONFIG_8                    0x08
#define RH_NRF905_CONFIG_8_RX_ADDRESS         0xff

#define RH_NRF905_CONFIG_9                    0x09
#define RH_NRF905_CONFIG_9_CRC_MODE_16BIT     0x80
#define RH_NRF905_CONFIG_9_CRC_EN             0x40
#define RH_NRF905_CONFIG_9_XOF                0x38
#define RH_NRF905_CONFIG_9_XOF_4MHZ           0x00
#define RH_NRF905_CONFIG_9_XOF_8MHZ           0x08
#define RH_NRF905_CONFIG_9_XOF_12MHZ          0x10
#define RH_NRF905_CONFIG_9_XOF_16MHZ          0x18
#define RH_NRF905_CONFIG_9_XOF_20MHZ          0x20
#define RH_NRF905_CONFIG_9_UP_CLK_EN          0x04
#define RH_NRF905_CONFIG_9_UP_CLK_FREQ        0x03
#define RH_NRF905_CONFIG_9_UP_CLK_FREQ_4MHZ   0x00
#define RH_NRF905_CONFIG_9_UP_CLK_FREQ_2MHZ   0x01
#define RH_NRF905_CONFIG_9_UP_CLK_FREQ_1MHZ   0x02
#define RH_NRF905_CONFIG_9_UP_CLK_FREQ_500KHZ 0x03

// Status register is always read as first byte
#define RH_NRF905_STATUS_AM                   0x80
#define RH_NRF905_STATUS_DR                   0x20

/////////////////////////////////////////////////////////////////////
/// \class RH_NRF905 RH_NRF905.h <RH_NRF905.h>
/// \brief Send and receive unaddressed, unreliable datagrams by nRF905 and compatible transceivers.
///
/// This base class provides basic functions for sending and receiving unaddressed, unreliable datagrams
/// of arbitrary length to 28 octets per packet. Use one of the Manager classes to get addressing and 
/// acknowledgement reliability, routing, meshes etc.
///
/// The nRF905 transceiver is configured to use Enhanced Shockburst with 16 Bit CRC, and 32 octet packets.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency
/// and with identical network addresses.
///
/// The nRF905 from Nordic Semiconductor http://www.nordicsemi.com/eng/Products/Sub-1-GHz-RF/nRF905
/// (http://www.nordicsemi.com/jpn/nordic/content_download/2452/29528/file/Product_Specification_nRF905_v1.5.pdf)
/// is a low-cost 433/868/915 MHz ISM transceiver module. It supports a number of channel frequencies at
/// 100kHz deviation and 50kHz bandwidth with Manchester encoding.
///
/// We tested with inexpensive nRF905 modules from eBay, similar to:
/// http://www.aliexpress.com/store/product/Free-ship-NRF905-433MHz-Wireless-Transmission-Module-Transceiver-Module-with-Antenna-for-the-433MHz-ISM-band/513046_607163305.html
///
/// This library provides functions for sending and receiving messages of up to 27 octets on any 
/// frequency supported by the nRF905.
///
/// Several nRF905 modules can be connected to an Arduino, permitting the construction of translators
/// and frequency changers, etc.
///
/// Example Arduino programs are included to show the main modes of use.
///
/// \par Packet Format
///
/// All messages sent and received by this class conform to this fixed length packet format
///
/// - 4 octets NETWORK ADDRESS
/// - 32 octets PAYLOAD, consisting of:
///   - 1 octet TO header
///   - 1 octet FROM header
///   - 1 octet ID header
///   - 1 octet FLAGS header
///   - 1 octet user message length header
///   - 0 to 27 octets of user message, trailing octets after the user message length are ignored
/// - 2 octets CRC 
///
/// All messages sent and received by this driver are 32 octets. The user message length is embedded in the message.
///
/// \par Connecting nRF905
///
/// The nRF905 is a 3.3V part is is *NOT* 5V tolerant. So you MUST use a 3.3V CPU such as Teensy, Arduino Due etc
/// or else provide for level shifters between the CPU and the nRF905. Failure to consider this will probably
/// break your nRF905.
///
/// The electrical connection between the nRF905 and the CPU require 3.3V, the 3 x SPI pins (SCK, SDI, SDO), 
/// a Chip Enable pin, a Transmit Enable pin and a Slave Select pin.
///
/// The examples below assume the commonly found cheap Chinese nRF905 modules. The RH_RF905 driver assumes the 
/// the nRF905 has a 16MHz crystal.
///
/// Connect the nRF905 to Teensy (or Arduino with suitable level shifters) like this
/// \code
///                 CPU          nRF905 module
///                 3V3----------VCC   (3.3V)
///             pin D8-----------CE    (chip enable in)
///             pin D9-----------TX_EN (transmit enable in)
///          SS pin D10----------CSN   (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------MOSI  (SPI Data in)
///        MISO pin D12----------MISO  (SPI data out)
///                 GND----------GND   (ground in)
/// \endcode
///
/// Caution: Arduino Due is a 3.3V part and is not 5V tolerant (so too is the nRF905 module
/// so they can be connected directly together. Unlike other Arduinos the Due has it default SPI 
/// connections on a dedicated 6 pin SPI header in the center of the board, which is 
/// physically compatible with Uno, Leonardo and Mega2560. A little dot marks pin 1 on the header.
/// You must connect to these
/// and *not* to the usual Arduino SPI pins Digital 11, 12 and 13.
/// See http://21stdigitalhome.blogspot.com.au/2013/02/arduino-due-hardware-spi.html
///
/// Connect the nRF905 to Arduino Due like this
/// \code
///                      CPU          nRF905 module
///                      3V3----------VCC   (3.3V)
///                  pin D8-----------CE    (chip enable in)
///                  pin D9-----------TX_EN (transmit enable in)
///               SS pin D10----------CSN   (chip select in)
///  SCK on SPI header pin 3----------SCK   (SPI clock in)
/// MOSI on SPI header pin 4----------MOSI  (SPI Data in)
/// MISO on SPI header pin 1----------MISO  (SPI data out)
///                      GND----------GND   (ground in)
/// \endcode
///
/// and you can then use the default constructor RH_NRF905(). 
/// You can override the default settings for the CE, TX_EN and CSN pins 
/// in the NRF905() constructor if you wish to connect the slave select CSN to other than the normal one for your 
/// CPU.
///
/// It is possible to have 2 radios conected to one CPU, provided each radio has its own 
/// CSN, TX_EN and CE line (SCK, MOSI and MISO are common to both radios)
///
/// \par Transmitter Power
///
/// You can control the transmitter power to be one of 4 power levels: -10, -2, 6 or 10dBm,
/// using the setRF() function, eg:
/// \code
/// nrf905.setRF(RH_NRF905::TransmitPower10dBm);
/// \endcode
///
/// We have made some actual power measurements against
/// programmed power for an nRF905 module from www.rfinchina.com under the following conditions:
/// - Teensy 3.1
/// - nRF905 module (with SMA antenna connector) wired to Teensy as described above, channel 108.
/// - 20cm SMA-SMA cable
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Tektronix TDS220 scope to measure the Vout from power head
/// \code
/// Program power           Measured Power
///    dBm                         dBm
///    -10                        -16
///    -2                         -8
///    6                           0
///    10                          8
/// \endcode
/// (Caution: we dont claim laboratory accuracy for these measurements)
/// You would not expect to get anywhere near these powers to air with a simple 1/4 wavelength wire antenna.
///
/// \par Example programs
///
/// Several example programs are provided. They work out of the box with Teensy 3.1 and Arduino Due 
/// connected as show above.
///
/// \par Radio Performance
///
/// Frequency accuracy may be debatable.
/// 
/// \par Memory
///
/// Memory usage of this class is minimal. The compiled client and server sketches are about 16000 bytes on Teensy. 
///
class RH_NRF905 : public RHNRFSPIDriver
{
public:
    /// \brief Convenient values for setting transmitter power in setRF()
    /// These are designed to agree with the values for RH_NRF905_CONFIG_1_PA_PWR after
    /// left shifting by 2
    /// To be passed to setRF();
    typedef enum
    {
	TransmitPowerm10dBm = 0,  ///< -10 dBm
	TransmitPowerm2dBm,       ///< -2 dBm
	TransmitPower6dBm,        ///< 6 dBm
	TransmitPower10dBm        ///< 10 dBm
    } TransmitPower;

    /// Constructor. You can have multiple instances, but each instance must have its own
    /// chip enable and slave select pin. 
    /// After constructing, you must call init() to initialise the interface
    /// and the radio module
    /// \param[in] chipEnablePin the Arduino pin to use to enable the chip for transmit/receive
    /// \param[in] txEnablePin the Arduino pin cponnected to the txEn pin on the radio that enable transmit mode
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the NRF905 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, 
    /// D10 for Maple, Teensy)
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_NRF905(uint8_t chipEnablePin = 8, uint8_t txEnablePin = 9, uint8_t slaveSelectPin = SS, RHGenericSPI& spi = hardware_spi);
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:g
    /// - Set the chip enable and chip select pins to output LOW, HIGH respectively.
    /// - Initialise the SPI output pins
    /// - Initialise the SPI interface library to 8MHz (Hint, if you want to lower
    /// the SPI frequency (perhaps where you have other SPI shields, low voltages etc), 
    /// call SPI.setClockDivider() after init()).
    /// -Flush the receiver and transmitter buffers
    /// - Set the radio to receive with powerUpRx();
    /// \return  true if everything was successful
    bool        init();

    /// Reads a single register from the NRF905
    /// \param[in] reg Register number, one of NR905_REG_*
    /// \return The value of the register
    uint8_t        spiReadRegister(uint8_t reg);

    /// Writes a single byte to the NRF905, and at the ame time reads the current STATUS register
    /// \param[in] reg Register number, one of NRF905_REG_*
    /// \param[in] val The value to write
    /// \return the current STATUS (read while the command is sent)
    uint8_t        spiWriteRegister(uint8_t reg, uint8_t val);

    /// Reads a number of consecutive registers from the NRF905 using burst read mode
    /// \param[in] reg Register number of the first register, one of NRF905_REG_*
    /// \param[in] dest Array to write the register values to. Must be at least len bytes
    /// \param[in] len Number of bytes to read
    /// \return the current STATUS (read while the command is sent)
    uint8_t           spiBurstReadRegister(uint8_t reg, uint8_t* dest, uint8_t len);

    /// Write a number of consecutive registers using burst write mode
    /// \param[in] reg Register number of the first register, one of NRF905_REG_*
    /// \param[in] src Array of new register values to write. Must be at least len bytes
    /// \param[in] len Number of bytes to write
    /// \return the current STATUS (read while the command is sent)
    uint8_t        spiBurstWriteRegister(uint8_t reg, uint8_t* src, uint8_t len);

    /// Reads and returns the device status register NRF905_REG_02_DEVICE_STATUS
    /// \return The value of the device status register
    uint8_t        statusRead();
  
    /// Sets the transmit and receive channel number.
    /// The RF frequency used is (422.4 + channel/10) * (1+hiFrequency) MHz
    /// \param[in] channel The channel number. 
    /// \param[in] hiFrequency false for low frequency band (422.4MHz and up), true for high frequency band (845MHz and up)
    /// \return true on success
    bool setChannel(uint16_t channel, bool hiFrequency = false);

    /// Sets the Network address.
    /// Only nodes with the same network address can communicate with each other. You 
    /// can set different network addresses in different sets of nodes to isolate them from each other.
    /// The default network address is 0xE7E7E7E7
    /// \param[in] address The new network address. Must match the network address of any receiving node(s).
    /// \param[in] len Number of bytes of address to set (1 to 4).
    /// \return true on success, false if len is not in the range 1-4 inclusive.
    bool setNetworkAddress(uint8_t* address, uint8_t len);

    /// Sets the transmitter power to use
    /// \param [in] power Transmitter power. One of NRF905::TransmitPower.
    /// \return true on success
    bool setRF(TransmitPower power);

    /// Sets the radio in power down mode.
    /// Sets chip enable to LOW.
    /// \return true on success
    void setModeIdle();

    /// Sets the radio in RX mode.
    /// Sets chip enable to HIGH to enable the chip in RX mode.
    /// \return true on success
    void setModeRx();

    /// Sets the radio in TX mode.
    /// Pulses the chip enable LOW then HIGH to enable the chip in TX mode.
    /// \return true on success
    void setModeTx();

    /// Sends data to the address set by setTransmitAddress()
    /// Sets the radio to TX mode
    /// \param [in] data Data bytes to send.
    /// \param [in] len Number of data bytes to set in the TX buffer. The actual size of the 
    /// transmitted data payload is set by setPayloadSize. Maximum message length actually 
    /// transmitted is RH_NRF905_MAX_MESSAGE_LEN = 27.
    /// \return true on success (which does not necessarily mean the receiver got the message, only that the message was
    /// successfully transmitted). Returns false if the requested message length exceeds RH_NRF905_MAX_MESSAGE_LEN.
    bool send(const uint8_t* data, uint8_t len);

    /// Blocks until the current message (if any) 
    /// has been transmitted
    /// \return true on success, false if the chip is not in transmit mode
    virtual bool waitPacketSent();

    /// Indicates if the chip is in transmit mode and 
    /// there is a packet currently being transmitted
    /// \return true if the chip is in transmit mode and there is a transmission in progress
    bool isSending();

    /// Prints the value of a single chip register
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// \return true on success
    bool printRegister(uint8_t reg);

    /// Prints the value of all chip registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// \return true on success
    bool printRegisters();

    /// Checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool recv(uint8_t* buf, uint8_t* len);

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

protected:
    /// Examine the revceive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

private:
    /// This idle mode chip configuration
    uint8_t             _configuration;

    /// the number of the chip enable pin
    uint8_t             _chipEnablePin;

    /// The number of the transmit enable pin
    uint8_t             _txEnablePin;

    /// Number of octets in the buffer
    uint8_t             _bufLen;
    
    /// The receiver/transmitter buffer
    uint8_t             _buf[RH_NRF905_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    bool                _rxBufValid;
};

/// @example nrf905_client.ino
/// @example nrf905_server.ino
/// @example nrf905_reliable_datagram_client.ino
/// @example nrf905_reliable_datagram_server.ino

#endif

// RH_Serial.h
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_Serial.h,v 1.14 2020/01/07 23:35:02 mikem Exp $

// Works with any serial port. Tested with Arduino Mega connected to Serial1
// Also works with 3DR Radio V1.3 Telemetry kit (serial at 57600baud)

#ifndef RH_Serial_h
#define RH_Serial_h

#include <RHGenericDriver.h>
#if (RH_PLATFORM == RH_PLATFORM_STM32F2)
 #define HardwareSerial USARTSerial
#elif defined (ARDUINO_ARCH_STM32F4)
 #include <libmaple/HardwareSerial.h>
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
 #include <UART.h>
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(ARDUINO_attinyxy6)
// AT Tiny Mega 3216 etc
 #define HardwareSerial UartClass
#else
 #include <HardwareSerial.h>
#endif

// Special characters
#define STX 0x02
#define ETX 0x03
#define DLE 0x10
#define SYN 0x16

// Maximum message length (including the headers) we are willing to support
#define RH_SERIAL_MAX_PAYLOAD_LEN 64

// The length of the headers we add.
// The headers are inside the payload and are therefore protected by the FCS
#define RH_SERIAL_HEADER_LEN 4

// This is the maximum message length that can be supported by this library. 
// It is an arbitrary limit.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrpyted
#ifndef RH_SERIAL_MAX_MESSAGE_LEN
#define RH_SERIAL_MAX_MESSAGE_LEN (RH_SERIAL_MAX_PAYLOAD_LEN - RH_SERIAL_HEADER_LEN)
#endif


/////////////////////////////////////////////////////////////////////
/// \class RH_Serial RH_Serial.h <RH_Serial.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a serial connection
///
/// This class sends and received packetized messages over a serial connection.
/// It can be used for point-to-point or multidrop, RS232, RS488 or other serial connections as
/// supported by your controller hardware.
/// It can also be used to communicate via radios with serial interfaces such as:
/// - APC220 Radio Data Module http://www.dfrobot.com/image/data/TEL0005/APC220_Datasheet.pdf
///   http://www.dfrobot.com/image/data/TEL0005/APC220_Datasheet.pdf
/// - 3DR Telemetry Radio https://store.3drobotics.com/products/3dr-radio
/// - HopeRF HM-TR module http://www.hoperf.com/upload/rf_app/HM-TRS.pdf
/// - Others
///
/// Compiles and runs on Linux, OSX and all the microprocessers and MCUs suported by
/// radiohead. On Linux and OSX, a RadioHead specific version of HardwareSerial (in RHutil/HardwareSerial.*)
/// encapsulates access to any serial port (or suported USB-serial converter)
///
/// The packetised messages include message encapsulation, headers, a message payload and a checksum.
/// It therefore can support robust binary message passing with error-detection and retransmission
/// when used with the appropriate manager. This allows reliable serial communicaitons even over very long
/// lines where noise might otherwise affect reliablity of the communications.
///
/// \par Packet Format
///
/// All messages sent and received by this RH_Serial Driver conform to this packet format:
/// \code
/// DLE 
/// STX
/// TO Header                (1 octet)
/// FROM Header              (1 octet)
/// ID Header                (1 octet)
/// FLAGS Header             (1 octet)
/// Message payload          (0 to 60 octets)
/// DLE
/// ETX
/// Frame Check Sequence FCS CCITT CRC-16 (2 octets)
/// \endcode
///
/// If any of octets from TO header through to the end of the payload are a DLE, 
/// then they are preceded by a DLE (ie DLE stuffing).
/// The FCS covers everything from the TO header to the ETX inclusive, but not any stuffed DLEs
///
/// \par Physical connection
///
/// The physical connection to your serial port will depend on the type of platform you are on.
///
/// For example, many arduinos only support a single Serial port on pins 0 and 1, 
/// which is shared with the USB host connections. On such Arduinos, it is not possible to use both 
/// RH_Serial on the Serial port as well as using the Serial port for debugand other printing or communications.
/// 
/// On Arduino Mega and Due, there are 4 serial ports:
/// - Serial: this is the serial port connected to the USB interface and the programming host.
/// - Serial1: on pins 18 (Tx) and 19 (Rx)
/// - Serial2: on pins 16 (Tx) and 17 (Rx)
/// - Serial3: on pins 14 (Tx) and 15 (Rx)
///
/// On Uno32, there are 2 serial ports:
/// - SerialUSB: this is the port for the USB host connection.
/// - Serial1: on pins 39 (Rx) and 40 (Tx) 
///
/// On Maple and Flymaple, there are 4 serial ports:
/// - SerialUSB: this is the port for the USB host connection.
/// - Serial1: on pins 7 (Tx) and 8 (Rx)
/// - Serial2: on pins 0 (Rx) and 1 (Tx)
/// - Serial3: on pins 29 (Tx) and 30 (Rx)
///
/// On Linux and OSX there can be any number of serial ports.
/// - On Linux, names like /dev/ttyUSB0 (for a FTDO USB-serial converter)
/// - On OSX, names like /dev/tty.usbserial-A501YSWL (for a FTDO USB-serial converter)
///
/// On STM32 F4 Discovery with Arduino and Arduino_STM32, there are 4 serial ports. We had success with port 2
/// (TX on pin PA2 and RX on pin PA3) and initialising the driver like this:
/// RH_Serial driver(Serial2);
///
/// Note that it is necessary for you to select which Serial port your RF_Serial will use and pass it to the 
/// contructor. On Linux you must pass an instance of HardwareSerial.
///
/// \par Testing
/// 
/// You can test this class and the RHReliableDatagram manager
/// on Unix and OSX with back-to-back connected FTDI USB-serial adapters.
/// Back-to-back means the TX of one is connected to the RX of the other and vice-versa. 
/// You should also join the ground pins.
///
/// Assume the 2 USB-serial adapters are connected by USB
/// and have been assigned device names: 
/// /dev/ttyUSB0 and /dev/ttyUSB1.
/// Build the example RHReliableDatagram client and server programs:
/// \code
/// tools/simBuild examples/serial/serial_reliable_datagram_server/serial_reliable_datagram_server.ino 
/// tools/simBuild examples/serial/serial_reliable_datagram_client/serial_reliable_datagram_client.ino
/// \endcode
/// In one window run the server, specifying the device to use as an environment variable:
/// \code
/// RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB1 ./serial_reliable_datagram_server 
/// \endcode
/// And in another window run the client, specifying the other device to use as an environment variable:
/// \code
/// RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB0 ./serial_reliable_datagram_client 
/// \endcode
/// You should see the 2 programs passing messages to each other.
/// 
class RH_Serial : public RHGenericDriver
{
public:
    /// Constructor
    /// \param[in] serial Reference to the HardwareSerial port which will be used by this instance.
    /// On Unix and OSX, this is an instance of RHutil/HardwareSerial. On 
    /// Arduino and other, it is an instance of the built in HardwareSerial class.
    RH_Serial(HardwareSerial& serial);

    /// Return the HardwareSerial port in use by this instance
    /// \return The current HardwareSerial as a reference
    HardwareSerial& serial();

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init();

    /// Tests whether a new message is available
    /// This can be called multiple times in a timeout loop.
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool available();

    /// Wait until a new message is available from the driver.
    /// Blocks until a complete message is received as reported by available()
  /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
  /// in multitaking environment like Linux to prevent waitAvailableTimeout
  /// using all the CPU while polling for receiver activity
    virtual void waitAvailable(uint16_t polldelay = 0);

    /// Wait until a new message is available from the driver or the timeout expires.
    /// Blocks until a complete message is received as reported by available() or the timeout expires.
    /// \param[in] timeout The maximum time to wait in milliseconds
  /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
  /// in multitaking environment like Linux to prevent waitAvailableTimeout
  /// using all the CPU while polling for receiver activity
    /// \return true if a message is available as reported by available(), false on timeout.
    virtual bool waitAvailableTimeout(uint16_t timeout, uint16_t polldelay = 0);

    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();


protected:
    /// \brief Defines different receiver states in teh receiver state machine
    typedef enum
    {
	RxStateInitialising = 0,  ///< Before init() is called
	RxStateIdle,              ///< Waiting for an STX
	RxStateDLE,               ///< Waiting for the DLE after STX
	RxStateData,              ///< Receiving data
	RxStateEscape,            ///< Got a DLE while receiving data.
	RxStateWaitFCS1,          ///< Got DLE ETX, waiting for first FCS octet
	RxStateWaitFCS2           ///< Waiting for second FCS octet
    } RxState;

    /// HAndle a character received from the serial port. IMplements
    /// the receiver state machine
    void  handleRx(uint8_t ch);

    /// Empties the Rx buffer
    void  clearRxBuf();

    /// Adds a charater to the Rx buffer
    void  appendRxBuf(uint8_t ch);

    /// Checks whether the Rx buffer contains valid data that is complete and uncorrupted
    /// Check the FCS, the TO address, and extracts the headers
    void  validateRxBuf();

    /// Sends a single data octet to the serial port.
    /// Implements DLE stuffing and keeps track of the senders FCS
    void  txData(uint8_t ch);

    /// Reference to the HardwareSerial port we will use
    HardwareSerial& _serial;

    /// The current state of the Rx state machine
    RxState         _rxState;

    /// Progressive FCS calc (CCITT CRC-16 covering all received data (but not stuffed DLEs), plus trailing DLE, ETX)
    uint16_t        _rxFcs;

    /// The received FCS at the end of the current message
    uint16_t        _rxRecdFcs; 

    /// The Rx buffer
    uint8_t         _rxBuf[RH_SERIAL_MAX_PAYLOAD_LEN];

    /// Current length of data in the Rx buffer
    uint8_t         _rxBufLen;

    /// True if the data in the Rx buffer is value and uncorrupted and complete message is available for collection
    bool            _rxBufValid;

    /// FCS for transmitted data
    uint16_t        _txFcs;
};

/// @example serial_reliable_datagram_client.ino
/// @example serial_reliable_datagram_server.ino
/// @example serial_gateway.ino
/// @example serial_encrypted_reliable_datagram_client.ino
/// @example serial_encrypted_reliable_datagram_server.ino

#endif

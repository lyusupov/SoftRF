// RHEncryptedDriver.h

// Generic encryption layer that could use any driver
// But will encrypt all data.
// Requires the Arduinolibs/Crypto library:
// https://github.com/rweather/arduinolibs
//
// Author: Philippe.Rochat'at'gmail.com
// Contributed to the RadioHead project by the author
// $Id: RHEncryptedDriver.h,v 1.4 2020/07/05 08:52:21 mikem Exp $

#ifndef RHEncryptedDriver_h
#define RHEncryptedDriver_h

#include <RHGenericDriver.h>
#if defined(RH_ENABLE_ENCRYPTION_MODULE) || defined(DOXYGEN)
#include <BlockCipher.h>

// Undef this if trailing 0 on each enrypted message is ok.
// This defined means a first byte of the payload is used to encode content length
// And the received message content is trimmed to this length
#define STRICT_CONTENT_LEN  

// Define this to allow encrypted content to span over 2 messages
// STRICT_CONTENT_LEN and ALLOW_MULTIPLE_MSG aren't compatible !!!
// With STRICT_CONTENT_LEN, receiver will try to extract length from every message !!!!
//#define ALLOW_MULTIPLE_MSG  

/////////////////////////////////////////////////////////////////////
/// \class RHEncryptedDriver RHEncryptedDriver <RHEncryptedDriver.h>
/// \brief Virtual Driver to encrypt/decrypt data. Can be used with any other RadioHead driver.
///
/// This driver acts as a wrapper for any other RadioHead driver, adding encryption and decryption of
/// messages that are passed to and from the actual radio driver. Only the message payload is encrypted,
/// and not the to/from address or flags. Any of the encryption ciphers supported by
/// ArduinoLibs Cryptographic Library http://rweather.github.io/arduinolibs/crypto.html may be used.
///
/// For successful communications, both sender and receiver must use the same cipher and the same key.
///
/// In order to enable this module you must uncomment #define RH_ENABLE_ENCRYPTION_MODULE at the bottom of RadioHead.h
/// But ensure you have installed the Crypto directory from arduinolibs first:
/// http://rweather.github.io/arduinolibs/index.html

class RHEncryptedDriver : public RHGenericDriver
{
public:
    /// Constructor.
    /// Adds a ciphering layer to messages sent and received by the actual transport driver.
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] blockcipher The blockcipher (from arduinolibs) that crypt/decrypt data. Ensure that
    /// the blockcipher has had its key set before sending or receiving messages.
    RHEncryptedDriver(RHGenericDriver& driver, BlockCipher& blockcipher);

    /// Calls the real driver's init()
    /// \return The value returned from the driver init() method;
    virtual bool init() { return _driver.init();};
    
    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool available() { return _driver.available();};

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    virtual bool send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver, which depends on the maximum length supported by the underlying transport driver.
    /// \return The maximum legal message length
    virtual  uint8_t maxMessageLength();

    /// Blocks until the transmitter 
    /// is no longer transmitting.
    virtual bool            waitPacketSent() { return _driver.waitPacketSent();} ;

    /// Blocks until the transmitter is no longer transmitting.
    /// or until the timeout occuers, whichever happens first
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \return true if the radio completed transmission within the timeout period. False if it timed out.
    virtual bool            waitPacketSent(uint16_t timeout) {return _driver.waitPacketSent(timeout);} ;

    /// Starts the receiver and blocks until a received message is available or a timeout
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \return true if a message is available
    virtual bool            waitAvailableTimeout(uint16_t timeout) {return _driver.waitAvailableTimeout(timeout);};

    /// Calls the waitCAD method in the driver
    /// \return The return value from teh drivers waitCAD() method
    virtual bool            waitCAD() { return _driver.waitCAD();};

    /// Sets the Channel Activity Detection timeout in milliseconds to be used by waitCAD().
    /// The default is 0, which means do not wait for CAD detection.
    /// CAD detection depends on support for isChannelActive() by your particular radio.
    void setCADTimeout(unsigned long cad_timeout) {_driver.setCADTimeout(cad_timeout);};

    /// Determine if the currently selected radio channel is active.
    /// This is expected to be subclassed by specific radios to implement their Channel Activity Detection
    /// if supported. If the radio does not support CAD, returns true immediately. If a RadioHead radio 
    /// supports isChannelActive() it will be documented in the radio specific documentation.
    /// This is called automatically by waitCAD().
    /// \return true if the radio-specific CAD (as returned by override of isChannelActive()) shows the
    /// current radio channel as active, else false. If there is no radio-specific CAD, returns false.
    virtual bool            isChannelActive() { return _driver.isChannelActive();};

    /// Sets the address of this node. Defaults to 0xFF. Subclasses or the user may want to change this.
    /// This will be used to test the adddress in incoming messages. In non-promiscuous mode,
    /// only messages with a TO header the same as thisAddress or the broadcast addess (0xFF) will be accepted.
    /// In promiscuous mode, all messages will be accepted regardless of the TO header.
    /// In a conventional multinode system, all nodes will have a unique address 
    /// (which you could store in EEPROM).
    /// You would normally set the header FROM address to be the same as thisAddress (though you dont have to, 
    /// allowing the possibilty of address spoofing).
    /// \param[in] thisAddress The address of this node.
    virtual void setThisAddress(uint8_t thisAddress) { _driver.setThisAddress(thisAddress);};

    /// Sets the TO header to be sent in all subsequent messages
    /// \param[in] to The new TO header value
    virtual void           setHeaderTo(uint8_t to){ _driver.setHeaderTo(to);};

    /// Sets the FROM header to be sent in all subsequent messages
    /// \param[in] from The new FROM header value
    virtual void           setHeaderFrom(uint8_t from){ _driver.setHeaderFrom(from);};

    /// Sets the ID header to be sent in all subsequent messages
    /// \param[in] id The new ID header value
    virtual void           setHeaderId(uint8_t id){ _driver.setHeaderId(id);};

    /// Sets and clears bits in the FLAGS header to be sent in all subsequent messages
    /// First it clears he FLAGS according to the clear argument, then sets the flags according to the 
    /// set argument. The default for clear always clears the application specific flags.
    /// \param[in] set bitmask of bits to be set. Flags are cleared with the clear mask before being set.
    /// \param[in] clear bitmask of flags to clear. Defaults to RH_FLAGS_APPLICATION_SPECIFIC
    ///            which clears the application specific flags, resulting in new application specific flags
    ///            identical to the set.
    virtual void           setHeaderFlags(uint8_t set, uint8_t clear = RH_FLAGS_APPLICATION_SPECIFIC) { _driver.setHeaderFlags(set, clear);};

    /// Tells the receiver to accept messages with any TO address, not just messages
    /// addressed to thisAddress or the broadcast address
    /// \param[in] promiscuous true if you wish to receive messages with any TO address
    virtual void           setPromiscuous(bool promiscuous){ _driver.setPromiscuous(promiscuous);};

    /// Returns the TO header of the last received message
    /// \return The TO header
    virtual uint8_t        headerTo() { return _driver.headerTo();};

    /// Returns the FROM header of the last received message
    /// \return The FROM header
    virtual uint8_t        headerFrom() { return _driver.headerFrom();};

    /// Returns the ID header of the last received message
    /// \return The ID header
    virtual uint8_t        headerId() { return _driver.headerId();};

    /// Returns the FLAGS header of the last received message
    /// \return The FLAGS header
    virtual uint8_t        headerFlags() { return _driver.headerFlags();};

    /// Returns the most recent RSSI (Receiver Signal Strength Indicator).
    /// Usually it is the RSSI of the last received message, which is measured when the preamble is received.
    /// If you called readRssi() more recently, it will return that more recent value.
    /// \return The most recent RSSI measurement in dBm.
    int16_t        lastRssi() { return _driver.lastRssi();};

    /// Returns the operating mode of the library.
    /// \return the current mode, one of RF69_MODE_*
    RHMode          mode() { return _driver.mode();};

    /// Sets the operating mode of the transport.
    void            setMode(RHMode mode) { _driver.setMode(mode);};

    /// Sets the transport hardware into low-power sleep mode
    /// (if supported). May be overridden by specific drivers to initialte sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// \return true if sleep mode is supported by transport hardware and the RadioHead driver, and if sleep mode
    ///         was successfully entered. If sleep mode is not suported, return false.
    virtual bool    sleep() { return _driver.sleep();};

    /// Returns the count of the number of bad received packets (ie packets with bad lengths, checksum etc)
    /// which were rejected and not delivered to the application.
    /// Caution: not all drivers can correctly report this count. Some underlying hardware only report
    /// good packets.
    /// \return The number of bad packets received.
    virtual uint16_t       rxBad() { return _driver.rxBad();};

    /// Returns the count of the number of 
    /// good received packets
    /// \return The number of good packets received.
    virtual uint16_t       rxGood() { return _driver.rxGood();};

    /// Returns the count of the number of 
    /// packets successfully transmitted (though not necessarily received by the destination)
    /// \return The number of packets successfully transmitted
    virtual uint16_t       txGood() { return _driver.txGood();};

private:
    /// The underlying transport river we are to use
    RHGenericDriver&        _driver;
    
    /// The CipherBlock we are to use for encrypting/decrypting
    BlockCipher&	    _blockcipher;
    
    /// Struct for with buffers for ciphering
    typedef struct
    {
	size_t  blockSize    = 0;
	uint8_t *inputBlock  = NULL;
	//uint8_t *outputBlock = NULL;		
    } CipherBlocks;
    
    CipherBlocks            _cipheringBlocks;
    
    /// Buffer to store encrypted/decrypted message
    uint8_t*                _buffer;
};

/// @example nrf24_encrypted_client.ino
/// @example nrf24_encrypted_server.ino
/// @example rf95_encrypted_client.ino
/// @example rf95_encrypted_server.ino
/// @example serial_encrypted_reliable_datagram_client.ino
/// @example serial_encrypted_reliable_datagram_server.ino


#else // RH_ENABLE_ENCRYPTION_MODULE
#error "You have included RHEncryptedDriver.h, but not enabled RH_ENABLE_ENCRYPTION_MODULE in RadioHead.h"
#endif

#endif

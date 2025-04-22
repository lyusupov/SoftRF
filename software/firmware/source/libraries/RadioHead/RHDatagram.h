// RHDatagram.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.h,v 1.14 2015/08/12 23:18:51 mikem Exp $

#ifndef RHDatagram_h
#define RHDatagram_h

#include <RHGenericDriver.h>

// This is the maximum possible message size for radios supported by RadioHead.
// Not all radios support this length, and many are much smaller
#define RH_MAX_MESSAGE_LEN 255

/////////////////////////////////////////////////////////////////////
/// \class RHDatagram RHDatagram.h <RHDatagram.h>
/// \brief Manager class for addressed, unreliable messages
///
/// Every RHDatagram node has an 8 bit address (defaults to 0).
/// Addresses (DEST and SRC) are 8 bit integers with an address of RH_BROADCAST_ADDRESS (0xff) 
/// reserved for broadcast.
///
/// \par Media Access Strategy
///
/// RHDatagram and the underlying drivers always transmit as soon as sendto() is called.
///
/// \par Message Lengths
///
/// Not all Radio drivers supported by RadioHead can handle the same message lengths. Some radios can handle
/// up to 255 octets, and some as few as 28. If you attempt to send a message that is too long for 
/// the underlying driver, sendTo() will return false and will not transmit the message. 
/// It is the programmers responsibility to make
/// sure that messages passed to sendto() do not exceed the capability of the radio. You can use the 
/// *_MAX_MESSAGE_LENGTH definitions or driver->maxMessageLength() to help.
///
/// \par Headers
///
/// Each message sent and received by a RadioHead driver includes 4 headers:<br>
/// \b TO The node address that the message is being sent to (broadcast RH_BROADCAST_ADDRESS (255) is permitted)<br>
/// \b FROM The node address of the sending node<br>
/// \b ID A message ID, distinct (over short time scales) for each message sent by a particilar node<br>
/// \b FLAGS A bitmask of flags. The most significant 4 bits are reserved for use by RadioHead. The least
/// significant 4 bits are reserved for applications.<br>
///
class RHDatagram
{
public:
    /// Constructor. 
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    RHDatagram(RHGenericDriver& driver, uint8_t thisAddress = 0);

    /// Initialise this instance and the 
    /// driver connected to it.
    bool init();

    /// Sets the address of this node. Defaults to 0. 
    /// This will be used to set the FROM address of all messages sent by this node.
    /// In a conventional multinode system, all nodes will have a unique address 
    /// (which you could store in EEPROM).
    /// \param[in] thisAddress The address of this node
    void setThisAddress(uint8_t thisAddress);

    /// Sends a message to the node(s) with the given address
    /// RH_BROADCAST_ADDRESS is a valid address which will cause the message
    /// to be accepted by all RHDatagram nodes within range.
    /// \param[in] buf Pointer to the binary message to send
    /// \param[in] len Number of octets to send (> 0)
    /// \param[in] address The address to send the message to.
    /// \return true if the message not too loing fot eh driver, and the message was transmitted.
    bool sendto(uint8_t* buf, uint8_t len, uint8_t address);

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available for this node, copy it to buf and return true
    /// The SRC address is placed in *from if present and not NULL.
    /// The DEST address is placed in *to if present and not NULL.
    /// If a message is copied, *len is set to the length.
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \param[in] from If present and not NULL, the referenced uint8_t will be set to the FROM address
    /// \param[in] to If present and not NULL, the referenced uint8_t will be set to the TO address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    bool recvfrom(uint8_t* buf, uint8_t* len, uint8_t* from = NULL, uint8_t* to = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

    /// Tests whether a new message is available
    /// from the Driver.
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received bythe transport, when it will be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop.
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    bool            available();

    /// Starts the Driver receiver and blocks until a valid received 
    /// message is available.
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
    void            waitAvailable(uint16_t polldelay = 0);

    /// Blocks until the transmitter 
    /// is no longer transmitting.
    bool            waitPacketSent();

    /// Blocks until the transmitter is no longer transmitting.
    /// or until the timeout occuers, whichever happens first
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \return true if the radio completed transmission within the timeout period. False if it timed out.
    bool            waitPacketSent(uint16_t timeout);

    /// Starts the Driver receiver and blocks until a received message is available or a timeout
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
    /// \return true if a message is available
    bool            waitAvailableTimeout(uint16_t timeout, uint16_t polldelay = 0);

    /// Sets the TO header to be sent in all subsequent messages
    /// \param[in] to The new TO header value
    void           setHeaderTo(uint8_t to);

    /// Sets the FROM header to be sent in all subsequent messages
    /// \param[in] from The new FROM header value
    void           setHeaderFrom(uint8_t from);

    /// Sets the ID header to be sent in all subsequent messages
    /// \param[in] id The new ID header value
    void           setHeaderId(uint8_t id);

    /// Sets and clears bits in the FLAGS header to be sent in all subsequent messages
    /// \param[in] set bitmask of bits to be set
    /// \param[in] clear bitmask of flags to clear
    void           setHeaderFlags(uint8_t set, uint8_t clear = RH_FLAGS_NONE);

    /// Returns the TO header of the last received message
    /// \return The TO header of the most recently received message.
    uint8_t        headerTo();

    /// Returns the FROM header of the last received message
    /// \return The FROM header of the most recently received message.
    uint8_t        headerFrom();

    /// Returns the ID header of the last received message
    /// \return The ID header of the most recently received message.
    uint8_t        headerId();

    /// Returns the FLAGS header of the last received message
    /// \return The FLAGS header of the most recently received message.
    uint8_t        headerFlags();

    /// Returns the address of this node.
    /// \return The address of this node
    uint8_t         thisAddress();

protected:
    /// The Driver we are to use
    RHGenericDriver&        _driver;

    /// The address of this node
    uint8_t         _thisAddress;
};

#endif

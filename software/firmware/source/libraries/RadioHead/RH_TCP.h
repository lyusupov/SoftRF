// RH_TCP.h
// Author: Mike McCauley (mikem@aierspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_TCP.h,v 1.4 2015/08/13 02:45:47 mikem Exp $
#ifndef RH_TCP_h
#define RH_TCP_h

#include <RHGenericDriver.h>
#include <RHTcpProtocol.h>

/////////////////////////////////////////////////////////////////////
/// \class RH_TCP RH_TCP.h <RH_TCP.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via sockets on a Linux simulator
///
/// \par Overview
///
/// This class is intended to support the testing of RadioHead manager classes and simulated sketches 
/// on a Linux host.
/// RH_TCP class sends messages to and from other simulator sketches via sockets to a 'Luminiferous Ether' 
/// simulator server (provided).
/// Multiple instances of simulated clients and servers can run on a single Linux server,
/// passing messages to each other via the etherSimulator.pl server.
///
/// Simple RadioHead sketches can be compiled and run on Linux using a build script and some support files.
///
/// \par Running simulated sketches
///
/// \code
/// cd whatever/RadioHead 
/// # build the client for Linux:
/// tools/simBuild examples/simulator/simulator_reliable_datagram_client/simulator_reliable_datagram_client.ino
/// # build the server for Linux:
/// tools/simBuild examples/simulator/simulator_reliable_datagram_server/simulator_reliable_datagram_server.ino
/// # in one window, run the simulator server:
/// tools/etherSimulator.pl
/// # in another window, run the server
/// ./simulator_reliable_datagram_server 
/// # in another window, run the client:
/// ./simulator_reliable_datagram_client
/// # see output:
/// Sending to simulator_reliable_datagram_server
/// got reply from : 0x02: And hello back to you
/// Sending to simulator_reliable_datagram_server
/// got reply from : 0x02: And hello back to you
/// Sending to simulator_reliable_datagram_server
/// got reply from : 0x02: And hello back to you
/// ...
/// \endcode
///
/// You can change the listen port and the simulated baud rate with 
/// command line arguments passed to etherSimulator.pl
///
/// \par Implementation
///
/// etherServer.pl is a conventional server written in Perl.
/// listens on a TCP socket (defaults to port 4000) for connections from sketch simulators
/// using RH_TCP as theur driver.
/// The simulated sketches send messages out to the 'ether' over the TCP connection to the etherServer.
/// etherServer manages the delivery of each message to any other RH_TCP sketches that are running.
///
/// \par Prerequisites
///
/// g++ compiler installed and in your $PATH
/// Perl
/// Perl POE library
///
class RH_TCP : public RHGenericDriver
{
public:
    /// Constructor
    /// \param[in] server Name and optionally the port number of the ether simulator server to contact.
    /// Format is "name[:port]", where name can be any valid host name or address (IPV4 or IPV6).
    /// The trailing :port is optional, and port can be any valid 
    /// port name or port number.
    RH_TCP(const char* server = "localhost:4000");

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it will be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool available();

    /// Wait until a new message is available from the driver.
    /// Blocks until a complete message is received as reported by available()
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
    virtual void waitAvailable(uint16_t polldelay = 0);

    /// Wait until a new message is available from the driver
    /// or the timeout expires
    /// Blocks until a complete message is received as reported by available()
    /// \param[in] timeout The maximum time to wait in milliseconds
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
    /// \return true if a message is available as reported by available()
    virtual bool waitAvailableTimeout(uint16_t timeout, uint16_t polldelay = 0);

    /// Turns the receiver on if it not already on.
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
    /// of 0 is NOT permitted. If the message is too long for the underlying radio technology, send() will
    /// return false and will not send the message.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// Sets the address of this node. Defaults to 0xFF. Subclasses or the user may want to change this.
    /// This will be used to test the adddress in incoming messages. In non-promiscuous mode,
    /// only messages with a TO header the same as thisAddress or the broadcast addess (0xFF) will be accepted.
    /// In promiscuous mode, all messages will be accepted regardless of the TO header.
    /// In a conventional multinode system, all nodes will have a unique address 
    /// (which you could store in EEPROM).
    /// You would normally set the header FROM address to be the same as thisAddress (though you dont have to, 
    /// allowing the possibilty of address spoofing).
    /// \param[in] address The address of this node.
    void setThisAddress(uint8_t address);

protected:

private:
    /// Connect to the address and port specified by the server constructor argument.
    /// Prepares the socket for use.
    bool connectToServer();

    /// Check for new messages from the ether simulator server
    /// \return true if no faults (not necessarily if there was an event)
    bool checkForEvents();

    /// Clear the receive buffer
    void clearRxBuf();

    /// Sends thisAddress to the ether simulator server
    /// in a RHTcpThisAddress message.
    /// \param[in] thisAddress The node address of this node
    /// \return true if successful
    bool sendThisAddress(uint8_t thisAddress);

    /// Sends a message to the ether simulator server for delivery to
    /// other nodes
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if successful
    bool sendPacket(const uint8_t* data, uint8_t len);

    /// Address and port of the server to which messages are sent
    /// and received using the protocol RHTcpPRotocol
    const char* _server;

    /// The TCP socket used to communicate with the message server
    int         _socket;

    /// Buffer to receive RHTcpProtocol messages
    uint8_t     _rxBuf[RH_TCP_MAX_PAYLOAD_LEN + 5];
    uint16_t    _rxBufLen;
    bool        _rxBufValid;

    /// Check whether the latest received message is complete and uncorrupted
    void            validateRxBuf();

    // Used in the interrupt handlers
    /// Buf is filled but not validated
    volatile bool   _rxBufFull;

};

/// @example simulator_reliable_datagram_client.ino
/// @example simulator_reliable_datagram_server.ino

#endif

// RHReliableDatagram.h
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHReliableDatagram.h,v 1.19 2019/07/14 00:18:48 mikem Exp $

#ifndef RHReliableDatagram_h
#define RHReliableDatagram_h

#include <RHDatagram.h>

/// The acknowledgement bit in the header FLAGS. This indicates if the payload is for an
/// ack for a successfully received message.
#define RH_FLAGS_ACK 0x80
/// The retry bit in the header FLAGS. This indicates that the payload is a retry for a
/// previously sent message.
#define RH_FLAGS_RETRY 0x40

/// This macro enables enhanced message deduplication behavior. This currently defaults
/// to 0 (off), but this may change to default to 1 (on) in future releases. Consumers who
/// want to enable this behavior should override this macro in their code and set it to 1.
/// It is most useful where a transmitter periodically wakes up and starts to transmit
/// starting again from the first sequence number.
///
/// Enhanced deduplication: Only messages containing the retry bit in the header
/// FLAGS will be evaluated for deduplication. This ensures that only messages that are
/// genuine retries will potentially be deduped. Note that this should not be enabled
/// if you will receive messages from devices using older versions of this library that
/// do not support the RETRY header. If you do, deduping of messages will be broken.
#ifndef RH_ENABLE_EXPLICIT_RETRY_DEDUP
 #define RH_ENABLE_EXPLICIT_RETRY_DEDUP 0
#endif

/// the default retry timeout in milliseconds
#define RH_DEFAULT_TIMEOUT 200

/// The default number of retries
#define RH_DEFAULT_RETRIES 3

/////////////////////////////////////////////////////////////////////
/// \class RHReliableDatagram RHReliableDatagram.h <RHReliableDatagram.h>
/// \brief RHDatagram subclass for sending addressed, acknowledged, retransmitted datagrams.
///
/// Manager class that extends RHDatagram to define addressed, reliable datagrams with acknowledgement and retransmission.
/// Based on RHDatagram, adds flags and sequence numbers. RHReliableDatagram is reliable in the sense
/// that messages are acknowledged by the recipient, and unacknowledged messages are retransmitted until acknowledged or the
/// retries are exhausted.
/// When addressed messages are sent (by sendtoWait()), it will wait for an ack, and retransmit
/// after timeout until an ack is received or retries are exhausted.
/// When addressed messages are collected by the application (by recvfromAck()), 
/// an acknowledgement is automatically sent to the sender.
///
/// You can use RHReliableDatagram to send broadcast messages, with a TO address of RH_BROADCAST_ADDRESS,
/// however broadcasts are not acknowledged or retransmitted and are therefore NOT actually reliable.
///
/// The retransmit timeout is randomly varied between timeout and timeout*2 to prevent collisions on all
/// retries when 2 nodes happen to start sending at the same time .
///
/// Each new message sent by sendtoWait() has its ID incremented.
///
/// An ack consists of a message with:
/// - TO set to the from address of the original message
/// - FROM set to this node address
/// - ID set to the ID of the original message
/// - FLAGS with the RH_FLAGS_ACK bit set
/// - 1 octet of payload containing ASCII '!' (since some drivers cannot handle 0 length payloads)
///
/// \par Media Access Strategy
///
/// RHReliableDatagram and the underlying drivers always transmit as soon as
/// sendtoWait() is called.  RHReliableDatagram waits for an acknowledgement,
/// and if one is not received after a timeout period the message is
/// transmitted again.  If no acknowledgement is received after several
/// retries, the transmissions is deemed to have failed.
/// No contention for media is detected.
/// This will be recognised as "pure ALOHA". 
/// The addition of Clear Channel Assessment (CCA) is desirable and planned.
///
/// There is no message queuing or threading in RHReliableDatagram. 
/// sendtoWait() waits until an acknowledgement is received, retransmitting
/// up to (by default) 3 retries time with a default 200ms timeout. 
/// During this transmit-acknowledge phase, any received message (other than the expected
/// acknowledgement) will be ignored. Your sketch will be unresponsive to new messages 
/// until an acknowledgement is received or the retries are exhausted. 
/// Central server-type sketches should be very cautious about their
/// retransmit strategy and configuration lest they hang for a long time
/// trying to reply to clients that are unreachable.
///
/// Caution: if you have a radio network with a mixture of slow and fast
/// processors and ReliableDatagrams, you may be affected by race conditions
/// where the fast processor acknowledges a message before the sender is ready
/// to process the acknowledgement. Best practice is to use the same processors (and
/// radios) throughout your network.
///
class RHReliableDatagram : public RHDatagram
{
public:
    /// Constructor. 
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    RHReliableDatagram(RHGenericDriver& driver, uint8_t thisAddress = 0);

    /// Sets the minimum retransmit timeout. If sendtoWait is waiting for an ack 
    /// longer than this time (in milliseconds), 
    /// it will retransmit the message. Defaults to 200ms. The timeout is measured from the end of
    /// transmission of the message. It must be at least longer than the the transmit 
    /// time of the acknowledgement (preamble+6 octets) plus the latency/poll time of the receiver. 
    /// For fast modulation schemes you can considerably shorten this time.
    /// Caution: if you are using slow packet rates and long packets 
    /// you may need to change the timeout for reliable operations.
    /// The actual timeout is randomly varied between timeout and timeout*2.
    /// \param[in] timeout The new timeout period in milliseconds
    void setTimeout(uint16_t timeout);

    /// Sets the maximum number of retries. Defaults to 3 at construction time. 
    /// If set to 0, each message will only ever be sent once.
    /// sendtoWait will give up and return false if there is no ack received after all transmissions time out
    /// and the retries count is exhausted.
    /// param[in] retries The maximum number a retries.
    void setRetries(uint8_t retries);

    /// Returns the currently configured maximum retries count.
    /// Can be changed with setRetries().
    /// \return The currently configured maximum number of retries.
    uint8_t retries();

    /// Send the message (with retries) and waits for an ack. Returns true if an acknowledgement is received.
    /// Synchronous: any message other than the desired ACK received while waiting is discarded.
    /// Blocks until an ACK is received or all retries are exhausted (ie up to retries*timeout milliseconds).
    /// If the destination address is the broadcast address RH_BROADCAST_ADDRESS (255), the message will 
    /// be sent as a broadcast, but receiving nodes do not acknowledge, and sendtoWait() returns true immediately
    /// without waiting for any acknowledgements.
    /// \param[in] address The address to send the message to.
    /// \param[in] buf Pointer to the binary message to send
    /// \param[in] len Number of octets to send
    /// \return true if the message was transmitted and an acknowledgement was received.
    bool sendtoWait(uint8_t* buf, uint8_t len, uint8_t address);

    /// If there is a valid message available for this node, send an acknowledgement to the SRC
    /// address (blocking until this is complete), then copy the message to buf and return true
    /// else return false. 
    /// If a message is copied, *len is set to the length.
    /// If from is not NULL, the SRC address is placed in *from.
    /// If to is not NULL, the DEST address is placed in *to.
    /// This is the preferred function for getting messages addressed to this node.
    /// If the message is not a broadcast, acknowledge to the sender before returning.
    /// You should be sure to call this function frequently enough to not miss any messages.
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \param[in] from If present and not NULL, the referenced uint8_t will be set to the SRC address
    /// \param[in] to If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf. False if
    /// - 1. There was no message received and waiting to be collected, or
    /// - 2. There was a message received but it was not addressed to this node, or
    /// - 3. There was a correctly addressed message but it was a duplicate of an earlier correctly received message
    bool recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* from = NULL, uint8_t* to = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

    /// Similar to recvfromAck(), this will block until either a valid message available for this node
    /// or the timeout expires. Starts the receiver automatically.
    /// You should be sure to call this function frequently enough to not miss any messages.
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \param[in] timeout Maximum time to wait in milliseconds
    /// \param[in] from If present and not NULL, the referenced uint8_t will be set to the SRC address
    /// \param[in] to If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    bool recvfromAckTimeout(uint8_t* buf, uint8_t* len,  uint16_t timeout, uint8_t* from = NULL, uint8_t* to = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

    /// Returns the number of retransmissions 
    /// we have had to send since starting or since the last call to resetRetransmissions().
    /// \return The number of retransmissions since initialisation.
    uint32_t retransmissions();

    /// Resets the count of the number of retransmissions 
    /// to 0. 
    void resetRetransmissions(); 

protected:
    /// Send an ACK for the message id to the given from address
    /// Blocks until the ACK has been sent
    void acknowledge(uint8_t id, uint8_t from);

    /// Checks whether the message currently in the Rx buffer is a new message, not previously received
    /// based on the from address and the sequence.  If it is new, it is acknowledged and returns true
    /// \return true if there is a message received and it is a new message
    bool haveNewMessage();

private:
    /// Count of retransmissions we have had to send
    uint32_t _retransmissions;

    /// The last sequence number to be used
    /// Defaults to 0
    uint8_t _lastSequenceNumber;

    // Retransmit timeout (milliseconds)
    /// Defaults to 200
    uint16_t _timeout;

    // Retries (0 means one try only)
    /// Defaults to 3
    uint8_t _retries;

    /// Array of the last seen sequence number indexed by node address that sent it
    /// It is used for duplicate detection. Duplicated messages are re-acknowledged when received 
    /// (this is generally due to lost ACKs, causing the sender to retransmit, even though we have already
    /// received that message)
    uint8_t _seenIds[256];
};

/// @example rf22_reliable_datagram_client.ino
/// @example rf22_reliable_datagram_server.ino

#endif


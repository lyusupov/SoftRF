// RH_TcpProtocol.h
// Author: Mike McCauley (mikem@aierspayce.com)
// Definition of protocol messages sent and received by RH_TCP
// Copyright (C) 2014 Mike McCauley
// $Id: RHTcpProtocol.h,v 1.3 2014/05/22 06:07:09 mikem Exp $

/// This file contains the definitions of message structures passed between
/// RH_TCP and the etherSimulator
#ifndef RH_TcpProtocol_h
#define RH_TcpProtocol_h

#define RH_TCP_MESSAGE_TYPE_NOP               0
#define RH_TCP_MESSAGE_TYPE_THISADDRESS       1
#define RH_TCP_MESSAGE_TYPE_PACKET            2

// Maximum message length (including the headers) we are willing to support
#define RH_TCP_MAX_PAYLOAD_LEN 255

// The length of the headers we add.
// The headers are inside the RF69's payload and are therefore encrypted if encryption is enabled
#define RH_TCP_HEADER_LEN 4


// This is the maximum message length that can be supported by this protocol. 
#define RH_TCP_MAX_MESSAGE_LEN (RH_TCP_MAX_PAYLOAD_LEN - RH_TCP_HEADER_LEN)

#pragma pack(push, 1) // No padding

/// \brief Generic RH_TCP simulator message structure
typedef struct
{
    uint32_t        length; ///< Number of octets following, in network byte order
    uint8_t         payload[RH_TCP_MAX_PAYLOAD_LEN + 1]; ///< Payload
}   RHTcpMessage;

/// \brief Generic RH_TCP  message structure with message type
typedef struct
{
    uint32_t        length; ///< Number of octets following, in network byte order
    uint8_t         type;  ///< One of RH_TCP_MESSAGE_TYPE_*
    uint8_t         payload[RH_TCP_MAX_PAYLOAD_LEN]; ///< Payload
}   RHTcpTypeMessage;

/// \brief RH_TCP message Notifies the server of thisAddress of this client
typedef struct
{
    uint32_t        length; ///< Number of octets following, in network byte order
    uint8_t         type;   ///< == RH_TCP_MESSAGE_TYPE_THISADDRESS
    uint8_t         thisAddress; ///< Node address
}   RHTcpThisAddress;

/// \brief RH_TCP radio message passed to or from the simulator
typedef struct
{
    uint32_t        length; ///< Number of octets following, in network byte order
    // 5 octets of header to follow for total of 9 octets
    uint8_t         type;   ///< == RH_TCP_MESSAGE_TYPE_PACKET
    uint8_t         to;     ///< Node address of the recipient
    uint8_t         from;   ///< Node address of the sender
    uint8_t         id;     ///< Message sequence number
    uint8_t         flags;  ///< Message flags
    uint8_t         payload[RH_TCP_MAX_MESSAGE_LEN]; ///< 0 or more, length deduced from length above
}   RHTcpPacket;

#pragma pack(pop)

#endif

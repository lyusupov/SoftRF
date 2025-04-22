// RHMesh.h
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHMesh.h,v 1.17 2020/08/04 09:02:14 mikem Exp $

#ifndef RHMesh_h
#define RHMesh_h

#include <RHRouter.h>

// Types of RHMesh message, used to set msgType in the RHMeshHeader
#define RH_MESH_MESSAGE_TYPE_APPLICATION                    0
#define RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST        1
#define RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE       2
#define RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE                  3

// Timeout for address resolution in milliecs
#define RH_MESH_ARP_TIMEOUT 4000

/////////////////////////////////////////////////////////////////////
/// \class RHMesh RHMesh.h <RHMesh.h>
/// \brief RHRouter subclass for sending addressed, optionally acknowledged datagrams
/// multi-hop routed across a network, with automatic route discovery
///
/// Manager class that extends RHRouter to add automatic route discovery within a mesh of adjacent nodes, 
/// and route signalling.
///
/// Unlike RHRouter, RHMesh can be used in networks where the network topology is fluid, or unknown, 
/// or if nodes can mode around or go in or out of service. When a node wants to send a 
/// message to another node, it will automatically discover a route to the destination node and use it. 
/// If the route becomes unavailable, a new route will be discovered.
///
/// \par Route Discovery
///
/// When a RHMesh mesh node is initialised, it doe not know any routes to any other nodes 
/// (see RHRouter for details on route and the routing table).
/// When you attempt to send a message with sendtoWait, will first check to see if there is a route to the 
/// destinastion node in the routing tabl;e. If not, it wil initialite 'Route Discovery'.
/// When a node needs to discover a route to another node, it broadcasts MeshRouteDiscoveryMessage 
/// with a message type of RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST. 
/// Any node that receives such a request checks to see if it is a request for a route to itself
/// (in which case it makes a unicast reply to the originating node with a 
/// MeshRouteDiscoveryMessage 
/// with a message type of RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE) 
/// otherwise it rebroadcasts the request, after adding itself to the list of nodes visited so 
/// far by the request.
///
/// If a node receives a RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST that already has itself 
/// listed in the visited nodes, it knows it has already seen and rebroadcast this request, 
/// and threfore ignores it. This prevents broadcast storms.
/// When a node receives a RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST it can use the list of 
/// nodes aready visited to deduce routes back towards the originating (requesting node). 
/// This also means that when the destination node of the request is reached, it (and all 
/// the previous nodes the request visited) will have a route back to the originating node. 
/// This means the unicast RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE 
/// reply will be routed successfully back to the original route requester.
///
/// The RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE sent back by the destination node contains 
/// the full list of nodes that were visited on the way to the destination.
/// Therefore, intermediate nodes that route the reply back towards the originating node can use the 
/// node list in the reply to deduce routes to all the nodes between it and the destination node.
///
/// Therefore, RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST and 
/// RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE together ensure the original requester and all 
/// the intermediate nodes know how to route to the source and destination nodes and every node along the path.
///
/// Note that there is a race condition here that can effect routing on multipath routes. For example, 
/// if the route to the destination can traverse several paths, last reply from the destination 
/// will be the one used.
///
/// \par Route Failure
///
/// RHRouter (and therefore RHMesh) use reliable hop-to-hop delivery of messages using 
/// hop-to-hop acknowledgements, but not end-to-end acknowledgements. When sendtoWait() returns, 
/// you know that the message has been delivered to the next hop, but not if it is (or even if it can be) 
/// delivered to the destination node. If during the course of hop-to-hop routing of a message, 
/// one of the intermediate RHMesh nodes finds it cannot deliver to the next hop 
/// (say due to a lost route or no acknwledgement from the next hop), it replies to the 
/// originator with a unicast MeshRouteFailureMessage RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE message. 
/// Intermediate nodes (on the way beack to the originator)
/// and the originating node use this message to delete the route to the destination 
/// node of the original message. This means that if a route to a destination becomes unusable 
/// (either because an intermediate node is off the air, or has moved out of range) a new route 
/// will be established the next time a message is to be sent.
///
/// \par Message Format
///
/// RHMesh uses a number of message formats layered on top of RHRouter:
/// - MeshApplicationMessage (message type RH_MESH_MESSAGE_TYPE_APPLICATION). 
///   Carries an application layer message for the caller of RHMesh
/// - MeshRouteDiscoveryMessage (message types RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST 
///   and RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE). Carries Route Discovery messages 
///   (broadcast) and replies (unicast).
/// - MeshRouteFailureMessage (message type RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE) Informs nodes of 
///   route failures.
///
/// Part of the Arduino RH library for operating with HopeRF RH compatible transceivers 
/// (see http://www.hoperf.com)
///
/// \par Memory
///
/// RHMesh programs require significant amount of SRAM, often approaching 2kbytes, 
/// which is beyond or at the limits of some Arduinos and other processors. Programs 
/// with additional software besides basic RHMesh programs may well require even more. If you have insufficient
/// SRAM for your program, it may result in failure to run, or wierd crashes and other hard to trace behaviour.
/// In this event you should consider a processor with more SRAM, such as the MotienoMEGA with 16k
/// (https://lowpowerlab.com/shop/moteinomega) or others.
///
/// \par Performance
/// This class (in the interests of simple implemtenation and low memory use) does not have
/// message queueing. This means that only one message at a time can be handled. Message transmission 
/// failures can have a severe impact on network performance.
/// If you need high performance mesh networking under all conditions consider XBee or similar.
class RHMesh : public RHRouter
{
public:

    /// The maximum length permitted for the application payload data in a RHMesh message
    #define RH_MESH_MAX_MESSAGE_LEN (RH_ROUTER_MAX_MESSAGE_LEN - sizeof(RHMesh::MeshMessageHeader))

    /// Structure of the basic RHMesh header.
    typedef struct
    {
	uint8_t             msgType;  ///< Type of RHMesh message, one of RH_MESH_MESSAGE_TYPE_*
    } MeshMessageHeader;

    /// Signals an application layer message for the caller of RHMesh
    typedef struct
    {
	MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_APPLICATION 
	uint8_t             data[RH_MESH_MAX_MESSAGE_LEN]; ///< Application layer payload data
    } MeshApplicationMessage;

    /// Signals a route discovery request or reply (At present only supports physical dest addresses of length 1 octet)
    typedef struct
    {
	MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
	uint8_t             destlen; ///< Reserved. Must be 1
	uint8_t             dest;    ///< The address of the destination node whose route is being sought
	uint8_t             route[RH_MESH_MAX_MESSAGE_LEN - 2]; ///< List of node addresses visited so far. Length is implcit
    } MeshRouteDiscoveryMessage;

    /// Signals a route failure
    typedef struct
    {
	MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE
	uint8_t             dest; ///< The address of the destination towards which the route failed
    } MeshRouteFailureMessage;

    /// Constructor. 
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    RHMesh(RHGenericDriver& driver, uint8_t thisAddress = 0);

    /// Sends a message to the destination node. Initialises the RHRouter message header 
    /// (the SOURCE address is set to the address of this node, HOPS to 0) and calls 
    /// route() which looks up in the routing table the next hop to deliver to.
    /// If no route is known, initiates route discovery and waits for a reply.
    /// Then sends the message to the next hop
    /// Then waits for an acknowledgement from the next hop 
    /// (but not from the destination node (if that is different).
    /// \param [in] buf The application message data
    /// \param [in] len Number of octets in the application message data. 0 is permitted
    /// \param [in] dest The destination node address. If the address is RH_BROADCAST_ADDRESS (255)
    /// the message will be broadcast to all the nearby nodes, but not routed or relayed.
    /// \param [in] flags Optional flags for use by subclasses or application layer, 
    ///             delivered end-to-end to the dest address. The receiver can recover the flags with recvFromAck().
    /// \return The result code:
    ///         - RH_ROUTER_ERROR_NONE Message was routed and delivered to the next hop 
    ///           (not necessarily to the final dest address)
    ///         - RH_ROUTER_ERROR_NO_ROUTE There was no route for dest in the local routing table
    ///         - RH_ROUTER_ERROR_UNABLE_TO_DELIVER Not able to deliver to the next hop 
    ///           (usually because it dod not acknowledge due to being off the air or out of range
    uint8_t sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags = 0);

    /// Starts the receiver if it is not running already, processes and possibly routes any received messages
    /// addressed to other nodes
    /// and delivers any messages addressed to this node.
    /// If there is a valid application layer message available for this node (or RH_BROADCAST_ADDRESS), 
    /// send an acknowledgement to the last hop
    /// address (blocking until this is complete), then copy the application message payload data
    /// to buf and return true
    /// else return false. 
    /// If a message is copied, *len is set to the length..
    /// If from is not NULL, the originator SOURCE address is placed in *source.
    /// If to is not NULL, the DEST address is placed in *dest. This might be this nodes address or 
    /// RH_BROADCAST_ADDRESS. 
    /// This is the preferred function for getting messages addressed to this node.
    /// If the message is not a broadcast, acknowledge to the sender before returning.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \param[in] source If present and not NULL, the referenced uint8_t will be set to the SOURCE address
    /// \param[in] dest If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// \param[in] hops If present and not NULL, the referenced uint8_t will be set to the HOPS
    /// (not just those addressed to this node).
    /// \return true if a valid message was received for this node and copied to buf
    bool recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL, uint8_t* hops = NULL);

    /// Starts the receiver if it is not running already.
    /// Similar to recvfromAck(), this will block until either a valid application layer 
    /// message available for this node
    /// or the timeout expires. 
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \param[in] timeout Maximum time to wait in milliseconds
    /// \param[in] source If present and not NULL, the referenced uint8_t will be set to the SOURCE address
    /// \param[in] dest If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// \param[in] hops If present and not NULL, the referenced uint8_t will be set to the HOPS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    bool recvfromAckTimeout(uint8_t* buf, uint8_t* len,  uint16_t timeout, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL, uint8_t* hops = NULL);

protected:

    /// Internal function that inspects messages being received and adjusts the routing table if necessary.
    /// Called by recvfromAck() immediately after it gets the message from RHReliableDatagram
    /// \param [in] message Pointer to the RHRouter message that was received.
    /// \param [in] messageLen Length of message in octets
    virtual void peekAtMessage(RoutedMessage* message, uint8_t messageLen);

    /// Internal function that inspects messages being received and adjusts the routing table if necessary.
    /// This is virtual, which lets subclasses override or intercept the route() function.
    /// Called by sendtoWait after the message header has been filled in.
    /// \param [in] message Pointer to the RHRouter message to be sent.
    /// \param [in] messageLen Length of message in octets
    virtual uint8_t route(RoutedMessage* message, uint8_t messageLen);

    /// Try to resolve a route for the given address. Blocks while discovering the route
    /// which may take up to 4000 msec.
    /// Virtual so subclasses can override.
    /// \param [in] address The physical address to resolve
    /// \return true if the address was resolved and added to the local routing table
    virtual bool doArp(uint8_t address);

    /// Tests if the given address of length addresslen is indentical to the
    /// physical address of this node.
    /// RHMesh always implements physical addresses as the 1 octet address of the node
    /// given by _thisAddress
    /// Called by recvfromAck() to test whether a RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST
    /// is for this node.
    /// Subclasses may want to override to implement more complicated or longer physical addresses
    /// \param [in] address Address of the pyysical addres being tested
    /// \param [in] addresslen Lengthof the address in bytes
    /// \return true if the physical address of this node is identical to address
    virtual bool isPhysicalAddress(uint8_t* address, uint8_t addresslen);

private:
    /// Temporary message buffer
    static uint8_t _tmpMessage[RH_ROUTER_MAX_MESSAGE_LEN];

};

/// @example rf22_mesh_client.ino
/// @example rf22_mesh_server1.ino
/// @example rf22_mesh_server2.ino
/// @example rf22_mesh_server3.ino

#endif


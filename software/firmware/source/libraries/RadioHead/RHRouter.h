// RHRouter.h
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHRouter.h,v 1.13 2020/08/04 09:02:14 mikem Exp $

#ifndef RHRouter_h
#define RHRouter_h

#include <RHReliableDatagram.h>

// Default max number of hops we will route
#define RH_DEFAULT_MAX_HOPS 30

// The default size of the routing table we keep
#define RH_ROUTING_TABLE_SIZE 10

// Error codes
#define RH_ROUTER_ERROR_NONE              0
#define RH_ROUTER_ERROR_INVALID_LENGTH    1
#define RH_ROUTER_ERROR_NO_ROUTE          2
#define RH_ROUTER_ERROR_TIMEOUT           3
#define RH_ROUTER_ERROR_NO_REPLY          4
#define RH_ROUTER_ERROR_UNABLE_TO_DELIVER 5

// This size of RH_ROUTER_MAX_MESSAGE_LEN is OK for Arduino Mega, but too big for
// Duemilanove. Size of 50 works with the sample router programs on Duemilanove.
#define RH_ROUTER_MAX_MESSAGE_LEN (RH_MAX_MESSAGE_LEN - sizeof(RHRouter::RoutedMessageHeader))
//#define RH_ROUTER_MAX_MESSAGE_LEN 50

// These allow us to define a simulated network topology for testing purposes
// See RHRouter.cpp for details
//#define RH_TEST_NETWORK 1
//#define RH_TEST_NETWORK 2
//#define RH_TEST_NETWORK 3
//#define RH_TEST_NETWORK 4

/////////////////////////////////////////////////////////////////////
/// \class RHRouter RHRouter.h <RHRouter.h>
/// \brief RHReliableDatagram subclass for sending addressed, optionally acknowledged datagrams
/// multi-hop routed across a network.
///
/// This is a Manager class that extends RHReliableDatagram to handle addressed messages
/// that are reliably transmitted and routed across a network of multiple RHRouter nodes.
/// Each message is transmitted reliably 
/// between each hop in order to get from the source node to the destination node.
///
/// With RHRouter, routes are hard wired. This means that each node must have programmed 
/// in it how to reach each of the other nodes it will be trying to communicate with. 
/// This means you must specify the next-hop node address for each of the destination nodes, 
/// using the addRouteTo() function. 
///
/// When sendtoWait() is called with a new message to deliver, and the destination address,
/// RHRouter looks up the next hop node for the destination node. It then uses 
/// RHReliableDatagram to (reliably) deliver the message to the next hop 
/// (which is expected also to be running an RHRouter). If that next-hop node is not
/// the final destination, it will also look up the next hop for the destination node and 
/// (reliably) deliver the message to the next hop. By this method, messages can be delivered 
/// across a network of nodes, even if each node cannot hear all of the others in the network.
/// Each time a message is received for another node and retransmitted to the next hop, 
/// the HOPS field in the header is incremented. If a message is received for routing to another node 
/// which has exceed the routers max_hops, the message will be dropped and ignored. 
/// This helps prevent infinite routing loops.
///
/// RHRouter supports messages with a dest of RH_BROADCAST_ADDRESS. Such messages are not routed, 
/// and are broadcast (once) to all nodes within range.
///
/// The recvfromAck() function is responsible not just for receiving and delivering 
/// messages addressed to this node (or RH_BROADCAST_ADDRESS), but 
/// it is also responsible for routing other message to their next hop. This means that it is important to 
/// call recvfromAck() or recvfromAckTimeout() frequently in your main loop. recvfromAck() will return 
/// false if it receives a message but it is not for this node.
///
/// RHRouter does not provide reliable end-to-end delivery, but uses reliable hop-to-hop delivery. 
/// If a message is unable to be delivered to an end node during to a delivery failure between 2 hops, 
/// the source node will not be told about it.
///
/// Note: This class is most useful for networks of nodes that are essentially static 
/// (i.e. the nodes dont move around), and for which the 
/// routing never changes. If that is not the case for your proposed network, see RHMesh instead.
///
/// \par The Routing Table
///
/// The routing table is a local table in RHRouter that holds the information about the next hop node 
/// address for each destination address you may want to send a message to. It is your responsibility 
/// to make sure every node in an RHRouter network has been configured with a unique address and the 
/// routing information so that messages are correctly routed across the network from source node to 
/// destination node. This is usually done once in setup() by calling addRouteTo(). 
/// The hardwired routing will in general be different on each node, and will depend on the physical 
/// topololgy of the network.
/// You can also use addRouteTo() to change a route and 
/// deleteRouteTo() to delete a route at run time. Youcan also clear the entire routing table
///
/// The Routing Table has limited capacity for entries (defined by RH_ROUTING_TABLE_SIZE, which is 10)
/// if more than RH_ROUTING_TABLE_SIZE are added, the oldest (first) one will be removed by calling 
/// retireOldestRoute()
///
/// \par Message Format
///
/// RHRouter add to the lower level RHReliableDatagram (and even lower level RH) class message formats. 
/// In those lower level classes, the hop-to-hop message headers are in the RH message headers, 
/// and are handled automcatically by tyhe RH hardware.
/// RHRouter and its subclasses add an end-to-end addressing header in the payload of the RH message, 
/// and before the RHRouter application data.
/// - 1 octet DEST, the destination node address (ie the address of the final 
///   destination node for this message)
/// - 1 octet SOURCE, the source node address (ie the address of the originating node that first sent 
///   the message).
/// - 1 octet HOPS, the number of hops this message has traversed so far.
/// - 1 octet ID, an incrementing message ID for end-to-end message tracking for use by subclasses. 
///   Not used by RHRouter.
/// - 1 octet FLAGS, a bitmask for use by subclasses. Not used by RHRouter.
/// - 0 or more octets DATA, the application payload data. The length of this data is implicit 
///   in the length of the entire message.
///
/// You should be careful to note that there are ID and FLAGS fields in the low level per-hop 
/// message header too. These are used only for hop-to-hop, and in general will be different to 
/// the ones at the RHRouter level.
///
/// \par Testing
///
/// Bench testing of such networks is notoriously difficult, especially simulating limited radio 
/// connectivity between some nodes.
/// To assist testing (both during RH development and for your own networks) 
/// RHRouter.cpp has the ability to 
/// simulate a number of different small network topologies. Each simulated network supports 4 nodes with 
/// addresses 1 to 4. It operates by pretending to not hear RH messages from certain other nodes.
/// You can enable testing with a \#define TEST_NETWORK in RHRouter.h
/// The sample programs rf22_mesh_* rely on this feature.
///
/// Part of the Arduino RH library for operating with HopeRF RH compatible transceivers 
/// (see http://www.hoperf.com)
class RHRouter : public RHReliableDatagram
{
public:

    /// Defines the structure of the RHRouter message header, used to keep track of end-to-end delivery parameters
    typedef struct
    {
	uint8_t    dest;       ///< Destination node address
	uint8_t    source;     ///< Originator node address
	uint8_t    hops;       ///< Hops traversed so far
	uint8_t    id;         ///< Originator sequence number
	uint8_t    flags;      ///< Originator flags
	// Data follows, Length is implicit in the overall message length
    } RoutedMessageHeader;

    /// Defines the structure of a RHRouter message
    typedef struct
    {
	RoutedMessageHeader header;    ///< end-to-end delivery header
	uint8_t             data[RH_ROUTER_MAX_MESSAGE_LEN]; ///< Application payload data
    } RoutedMessage;

    /// Values for the possible states for routes
    typedef enum
    {
	Invalid = 0,           ///< No valid route is known
	Discovering,           ///< Discovering a route (not currently used)
	Valid                  ///< Route is valid
    } RouteState;

    /// Defines an entry in the routing table
    typedef struct
    {
	uint8_t      dest;      ///< Destination node address
	uint8_t      next_hop;  ///< Send via this next hop address
	uint8_t      state;     ///< State of this route, one of RouteState
    } RoutingTableEntry;

    /// Constructor. 
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    RHRouter(RHGenericDriver& driver, uint8_t thisAddress = 0);

    /// Initialises this instance and the radio module connected to it.
    /// Overrides the init() function in RH.
    /// Sets max_hops to the default of RH_DEFAULT_MAX_HOPS (30)
    bool init();


    /// Sets the flag determining if the node will participate in routing.
    /// if isa_router is true, the node will be a full participant. If false the node
    /// will only respond to
    /// packets directed to its address, and act only as a leaf node in the network. The default is true.
    /// \param[in] isa_router true or false
    void setIsaRouter(bool isa_router);

    /// Sets the max_hops to the given value
    /// This controls the maximum number of hops allowed between source and destination nodes
    /// Messages that are not delivered by the time their HOPS field exceeds max_hops on a 
    /// routing node will be dropped and ignored.
    /// \param [in] max_hops The new value for max_hops
    void setMaxHops(uint8_t max_hops);

    /// Adds a route to the local routing table, or updates it if already present.
    /// If there is not enough room the oldest (first) route will be deleted by calling retireOldestRoute().
    /// \param [in] dest The destination node address. RH_BROADCAST_ADDRESS is permitted.
    /// \param [in] next_hop The address of the next hop to send messages destined for dest
    /// \param [in] state The satte of the route. Defaults to Valid
    void addRouteTo(uint8_t dest, uint8_t next_hop, uint8_t state = Valid);

    /// Finds and returns a RoutingTableEntry for the given destination node
    /// \param [in] dest The desired destination node address.
    /// \return pointer to a RoutingTableEntry for dest
    RoutingTableEntry* getRouteTo(uint8_t dest);

    /// Deletes from the local routing table any route for the destination node.
    /// \param [in] dest The destination node address
    /// \return true if the route was present
    bool deleteRouteTo(uint8_t dest);

    /// Deletes the oldest (first) route from the 
    /// local routing table
    void retireOldestRoute();

    /// Clears all entries from the 
    /// local routing table
    void clearRoutingTable();

    /// If RH_HAVE_SERIAL is defined, this will print out the contents of the local 
    /// routing table using Serial
    void printRoutingTable();

    /// Method for iterating through the current routing table
    /// \param [inout] RTE_p If a valid entry is found, the entry is copied to this structure
    ///    caller is responsible for alloocating and deallocating the structure.
    /// \param [inout] lastIndex_p points to the index to start searching from. Set to the
    ///   index of the next valid route found. Set this to -1 to start the search.
    /// \return false if next entry is valid, true if finished with table.
    bool getNextValidRoutingTableEntry(RoutingTableEntry *RTE_p, int *lastIndex_p); //blase 7/27/20 


    /// Sends a message to the destination node. Initialises the RHRouter message header 
    /// (the SOURCE address is set to the address of this node, HOPS to 0) and calls 
    /// route() which looks up in the routing table the next hop to deliver to and sends the 
    /// message to the next hop. Waits for an acknowledgement from the next hop 
    /// (but not from the destination node (if that is different).
    /// \param [in] buf The application message data
    /// \param [in] len Number of octets in the application message data. 0 is permitted
    /// \param [in] dest The destination node address
    /// \param [in] flags Optional flags for use by subclasses or application layer, 
    ///             delivered end-to-end to the dest address. The receiver can recover the flags with recvFromAck().
    /// \return The result code:
    ///         - RH_ROUTER_ERROR_NONE Message was routed and delivered to the next hop 
    ///           (not necessarily to the final dest address)
    ///         - RH_ROUTER_ERROR_NO_ROUTE There was no route for dest in the local routing table
    ///         - RH_ROUTER_ERROR_UNABLE_TO_DELIVER Not able to deliver to the next hop 
    ///           (usually because it dod not acknowledge due to being off the air or out of range
    uint8_t sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags = 0);

    /// Similar to sendtoWait() above, but spoofs the source address.
    /// For internal use only during routing
    /// \param [in] buf The application message data.
    /// \param [in] len Number of octets in the application message data. 0 is permitted.
    /// \param [in] dest The destination node address.
    /// \param [in] source The (fake) originating node address.
    /// \param [in] flags Optional flags for use by subclasses or application layer, 
    ///             delivered end-to-end to the dest address. The receiver can recover the flags with recvFromAck().
    /// \return The result code:
    ///         - RH_ROUTER_ERROR_NONE Message was routed and deliverd to the next hop 
    ///           (not necessarily to the final dest address)
    ///         - RH_ROUTER_ERROR_NO_ROUTE There was no route for dest in the local routing table
    ///         - RH_ROUTER_ERROR_UNABLE_TO_DELIVER Noyt able to deliver to the next hop 
    ///           (usually because it dod not acknowledge due to being off the air or out of range
    uint8_t sendtoFromSourceWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t source, uint8_t flags = 0);

    /// Starts the receiver if it is not running already.
    /// If there is a valid message available for this node (or RH_BROADCAST_ADDRESS), 
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
    /// \return true if a valid message was recvived for this node copied to buf
    bool recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL, uint8_t* hops = NULL);

    /// Starts the receiver if it is not running already.
    /// Similar to recvfromAck(), this will block until either a valid message available for this node
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

    /// Lets sublasses peek at messages going 
    /// past before routing or local delivery.
    /// Called by recvfromAck() immediately after it gets the message from RHReliableDatagram
    /// \param [in] message Pointer to the RHRouter message that was received.
    /// \param [in] messageLen Length of message in octets
    virtual void peekAtMessage(RoutedMessage* message, uint8_t messageLen);

    /// Finds the next-hop route and sends the message via RHReliableDatagram::sendtoWait().
    /// This is virtual, which lets subclasses override or intercept the route() function.
    /// Called by sendtoWait after the message header has been filled in.
    /// \param [in] message Pointer to the RHRouter message to be sent.
    /// \param [in] messageLen Length of message in octets
    virtual uint8_t route(RoutedMessage* message, uint8_t messageLen);

    /// Deletes a specific rout entry from therouting table
    /// \param [in] index The 0 based index of the routing table entry to delete
    void deleteRoute(uint8_t index);

    /// The last end-to-end sequence number to be used
    /// Defaults to 0
    uint8_t _lastE2ESequenceNumber;

    /// The maximum number of hops permitted in routed messages.
    /// If a routed message would exceed this number of hops it is dropped and ignored.
    uint8_t              _max_hops;
    
    /// Flag to set if packets are forwarded or not
    bool _isa_router;

private:

    /// Temporary mesage buffer
    static RoutedMessage _tmpMessage;

    /// Local routing table
    RoutingTableEntry    _routes[RH_ROUTING_TABLE_SIZE];
};

/// @example rf22_router_client.ino
/// @example rf22_router_server1.ino
/// @example rf22_router_server2.ino
/// @example rf22_router_server3.ino
#endif


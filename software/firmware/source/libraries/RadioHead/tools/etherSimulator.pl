#!/usr/bin/perl
#
# etherSimulator.pl
# Simulates the luminiferous ether for RH_Simulator.
# Connects multiple instances of RH_Simulator clients together and passes 
# simulated messages between them.

use Getopt::Long;
use strict;

# Configurable variables
my $help;
my $config;
my $port = 4000;
$port = $main::opt_p
    if $main::opt_p;
my $bps = 10000;
$bps = $main::opt_b
    if $main::opt_b;

# Config that shows probability of successful transmission between nodes
# Read from config file
my %netconfig;

use warnings;
use POE qw(Component::Server::TCP Filter::Block);
use strict;

my @options = 
    (
     'h'     => \$help,                # Help, show usage
     'c=s'   => \$config,              # Config file
     'b=n'   => \$bps,                 # Bits per second simulated baud rate
     'p=n'   => \$port,                # port number
    );

&GetOptions(@options) || &usage;
&usage if $help;

readConfig($config) if defined $config;

sub usage
{
    print "usage: $0 [-h] [-c configfile] [-b bitspersec] [-p portnumber]\n";
    exit;
}

# config file for etherSimulator.pl
# Specify the probability of correct delivery between nodea and nodeb (bidirectional)
# probability:nodea:nodeb:probability
# nodea and nodeb are integers 0 to 255
# probability is a float range 0.0 to 1.0
# In this example, the probability of successful transmission
# between nodes 10 and 2 (and vice versa) is given as 0.5 (ie 50% chance)
# probability:10:2:0.5
sub readConfig
{
    my ($config) = @_;

    if (open(CONFIG, $config))
    {
	while (<CONFIG>)
	{
	    if (/^probability:(\d{1,3}):(\d{1,3}):(\d+(\.\d+))/)
	    {
		$netconfig{$1}{$2} = $3;
		$netconfig{$2}{$1} = $3; # Bidirectional
	    }
	}
	close(CONFIG);
    }
    else
    {
	print STDERR "Could not open config file $config: $!\n";
	exit;
    }
}

# See RHTcpProtocol.h
# messages to and from us are preceded by the payload length as uint32_t in network byte order
sub encoder
{
    my $stuff = shift;
    substr($$stuff, 0, 0) = pack('N', length($$stuff));
    return;
}

sub decoder
{
    my $stuff = shift;
    return if (length($$stuff) < 4);
    my ($length) = unpack('N', $$stuff);
    return if (length($$stuff) < $length+4);
    return $length + 4;
}

# Filter to assemble and disassemble messages accordiong to precending length
my $filter = POE::Filter::Block->new( LengthCodec => [ \&encoder, \&decoder ] );

# Message types
# See RH_TcpProtocol.h
my $RH_TCP_MESSAGE_TYPE_NOP                = 0; # Not used
my $RH_TCP_MESSAGE_TYPE_THISADDRESS        = 1; # Specifies the thisAddress of the connected sketch
my $RH_TCP_MESSAGE_TYPE_PACKET             = 2; # Message to/from the connected sketch

my %clients;

# Look up the source and dest nodes in the netconfig and return the 0.0 to 1.0 probability
# of successful delivery
sub probabilityOfSuccessfulDelivery
{
    my ($from, $to) = @_;

    return $netconfig{$from}{$to}
        if exists $netconfig{$from}{$to};
    # If no explicit probability, use 1.0 (certainty)
    return 1.0;
}

# Return true if the message is simulted to have been received successfully
# taking into account the probability of sucessful delivery
sub willDeliverFromTo
{
    my ($from, $to) = @_;

    my $prob = probabilityOfSuccessfulDelivery($from, $to);
    return 1 
	if rand() < $prob;
    return 0;
}

sub deliverMessages
{
    my ($key, $value);
    while (($key, $value) = each(%clients))
    {
	next unless defined $$value{'packet'}; # No packet waiting for delivery
	# Find how long since the message was transmitted and see it its time to 
	# deliver it to the client.
	# We are waiting here for the transmission time of the message to elapse
	# given the message length and the bits per second
	my $elapsed = Time::HiRes::tv_interval([$$value{'packetreceived'}], [Time::HiRes::gettimeofday]);
	if ($elapsed > length($$value{'packet'}) * 8 / $bps)
	{
	    $$value{'client'}->put(pack('Ca*', $RH_TCP_MESSAGE_TYPE_PACKET, $$value{'packet'}));
	    delete $$value{'packet'}; # Delivered, forget it
	}
    }
}

POE::Session->create(
  inline_states => {
    _start => sub {
      $_[KERNEL]->delay(tick => 1);
    },

    tick => sub {
      deliverMessages();
      $_[KERNEL]->delay(tick => 0.001);
    },
  },
);

POE::Component::Server::TCP->new(
    Port => $port,

    ClientConnected => sub {
	my $client = $_[HEAP]{client};
	# Create a new object to hold data about RH_TCP messages to and from this client
	$clients{$client} = {'client' => $client};
    },

    ClientInput => sub {
	my $client = $_[HEAP]{client};
	my $client_input = $_[ARG0];
	my $client_id =  $_[ARG1];
	my ($length, $type) = unpack('NC', $client_input);
	if ($type == $RH_TCP_MESSAGE_TYPE_THISADDRESS)
	{
	    # Client notifies us of its node ID
	    my ($length, $type, $thisaddress) = unpack('NCC', $client_input);
	    # Set the client objects thisaddress
	    $clients{$client}{'thisaddress'} = $thisaddress;
	}
	elsif ($type == $RH_TCP_MESSAGE_TYPE_PACKET)
	{
	    # New packet for transmission
	    my ($length, $type, $packet) = unpack('NCa*', $client_input);
	    # Try to deliver the packet to all the other clients
	    my ($key, $value);
	    while (($key, $value) = each(%clients))
	    {
		next if ($key eq $client); # Dont deliver back to the same client

		# Check the network config and see if delivery to this node is possible
		next unless willDeliverFromTo($clients{$client}{'thisaddress'}, $$value{thisaddress});

		# The packet reached this destination, see if it collided with 
		# another packet
		if (defined $$value{'packet'})
		{
		    # Collision with waiting packet, delete it
		    delete $$value{'packet'};
		}
		else
		{
		    # New packet, queue it for delivery to the client after the
		    # nominal transmission time is complete
		    $$value{'packet'} = $packet;
		    $$value{'packetreceived'} = Time::HiRes::gettimeofday();
		}
	    }
	}
    },

    ClientDisconnected => sub {
	my $client = $_[HEAP]{client};
	delete $clients{$client};
    },
    ClientFilter => $filter, # Handles prepended lengths to 
    );

POE::Kernel->run;
exit;

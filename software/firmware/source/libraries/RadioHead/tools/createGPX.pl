#!/usr/bin/perl
#
# createGPX.pl
# Takes the dumps from rf95_client2 and rf95_server2, determines which messages 
# were received on both ends and produces 3 tracks that can be load as gpx-files
# into google maps. This can be used to determine the range between client and
# server or visualize where communication between server and client was possible.  
# The server gps data is not used, i.e. it is assumed that the server is stationary.
#
# 1) Track containing all points that were sent by rf95_client2, received by 
#    rf95_server2 +  the response of rf95_server2 was received by rf95_client2  
# 2) Track containng all points that were sent by rf95_client2, received by  
#    rf95_server2, but the response of rf95_server2 was NOT recevied by 
#    rf95_client2
# 3) Track containg all points that were sent by rf95_client2, but not 
#    received by rf95_server2. Consequently rf95_server did not sent a 
#    response.
#
# To dump the output, the programs can be started as follows:
#  On server: rf95_server2 > dump_file_server 
#  On client: rf95_client2 > dump_file_client

use Geo::Gpx;
use Getopt::Long;
use strict;

my %serverMessages;
my %clientMessages;
my %allMessages;
my $sentOnlyMessages = Geo::Gpx->new();
my $receivedOnlyMessages = Geo::Gpx->new();
my $acknowledgedMessages = Geo::Gpx->new();
my $help;
my $serverFile;
my $clientFile;
my $resultFilePrefix;

my @options = 
    (
     'h'     => \$help,                # Help, show usage
     's=s'   => \$serverFile,          # file containing dump of server
     'c=s'   => \$clientFile,          # file containing dump of client
     'p=s'   => \$resultFilePrefix     # prefix for 3 results files
    );

&GetOptions(@options) || &usage;
&usage if $help;
if ($serverFile ne "")
{
 print "ServerFile: " . $serverFile . "\n";
 &readServerFile($serverFile);
 #&dumpServerData;
}
if ($clientFile ne "")
{
 print "ClientFile: " . $clientFile . "\n";
 &readClientFile($clientFile);
 #&dumpClientData;
}
&classifyData if (($serverFile ne "") && ($clientFile ne ""));

if ($resultFilePrefix ne "")
{
 print "Prefix of resulting gpx-files: " . $resultFilePrefix . "\n";
 open SENTONLYFILE, ">" . $resultFilePrefix . "_SentOnly.gpx";
 open RECEIVEDONLYFILE, ">" . $resultFilePrefix . "_ReceivedOnly.gpx";
 open ACKNOWLEDGEFILE, ">" . $resultFilePrefix . "_Acknowleded.gpx";

 print SENTONLYFILE $sentOnlyMessages->xml( '1.1' );
 print RECEIVEDONLYFILE $receivedOnlyMessages->xml( '1.1' ) ;
 print ACKNOWLEDGEFILE $acknowledgedMessages->xml( '1.1' ) ;

 close SENTONLYFILE;
 close RECEIVEDONLYFILE;
 close ACKNOWLEDGEFILE; 
}

sub usage
{
    print "usage: $0 [-h] [-s serverfile] [-c clientfile] [-p prefix for result files] \n";
    exit;
}


###################################################################################
# classifyData  
#
# Using the sequence number and the timpstamp of the client data, the messages
# are classified in 3 groups:
# (1) full contact, i.e. message and its reply was received on both ends
# (2) received only, i.e. message was received by server, but reply to client 
#     got lost
# (3) no contact, i.e. message was sent, but not received by server 
#
###################################################################################
sub classifyData
{
  my $item;
  my $seqNr;
  my $timestamp; 
  my $longitude;
  my $latitude;
  my $reply;
  my $rssiAtServer;
  my $serverLongitude;
  my $serverLatitude;
  my $serverReply;

  foreach $item (sort keys %clientMessages)
  {
      ($latitude,$longitude,$reply) = split(/~/,$clientMessages{$item});
      ($seqNr,$timestamp) = split(/~/,$item);

      #print "$item => $clientMessages{$item}\n";
      if ($latitude =~ /(\d\d)(\d\d\.\d\d\d\d)/)
      {
         #print $latitude;
         $latitude = $1 +  $2/60; 
         #print "Latitude  " . $latitude . "\n";
      }
      if ($longitude =~ /(\d\d\d)(\d\d\.\d\d\d\d)/)
      {
         #print $longitude;
         $longitude = $1 +  $2/60; 
         #print "Longitude  " . $longitude . "\n";
      }

      if ($serverMessages{$item})
      {
         # this item was received by the server 
         # print "$item => $clientMessages{$item}\n";
         # print "$item => $serverMessages{$item}\n";
         # print "\n\n";
         ($serverLatitude,$serverLongitude,$rssiAtServer, $serverReply) = split(/~/,$serverMessages{$item});

         if ($reply eq "NULL")
         {
          # client message received, but server message lost
          #print "RECEIVED ONLY; $item => $clientMessages{$item}\n\n\n";

          my $wpt = {
                 lat           => $latitude,
                 lon           => $longitude,
                 name          => $seqNr . "@" . $timestamp,
                 cmt           => 'sent and received with RSSI=' .  $rssiAtServer . ', but response got lost',
          };
          $receivedOnlyMessages->add_waypoint( $wpt );
         }
         else
         {
          # client message received by server, client received acknowledge message sent by server 
          # print "ACKNOWLEDGED; $item => $clientMessages{$item}\n\n\n";

          # sanity check: the reply string should be identical
          if ($reply ne $serverReply) 
          {
            print "WARNING: Sent and received messages not identical (Sent=$reply/Received=$serverReply)...\n";
          }
          my $wpt = {
                 lat           => $latitude,
                 lon           => $longitude,
                 name          => $seqNr . "@" . $timestamp,
                 cmt           => 'sent and received with RSSI=' .  $rssiAtServer . ', acknowledge received by client',
          };
          $acknowledgedMessages->add_waypoint( $wpt );
         }
      }
      else
      {
        # client message was sent but not received
        # print "SENTONLY; $item => $clientMessages{$item}\n\n\n";
         if ($reply eq "NULL")
         {
            my $wpt = {
                 lat           => $latitude,
                 lon           => $longitude,
                 name          => $seqNr . "@" . $timestamp,
                 cmt           => 'sent but not received',
            };
            $sentOnlyMessages->add_waypoint( $wpt );
         }
         else
         {
            my $seqNr;
            my $timestamp;
            ($seqNr,$timestamp) = split (/~/, $item);
            print "WARNING: There should by no reply to message $seqNr @ $timestamp...\n";
         }
      }
   }


   # sanity check: Verify that each of the server message
   # was actually sent by the client
   foreach $item (sort keys %serverMessages)
   {
      if (not exists ($clientMessages{$item}) )
      {
        ($seqNr,$timestamp) = split(/~/,$item);
        ($serverLatitude,$serverLongitude,$rssiAtServer, $serverReply) = split(/~/,$serverMessages{$item});
        print "WARNING: Data inconistency in $seqNr @ $timestamp. Message received but never sent...\n"
      }
   }
}

###################################################################################
# readServerFile
#
# read data in format created by rf95_server2. To create file, standard output of 
# rf95_server2 needs to be redirected into a file, i.e. rf95_server2 > file.log
###################################################################################
sub readServerFile
{
    my ($serverFile) = @_;
    if (open(SERVERFILE, $serverFile))
    {
        while (<SERVERFILE>)
        {
           # format: got request: "(00004):163213.000,4846.8686N,00912.2478E" 
           #         RSSI: -82
           #         Sent a reply: "R:(00004):20191220163216RM,4846.8303N,00912.27"
           if (/^got request:\ \"\((\d{5})\):([\d|\.]+),([\d|\.]+)([N|S]),([\d|\.]+)([E|W])\"/)
           {
                my $seqNr = $1;
                my $timestamp= $2;
                my $latitude = $3;
                my $longitude = $5;
                my $rssi;
                my $reply;
                $longitude = (-1) * $longitude if ($6 eq "W");
		$latitude = (-1) * $latitude if ($4 eq "S");
               
                # get RSSI 
                my $line = <SERVERFILE>;
                if ($line =~ /RSSI:\ ([-|+|\d]+)/)
                {
                   $rssi = $1;
                }

                # get reply
                $line = <SERVERFILE>; 
                if ($line =~ /Sent a reply:\ \"(.+)\"/)
                {
                   $reply=$1;
                   if ($reply =~ /R:\((\d{5})\)/) 
                   {
                      if ($1 ne $seqNr)
                      {
                        print "WARNING: File structure seems to be broken. No matching reply for sequence numer '$seqNr'.";     
                        print "File name: $serverFile\n";
                      }
                   }
                }
		$serverMessages{$seqNr ."~" . $timestamp} = $latitude . "~" . $longitude . "~" . $rssi ."~" . $reply ;
           }

        }
        close(SERVERFILE);
    }
    else
    {
        print STDERR "Could not open config file $serverFile: $!\n";
        exit;
    }
}
 
###################################################################################
# readClientFile
#
# read data in format created by rf95_client2. To create file, standard output of 
# rf95_client2 needs to be redirected into a file, i.e. rf95_client2 > file.log
###################################################################################
sub readClientFile
{
    my ($clientFile) = @_;
    if (open(CLIENTFILE, $clientFile))
    {
        while (<CLIENTFILE>)
        {
           # format: Message="(00004):165718.000,4846.8668N,00912.2344E"
           #         got reply: "R:(00004):20191220165722RM,4846.8729N,00912.24"
           # or
           #         Message="(00206):172330.000,4846.4587N,00912.3557E"
           #         No reply, is rf95_server running?

           if (/^\Message="\((\d{5})\):(.+),(.+)([N|S]),(.+)([E|W])\"/)
           {
                my $seqNr = $1;
                my $timestamp= $2;
                my $latitude = $3;
                my $longitude = $5;
                my $reply;
                $longitude = (-1) * $longitude if ($6 eq "W");
		$latitude = (-1) * $latitude if ($4 eq "S");
               
                # get reply
                my $line = <CLIENTFILE>; 
                if ($line =~ /got reply:\ \"(.+)\"/)
                {
                   $reply=$1;
                   if ($reply =~ /R:\((\d{5})\)/) 
                   {
                      if ($1 ne $seqNr)
                      {
                        print "WARNING: File structure seems to be broken. No matching reply for sequence numer '$seqNr'.";     
                        print "File name: $clientFile\n";
                      }
                   }
                }
		if ($line =~ /No reply/)
                {
                   $reply = "NULL";
                }
		$clientMessages{$seqNr ."~" . $timestamp} = $latitude . "~" . $longitude . "~"  . $reply ;
           }

        }
        close(CLIENTFILE);
    }
    else
    {
        print STDERR "Could not open config file $clientFile: $!\n";
        exit;
    }
}


###################################################################################
# dumpServerData
#
# Once the data file of rf95_server2 is successfully read, it can be dumped with
# this routine. This routine is intended for debugging
# 
###################################################################################
sub dumpServerData()
{
  my $item;
  foreach $item (sort keys %serverMessages)
  {
      print "$item => $serverMessages{$item}\n";
  }
}


###################################################################################
# dumpClientData
#
# Once the data file of rf95_client2 is successfully read, it can be dumped with
# this routine. This routine is intended for debugging
# 
###################################################################################
sub dumpClientData()
{
  my $item;
  foreach $item (sort keys %clientMessages)
  {
      print "$item => $clientMessages{$item}\n";
  }
}

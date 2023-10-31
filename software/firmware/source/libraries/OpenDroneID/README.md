# ID Open
An Arduino/ESP32 class to act as a wrapper around opendroneid.

Supports BLE 4, WiFi NAN and WiFi beacon.

Runs on a cheap ESP32 dev board.

Needs opendroneid.c, opendroneid.h, odid_wifi.h and wifi.c from [opendroneid](https://github.com/opendroneid/opendroneid-core-c/tree/master/libopendroneid) to be copied into the id_open directory.

Last tested with opendroneid release 2.0.

If you are thinking of using this to make remote IDs for use in the US or EU, there are problems.
  * It looks like both of these jurisdictions are going to require IDs to transmit ANSI/CTA serial numbers. (See the table at the bottom of [this page](https://github.com/opendroneid/opendroneid-core-c/).)
  * The FAA are requiring remote IDs to be tamper resistant (see their [acceptance of the ASTM Means of Compliance](https://www.federalregister.gov/documents/2022/08/11/2022-16997/accepted-means-of-compliance-remote-identification-of-unmanned-aircraft) ). I don't see how this can be done with an open source, home built ID.

There is a report of an ESP32 that will not simultaneously do WiFi and Bluetooth remote ID (see issue #18). If your ESP32 goes into a reboot loop when both are enabled, try one or the other. This may be related to the ESP IDF version.

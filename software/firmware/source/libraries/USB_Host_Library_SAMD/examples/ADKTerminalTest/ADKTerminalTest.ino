/*
  Copyright (c) 2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
//#include "variant.h"
#include "Arduino.h"
#include <stdio.h>
#include <adk.h>


// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost usb;
ADK adk(&usb,"Arduino SA",
            "Arduino_Terminal",
            "Arduino Terminal for Android",
            "1.0",
            "http://labs.arduino.cc/uploads/ADK/ArduinoTerminal/ThibaultTerminal_ICS_0001.apk",
            "1");

void setup(void)
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("\r\nADK demo start");

  if (usb.Init())
	SerialDebug.println("USB host did not start.");

  delay(20);
}

#define RCVSIZE 128

void loop(void)
{
	uint8_t buf[RCVSIZE];
	uint16_t nbread = RCVSIZE;
	char helloworld[] = "Hello World!\r\n";

	usb.Task();

	if( adk.isReady() == false ) {
		return;
	}
	/* Write hello string to ADK */
	adk.SndData(strlen(helloworld), (uint8_t *)helloworld);

	delay(1000);

	/* Read data from ADK and print to UART */
	adk.RcvData(&nbread, buf);
	if (nbread > 0)
	{
		SerialDebug.print("RCV: ");
		for (uint32_t i = 0; i < nbread; ++i)
		{
			SerialDebug.print((char)buf[i]);
		}
		SerialDebug.print("\r\n");
	}	
}

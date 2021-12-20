/*
 Example sketch for the RFCOMM/SPP Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <SPP.h>
#include <usbhub.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
//USBHub Hub1(&UsbH); // Some dongles have a hub inside

BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so

const uint8_t length = 2; // Set the number of instances here
SPP *SerialBT[length]; // We will use this pointer to store the instances, you can easily make it larger if you like, but it will use a lot of RAM!

bool firstMessage[length] = { true }; // Set all to true

void setup() {
  for (uint8_t i = 0; i < length; i++)
    SerialBT[i] = new SPP(&Btd); // This will set the name to the default: "Arduino" and the pin to "0000" for all connections

  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // Halt
  }
  SerialDebug.print(F("\r\nSPP Bluetooth Library Started"));
}

void loop() {
  UsbH.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  for (uint8_t i = 0; i < length; i++) {
    if (SerialBT[i]->connected) {
      if (firstMessage[i]) {
        firstMessage[i] = false;
        SerialBT[i]->println(F("Hello from Arduino")); // Send welcome message
      }
      if (SerialBT[i]->available())
        SerialDebug.write(SerialBT[i]->read());
    }
    else
      firstMessage[i] = true;
  }

  // Set the connection you want to send to using the first character
  // For instance "0Hello World" would send "Hello World" to connection 0
  if (SerialDebug.available()) {
    delay(10); // Wait for the rest of the data to arrive
    uint8_t id = SerialDebug.read() - '0'; // Convert from ASCII
    if (id < length && SerialBT[id]->connected) { // Make sure that the id is valid and make sure that a device is actually connected
      while (SerialDebug.available()) // Check if data is available
        SerialBT[id]->write(SerialDebug.read()); // Send the data
    }
  }
}

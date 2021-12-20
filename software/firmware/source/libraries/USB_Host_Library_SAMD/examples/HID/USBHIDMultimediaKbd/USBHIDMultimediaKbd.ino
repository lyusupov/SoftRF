#include <hidcomposite.h>
#include <usbhub.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

// Override HIDComposite to be able to select which interface we want to hook into
class HIDSelector : public HIDComposite
{
public:
    HIDSelector(USBHost *p) : HIDComposite(p) {};

protected:
    void ParseHIDData(HID *hid, uint8_t ep, bool is_rpt_id, uint8_t len, uint8_t *buf); // Called by the HIDComposite library
    bool SelectInterface(uint8_t iface, uint8_t proto);
};

// Return true for the interface we want to hook into
bool HIDSelector::SelectInterface(uint8_t iface, uint8_t proto)
{
  SerialDebug.print("HIDSelector::SelectInterface("); SerialDebug.print(iface);
  SerialDebug.print(','); SerialDebug.print(proto); SerialDebug.println(')');
  if (proto != 0)
    return true;

  return false;
}

// Will be called for all HID data received from the USB interface
void HIDSelector::ParseHIDData(HID *hid, uint8_t ep, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  SerialDebug.print("HIDSelector::ParseHIDData(,"); SerialDebug.print(ep);
  SerialDebug.print(','); SerialDebug.print(is_rpt_id);
  SerialDebug.print(','); SerialDebug.print(len);
  SerialDebug.print(','); SerialDebug.print((uint32_t)buf, HEX); SerialDebug.println(')');
  if (len && buf)  {
    for (uint8_t i = 0; i < len; i++) {
      SerialDebug.print(buf[i], HEX); SerialDebug.print(' ');
    }
    SerialDebug.println();
  }
}

USBHost     UsbH;
//USBHub     Hub(&UsbH);
HIDSelector    hidSelector(&UsbH);

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
    SerialDebug.println("USB host did not start.");

  delay( 200 );
}

void loop()
{
  UsbH.Task();
}

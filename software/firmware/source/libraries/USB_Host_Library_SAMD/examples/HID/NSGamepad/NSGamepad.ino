/* Hori HoriPAD gamepad driver for Nintendo Switch */

#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

const char *button_names[] = {
  "Y",
  "B",
  "A",
  "X",
  "LeftTrigger",
  "RightTrigger",
  "LeftThrottle",
  "RightThrottle",
  "Minus",
  "Plus",
  "LeftStick",
  "RightStick",
  "Home",
  "Capture",
  "Reserved1",
  "Reserved2",
};

enum NSButtons {
  NSButton_Y = 0,
  NSButton_B,
  NSButton_A,
  NSButton_X,
  NSButton_LeftTrigger,
  NSButton_RightTrigger,
  NSButton_LeftThrottle,
  NSButton_RightThrottle,
  NSButton_Minus,
  NSButton_Plus,
  NSButton_LeftStick,
  NSButton_RightStick,
  NSButton_Home,
  NSButton_Capture,
  NSButton_Reserved1,
  NSButton_Reserved2,
};

struct GamePadEventData
{
  uint16_t  buttons;
  uint8_t   dPad;
  uint8_t   leftXAxis;
  uint8_t   leftYAxis;
  uint8_t   rightXAxis;
  uint8_t   rightYAxis;
  uint8_t   filler;
}__attribute__((packed));

class JoystickEvents
{
  public:
    virtual void OnGamePadChanged(const GamePadEventData *evt);
};

#define RPT_GAMEPAD_LEN sizeof(GamePadEventData)

class JoystickReportParser : public HIDReportParser
{
  JoystickEvents *joyEvents;

  uint8_t oldPad[RPT_GAMEPAD_LEN];

  public:
  JoystickReportParser(JoystickEvents *evt);

  virtual void Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf);
};


JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
  joyEvents(evt)
{}

void JoystickReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
  // Checking if there are changes in report since the method was last called
  bool match = (sizeof(oldPad) == len) && (memcmp(oldPad, buf, len) == 0);

  // Calling Game Pad event handler
  if (!match && joyEvents) {
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);
    memcpy(oldPad, buf, len);
  }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt)
{
  Serial.print("left_X: ");
  PrintHex<uint16_t>(evt->leftXAxis, 0x80);
  Serial.print(" left_Y: ");
  PrintHex<uint16_t>(evt->leftYAxis, 0x80);
  Serial.print(" right_X: ");
  PrintHex<uint16_t>(evt->rightXAxis, 0x80);
  Serial.print(" right_Y: ");
  PrintHex<uint16_t>(evt->rightYAxis, 0x80);
  Serial.print(" dPad: ");
  PrintHex<uint8_t>(evt->dPad, 0x80);
  Serial.print(" Buttons: ");
  PrintHex<uint16_t>(evt->buttons, 0x80);
  Serial.print(' ');
  if (evt->buttons) {
    for (int i = 0; i < 16; i++) {
      uint16_t button_mask = 1 << i;
      if (evt->buttons & button_mask) {
        Serial.print(button_names[i]);
        Serial.print(',');
      }
    }
  }
  Serial.println();
}

USBHost                                         UsbH;
USBHub                                          Hub(&UsbH);
HIDUniversal                                    Hid(&UsbH);
JoystickEvents                                  JoyEvents;
JoystickReportParser                            Joy(&JoyEvents);

void setup()
{
  Serial.begin( 115200 );
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  if (UsbH.Init())
    Serial.println("USB host did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );
}

void loop()
{
  UsbH.Task();
}

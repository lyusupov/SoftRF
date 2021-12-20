/*
 * Works with Dymo M10 USB postage scale. This scale automatically powers off
 * after 3 minutes of no activity. This is based on the scale example but
 * removes the LCD code.
 */

#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

const char * const UNITS[] = {
  "units",        // unknown unit
  "mg",           // milligram
  "g",            // gram
  "kg",           // kilogram
  "cd",           // carat
  "taels",        // lian
  "gr",           // grain
  "dwt",          // pennyweight
  "tonnes",       // metric tons
  "tons",         // avoir ton
  "ozt",          // troy ounce
  "oz",           // ounce
  "lbs"           // pound
};

/* Scale status constants */
#define REPORT_FAULT    0x01
#define ZEROED          0x02
#define WEIGHING        0x03
#define WEIGHT_VALID    0x04
#define WEIGHT_NEGATIVE 0x05
#define OVERWEIGHT      0x06
#define CALIBRATE_ME    0x07
#define ZERO_ME         0x08

const char * const STATUS[] = {
  NULL,                         // 0
  "Report fault",               // 1
  "Scale zero set",             // 2
  "Weighing...",                // 3
  "Weight: ",                   // 4
  "Negative weight",            // 5
  "Max weight reached",         // 6
  "Scale calibration required", // 7
  "Scale zeroing required"      // 8
};

/* input data report */
struct ScaleEventData
{
  uint8_t reportID;	//must be 3
  uint8_t status;
  uint8_t unit;
  int8_t exp;			//scale factor for the weight
  uint16_t weight;	//
};

class ScaleEvents
{

  public:

    virtual void OnScaleChanged(const ScaleEventData *evt);
};

#define RPT_SCALE_LEN	sizeof(ScaleEventData)/sizeof(uint8_t)

class ScaleReportParser : public HIDReportParser
{
  public:
    ScaleReportParser(ScaleEvents *evt);
    virtual void Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf);

  private:
    uint8_t oldScale[RPT_SCALE_LEN];
    ScaleEvents		*scaleEvents;
};

ScaleReportParser::ScaleReportParser(ScaleEvents *evt) :
  scaleEvents(evt)
{
	memset(oldScale, 0, RPT_SCALE_LEN);
}

void ScaleReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
  // If new report same as old report, ignore it.
  if (len != RPT_SCALE_LEN) return;
  if (memcmp(buf, oldScale, RPT_SCALE_LEN) == 0) return;

  // Calling scale event handler
  if (scaleEvents) {
    scaleEvents->OnScaleChanged((const ScaleEventData*)buf);
    memcpy(oldScale, buf, RPT_SCALE_LEN);
  }
}

void ScaleEvents::OnScaleChanged(const ScaleEventData *evt)
{
  if( evt->reportID != 3 ) {
    SerialDebug.println("Invalid report!");
    return;
  }

  switch( evt->status ) {
    case REPORT_FAULT:
    case ZEROED:
    case WEIGHING:
    case WEIGHT_NEGATIVE:
    case OVERWEIGHT:
    case CALIBRATE_ME:
    case ZERO_ME:
      SerialDebug.println(STATUS[evt->status]);
      break;

    case WEIGHT_VALID:
      SerialDebug.print( STATUS[evt->status] );
      SerialDebug.print( evt->weight * pow( 10, evt->exp ) );
      SerialDebug.print(' ');
      SerialDebug.println( UNITS[ evt->unit ]);
      break;

    default:
      SerialDebug.print("Undefined status code: ");
      SerialDebug.println( evt->status );
      break;
  }
}

USBHost             UsbH;
USBHub              Hub(&UsbH);
HIDUniversal        Hid(&UsbH);
ScaleEvents         Events;
ScaleReportParser   Scale(&Events);

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
    SerialDebug.println("USB host did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Scale))
    SerialDebug.println("SetReportParser failed");
}

void loop()
{
  UsbH.Task();
}

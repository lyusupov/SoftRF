/* USB Host to PL2303-based USB GPS unit interface */
/* Navibee GM720 receiver - Sirf Star III */
/* Mikal Hart's TinyGPS library */
/* test_with_gps_device library example modified for PL2302 access */

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

/* USB support */
#include <usbhub.h>

/* CDC support */
#include <cdcacm.h>
#include <cdcprolific.h>

#include <TinyGPS.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
    Modified to be used with USB Host Shield Library r2.0
    and USB Host Shield 2.0
*/

class PLAsyncOper : public CDCAsyncOper
{
public:
    uint8_t OnInit(ACM *pacm);
};

uint8_t PLAsyncOper::OnInit(ACM *pacm)
{
    uint8_t rcode;

    // Set DTR = 1
    rcode = pacm->SetControlLineState(1);

    if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
        return rcode;
    }

    LINE_CODING lc;
    lc.dwDTERate  = 4800;   //default serial speed of GPS unit
    lc.bCharFormat  = 0;
    lc.bParityType  = 0;
    lc.bDataBits  = 8;

    rcode = pacm->SetLineCoding(&lc);

    if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);
    }

    return rcode;
}

USBHost     UsbH;
//USBHub     Hub(&UsbH);
PLAsyncOper  AsyncOper;
PL2303       Pl(&UsbH, &AsyncOper);
TinyGPS gps;

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int16_t digits = 2);

void setup()
{

  SerialDebug.begin(115200);

  SerialDebug.print("Testing TinyGPS library v. "); SerialDebug.println(TinyGPS::library_version());
  SerialDebug.println("by Mikal Hart");
  SerialDebug.println();
  SerialDebug.print("Sizeof(gpsobject) = "); SerialDebug.println(sizeof(TinyGPS));
  SerialDebug.println();
  /* USB Initialization */
  if (UsbH.Init()) {
      SerialDebug.println("USB host did not start");
  }

  delay( 200 );
}

void loop()
{
  UsbH.Task();

  if( Pl.isReady()) {

    bool newdata = false;
    uint32_t start = (uint32_t)millis();

    // Every 5 seconds we print an update
    while ((int32_t)((uint32_t)millis() - start) < 5000) {
      if( feedgps()) {
        newdata = true;
      }
    }//while (millis()...

    if (newdata) {
      SerialDebug.println("Acquired Data");
      SerialDebug.println("-------------");
      gpsdump(gps);
      SerialDebug.println("-------------");
      SerialDebug.println();
    }//if( newdata...
  }//if( UsbH.getUsbTaskState() == USB_STATE_RUNNING...
}

void printFloat(double number, int16_t digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     SerialDebug.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  uint32_t int_part = (uint32_t)number;
  double remainder = number - (double)int_part;
  SerialDebug.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    SerialDebug.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    SerialDebug.print(toPrint);
    remainder -= toPrint;
  }
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  uint8_t month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  SerialDebug.print("Lat/Long(10^-5 deg): "); SerialDebug.print(lat); SerialDebug.print(", "); SerialDebug.print(lon);
  SerialDebug.print(" Fix age: "); SerialDebug.print(age); SerialDebug.println("ms.");

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  SerialDebug.print("Lat/Long(float): "); printFloat(flat, 5); SerialDebug.print(", "); printFloat(flon, 5);
  SerialDebug.print(" Fix age: "); SerialDebug.print(age); SerialDebug.println("ms.");

  feedgps();

  gps.get_datetime(&date, &time, &age);
  SerialDebug.print("Date(ddmmyy): "); SerialDebug.print(date); SerialDebug.print(" Time(hhmmsscc): "); SerialDebug.print(time);
  SerialDebug.print(" Fix age: "); SerialDebug.print(age); SerialDebug.println("ms.");

  feedgps();

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  SerialDebug.print("Date: "); SerialDebug.print(static_cast<int>(month)); SerialDebug.print("/"); SerialDebug.print(static_cast<int>(day)); SerialDebug.print("/"); SerialDebug.print(year);
  SerialDebug.print("  Time: "); SerialDebug.print(static_cast<int>(hour)); SerialDebug.print(":"); SerialDebug.print(static_cast<int>(minute)); SerialDebug.print(":"); SerialDebug.print(static_cast<int>(second)); SerialDebug.print("."); SerialDebug.print(static_cast<int>(hundredths));
  SerialDebug.print("  Fix age: ");  SerialDebug.print(age); SerialDebug.println("ms.");

  feedgps();

  SerialDebug.print("Alt(cm): "); SerialDebug.print(gps.altitude()); SerialDebug.print(" Course(10^-2 deg): "); SerialDebug.print(gps.course()); SerialDebug.print(" Speed(10^-2 knots): "); SerialDebug.println(gps.speed());
  SerialDebug.print("Alt(float): "); printFloat(gps.f_altitude()); SerialDebug.print(" Course(float): "); printFloat(gps.f_course()); SerialDebug.println();
  SerialDebug.print("Speed(knots): "); printFloat(gps.f_speed_knots()); SerialDebug.print(" (mph): ");  printFloat(gps.f_speed_mph());
  SerialDebug.print(" (mps): "); printFloat(gps.f_speed_mps()); SerialDebug.print(" (kmph): "); printFloat(gps.f_speed_kmph()); SerialDebug.println();

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  SerialDebug.print("Stats: characters: "); SerialDebug.print(chars); SerialDebug.print(" sentences: "); SerialDebug.print(sentences); SerialDebug.print(" failed checksum: "); SerialDebug.println(failed);
}

bool feedgps()
{
  uint32_t rcode;
  uint8_t  buf[64];    //serial buffer equals Max.packet size of bulk-IN endpoint
  uint16_t rcvd = 64;
    {
        /* reading the GPS */
        rcode = Pl.RcvData(&rcvd, buf);
         if (rcode && rcode != USB_ERRORFLOW)
            ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
            rcode = false;
            if( rcvd ) { //more than zero bytes received
              for( uint16_t i=0; i < rcvd; i++ ) {
                if( gps.encode((char)buf[i])) { //feed a character to gps object
                  rcode = true;
                }//if( gps.encode(buf[i]...
              }//for( uint16_t i=0; i < rcvd; i++...
            }//if( rcvd...
    }
  return( rcode );
}


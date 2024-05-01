#ifndef __OGN_H__
#define __OGN_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>
#ifndef __AVR__
#include <time.h>
#endif

#include <math.h>

#include "intmath.h"

#include "bitcount.h"
#include "nmea.h"

#include "ldpc.h"

#include "format.h"

#include "ads-l.h"

/*
class OGN_SlowPacket       // "slow packet" for transmitting position encoded in packet transmission times
{ public:

  union
  { uint32_t Word[12];     // OGN packet as 32-bit words
    uint8_t  Byte[45];     // OGN packet as  8-bit bytes

    struct                // OGN packet as HeaderWord+Position+FEC
    { uint32_t Header;     //    ECRR PMTT AAAA AAAA AAAA AAAA AAAA AAAA
                           // E=Emergency, C=enCrypt/Custom, RR=Relay count, P=Parity, M=isMeteo/Telemetry, TT=address Type, AA..=Address:24-bit
                           // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                           // This option is indented to pass any type of custom data not foreseen otheriwse

      uint32_t Data[4];// 0: QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:20
                           // 1: MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:20
                           // 2: RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
                           // 3: BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10

      uint32_t FEC[7];     // Gallager code: 194 check bits for 160 user bits
    } ;

} ;
*/
                           // the packet description here is how it look on the little-endian CPU before sending it to the RF chip
                           // nRF905, CC1101, SPIRIT1, RFM69 chips actually reverse the bit order within every byte
                           // thus on the air the bits appear MSbit first for every byte transmitted

class OGN_Packet           // Packet structure for the OGN tracker
{ public:

   static const int Words =  5;
   static const int Bytes = 20;

 union
 { uint32_t HeaderWord;    //    ECRR PMTT AAAA AAAA AAAA AAAA AAAA AAAA
                           // E=Emergency, C=enCrypt/Custom, RR=Relay count, P=Parity, M=isMeteo/Other, TT=address Type, AA..=Address:24-bit
                           // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                           // This option is indented to pass any type of custom data not foreseen otheriwse
   struct
   { unsigned int Address    :24; // aircraft address
     unsigned int AddrType   : 2; // address type: 0 = random, 1 = ICAO, 2 = FLARM, 3 = OGN
     unsigned int Other      : 1; // 0 = position packet, 1 = other information like status
     unsigned int Parity     : 1; // parity takes into account bits 0..27 thus only the 28 lowest bits
     unsigned int RelayCount : 2; // 0 = direct packet, 1 = relayed once, 2 = relayed twice, ...
     unsigned int Encrypted  : 1; // packet is encrypted
     unsigned int Emergency  : 1; // aircraft in emergency (not used for now)
   } Header ;

 } ;

 union
 { uint32_t Data[4];       // 0: QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:20
                           // 1: MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:20
                           // 2: RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
                           // 3: BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10

                           // meteo/telemetry types: Meteo conditions, Thermal wind/climb, Device status, Precise time,

                           // meteo report: Humidity, Barometric pressure, Temperature, wind Speed/Direction
                           // 2: HHHH HHHH SSSS SSSS SSAA AAAA AAAA AAAA
                           // 3: TTTT TTTT YYYY BBBB BBBB BBDD DDDD DDDD  YYYY = report tYpe (meteo, thermal, water level, other telemetry)

                           // Device status: Time, baro pressure+temperature, GPS altitude, supply voltage, TX power, RF noise, software version, software features, hardware features,
                           // 0: UUUU UUUU UUUU UUUU UUUU UUUU UUUU UUUU  UU..=Unix time
                           // 1: SSSS SSSS SSSS SSSS TTTT TTTT HHHH HHHH  SS..=slot time, TT..=temperature, HH..=humidity
                           // 2: BBBB BBBB BBBB BBBB BBAA AAAA AAAA AAAA  Baro pressure[0.5Pa], GPS altitude
                           // 3: VVVV VVVV YYYY HHHH HHHH XXXX VVVV VVVV  VV..=firmware version, YYYY = report type, TT..=Temperatature, XX..=TxPower, VV..=battery voltage

                           // Pilot status:
                           // 0: NNNN NNNN NNNN NNNN NNNN NNNN NNNN NNNN  Name: 9 char x 7bit or 10 x 6bit or Huffman encoding ?
                           // 1: NNNN NNNN NNNN NNNN NNNN NNNN NNNN NNNN
   struct
   {   signed int    Latitude:24; //                  // QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:24
     unsigned int        Time: 6; // [sec]            // time, just second thus ambiguity every every minute
     unsigned int  FixQuality: 2; //
       signed int   Longitude:24; //                  // MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:24
     unsigned int         DOP: 6; //                  // GPS Dilution of Precision
     unsigned int     BaroMSB: 1; //                  // negated bit #8 of the altitude difference between baro and GPS
     unsigned int     FixMode: 1; //
     unsigned int    Altitude:14; // [m] VR           // RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
     unsigned int       Speed:10; // [0.1m/s] VR
     unsigned int    TurnRate: 8; // [0.1deg/s] VR
     unsigned int     Heading:10; //                  // BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10
     unsigned int   ClimbRate: 9; // [0.1m/s] VR
     unsigned int     Stealth: 1;
     unsigned int    AcftType: 4; // [0..15]          // type of aircraft: 1 = glider, 2 = towplane, 3 = helicopter, ...
     unsigned int BaroAltDiff: 8; // [m]              // lower 8 bits of the altitude difference between baro and GPS
   } Position;

   struct
   {   signed int    Latitude:24; //                  // Latitude
     unsigned int        Time: 6; // [sec]            // time, just second thus ambiguity every every minute
     unsigned int            : 2; //
       signed int   Longitude:24; //                  // Longitude
     unsigned int            : 6; //                  //
     unsigned int     BaroMSB: 1; //                  // negated bit #8 of the altitude difference between baro and GPS
     unsigned int            : 1; //
     unsigned int    Altitude:14; // [m] VR           // RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
     unsigned int       Speed:10; // [0.1m/s] VR
     unsigned int            : 8; //
     unsigned int     Heading:10; //                  // BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10
     unsigned int   ClimbRate: 9; // [0.1m/s] VR
     unsigned int            : 1;
     unsigned int  ReportType: 4; //                  // 1 for wind/thermal report
     unsigned int BaroAltDiff: 8; // [m]              // lower 8 bits of the altitude difference between baro and GPS
   } Wind;

   struct
   { unsigned int Pulse     : 8; // [bpm]               // pilot: heart pulse rate
     unsigned int Oxygen    : 7; // [%]                 // pilot: oxygen level in the blood
     unsigned int           : 5;
     unsigned int RxRate    : 4; // [/min]              // log2 of received packet rate
     unsigned int Time      : 6; // [sec]               // same as in the position packet
     unsigned int FixQuality: 2;
     unsigned int AudioNoise: 8; // [dB]                //
     unsigned int RadioNoise: 8; // [dBm]               // noise seen by the RF chip
     unsigned int Temperature:9; // [0.1degC] VR        // temperature by the baro or RF chip
     unsigned int Humidity  : 7; // [%]                 // humidity
     unsigned int Altitude  :14; // [m] VR              // same as in the position packet
     unsigned int Pressure  :14; // [0.08hPa]           // barometric pressure
     unsigned int Satellites: 4; // [ ]
     unsigned int Firmware  : 8; // [ ]                 // firmware version
     unsigned int Hardware  : 8; // [ ]                 // hardware version
     unsigned int TxPower   : 4; // [dBm]               // RF trancmitter power
     unsigned int ReportType: 4; // [ ]                 // 0 for the status report
     unsigned int Voltage   : 8; // [1/64V] VR          // supply voltager
   } Status;

  } ;

   uint8_t  *Byte(void) const { return (uint8_t  *)&HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&HeaderWord; } // packet as words

   // void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator

#ifdef __AVR__

#endif

#ifndef __AVR__

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX\n",
             (long int)HeaderWord, (long int)Data[0], (long int)Data[1],
             (long int)Data[2], (long int)Data[3] ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { printf(" %02X", Byte()[Idx]); }
     printf("\n"); }

   int WriteDeviceStatus(char *Out)
   { return sprintf(Out, " h%02X v%02X %dsat/%d %ldm %3.1fhPa %+4.1fdegC %d%% %4.2fV %d/%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, (long int)DecodeAltitude(), 0.08*Status.Pressure, 0.1*DecodeTemperature(), Status.Humidity,
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
   }

   void Print(void) const
   { if(!Header.Other) { PrintPosition(); return; }
     if(Status.ReportType==0) { PrintDeviceStatus(); return; }
   }

   void PrintDeviceStatus(void) const
   { printf("%c:%06lX R%c%c %02ds:",
             '0'+Header.AddrType, (long int)Header.Address, '0'+Header.RelayCount, Header.Emergency?'E':' ', Status.Time);
     printf(" h%02X v%02X %dsat/%d %ldm %3.1fhPa %+4.1fdegC %d%% %4.2fV Tx:%ddBm Rx:%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, (long int)DecodeAltitude(), 0.08*Status.Pressure, 0.1*DecodeTemperature(), Status.Humidity,
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
     printf("\n");
   }

   void PrintPosition(void) const
   { printf("%c%X:%c:%06lX R%c%c",
            Position.Stealth ?'s':' ', (int)Position.AcftType, '0'+Header.AddrType, (long int)Header.Address, '0'+Header.RelayCount,
	    Header.Emergency?'E':' ');
     printf(" %d/%dD/%4.1f", (int)Position.FixQuality, (int)Position.FixMode+2, 0.1*(10+DecodeDOP()) );
     if(Position.Time<60) printf(" %02ds:", (int)Position.Time);
                 else printf(" ---:");
     printf(" [%+10.6f, %+11.6f]deg %ldm",
            0.0001/60*DecodeLatitude(), 0.0001/60*DecodeLongitude(), (long int)DecodeAltitude() );
     if(hasBaro())
     { printf("[%+dm]", (int)getBaroAltDiff() ); }
     printf(" %3.1fm/s %05.1fdeg %+4.1fm/s %+4.1fdeg/s",
            0.1*DecodeSpeed(), 0.1*DecodeHeading(), 0.1*DecodeClimbRate(), 0.1*DecodeTurnRate() );
     printf("\n");
   }

   int8_t ReadAPRS(const char *Msg)                                                 // read an APRS position message
   { Clear();

     const char *Data  = strchr(Msg, ':'); if(Data==0) return -1; // where the time/position data starts
     Data++;
     const char *Dest  = strchr(Msg, '>'); if(Dest==0) return -1; // where the destination call is
     Dest++;
     const char *Comma = strchr(Dest, ',');                       // the first comma after the destination call

     Position.AcftType=0xF;

     uint8_t AddrType;
     uint32_t Address;
          if(memcmp(Msg, "RND", 3)==0) AddrType=0;
     else if(memcmp(Msg, "ICA", 3)==0) AddrType=1;
     else if(memcmp(Msg, "FLR", 3)==0) AddrType=2;
     else if(memcmp(Msg, "OGN", 3)==0) AddrType=3;
     else AddrType=4;
     if(AddrType<4)
     { if(Read_Hex(Address, Msg+3)==6) Header.Address=Address;
       Header.AddrType=AddrType; }

     if(Comma)
     { if(memcmp(Comma+1, "RELAY*" , 6)==0) Header.RelayCount=1;
       else if(Comma[10]=='*') Header.RelayCount=1;
     }

     if(Data[0]!='/') return -1;
     int8_t Time;
     if(Data[7]=='h')                                            // HHMMSS UTC time
     { Time=Read_Dec2(Data+5); if(Time<0) return -1; }
     else if(Data[7]=='z')                                       // DDHHMM UTC time
     { Time=0; }
     else return -1;
     Position.Time=Time;
     Data+=8;

     Position.FixMode=1;
     Position.FixQuality=1;
     EncodeDOP(0xFF);

     int8_t LatDeg  = Read_Dec2(Data);   if(LatDeg<0) return -1;
     int8_t LatMin  = Read_Dec2(Data+2); if(LatMin<0) return -1;
     if(Data[4]!='.') return -1;
     int8_t LatFrac = Read_Dec2(Data+5); if(LatFrac<0) return -1;
     int32_t Latitude = (int32_t)LatDeg*600000 + (int32_t)LatMin*10000 + (int32_t)LatFrac*100;
     char LatSign = Data[7];
     Data+=8+1;

     int16_t LonDeg  = Read_Dec3(Data);   if(LonDeg<0) return -1;
     int8_t  LonMin  = Read_Dec2(Data+3); if(LonMin<0) return -1;
     if(Data[5]!='.') return -1;
     int8_t LonFrac = Read_Dec2(Data+6); if(LonFrac<0) return -1;
     int32_t Longitude = (int32_t)LonDeg*600000 + (int32_t)LonMin*10000 + (int32_t)LonFrac*100;
     char LonSign = Data[8];
     Data+=9+1;

     int16_t Speed=0;
     int16_t Heading=0;
     if(Data[3]=='/')
     { Heading=Read_Dec3(Data);
       Speed=Read_Dec3(Data+4);
       Data+=7; }
     EncodeHeading(Heading*10);
     EncodeSpeed(((int32_t)Speed*337146+0x8000)>>16);

     uint32_t Altitude=0;
     if( (Data[0]=='/') && (Data[1]=='A') && (Data[2]=='=') && (Read_UnsDec(Altitude, Data+3)==6) )
     { Altitude = (3*Altitude+5)/10;
       Data+=9; }
     EncodeAltitude(Altitude);

     for( ; ; )
     { if(Data[0]!=' ') break;
       Data++;

       if( (Data[0]=='!') && (Data[1]=='W') && (Data[4]=='!') )
       { Latitude  += (Data[2]-'0')*10;
         Longitude += (Data[3]-'0')*10;
         Data+=5; continue; }

       if( (Data[0]=='i') && (Data[1]=='d') )
       { uint32_t ID; Read_Hex(ID, Data+2);
         Header.Address    = ID&0x00FFFFFF;
         Header.AddrType   = (ID>>24)&0x03;
         Position.AcftType = (ID>>26)&0x0F;
         Position.Stealth  = ID>>31;
         Data+=10; continue; }

       if( (Data[0]=='F') && (Data[1]=='L') && (Data[5]=='.') )
       { int16_t FLdec=Read_Dec3(Data+2);
         int16_t FLfrac=Read_Dec2(Data+6);
         if( (FLdec>=0) && (FLfrac>=0) )
         { uint32_t StdAlt = FLdec*100+FLfrac;
           StdAlt = (StdAlt*3+5)/10;
           EncodeStdAltitude(StdAlt); }
         Data+=8; continue; }

       if( (Data[0]=='+') || (Data[0]=='-') )
       { int32_t Value; int8_t Len=Read_Float1(Value, Data);
         if(Len>0)
         { Data+=Len;
           if(memcmp(Data, "fpm", 3)==0) { EncodeClimbRate(Value/200); Data+=3; continue; }
           if(memcmp(Data, "rot", 3)==0) { EncodeTurnRate(3*Value);     Data+=3; continue; }
         }
       }

       if( (Data[0]=='g') && (Data[1]=='p') && (Data[2]=='s') )
       { int16_t HorPrec=Read_Dec2(Data+3);
         if(HorPrec<0) HorPrec=Read_Dec1(Data[3]);
         if(HorPrec>=0)
         { uint16_t DOP=HorPrec*5; if(DOP<10) DOP=10; else if(DOP>230) DOP=230;
           EncodeDOP(DOP-10); Data+=5; }
       }
       while(Data[0]>' ') Data++;
     }

     if(LatSign=='S') Latitude=(-Latitude); else if(LatSign!='N') return -1;
     EncodeLatitude(Latitude);
     if(LonSign=='W') Longitude=(-Longitude); else if(LonSign!='E') return -1;
     EncodeLongitude(Longitude);

     return 0; }

#endif // __AVR__

   int calcDistanceVector(int32_t &LatDist, int32_t &LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000, int32_t MaxDist=0x7FFF)
   { LatDist = ((DecodeLatitude()-RefLat)*1517+0x1000)>>13;           // convert from 1/600000deg to meters (40000000m = 360deg) => x 5/27 = 1517/(1<<13)
     if(abs(LatDist)>MaxDist) return -1;
     LonDist = ((DecodeLongitude()-RefLon)*1517+0x1000)>>13;
     if(abs(LonDist)>(4*MaxDist)) return -1;
             LonDist = (LonDist*LatCos+0x800)>>12;
     if(abs(LonDist)>MaxDist) return -1;
     return 1; }

   void setDistanceVector(int32_t LatDist, int32_t LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000)
   { EncodeLatitude(RefLat+(LatDist*27)/5);
     LonDist = (LonDist<<12)/LatCos;                                  // LonDist/=cosine(Latitude)
     EncodeLongitude(RefLon+(LonDist*27)/5); }

   // uint8_t WritePFLAA(char *NMEA, GPS_Position &Position)
   // { return  WritePFLAA(NMEA, Position.Latitude, Position.Longitude, (Position.Altitude+5)/10, Position.LatitudeCosine); }

   uint8_t WritePFLAA(char *NMEA, int32_t RefLat, int32_t RefLon, int32_t RefAlt, uint16_t LatCos)
   { int32_t LatDist, LonDist;
     if(calcDistanceVector(LatDist, LonDist, RefLat, RefLon, LatCos)<0) return 0;     // return zero, when distance too large
     int32_t AltDist = DecodeAltitude()-RefAlt;
     return WritePFLAA(NMEA, LatDist, LonDist, AltDist); }                            // return number of formatted characters

   uint8_t WritePFLAA(char *NMEA, int32_t LatDist, int32_t LonDist, int32_t AltDist)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$PFLAA,0,");                    // sentence name and alarm-level (but no alarms for trackers)
     Len+=Format_SignDec(NMEA+Len, LatDist);
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, LonDist);
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, AltDist);                       // [m] relative altitude
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Header.AddrType;                              // address-type (3=OGN)
     NMEA[Len++]=',';
     uint32_t Addr = Header.Address;                               // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));               // XXXXXX 24-bit address: RND, ICAO, FLARM, OGN
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, DecodeHeading(), 4, 1);          // [deg] heading (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, DecodeTurnRate(), 2, 1);        // [deg/sec] turn rate
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, DecodeSpeed(), 2, 1);            // [approx. m/s] ground speed
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, DecodeClimbRate(), 2, 1);       // [m/s] climb/sink rate
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(Position.AcftType);                      // [0..F] aircraft-type: 1=glider, 2=tow plane, etc.
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }                                                 // return number of formatted characters

   uint8_t Print(char *Out) const
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     // Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     // Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Position.Time, 2);
     Out[Len++]=' ';
     Len+=PrintLatitude(Out+Len, DecodeLatitude());
     Out[Len++]=' ';
     Len+=PrintLongitude(Out+Len, DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   static uint8_t PrintLatitude(char *Out, int32_t Lat)
   { uint8_t Len=0;
     char Sign='N';
     if(Lat<0) { Sign='S'; Lat=(-Lat); }
     uint32_t Deg=Lat/600000;
     Lat -= 600000*Deg;
     Len+=Format_UnsDec(Out+Len, Deg, 2, 0);
     Len+=Format_UnsDec(Out+Len, Lat, 6, 4);
     Out[Len++]=Sign;
     return Len; }

   static uint8_t PrintLongitude(char *Out, int32_t Lon)
   { uint8_t Len=0;
     char Sign='E';
     if(Lon<0) { Sign='W'; Lon=(-Lon); }
     uint32_t Deg=Lon/600000;
     Lon -= 600000*Deg;
     Len+=Format_UnsDec(Out+Len, Deg, 3, 0);
     Len+=Format_UnsDec(Out+Len, Lon, 6, 4);
     Out[Len++]=Sign;
     return Len; }

   // OGN_Packet() { Clear(); }
   void Clear(void) { HeaderWord=0; Data[0]=0; Data[1]=0; Data[2]=0; Data[3]=0; }

   // uint8_t getAddrType(void) const   { return Header.AddrType; }      // Address type: 0 = Random, 1 = ICAO, 2 = FLARM, 3 = OGN
   // void    setAddrType(uint8_t Type) { Header.AddrType = Type; }

   // uint32_t getAddress(void) const   { return Header.Address; }        // Address: 24-bit
   // void     setAddress(uint32_t Address) { Header.Address = Address; }

   // uint8_t getRelayCount(void) const { return (HeaderWord>>28)&0x03; } // how many time the packet has been relayed
   // void    setRelayCount(uint8_t Count) { HeaderWord = (HeaderWord&0xCFFFFFFF) | ((uint32_t)(Count&0x03)<<28); }
   // void    incRelayCount(void)       { HeaderWord+=0x10000000; }

   // bool  isOther(void) const  { return HeaderWord &  0x04000000; }          // this is a meteo or other report: sends wind speed/direction, pressure, temperatue and humidity
   // void setOther(void)        {        HeaderWord |= 0x04000000; }
   // void clrOther(void)        {        HeaderWord &= 0xFBFFFFFF; }

   // bool isEmergency(void)   const { return HeaderWord &  0x80000000; } // emergency declared or detected (high-g shock ?)
   // void setEmergency(void)        {        HeaderWord |= 0x80000000; }
   // void clrEmergency(void)        {        HeaderWord &= 0x7FFFFFFF; }

   // bool  isEncrypted(void) const  { return HeaderWord &  0x40000000; } // position can be encrypted with a public key (competitions, etc.)
   // void setEncrypted(void)        {        HeaderWord |= 0x40000000; } // when in Emergency it must not be encrypted
   // void clrEncrypted(void)        {        HeaderWord &= 0xBFFFFFFF; }

   uint32_t getAddressAndType(void) const { return HeaderWord&0x03FFFFFF; } // Address with address-type: 26-bit
   void     setAddressAndType(uint32_t AddrAndType) { HeaderWord = (HeaderWord&0xFC000000) | (AddrAndType&0x03FFFFFF); }

   bool goodAddrParity(void) const  { return ((Count1s(HeaderWord&0x0FFFFFFF)&1)==0); }  // Address parity should be EVEN
   void calcAddrParity(void)        { if(!goodAddrParity()) HeaderWord ^= 0x08000000; }  // if not correct parity, flip the parity bit

   bool  isStealth(void) const        { return Position.Stealth; }   // position not to be displayed on public webpages
   void setStealth(uint8_t Stealth=1) { Position.Stealth = Stealth; }
   void clrStealth(void)              {        Position.Stealth = 0; }

   // uint8_t  getAcftType(void) const   { return Position.AcftType; }
   // void     setAcftType(uint8_t Type) { Position.AcftType = Type; }

   // uint8_t  getTime(void) const { return Position.Time; }              // 6 lower bits of the UnitTime or the second counter ?
   // void     setTime(uint8_t Time) { Position.Time = Time; }

   // uint8_t  getFixMode(void) const { return Position.FixMode; }           // 0 = 2-D, 1 = 3-D
   // void     setFixMode(uint8_t Mode) { Position.FixMode = Mode; }

   bool hasBaro(void) const             { return Position.BaroMSB || Position.BaroAltDiff; }
   void clrBaro(void)                   { Position.BaroMSB=0; Position.BaroAltDiff=0; }
   int16_t getBaroAltDiff(void) const   { int16_t AltDiff=Position.BaroAltDiff; if(Position.BaroMSB==0) AltDiff|=0xFF00; return AltDiff; }
   void setBaroAltDiff(int16_t AltDiff)
   { if(AltDiff<(-255)) AltDiff=(-255); else if(AltDiff>255) AltDiff=255;
     Position.BaroMSB = (AltDiff&0xFF00)==0; Position.BaroAltDiff=AltDiff&0xFF; }
   void EncodeStdAltitude(int32_t StdAlt) { setBaroAltDiff((StdAlt-DecodeAltitude())); }
   int32_t DecodeStdAltitude(void) const { return (DecodeAltitude()+getBaroAltDiff()); }

   // uint8_t  getFixQuality(void) const   { return Position.FixQuality; }        // 0 = no fix, 1 = GPS, 2 = diff. GPS, 3 = other
   // void     setFixQuality(uint8_t Qual) { Position.FixQuality = Qual; }

   static uint16_t EncodeUR2V8(uint16_t Value)                                 // Encode unsigned 12bit (0..3832) as 10bit
   {      if(Value<0x100) { }
     else if(Value<0x300) Value = 0x100 | ((Value-0x100)>>1);
     else if(Value<0x700) Value = 0x200 | ((Value-0x300)>>2);
     else if(Value<0xF00) Value = 0x300 | ((Value-0x700)>>3);
     else                 Value = 0x3FF;
     return Value; }

   static uint16_t DecodeUR2V8(uint16_t Value)                                 // Decode 10bit 0..0x3FF
   { uint16_t  Range = Value>>8;
     Value &= 0x0FF;
     if(Range==0) return Value;              // 000..0FF
     if(Range==1) return 0x101+(Value<<1);   // 100..2FE
     if(Range==2) return 0x302+(Value<<2);   // 300..6FC
                  return 0x704+(Value<<3); } // 700..EF8                       // in 12bit (0..3832)

   static uint8_t EncodeUR2V5(uint16_t Value)                                  // Encode unsigned 9bit (0..472) as 7bit
   {      if(Value<0x020) { }
     else if(Value<0x060) Value = 0x020 | ((Value-0x020)>>1);
     else if(Value<0x0E0) Value = 0x040 | ((Value-0x060)>>2);
     else if(Value<0x1E0) Value = 0x060 | ((Value-0x0E0)>>3);
     else                 Value = 0x07F;
     return Value; }

   static uint16_t DecodeUR2V5(uint16_t Value)                                 // Decode 7bit as unsigned 9bit (0..472)
   { uint8_t Range = (Value>>5)&0x03;
             Value &= 0x1F;
          if(Range==0) { }                            // 000..01F
     else if(Range==1) { Value = 0x021+(Value<<1); }  // 020..05E
     else if(Range==2) { Value = 0x062+(Value<<2); }  // 060..0DC
     else              { Value = 0x0E4+(Value<<3); }  // 0E0..1D8 => max. Value = 472
     return Value; }

   static uint8_t EncodeSR2V5(int16_t Value)                                  // Encode signed 10bit (-472..+472) as 8bit
   { uint8_t Sign=0; if(Value<0) { Value=(-Value); Sign=0x80; }
     Value = EncodeUR2V5(Value);
     return Value | Sign; }

   static  int16_t DecodeSR2V5( int16_t Value)                                // Decode
   { int16_t Sign =  Value&0x80;
     Value = DecodeUR2V5(Value&0x7F);
     return Sign ? -Value: Value; }

   static uint16_t EncodeUR2V6(uint16_t Value)                                // Encode unsigned 10bit (0..952) as 8 bit
   {      if(Value<0x040) { }
     else if(Value<0x0C0) Value = 0x040 | ((Value-0x040)>>1);
     else if(Value<0x1C0) Value = 0x080 | ((Value-0x0C0)>>2);
     else if(Value<0x3C0) Value = 0x0C0 | ((Value-0x1C0)>>3);
     else                 Value = 0x0FF;
     return Value; }

   static uint16_t DecodeUR2V6(uint16_t Value)                                // Decode 8bit as unsigned 10bit (0..952)
   { uint16_t Range  = (Value>>6)&0x03;
             Value &= 0x3F;
          if(Range==0) { }                            // 000..03F
     else if(Range==1) { Value = 0x041+(Value<<1); }  // 040..0BE
     else if(Range==2) { Value = 0x0C2+(Value<<2); }  // 0C0..1BC
     else              { Value = 0x1C4+(Value<<3); }  // 1C0..3B8 => max. Value = 952
     return Value; }

   static uint16_t EncodeSR2V6(int16_t Value)                                 // Encode signed 11bit (-952..+952) as 9bit
   { uint16_t Sign=0; if(Value<0) { Value=(-Value); Sign=0x100; }
     Value = EncodeUR2V6(Value);
     return Value | Sign; }

   static  int16_t DecodeSR2V6( int16_t Value)                                // Decode 9bit as signed 11bit (-952..+952)
   { int16_t Sign =  Value&0x100;
     Value = DecodeUR2V6(Value&0x00FF);
     return Sign ? -Value: Value; }

   void EncodeLatitude(int32_t Latitude)                                // encode Latitude: units are 0.0001/60 degrees
   { Position.Latitude = Latitude>>3; }

   int32_t DecodeLatitude(void) const
   { int32_t Latitude = Position.Latitude;
     // if(Latitude&0x00800000) Latitude|=0xFF000000;
     Latitude = (Latitude<<3)+4; return Latitude; }

   void EncodeLongitude(int32_t Longitude)                             // encode Longitude: units are 0.0001/60 degrees
   { Position.Longitude = Longitude>>=4; }

   int32_t DecodeLongitude(void) const
   { int32_t Longitude = Position.Longitude;
     // if(Longitude&0x00800000) Longitude|=0xFF000000;
     Longitude = (Longitude<<4)+8; return Longitude; }

   static uint16_t EncodeUR2V12(uint16_t Value)                        // encode unsigned 16-bit (0..61432) as 14-bit
   {      if(Value<0x1000) { }
     else if(Value<0x3000) Value = 0x1000 | ((Value-0x1000)>>1);
     else if(Value<0x7000) Value = 0x2000 | ((Value-0x3000)>>2);
     else if(Value<0xF000) Value = 0x3000 | ((Value-0x7000)>>3);
     else                  Value = 0x3FFF;
     return Value; }

   static uint16_t DecodeUR2V12(uint16_t Value)
   { uint16_t Range = Value>>12;
              Value &=0x0FFF;
     if(Range==0) return         Value;       // 0000..0FFF
     if(Range==1) return 0x1001+(Value<<1);   // 1000..2FFE
     if(Range==2) return 0x3002+(Value<<2);   // 3000..6FFC
                  return 0x7004+(Value<<3); } // 7000..EFF8 => max: 61432

   void EncodeAltitude(int32_t Altitude)                               // encode altitude in meters
   { if(Altitude<0)      Altitude=0;
     Position.Altitude = EncodeUR2V12((uint16_t)Altitude); }

   int32_t DecodeAltitude(void) const                                   // return Altitude in meters
   { return DecodeUR2V12(Position.Altitude); }

   void EncodeDOP(uint8_t DOP)
   {      if(DOP<0)    DOP=0;
     else if(DOP<0x10) { }
     else if(DOP<0x30) DOP = 0x10 | ((DOP-0x10)>>1);
     else if(DOP<0x70) DOP = 0x20 | ((DOP-0x30)>>2);
     else if(DOP<0xF0) DOP = 0x30 | ((DOP-0x70)>>3);
     else              DOP = 0x3F;
     Position.DOP = DOP; }

   uint8_t DecodeDOP(void) const
   { uint8_t DOP = Position.DOP;
     uint8_t Range = DOP>>4;
     DOP &= 0x0F;
     if(Range==0) return       DOP;              // 00..0F
     if(Range==1) return 0x11+(DOP<<1);          // 10..2E
     if(Range==2) return 0x32+(DOP<<2);          // 30..6C
                  return 0x74+(DOP<<3); }        // 70..E8 => max. DOP = 232*0.1=23.2

   void EncodeSpeed(int16_t Speed)                                       // speed in 0.2 knots (or 0.1m/s)
   {      if(Speed<0)     Speed=0;
     else Speed=EncodeUR2V8(Speed);
     Position.Speed = Speed; }

   int16_t DecodeSpeed(void) const           // return speed in 0.2 knots or 0.1m/s units
   { return DecodeUR2V8(Position.Speed); }   // => max. speed: 3832*0.2 = 766 knots

   int16_t DecodeHeading(void) const         // return Heading in 0.1 degree units 0..359.9 deg
   { int32_t Heading = Position.Heading;
     return (Heading*3600+512)>>10; }

   void EncodeHeading(int16_t Heading)
   { Position.Heading = (((int32_t)Heading<<10)+180)/3600; }

   void setHeadingAngle(uint16_t HeadingAngle)
   { Position.Heading = (((HeadingAngle+32)>>6)); }

   uint16_t getHeadingAngle(void) const
   { return (uint16_t)Position.Heading<<6; }

   void EncodeTurnRate(int16_t Turn)         // [0.1 deg/sec]
   { Position.TurnRate = EncodeSR2V5(Turn); }

   int16_t DecodeTurnRate(void) const
   { return DecodeSR2V5(Position.TurnRate); }

   void EncodeClimbRate(int16_t Climb)
   { Position.ClimbRate = EncodeSR2V6(Climb); }

   int16_t DecodeClimbRate(void) const
   { return DecodeSR2V6(Position.ClimbRate); }

// --------------------------------------------------------------------------------------------------------------
// Status fields

   void EncodeTemperature(int16_t Temp)   { Status.Temperature=EncodeSR2V5(Temp-200); } // [0.1degC]
   int16_t DecodeTemperature(void) const  { return 200+DecodeSR2V5(Status.Temperature); }

   void EncodeVoltage(uint16_t Voltage)   { Status.Voltage=EncodeUR2V6(Voltage); }      // 
  uint16_t DecodeVoltage(void) const      { return DecodeUR2V6(Status.Voltage); }

// --------------------------------------------------------------------------------------------------------------

   void Encrypt (const uint32_t Key[4]) { XXTEA_Encrypt(Data, 4, Key, 8); }        // encrypt with given Key
   void Decrypt (const uint32_t Key[4]) { XXTEA_Decrypt(Data, 4, Key, 8); }        // decrypt with given Key

   void Whiten  (void) { TEA_Encrypt_Key0(Data, 8); TEA_Encrypt_Key0(Data+2, 8); } // whiten the position
   void Dewhiten(void) { TEA_Decrypt_Key0(Data, 8); TEA_Decrypt_Key0(Data+2, 8); } // de-whiten the position

// ==============================================================================================
// TEA encryption/decryption
// Data is 2 x 32-bit word
// Key  is 4 x 32-bit word

   static void TEA_Encrypt (uint32_t* Data, const uint32_t *Key, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                         // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=0;         // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];     // cache key
     for (int i=0; i < Loops; i++)                            // basic cycle start
     { sum += delta;
       v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3); }  // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   static void TEA_Decrypt (uint32_t* Data, const uint32_t *Key, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                           // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];       // cache key
     for (int i=0; i < Loops; i++)                              // basic cycle start */
     { v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
       v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       sum -= delta; }                                          // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   static void TEA_Encrypt_Key0 (uint32_t* Data, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                          // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=0;          // a key schedule constant
     for (int i=0; i < Loops; i++)                             // basic cycle start
     { sum += delta;
       v0 += (v1<<4) ^ (v1 + sum) ^ (v1>>5);
       v1 += (v0<<4) ^ (v0 + sum) ^ (v0>>5); }                 // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   static void TEA_Decrypt_Key0 (uint32_t* Data, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                           // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
     for (int i=0; i < Loops; i++)                              // basic cycle start */
     { v1 -= (v0<<4) ^ (v0 + sum) ^ (v0>>5);
       v0 -= (v1<<4) ^ (v1 + sum) ^ (v1>>5);
       sum -= delta; }                                          // end cycle
     Data[0]=v0; Data[1]=v1;
   }

// ==============================================================================================
// XXTEA encryption/decryption

   static uint32_t XXTEA_MX(uint8_t E, uint32_t Y, uint32_t Z, uint8_t P, uint32_t Sum, const uint32_t Key[4])
   { return ((((Z>>5) ^ (Y<<2)) + ((Y>>3) ^ (Z<<4))) ^ ((Sum^Y) + (Key[(P&3)^E] ^ Z))); }

   static void XXTEA_Encrypt(uint32_t *Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops)
   { const uint32_t Delta = 0x9e3779b9;
     uint32_t Sum = 0;
     uint32_t Z = Data[Words-1]; uint32_t Y;
     for( ; Loops; Loops--)
     { Sum += Delta;
       uint8_t E = (Sum>>2)&3;
       for (uint8_t P=0; P<(Words-1); P++)
       { Y = Data[P+1];
         Z = Data[P] += XXTEA_MX(E, Y, Z, P, Sum, Key); }
       Y = Data[0];
       Z = Data[Words-1] += XXTEA_MX(E, Y, Z, Words-1, Sum, Key);
     }
   }

   static void XXTEA_Decrypt(uint32_t *Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops)
   { const uint32_t Delta = 0x9e3779b9;
     uint32_t Sum = Loops*Delta;
     uint32_t Y = Data[0]; uint32_t Z;
     for( ; Loops; Loops--)
     { uint8_t E = (Sum>>2)&3;
       for (uint8_t P=Words-1; P; P--)
       { Z = Data[P-1];
         Y = Data[P] -= XXTEA_MX(E, Y, Z, P, Sum, Key); }
       Z = Data[Words-1];
       Y = Data[0] -= XXTEA_MX(E, Y, Z, 0, Sum, Key);
       Sum -= Delta;
     }
   }

// ==============================================================================================

   static uint8_t Gray(uint8_t Binary) { return Binary ^ (Binary>>1); }

   static uint8_t Binary(uint8_t Gray)
   { Gray = Gray ^ (Gray >> 4);
     Gray = Gray ^ (Gray >> 2);
     Gray = Gray ^ (Gray >> 1);
     return Gray; }

  uint8_t getTxSlot(uint8_t Idx) const // Idx=0..15
  { const uint32_t *DataPtr = Data;
    uint32_t  Mask=1; Mask<<=Idx;
    uint8_t Slot=0;
    for(uint8_t Bit=0; Bit<6; Bit++)
    { Slot>>=1;
      if(DataPtr[Bit]&Mask) Slot|=0x20;
      Mask<<=1; Slot>>=1; }
    return Gray(Slot); }

} ;

// ---------------------------------------------------------------------------------------------------------------------

class OGN_TxPacket                                    // OGN packet with FEC code, like for transmission
{ public:
   static const int     Words =  7;
   static const int     Bytes = 26;

   OGN_Packet Packet;     // OGN packet

   uint32_t FEC[2];       // Gallager code: 48 check bits for 160 user bits

  public:

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Packet.Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Packet.Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Packet.Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Packet.Position.Time, 2);
     Out[Len++]=' ';
     Len+=Packet.PrintLatitude(Out+Len, Packet.DecodeLatitude());
     Out[Len++]=' ';
     Len+=Packet.PrintLongitude(Out+Len, Packet.DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)Packet.DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, Packet.DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, Packet.DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3], (long int)FEC[0],
             (long int)FEC[1], (int)checkFEC() ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { printf(" %02X", Packet.Byte()[Idx]); }
     printf("\n"); }

   // void calcFEC(void)                            { LDPC_Encode(&Packet.HeaderWord, FEC); }       // calculate the 48-bit parity check
   // void calcFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&PacketHeaderWord,  FEC, ParityGen); }

#ifndef __AVR__
   void    calcFEC(void)                   { LDPC_Encode(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)
#else
   void    calcFEC(void)                   { LDPC_Encode(Packet.Byte()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Byte()); }        // returns number of parity checks that fail (0 => no errors, all fine)
#endif

   uint8_t  *Byte(void) const { return (uint8_t  *)&Packet.HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&Packet.HeaderWord; } // packet as words

   void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator
/*
   uint8_t calcErrorPattern(uint8_t *ErrPatt, const uint8_t *OtherPacket) const
   { uint8_t ByteIdx=0; const uint32_t *WordPtr=Packet.Word();
     for(uint8_t WordIdx=0; WordIdx<Words; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=Bytes) break;
         ErrPatt[ByteIdx]=Packet[ByteIdx]^Word; ByteIdx++;
         Word>>=8; }
     }
     return Bytes; }
*/
} ;

// ---------------------------------------------------------------------------------------------------------------------

class OGN_RxPacket                                        // OGN packet with FEC code and some reception info
{ public:
   static const int     Words =  7;
   static const int     Bytes = 26;

   OGN_Packet Packet;

   uint32_t FEC[2];       // Gallager code: 48 check bits for 160 user bits

   union
   { uint8_t State;       //
     struct
     { bool      :1;      //
       bool Ready:1;      // is ready for transmission
       bool Sent :1;      // has already been transmitted out
       bool Corr :1;      // correctly received or corrected by FEC
       uint8_t RxErr:4;   // number of bit errors corrected upon reception
     } ;
   } ;

   uint8_t RxChan;        // RF channel where the packet was received
   uint8_t RxRSSI;        // [-0.5dBm]
   uint8_t Rank;          // rank: low altitude and weak signal => high rank

  public:

   OGN_RxPacket() { Clear(); }
   void Clear(void) { Packet.Clear(); State=0; Rank=0; }

   void   clrReady(void)       {        State &= 0xFE; } // is ready for transmission
   void   setReady(void)       {        State |= 0x01; }
   uint8_t isReady(void) const { return State &  0x01; }

   void   clrSent(void)        {        State &= 0xFD; } // has already been transmitted out
   void   setSent(void)        {        State |= 0x02; }
   uint8_t isSent(void)  const { return State &  0x02; }

   void   clrAlloc(void)       {        State &= 0x7F; } // allocated = being used (when in a pipe)
   void   setAlloc(void)       {        State |= 0x80; }
   uint8_t isAlloc(void) const { return State &  0x80; }

   uint8_t  *Byte(void) const { return (uint8_t  *)&Packet.HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&Packet.HeaderWord; } // packet as words

   void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator

   uint8_t calcErrorPattern(uint8_t *ErrPatt, const uint8_t *OtherPacket) const
   { uint8_t ByteIdx=0; const uint32_t *WordPtr=Packet.Word();
     for(uint8_t WordIdx=0; WordIdx<Words; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=Bytes) break;
         ErrPatt[ByteIdx]=OtherPacket[ByteIdx]^Word; ByteIdx++;
         Word>>=8; }
     }
     return Bytes; }

   // void calcFEC(void)                            { LDPC_Encode(&Packet.HeaderWord, FEC); }       // calculate the 48-bit parity check
   // void calcFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&PacketHeaderWord,  FEC, ParityGen); }
#ifndef __AVR__
   void    calcFEC(void)                   { LDPC_Encode(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)
#else
   void    calcFEC(void)                   { LDPC_Encode(Packet.Byte()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Byte()); }        // returns number of parity checks that fail (0 => no errors, all fine)
#endif

   int BitErr(OGN_RxPacket &RefPacket) const // return number of different data bits between this Packet and RefPacket
   { return Count1s(Packet.HeaderWord^RefPacket.Packet.HeaderWord)
           +Count1s(Packet.Data[0]^RefPacket.Packet.Data[0])
           +Count1s(Packet.Data[1]^RefPacket.Packet.Data[1])
           +Count1s(Packet.Data[2]^RefPacket.Packet.Data[2])
           +Count1s(Packet.Data[3]^RefPacket.Packet.Data[3])
           +Count1s(FEC[0]^RefPacket.FEC[0])
           +Count1s((FEC[1]^RefPacket.FEC[1])&0xFFFF); }

   void calcRelayRank(int32_t RxAltitude)                               // [0.1m] altitude of reception
   { if(Packet.Header.Emergency) { Rank=0xFF; return; }                 // emergency packets always highest rank
     Rank=0;
     if(Packet.Header.Other) return;                                    // only relay position packets
     if(Packet.Position.Time>=60) return;
     if(Packet.Header.RelayCount>0) return;                             // no rank for relayed packets (only single relay)
     if(RxRSSI>32)                                                      // [-0.5dB] weaker signal => higher rank
       Rank += (RxRSSI-32)>>2;                                          // 1 point/2dB lower signal
     RxAltitude -= 10*Packet.DecodeAltitude();                          // [0.1m] lower altitude => higher rank
     if(RxAltitude>0)
       Rank += RxAltitude>>10;                                          // 1 point/100m of altitude below
     int16_t ClimbRate = Packet.DecodeClimbRate();                      // [0.1m/s] higher sink rate => higher rank
     if(ClimbRate<0)
       Rank += (-ClimbRate)>>3;                                         // 1 point/0.8m/s of sink
   }

   uint8_t ReadPOGNT(const char *NMEA)
   { uint8_t Len=0;
     if(memcmp(NMEA, "$POGNT,", 7)!=0) return -1;
     Len+=7;

     if(NMEA[Len+2]!=',') return -1;
     int8_t Time=Read_Dec2(NMEA+Len);
     if( (Time<0) || (Time>=60) ) return -1;
     Packet.Position.Time=Time;
     Len+=3;

     if(NMEA[Len+1]!=',') return -1;
     int8_t AcftType=Read_Hex1(NMEA[Len]);
     if(AcftType<0) return -1;
     Packet.Position.AcftType=AcftType;
     Len+=2;

     if(NMEA[Len+1]!=',') return -1;
     int8_t AddrType=Read_Hex1(NMEA[Len]);
     if((AddrType<0) || (AddrType>=4) ) return -1;
     Packet.Header.AddrType=AddrType;
     Len+=2;

     uint32_t Addr;
     int8_t Ret=Read_Hex(Addr, NMEA+Len); if(Ret<=0) return -1;
     if(NMEA[Len+Ret]!=',') return -1;
     Packet.Header.Address=Addr;
     Len+=Ret+1;

     if(NMEA[Len+1]!=',') return -1;
     int8_t Relay=Read_Hex1(NMEA[Len]);
     if((Relay<0) || (Relay>=4) ) return -1;
     Packet.Header.RelayCount=Relay;
     Len+=2;

     return Len; }

   uint8_t WritePOGNT(char *NMEA)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$POGNT,");                             // sentence name
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)Packet.Position.Time, 2);     // [sec] time
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(Packet.Position.AcftType);                      // [0..F] aircraft-type: 1=glider, 2=tow plane, etc.
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Header.AddrType;                              // [0..3] address-type: 1=ICAO, 2=FLARM, 3=OGN
     NMEA[Len++]=',';
     uint32_t Addr = Packet.Header.Address;                               // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Header.RelayCount;                            // [0..3] counts retransmissions
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Position.FixQuality;                          // [] fix quality
     NMEA[Len++]='0'+Packet.Position.FixMode;                             // [] fix mode
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)(Packet.DecodeDOP()+10),2,1); // [] Dilution of Precision
     NMEA[Len++]=',';
     Len+=Packet.PrintLatitude(NMEA+Len, Packet.DecodeLatitude());        // [] Latitude
     NMEA[Len++]=',';
     Len+=Packet.PrintLongitude(NMEA+Len, Packet.DecodeLongitude());      // [] Longitude
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint32_t)Packet.DecodeAltitude());     // [m] Altitude (by GPS)
     NMEA[Len++]=',';
     if(Packet.hasBaro())
       Len+=Format_SignDec(NMEA+Len, (int32_t)Packet.getBaroAltDiff());   // [m] Standard Pressure Altitude (by Baro)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeClimbRate(), 2, 1);       // [m/s] climb/sink rate (by GPS or pressure sensor)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeSpeed(), 2, 1);         // [m/s] ground speed (by GPS)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeHeading(), 4, 1);          // [deg] heading (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeTurnRate(), 2, 1);         // [deg/s] turning rate (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, -(int16_t)RxRSSI/2);            // [dBm] received signal level
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)RxErr);                // [bits] corrected transmisison errors
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Packet.Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Packet.Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Packet.Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Packet.Position.Time, 2);
     Out[Len++]=' ';
     Len+=Packet.PrintLatitude(Out+Len, Packet.DecodeLatitude());
     Out[Len++]=' ';
     Len+=Packet.PrintLongitude(Out+Len, Packet.DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)Packet.DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, Packet.DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, Packet.DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3],
             (long int)FEC[0], (long int)FEC[1], (int)checkFEC() ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<26; Idx++)
     { printf(" %02X", Packet.Byte()[Idx]); }
     printf(" (%d)\n", LDPC_Check(Packet.Byte())); }

} ;

#ifdef WITH_PPM

class OGN_PPM_Packet                                        // OGN packet with FEC code and some reception info
{ public:
   static const int     Words = 12;

   OGN_Packet Packet;

   uint32_t FEC[7];       // Gallager code: 194 check bits for 160 user bits

  public:

   void    calcFEC(void)                   { LDPC_Encode_n354k160(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check_n354k160(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)

   uint32_t *Word(void) const { return Packet.Word(); }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %08lX %08lX %08lX %08lX %08lX %01lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3],
             (long int)FEC[0], (long int)FEC[1], (long int)FEC[2], (long int)FEC[2],
             (long int)FEC[4], (long int)FEC[5], (long int)FEC[6], (int)checkFEC() ); }

   static uint8_t Gray(uint8_t Binary) { return Binary ^ (Binary>>1); }

   static uint8_t Binary(uint8_t Gray)
   { Gray = Gray ^ (Gray >> 4);
     Gray = Gray ^ (Gray >> 2);
     Gray = Gray ^ (Gray >> 1);
     return Gray; }

   uint8_t getSymbol(uint16_t Idx)
   { if(Idx>=59) return 0xFF;
     uint32_t *Word = Packet.Word();
     uint8_t Symbol=0; uint8_t SymbMask=1;
     for(uint8_t Bit=0; Bit<6; Bit++, Idx+=59 )
     { uint8_t WordIdx=Idx>>5; uint8_t BitIdx=Idx&31;
       uint32_t Mask=1; Mask<<=BitIdx;
       if(Word[WordIdx]&Mask) Symbol|=SymbMask;
       SymbMask<<=1; }
     return Gray(Symbol); }

   void clear(void)
   { memset(Packet.Word(), 0, Words*4); }

   void setSymbol(uint16_t Idx, uint8_t Symbol)
   { if(Idx>=59) return;
     Symbol = Binary(Symbol);
     uint32_t *Word = Packet.Word();
     for(uint8_t Bit=0; Bit<6; Bit++, Idx+=59 )
     { if(Symbol&1)
       { uint8_t WordIdx=Idx>>5; uint8_t BitIdx=Idx&31;
         uint32_t Mask=1; Mask<<=BitIdx;
         Word[WordIdx]|=Mask; }
       Symbol>>=1; }
   }

} ;

#endif // WITH_PPM

// ---------------------------------------------------------------------------------------------------------------------

template<uint8_t Size=8>
 class OGN_PrioQueue
{ public:
   // static const uint8_t Size = 8;            // number of packets kept
   OGN_RxPacket         Packet[Size];        // OGN packets
   uint16_t             Sum;                 // sum of all ranks
   uint8_t              Low, LowIdx;         // the lowest rank and the index of it

  public:
   void Clear(void)                                                           // clear (reset) the queue
   { for(uint8_t Idx=0; Idx<Size; Idx++)                                      // clear every packet
     { Packet[Idx].Clear(); }
     Sum=0; Low=0; LowIdx=0; }                                                // clear the rank sum, lowest rank

   OGN_RxPacket * operator [](uint8_t Idx) { return Packet+Idx; }

   uint8_t getNew(void)                                                       // get (index of) a free or lowest rank packet
   { Sum-=Packet[LowIdx].Rank; Packet[LowIdx].Rank=0; Low=0; return LowIdx; } // remove old packet from the rank sum

   void addNew(uint8_t NewIdx)                                                // add the new packet to the queue
   { uint32_t AddressAndType = Packet[NewIdx].Packet.getAddressAndType();     // get ID of this packet: ID is address-type and address (2+24 = 26 bits)
     for(uint8_t Idx=0; Idx<Size; Idx++)                                      // look for other packets with same ID
     { if(Idx==NewIdx) continue;                                              // avoid the new packet
       if(Packet[Idx].Packet.getAddressAndType() == AddressAndType)           // if another packet with same ID:
       { clean(Idx); }                                                        // then remove it: set rank to zero
     }
     uint8_t Rank=Packet[NewIdx].Rank; Sum+=Rank;                             // add the new packet to the rank sum
     if(NewIdx==LowIdx) reCalc();
     else { if(Rank<Low) { Low=Rank; LowIdx=NewIdx; } }
     // if(NewIdx!=LowIdx)                                                       //
     // { if(Rank<=Low) { Low=Rank; LowIdx=NewIdx; } }
     // else reCalc();
   }

   uint8_t getRand(uint32_t Rand) const                                       // get a position by random selection but probabilities prop. to ranks
   { if(Sum==0) return Rand%Size;                                             //
     uint16_t RankIdx = Rand%Sum;
     uint8_t Idx; uint16_t RankSum=0;
     for(Idx=0; Idx<Size; Idx++)
     { uint8_t Rank=Packet[Idx].Rank; if(Rank==0) continue;
       RankSum+=Rank; if(RankSum>RankIdx) return Idx; }
     return Rand%Size; }

   void reCalc(void)                                                           // find the lowest rank and calc. the sum of all ranks
   { Sum=Low=Packet[0].Rank; LowIdx=0;                                         // take minimum at the first slot
     for(uint8_t Idx=1; Idx<Size; Idx++)                                       // loop over all other slots
     { uint8_t Rank=Packet[Idx].Rank;
       Sum+=Rank;                                                              // sum up the ranks
       if(Rank<Low) { Low=Rank; LowIdx=Idx; }                                  // update the minimum
     }
   }

   void cleanTime(uint8_t Time)                                                // clean up slots of given Time
   { for(int Idx=0; Idx<Size; Idx++)
     { if( (Packet[Idx].Rank) && (Packet[Idx].Packet.Position.Time==Time) )
       { clean(Idx); }
     }
   }

   void clean(uint8_t Idx)                                                      // clean given slot
   { Sum-=Packet[Idx].Rank; Packet[Idx].Rank=0; Low=0; LowIdx=Idx; }

   void decrRank(uint8_t Idx, uint8_t Decr=1)                                   // decrement rank of given slot
   { uint8_t Rank=Packet[Idx].Rank; if(Rank==0) return;                         // if zero already: do nothing
     if(Decr>Rank) Decr=Rank;                                                   // if to decrement by more than the rank already: reduce the decrement
     Rank-=Decr; Sum-=Decr;                                                     // decrement the rank and the sum of ranks
     if(Rank<Low) { Low=Rank; LowIdx=Idx; }                                     // if new minimum: update the minimum.
     Packet[Idx].Rank=Rank; }                                                   // update the rank of this slot

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     for(uint8_t Idx=0; Idx<Size; Idx++)
     { uint8_t Rank=Packet[Idx].Rank;
       Out[Len++]=' '; Len+=Format_Hex(Out+Len, Rank);
       if(Rank)
       { Out[Len++]='/'; Len+=Format_Hex(Out+Len, Packet[Idx].Packet.getAddressAndType() );
         Out[Len++]=':'; Len+=Format_UnsDec(Out+Len, Packet[Idx].Packet.Position.Time, 2 ); }
     }
     Out[Len++]=' '; Len+=Format_Hex(Out+Len, Sum);
     Out[Len++]='/'; Len+=Format_Hex(Out+Len, LowIdx);
     Out[Len++]='\n'; Out[Len]=0; return Len; }

} ;

class GPS_Position
{ public:

  union
  { uint8_t Flags;              // bit #0 = GGA and RMC had same Time
    struct
    { bool Complete :1;
      bool Baro     :1;
    } ;
  } ;

   int8_t FixQuality;           // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
   int8_t FixMode;              // 0 = not set (from GSA) 1 = none, 2 = 2-D, 3 = 3-D
   int8_t Satellites;           // number of active satellites

   int8_t  Year, Month, Day;    // Date (UTC) from GPS
   int8_t  Hour, Min, Sec;      // Time-of-day (UTC) from GPS
   int8_t  FracSec;             // [1/100 sec] some GPS-es give second fraction with the time-of-day

   uint8_t PDOP;                // [0.1] dilution of precision
   uint8_t HDOP;                // [0.1] horizontal dilution of precision
   uint8_t VDOP;                // [0.1] vertical dilution of precision

   int16_t Speed;               // [0.1 m/s] speed-over-ground
   int16_t Heading;             // [0.1 deg]  heading-over-ground

   int16_t ClimbRate;           // [0.1 meter/sec)
   int16_t TurnRate;            // [0.1 deg/sec]

   int16_t GeoidSeparation;     // [0.1 meter] difference between Geoid and Ellipsoid
   int32_t Altitude;            // [0.1 meter] height above Geoid (sea level)

   int32_t Latitude;            // [0.0001/60 deg] about 0.018m accuracy (to convert to u-Blox GPS 1e-7deg units mult by 50/3)
   int32_t Longitude;           // [0.0001/60 deg]
  uint16_t LatitudeCosine;      // [2^-12] Latitude cosine for distance calculation

   int16_t Temperature;         // [0.1 degC]
  uint32_t Pressure;            // [0.25 Pa]   from pressure sensor
   int32_t StdAltitude;         // [0.1 meter] standard pressure altitude (from the pressure sensor and atmosphere calculator)

  public:

   GPS_Position() { Clear(); }

   void Clear(void)
   { Flags=0; FixQuality=0; FixMode=0; PDOP=0; HDOP=0; VDOP=0;
     setDefaultDate(); setDefaultTime();
     Latitude=0; Longitude=0; LatitudeCosine=0x4000; // LatitudeCosine=0x7FFFFFFF;
     Altitude=0; GeoidSeparation=0;
     Speed=0; Heading=0; ClimbRate=0; TurnRate=0;
     StdAltitude=0; Temperature=0; }

   void setDefaultDate() { Year=00; Month=1; Day=1; }
   void setDefaultTime() { Hour=0;  Min=0;   Sec=0; FracSec=0; }

   // void CopyTime(OGN_Position *Previous) { }

   // bool  isComplete(void) const { return Flags&0x01; } // have both RMC and GGA sentences been received and for same time ?
   // void setComplete(void)       { Flags|=0x01; }
   // void clrComplete(void)       { Flags&=0xFE; }

   // bool hasBaro(void)     const { return Flags&0x02; } // has data from pressure sensor
   // void setBaro(void)           { Flags|=0x02; }
   // void clrBaro(void)           { Flags&=0xFD; }

   bool isTimeValid(void) const                      // is the GPS time-of-day valid ?
   { return (Hour>=0) && (Min>=0) && (Sec>=0); }     // all data must have been correctly read: negative means not correctly read)

   bool isDateValid(void) const                      // is the GPS date valid ?
   { return (Year>=0) && (Month>=0) && (Day>=0); }

   bool isValid(void) const                          // is GPS lock there ?
   { if(!isTimeValid()) return 0;                    // is GPS time valid/present ?
     if(!isDateValid()) return 0;                    // is GPS date valid/present ?
     if(FixQuality==0) return 0;                     // Fix quality must be 1=GPS or 2=DGPS
     if(FixMode==1) return 0;                        // if GSA says "no lock" (when GSA is not there, FixMode=0)
     if(Satellites<=0) return 0;                     // if number of satellites none or invalid
     return 1; }

   void copyTime(GPS_Position &RefPosition)           // copy HH:MM:SS.SSS from another record
   { FracSec = RefPosition.FracSec;
     Sec     = RefPosition.Sec;
     Min     = RefPosition.Min;
     Hour    = RefPosition.Hour; }

   void copyDate(GPS_Position &RefPosition)           // copy YY:MM:DD from another record
   { Day     = RefPosition.Day;
     Month   = RefPosition.Month;
     Year    = RefPosition.Year; }

   void copyTimeDate(GPS_Position &RefPosition) { copyTime(RefPosition); copyDate(RefPosition); }

   uint8_t incrTime(void)                            // increment HH:MM:SS by one second
   { Sec++; if(Sec<60) return 0;
     Sec=0;
     Min++; if(Min<60) return 0;
     Min=0;
     Hour++; if(Hour<24) return 0;
     Hour=0;
     return 1; }                                     // return 1 if date needs to be incremented

   uint8_t MonthDays(void)                           // number of days per month
   { const uint16_t Table = 0x0CD5;
     // const uint8_t Table[12] = { 31,28,31,30, 31,30,31,31, 30,31,30,31 };
     if( (Month<1) || (Month>12) ) return 0;
     if( Month==2) return 28+isLeapYear();
     return 30 + ((Table>>(Month-1))&1); }

   void incrDate(void)                               // increment YY:MM:DD by one day
   { Day++; if(Day<MonthDays()) return;
     Day=1; Month++; if(Month<12) return;
     Month=1; Year++; }

   void incrTimeData(void) { if(incrTime()) incrDate(); }

#ifndef __AVR__ // there is not printf() with AVR
   void PrintDateTime(void) const { printf("%02d.%02d.%04d %02d:%02d:%05.2f", Day, Month, 2000+Year, Hour, Min, Sec+0.01*FracSec ); }
   void PrintTime(void)     const { printf("%02d:%02d:%05.2f", Hour, Min, Sec+0.01*FracSec ); }

   int PrintDateTime(char *Out) const { return sprintf(Out, "%02d.%02d.%04d %02d:%02d:%02d.%02d", Day, Month, Year, Hour, Min, Sec, FracSec ); }
   int PrintTime(char *Out)     const { return sprintf(Out, "%02d:%02d:%02d.%02d", Hour, Min, Sec, FracSec ); }

   void Print(void) const
   { printf("Time/Date = "); PrintDateTime(); printf("\n"); // printf(" = %10ld.%03dsec\n", (long int)UnixTime, mSec);
     printf("FixQuality/Mode=%d/%d: %d satellites DOP/H/V=%3.1f/%3.1f/%3.1f\n", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf("FixQuality=%d: %d satellites HDOP=%3.1f\n", FixQuality, Satellites, 0.1*HDOP);
     printf("Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m LatCosine=%+6.3f\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation, 1.0/(1<<12)*LatitudeCosine);
     printf("Speed/Heading = %3.1fm/s %05.1fdeg\n", 0.1*Speed, 0.1*Heading);
   }

   int Print(char *Out) const
   { int Len=0;
     Len+=sprintf(Out+Len, "Time/Date = "); Len+=PrintDateTime(Out+Len); printf("\n"); // Len+=sprintf(Out+Len, " = %10ld.%02dsec\n", (long int)UnixTime, FracSec);
     Len+=sprintf(Out+Len, "FixQuality/Mode=%d/%d: %d satellites DOP/H/V=%3.1f/%3.1f/%3.1f\n", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     Len+=sprintf(Out+Len, "Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     Len+=sprintf(Out+Len, "Speed/Heading = %3.1fm/s %05.1fdeg\n", 0.1*Speed, 0.1*Heading);
     return Len; }

   void PrintLine(void) const
   { PrintTime();
     printf(" %d/%d/%02d/%4.1f/%4.1f/%4.1f", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf(" [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf(" %4.1fm/s %05.1fdeg", 0.1*Speed, 0.1*Heading);
     printf("\n"); }

   int PrintLine(char *Out) const
   { int Len=PrintDateTime(Out);
     Len+=sprintf(Out+Len, " %d/%d/%02d", FixQuality, FixMode, Satellites);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, PDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, HDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, VDOP, 2, 1);
     Out[Len++]=' ';
     Out[Len++]='['; Len+=Format_SignDec(Out+Len, Latitude/60, 6, 4);
     Out[Len++]=','; Len+=Format_SignDec(Out+Len, Longitude/60, 7, 4);
     Out[Len++]=']'; Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, Altitude, 4, 1); Out[Len++]='m';
     Out[Len++]='/'; Len+=Format_SignDec(Out+Len, GeoidSeparation, 4, 1); Out[Len++]='m';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Speed,     2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Heading,   4, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]='\n'; Out[Len++]=0; return Len; }
#endif // __AVR__

   int8_t ReadNMEA(NMEA_RxMsg &RxMsg)
   {      if(RxMsg.isGPGGA()) return ReadGGA(RxMsg);
     else if(RxMsg.isGNGGA()) return ReadGGA(RxMsg);
     else if(RxMsg.isGPRMC()) return ReadRMC(RxMsg);
     else if(RxMsg.isGNRMC()) return ReadRMC(RxMsg);
     else if(RxMsg.isGPGSA()) return ReadGSA(RxMsg);
     else if(RxMsg.isGNGSA()) return ReadGSA(RxMsg);
     else return 0; }

   int8_t ReadNMEA(const char *NMEA)
   { int Err=0;
     Err=ReadGGA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadGSA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadRMC(NMEA); if(Err!=(-1)) return Err;
     return 0; }

   int8_t ReadGGA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<14) return -1;                                                        // no less than 14 paramaters
     if(ReadTime((const char *)RxMsg.ParmPtr(0))>0) Complete=1; else Complete=0;          // read time and check if same as the RMC says
     FixQuality =Read_Dec1(*RxMsg.ParmPtr(5)); if(FixQuality<0) FixQuality=0;             // fix quality: 0=invalid, 1=GPS, 2=DGPS
     Satellites=Read_Dec2((const char *)RxMsg.ParmPtr(6));                                // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(RxMsg.ParmPtr(6)[0]);
     if(Satellites<0) Satellites=0;
     ReadHDOP((const char *)RxMsg.ParmPtr(7));                                            // horizontal dilution of precision
     ReadLatitude(*RxMsg.ParmPtr(2), (const char *)RxMsg.ParmPtr(1));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(4), (const char *)RxMsg.ParmPtr(3));                    // Longitude
     ReadAltitude(*RxMsg.ParmPtr(9), (const char *)RxMsg.ParmPtr(8));                     // Altitude
     ReadGeoidSepar(*RxMsg.ParmPtr(11), (const char *)RxMsg.ParmPtr(10));                 // Geoid separation
     // calcLatitudeCosine();
     return 1; }

   int8_t ReadGGA(const char *GGA)
   { if( (memcmp(GGA, "$GPGGA", 6)!=0) && (memcmp(GGA, "$GNGGA", 6)!=0) ) return -1;                                           // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GGA)<14) return -2;                           // index parameters and check the sum
     if(ReadTime(GGA+Index[0])>0) Complete=1; else Complete=0;
     FixQuality =Read_Dec1(GGA[Index[5]]); if(FixQuality<0) FixQuality=0;                 // fix quality
     Satellites=Read_Dec2(GGA+Index[6]);                                                  // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(GGA[Index[6]]);
     if(Satellites<0) Satellites=0;
     ReadHDOP(GGA+Index[7]);                                                              // horizontal dilution of precision
     ReadLatitude( GGA[Index[2]], GGA+Index[1]);                                          // Latitude
     ReadLongitude(GGA[Index[4]], GGA+Index[3]);                                          // Longitude
     ReadAltitude(GGA[Index[9]], GGA+Index[8]);                                           // Altitude
     ReadGeoidSepar(GGA[Index[11]], GGA+Index[10]);                                       // Geoid separation
     // calcLatitudeCosine();
     return 1; }

   int8_t ReadGSA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<17) return -1;
     FixMode =Read_Dec1(*RxMsg.ParmPtr(1)); if(FixMode<0) FixMode=0;                      // fix mode
     ReadPDOP((const char *)RxMsg.ParmPtr(14));                                           // total dilution of precision
     ReadHDOP((const char *)RxMsg.ParmPtr(15));                                           // horizontal dilution of precision
     ReadVDOP((const char *)RxMsg.ParmPtr(16));                                           // vertical dilution of precision
     return 1; }

   int8_t ReadGSA(const char *GSA)
   { if( (memcmp(GSA, "$GPGSA", 6)!=0) && (memcmp(GSA, "$GNGSA", 6)!=0) ) return -1;      // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GSA)<17) return -2;                           // index parameters and check the sum
     FixMode =Read_Dec1(GSA[Index[1]]); if(FixMode<0) FixMode=0;
     ReadPDOP(GSA+Index[14]);
     ReadHDOP(GSA+Index[15]);
     ReadVDOP(GSA+Index[16]);
     return 1; }

   int ReadRMC(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<12) return -1;                                                        // no less than 12 parameters
     if(ReadTime((const char *)RxMsg.ParmPtr(0))>0) Complete=1; else Complete=0;          // read time and check if same as the GGA says
     if(ReadDate((const char *)RxMsg.ParmPtr(8))<0) setDefaultDate();                     // date
     ReadLatitude(*RxMsg.ParmPtr(3), (const char *)RxMsg.ParmPtr(2));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(5), (const char *)RxMsg.ParmPtr(4));                    // Longitude
     ReadSpeed((const char *)RxMsg.ParmPtr(6));                                           // Speed
     ReadHeading((const char *)RxMsg.ParmPtr(7));                                         // Heading
     calcLatitudeCosine();
     return 1; }

   int8_t ReadRMC(const char *RMC)
   { if( (memcmp(RMC, "$GPRMC", 6)!=0) && (memcmp(RMC, "$GNRMC", 6)!=0) ) return -1;      // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, RMC)<12) return -2;                           // index parameters and check the sum
     if(ReadTime(RMC+Index[0])>0) Complete=1; else Complete=0;
     if(ReadDate(RMC+Index[8])<0) setDefaultDate();
     ReadLatitude( RMC[Index[3]], RMC+Index[2]);
     ReadLongitude(RMC[Index[5]], RMC+Index[4]);
     ReadSpeed(RMC+Index[6]);
     ReadHeading(RMC+Index[7]);
     calcLatitudeCosine();
     return 1; }

   int8_t calcDifferences(GPS_Position &RefPos) // calculate climb rate and turn rate with an earlier reference position
   { ClimbRate=0; TurnRate=0;
     if(RefPos.FixQuality==0) return 0;
     int TimeDiff=Sec-RefPos.Sec; if(TimeDiff<(-30)) TimeDiff+=60;
     if(TimeDiff==0) return 0;
     ClimbRate = Altitude-RefPos.Altitude;
     TurnRate = Heading-RefPos.Heading;
     if(TurnRate>1800) TurnRate-=3600; else if(TurnRate<(-1800)) TurnRate+=3600;
     if(Baro && RefPos.Baro && (abs(Altitude-StdAltitude)<2500) )
     { ClimbRate = StdAltitude-RefPos.StdAltitude; }
     if(TimeDiff==1)
     { }
     else if(TimeDiff==2)
     { ClimbRate=(ClimbRate+1)>>1;
       TurnRate=(TurnRate+1)>>1; }
     else
     { ClimbRate/=TimeDiff;
       TurnRate/=TimeDiff; }
     return TimeDiff; }

   void Encode(ADSL_Packet &Packet) const
   { Packet.setAlt((Altitude+GeoidSeparation+5)/10);
     Packet.setLatOGN(Latitude);
     Packet.setLonOGN(Longitude);
     Packet.TimeStamp = (Sec*4+FracSec/25)%60;
     Packet.setSpeed(((uint32_t)Speed*4+5)/10);
     Packet.setClimb(((int32_t)ClimbRate*8+5)/10);
     // if(hasClimb) Packet.setClimb(((int32_t)ClimbRate*8+5)/10);
     //        else  Packet.clrClimb();
     Packet.setTrack(((uint32_t)Heading*32+112)/225);
     Packet.Integrity[0]=0; Packet.Integrity[1]=0;
     if((FixQuality>0)&&(FixMode>=2))
     { Packet.setHorAccur((HDOP*2+5)/10);
       Packet.setVerAccur((VDOP*3+5)/10); }
   }

   void Encode(OGN_Packet &Packet) const
   { Packet.Position.FixQuality = FixQuality<3 ? FixQuality:3;
     if((FixQuality>0)&&(FixMode>=2)) Packet.Position.FixMode = FixMode-2;
                                 else Packet.Position.FixMode = 0;
     if(PDOP>0) Packet.EncodeDOP(PDOP-10);                              // encode PDOP from GSA
           else Packet.EncodeDOP(HDOP-10);                              // or if no GSA: use HDOP
     int ShortTime=Sec;
     if(FracSec>=50) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.Position.Time=ShortTime;
     Packet.EncodeLatitude(Latitude);
     Packet.EncodeLongitude(Longitude);
     Packet.EncodeSpeed(Speed);
     Packet.EncodeHeading(Heading);
     Packet.EncodeClimbRate(ClimbRate);
     Packet.EncodeTurnRate(TurnRate);
     Packet.EncodeAltitude((Altitude+5)/10);
     if(Baro) Packet.EncodeStdAltitude((StdAltitude+5)/10);
         else Packet.clrBaro();
   }

   void EncodeStatus(OGN_Packet &Packet) const
   { Packet.Status.ReportType=0;
     int ShortTime=Sec;
     if(FracSec>=50) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.Status.Time=ShortTime;
     Packet.Status.FixQuality = FixQuality<3 ? FixQuality:3;
     Packet.Status.Satellites = Satellites<15 ? Satellites:15;
     Packet.EncodeAltitude((Altitude+5)/10);
     if(Baro)
     { Packet.EncodeTemperature(Temperature);
       Packet.Status.Pressure = (Pressure+16)>>5; }
     else
     { Packet.Status.Pressure = 0; }
     Packet.Status.Humidity=0;
   }

   // uint8_t getFreqPlan(void) const // get the frequency plan from Lat/Lon: 1 = Europe + Africa, 2 = USA/CAnada, 3 = Australia + South America, 4 = New Zeeland
   // { if( (Longitude>=(-20*600000)) && (Longitude<=(60*600000)) ) return 1; // between -20 and 60 deg Lat => Europe + Africa: 868MHz band
   //   if( Latitude<(20*600000) )                                            // below 20deg latitude
   //   { if( ( Longitude>(164*600000)) && (Latitude<(-30*600000)) && (Latitude>(-48*600000)) ) return 4;  // => New Zeeland
   //     return 3; }                                                         // => Australia + South America: upper half of 915MHz band
   //   return 2; }                                                           // => USA/Canada: full 915MHz band

   // static int32_t calcLatDistance(int32_t Lat1, int32_t Lat2)             // [m] distance along latitude
   // { return ((int64_t)(Lat2-Lat1)*0x2f684bda+0x80000000)>>32; }

   // static int32_t calcLatAngle32(int32_t Lat)                             // convert latitude to 32-bit integer angle
   // { return ((int64_t)Lat*2668799779u+0x4000000)>>27; }

   static int16_t calcLatAngle16(int32_t Lat)                             // convert latitude to 16-bit integer angle
   { return ((int64_t)Lat*1303125+0x80000000)>>32; }

   // static int32_t calcLatCosine(int32_t LatAngle)                         // calculate the cosine of the latitude 32-bit integer angle
   // { return IntSine((uint32_t)(LatAngle+0x40000000)); }

   // static int32_t calcLatCosine(int16_t LatAngle)                         // calculate the cosine of the latitude 16-bit integer angle
   // { return IntSine((uint16_t)(LatAngle+0x4000)); }

   static int16_t calcLatCosine(int16_t LatAngle)
   { return Icos(LatAngle); }

   // int32_t getLatDistance(int32_t RefLatitude) const                      // [m] distance along latitude
   // { return calcLatDistance(RefLatitude, Latitude); }

   // int32_t getLonDistance(int32_t RefLongitude) const                     // [m] distance along longitude
   // { int32_t Dist = calcLatDistance(RefLongitude, Longitude);             //
   //   int16_t LatAngle =  calcLatAngle16(Latitude);
   //   int32_t LatCos = calcLatCosine(LatAngle);
   //   // printf("Latitude=%+d, LatAngle=%04X LatCos=%08X\n", Latitude, (uint16_t)LatAngle, LatCos);
   //   return ((int64_t)Dist*LatCos+0x40000000)>>31; }                      // distance corrected by the latitude cosine

   void    calcLatitudeCosine(void)
   { int16_t LatAngle =  calcLatAngle16(Latitude);
       LatitudeCosine = calcLatCosine(LatAngle); }

  private:

   int8_t ReadLatitude(char Sign, const char *Value)
   { int8_t Deg=Read_Dec2(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+2); if(Min<0) return -1;
     if(Value[4]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+5); if(FracMin<0) return -1;
     // printf("Latitude: %c %02d %02d %04d\n", Sign, Deg, Min, FracMin);
     Latitude = Times60((int16_t)Deg) + Min;
     Latitude = Latitude*(int32_t)10000 + FracMin;
     // printf("Latitude: %d\n", Latitude);
     if(Sign=='S') Latitude=(-Latitude);
     else if(Sign!='N') return -1;
     // printf("Latitude: %d\n", Latitude);
     return 0; }                                    // Latitude units: 0.0001/60 deg

   int8_t ReadLongitude(char Sign, const char *Value)
   { int16_t Deg=Read_Dec3(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+3); if(Min<0) return -1;
     if(Value[5]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+6); if(FracMin<0) return -1;
     Longitude = Times60((int16_t)Deg) + Min;
     Longitude = Longitude*(int32_t)10000 + FracMin;
     if(Sign=='W') Longitude=(-Longitude);
     else if(Sign!='E') return -1;
     return 0; }                                    // Longitude units: 0.0001/60 deg

   int8_t ReadAltitude(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(Altitude, Value); }          // Altitude units: 0.1 meter

   int8_t ReadGeoidSepar(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(GeoidSeparation, Value); }   // GeoidSepar units: 0.1 meter

   int8_t ReadSpeed(const char *Value)
   { int32_t Knots;
     if(Read_Float1(Knots, Value)<1) return -1;      // Speed: 0.1 knots
     Speed=(527*Knots+512)>>10; return 0; }          // convert speed to 0.1 meter/sec

   int8_t ReadHeading(const char *Value)
   { return Read_Float1(Heading, Value); }           // Heading units: 0.1 degree

   int8_t ReadPDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     PDOP=DOP; return 0; }

   int ReadHDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     HDOP=DOP; return 0; }

   int ReadVDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     VDOP=DOP; return 0; }

   int8_t ReadTime(const char *Value)
   { int8_t Prev; int8_t Same=1;
     Prev=Hour;
     Hour=Read_Dec2(Value);  if(Hour<0) return -1; // read hour (two digits)
     if(Prev!=Hour) Same=0;
     Prev=Min;
     Min=Read_Dec2(Value+2); if(Min<0)  return -1; // read minute (two digits)
     if(Prev!=Min) Same=0;
     Prev=Sec;
     Sec=Read_Dec2(Value+4); if(Sec<0)  return -1; // read second (two digits)
     if(Prev!=Sec) Same=0;
     Prev=FracSec;
     if(Value[6]=='.')                            // is there a second fraction ?
     { FracSec=Read_Dec2(Value+7); if(FracSec<0) return -1; }
     if(Prev!=FracSec) Same=0;
     return Same; }                             // return 1 when time did not change (both RMC and GGA were for same time)

   int8_t ReadDate(const char *Param)
   { Day=Read_Dec2(Param);     if(Day<0)   return -1; // read calendar year (two digits - thus need to be extended to four)
     Month=Read_Dec2(Param+2); if(Month<0) return -1; // read calendar month
     Year=Read_Dec2(Param+4);  if(Year<0)  return -1; // read calendar day
     return 0; }

   int8_t static IndexNMEA(uint8_t Index[20], const char *Seq) // index parameters and verify the NMEA checksum
   { if(Seq[0]!='$') return -1;
     if(Seq[6]!=',') return -1;
     uint8_t Check=Seq[1]^Seq[2]^Seq[3]^Seq[4]^Seq[5]^Seq[6];
     Index[0]=7; int8_t Params=1; int8_t Ptr;
     for(Ptr=7; ; )
     { char ch=Seq[Ptr++]; if(ch<' ') return -1;
       if(ch=='*') break;
       Check^=ch;
       if(ch==',') { Index[Params++]=Ptr; }
     }
     if(Seq[Ptr++]!=HexDigit(Check>>4)  ) { /* printf("H:%c:%c <=> %02X\n", Seq[Ptr-1],Seq[Ptr  ], Check); */ return -2; }
     if(Seq[Ptr++]!=HexDigit(Check&0x0F)) { /* printf("L:%c:%c <=> %02X\n", Seq[Ptr-2],Seq[Ptr-1], Check); */ return -2; }
     // printf("%s => [%d]\n", Seq, Params);
     return Params; }

  public:

   uint32_t getUnixTime(void)                               // return the Unix timestamp (tested 2000-2099)
   { uint16_t Days = DaysSince00() + DaysSimce1jan();
     return Times60(Times60(Times24((uint32_t)(Days+10957)))) + Times60((uint32_t)(Times60((uint16_t)Hour) + Min)) + Sec; }

   uint32_t getFatTime(void)                                // return timestamp in FAT format
   { uint16_t Date = ((uint16_t)(Year+20)<<9) | ((uint16_t)Month<<5) | Day;
     uint16_t Time = ((uint16_t)Hour<<11) | ((uint16_t)Min<<5) | (Sec>>1);
     return ((uint32_t)Date<<16) | Time; }

  private:

   uint8_t isLeapYear(void) { return (Year&3)==0; }

#ifdef __AVR__
   int16_t DaysSimce1jan(void)
   { static const uint8_t DaysDiff[12] PROGMEM = { 0, 3, 3, 6, 8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = Times28((uint16_t)(Month-(int8_t)1)) + pgm_read_byte(DaysDiff+(Month-1)) + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#else
   int16_t DaysSimce1jan(void)        //  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
   { static const uint8_t DaysDiff[12] = { 0,  3,  3,  6,  8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = Times28((uint16_t)(Month-(int8_t)1)) + DaysDiff[Month-1] + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#endif

   uint16_t DaysSince00(void)
   { uint16_t Days = 365*Year;
     if(Year>0) Days += ((Year-1)>>2)+1;
     return Days; }

   template <class Type>
     static Type Times60(Type X) { return ((X<<4)-X)<<2; }

   template <class Type>
     static Type Times28(Type X) { X+=(X<<1)+(X<<2); return X<<2; }

   template <class Type>
     static Type Times24(Type X) { X+=(X<<1);        return X<<3; }

} ;

#endif // of __OGN_H__


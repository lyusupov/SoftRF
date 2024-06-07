#ifndef __ADSL_H__
#define __ADSL_H__

#include <stdlib.h>
#include <math.h>
// #include <string.h>
// #include "radiodemod.h"
// #include "intmath.h"
#include "ognconv.h"
#include "bitcount.h"
#include "format.h"
// #include "crc1021.h"

class ADSL_Packet
{ public:

   const static uint8_t TxBytes = 27; // including SYNC, Length, actual packet content (1+20 bytes) and 3-byte CRC
   const static uint8_t SYNC1 = 0x72; // two SYNC bytes - Lemgth byte can be considered the 3rd SYNC byte as it is fixed
   const static uint8_t SYNC2 = 0x4B;

   uint8_t SYNC[2];          // two bytes for correct alignment: can contain the last two SYNC bytes
   uint8_t Length;           // [bytes] packet length = 24 = 0x18 (excluding length but including the 24-bit CRC)
   uint8_t Version;          // Version[4]/Sigmature[1]/Key[2]/Reserved[1]
   union                     // 20 bytes
   { uint32_t Word[5];       // this part to be scrambled/encrypted, is aligned to 32-bit
     struct                  // this is aligned to 32-bit
     { uint8_t Type;         // 0x02=iConspicuity, 0x41=Telemetry, bit #7 = Unicast
       uint8_t Address  [4]; // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       union
       { uint8_t Meta     [2]; // Time[6]/Cat[5]/Emergency[3]/FlightState[2]
         struct
         { uint8_t TimeStamp   :6; // [0.25sec]
           uint8_t FlightState :2; // 0=unknown, 1=ground, 2=airborne
           uint8_t AcftCat     :5; // 1=light, 2=small-heavy, 3=heli, 4=glider, 5=baloon/airship, 6=para/hang-glider, 7=skydiver,
           uint8_t Emergency   :3; // 1=OK
         } __attribute__((packed));
       } ;
       uint8_t Position[11]; // Lat[24]/Lon[24]/Speed[8]/Alt[14]/Climb[9]/Track[9]
       union
       { uint8_t Integrity[2]; // SourceInteg[2]/DesignAssurance[2]/NavigationIntegrity[4]/NorizAccuracy[3]/VertAccuracy[2]/ValocityAccuracy[2]/Reserved[1]
         struct
         { uint8_t SourceIntegrity:2; // 3=1e-7/h, 2=1e-5/h, 1=1e-3/h  <= SIL
           uint8_t DesignAssurance:2; // 3=B, 2=C, 1=D                 <= SDA
           uint8_t NavigIntegrity :4; // 12=7.5m, 11=25m, 10=75m       <= NIC  +1
           uint8_t HorizAccuracy  :3; // 7=3m, 6=10m, 5=30m. 4=92.5m   <= NACp -4
           uint8_t VertAccuracy   :2; // 3=15m, 2=45m, 1=150m          <= GVA
           uint8_t VelAccuracy    :2; // 3=1m/s 2=3m/s 3=10m/s         <= NACv
           uint8_t Reserved       :1; //
         } __attribute__((packed));
       } ;
     } __attribute__((packed)) ;

     struct
     { uint8_t Type;             // 0x02=iConspicuity, bit #7 = Unicast, 0x42 = telemetry
       uint8_t Address  [4];     // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       struct                    //
       { uint8_t  TimeStamp:6;   // [0.25 sec] 0..14.75
         uint8_t  TelemType:2;   // 0=telemetry, 1=info, 2=flight
       } __attribute__((packed)) Header;  // 1 byte
       struct
       { uint8_t Satellites:4;   // 1-15
         uint8_t        DOP:5;   // [0.125] 1.0-3.75
         uint8_t        SNR:5;   // [dB] 10-40 dB, bias by 10 dB
         uint8_t FixQuality:2;   //
       } __attribute__((packed)) GPS;     // 2 bytes
       struct
       { int16_t  HeightDiff:10; // [m] +/-500m HAE-pAlt
         uint16_t   Pressure:14; // [8 Pa]
          int8_t Temperature: 8; // [1/2 deg] -63..+63
         uint8_t    Humidity: 6; //
         uint8_t    Spare   : 2;
       } __attribute__((packed)) Baro;    // 5 bytes
       struct
       { uint16_t Voltage : 10;  // [4mV] VR 0.00-15.32V EncodeUR2V8()
         uint8_t  Capacity:  6;  // [100%/64] 0..100%
       } __attribute__((packed)) Battery; // 2 bytes
       struct
       { uint8_t RxRate:  6;     // [0.25Hz] VR 0-232/4 EncodeUR2V4()
         uint8_t RxNoise: 6;     // [dB] -120..-60dBm
         uint8_t TxPower: 4;     // [dBm] 10..25
       } __attribute__((packed)) Radio;   // 2 bytes
       struct
       { uint8_t Accel:8;
         uint8_t  Gyro:8;
         uint8_t Spare:8;
       } __attribute__((packed)) IMU;     // 3 bytes
     } __attribute__((packed)) Telemetry;

     struct
     { uint8_t Type;             // 0x02=iConspicuity, bit #7 = Unicast, 0x42 = telemetry
       uint8_t Address  [4];     // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       struct                    //
       { uint8_t  InfoType :6;   //
         uint8_t  TelemType:2;   // 1 = Info
       } __attribute__((packed)) Header;  // 1 byte
       char Msg[14];
     } __attribute__((packed)) Info;

     struct
     { uint8_t Type;             // 0x02=iConspicuity, bit #7 = Unicast, 0x42 = telemetry
       uint8_t Address  [4];     // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       struct                    //
       { uint8_t  TimeStamp:6;   // [0.25 sec] 0..14.75
         uint8_t  TelemType:2;   // 2 = flight-status
       } __attribute__((packed)) Header;  // 1 byte
       struct
       { uint16_t   Time:16;     // [sec] takeoff/landing time
         uint16_t  Dist :16;     // [km] distance flown
          uint8_t   Dir : 8;     // [cyclic] takeoff/landing direction
          bool  Landing : 1;     // 0=takeoff, 1=landing
          uint8_t Spare : 7;     //
          int32_t   Lat :20;     // [cyclic] takeoff/landing latitude
         uint16_t   Alt :12;     // [m] takeoff/landing altitude HAE
          int32_t   Lon :20;     // [cyclic] takeoff/landing longitude
         uint16_t MaxAlt:12;     // [m] max. flight altitude HAE
       } __attribute__((packed)) Takeoff; // 3x4 = 14 btyes

     } __attribute__((packed)) Flight;

   } ;
   uint8_t CRC24[3];           // 24-bit (is aligned to 32-bit)

// --------------------------------------------------------------------------------------------------------

   static const uint8_t InfoParmNum = 15; // [int]  number of info-parameters and their names
   static const char *InfoParmName(uint8_t Idx) { static const char *Name[InfoParmNum] =
                                                  { "Pilot", "Manuf", "Model", "Type", "SN", "Reg", "ID", "Class",
                                                    "Task" , "Base" , "ICE"  , "PilotID", "Hard", "Soft", "Crew" } ;
                                                  return Idx<InfoParmNum ? Name[Idx]:0; }

// --------------------------------------------------------------------------------------------------------

  public:
   void Init(uint8_t Type=0x02)
   { SYNC[0]=SYNC1; SYNC[1]=SYNC2;
     Length=TxBytes-3; Version=0x00;
     for(int Idx=0; Idx<5; Idx++)
       Word[Idx]=0;
     this->Type=Type; }

   void Print(void) const
   { if(Type==0x02)
       printf(" v%02X %4.1fs: %02X:%06X [%+09.5f,%+010.5f]deg %dm %+4.1fm/s %05.1fdeg %3.1fm/s\n",
         Version, 0.25*TimeStamp, getAddrTable(), getAddress(), FNTtoFloat(getLat()), FNTtoFloat(getLon()),
         getAlt(), 0.125*getClimb(), (45.0/0x40)*getTrack(), 0.25*getSpeed());
     else
       printf(" v%02X %4.1fs: %02X:%06X\n",
         Version, 0.25*TimeStamp, getAddrTable(), getAddress() );
   }

   int PrintFlight(char *Out, uint32_t Time=0) const // type #2 = Flight status
   { int Len=0;
     Len+=sprintf(Out+Len, " %s", Flight.Takeoff.Landing?"Landing:":"Takeoff:");
     Len+=sprintf(Out+Len, "[%+09.5f,%+010.5f] %3dm %03.0f %dm/%dkm",
             1e-7*FNTtoUBX(Flight.Takeoff.Lat<<4), 1e-7*FNTtoUBX(Flight.Takeoff.Lon<<4),
             UnsVRdecode<int32_t,12>(Flight.Takeoff.Alt<<2)-316, (90.0/0x40)*Flight.Takeoff.Dir,
             UnsVRdecode<int32_t,12>(Flight.Takeoff.MaxAlt<<2)-316, Flight.Takeoff.Dist);
     if(Time)
     { uint16_t Diff=Time;
       Diff=Flight.Takeoff.Time-Diff;
       Time-=Diff;
       Out[Len++]=' ';
       Len+=Format_HHMMSS(Out+Len, Time);
       Out[Len]=0; }
     return Len; }

   int PrintInfo(char *Out) const // type #1 = Info
   { int Len=0;
     if(Info.Header.InfoType<InfoParmNum) Len+=sprintf(Out+Len, " %s=%.14s", InfoParmName(Info.Header.InfoType), Info.Msg);
                                     else Len+=sprintf(Out+Len, " #%02X=%.14s", Info.Header.InfoType, Info.Msg);
     return Len; }

   int PrintTelemetry(char *Out) const // type #0 = Telemetry
   { int Len=0;
     Len+=sprintf(Out+Len, " %dsat/%d/%3.1f/%ddB",
                    Telemetry.GPS.Satellites, Telemetry.GPS.FixQuality, 0.25*Telemetry.GPS.DOP, Telemetry.GPS.SNR+10);
     if(Telemetry.Baro.Pressure)            Len+=sprintf(Out+Len, " %3.1fhPa", 0.08*Telemetry.Baro.Pressure);
     if(Telemetry.Baro.HeightDiff!=(-512))  Len+=sprintf(Out+Len, "/%+dm",          Telemetry.Baro.HeightDiff);
     if(Telemetry.Baro.Temperature!=(-128)) Len+=sprintf(Out+Len, "/%+4.1fdegC",0.5*Telemetry.Baro.Temperature);
     if(Telemetry.Baro.Humidity)            Len+=sprintf(Out+Len, "/%d%%",    ((int)Telemetry.Baro.Humidity*100+32)>>6);
     Len+=sprintf(Out+Len, " %+5.2fV/%d%%",
               0.004*DecodeUR2V8(Telemetry.Battery.Voltage), ((int)Telemetry.Battery.Capacity*100+32)>>6);
     Len+=sprintf(Out+Len, " %d/%+ddBm/%3.1fHz",
               Telemetry.Radio.TxPower+10, (int8_t)Telemetry.Radio.RxNoise-120, 0.25*DecodeUR2V4(Telemetry.Radio.RxRate));
     return Len; }

   int Print(char *Out) const
   { Out[0]=0;
     if(Type==0x02)
       return sprintf(Out, "%02X:%06X %4.1fs [%+09.5f,%+010.5f]deg %dm %+4.1fm/s %05.1fdeg %3.1fm/s",
         getAddrTable(), getAddress(), 0.25*TimeStamp, FNTtoFloat(getLat()), FNTtoFloat(getLon()),
         getAlt(), 0.125*getClimb(), (45.0/0x40)*getTrack(), 0.25*getSpeed());
     if(Type==0x42)
     { int Len=0;
       if(Telemetry.Header.TelemType==0)
       { Len=sprintf(Out, "%02X:%06X %4.1fs", getAddrTable(), getAddress(), 0.25*Telemetry.Header.TimeStamp);
         Len+=PrintTelemetry(Out+Len); }
       else if(Telemetry.Header.TelemType==1)
       { Len=sprintf(Out, "%02X:%06X ", getAddrTable(), getAddress() );
         Len+=PrintInfo(Out+Len); }
       else if(Telemetry.Header.TelemType==2)
       { int Len=sprintf(Out, "%02X:%06X ", getAddrTable(), getAddress() );
         Len+=PrintFlight(Out+Len); }
       else
       { Len+=sprintf(Out, "%02X:%06X %d:%02X (telemetry/diagnostic)",
                getAddrTable(), getAddress(), Telemetry.Header.TelemType, Telemetry.Header.TimeStamp); }
       return Len; }
     return 0; }

   uint8_t Dump(char *Out)
   { uint8_t Len=0;
     Len+=Format_Hex(Out+Len, Length);
     Len+=Format_Hex(Out+Len, Version);
     for(int Idx=0; Idx<5; Idx++)
     { Out[Len++]=' '; Len+=Format_Hex(Out+Len, Word[Idx]); }
     Out[Len++]=' ';
     Len+=Format_Hex(Out+Len, CRC24[0]);
     Len+=Format_Hex(Out+Len, CRC24[1]);
     Len+=Format_Hex(Out+Len, CRC24[2]);
     return Len; }

   uint8_t  getRelay(void)     const { return Address[3]&0x80; }
   void     setRelay(uint8_t Relay)  { Address[3] = (Address[3]&0x7F) | (Relay<<7); }

   static uint32_t get3bytes(const uint8_t *Byte) { int32_t Word=Byte[2]; Word<<=8; Word|=Byte[1]; Word<<=8; Word|=Byte[0]; return Word; }
   static void     set3bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; }

   static uint32_t get4bytes(const uint8_t *Byte)
   { uint32_t A = Byte[0];
     uint32_t B = Byte[1];
     uint32_t C = Byte[2];
     uint32_t D = Byte[3];
     return A | (B<<8) | (C<<16) | (D<<24); }
   // { uint32_t Word =Byte[3]; Word<<=8;
   //            Word|=Byte[2]; Word<<=8;
   //            Word|=Byte[1]; Word<<=8;
   //            Word|=Byte[0];
   //   return Word; }
   static void     set4bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; Byte[3]=Word>>24; }

   uint32_t getAddress(void) const
   { uint32_t Addr = get4bytes(Address); return (Addr>>6)&0x00FFFFFF; }

   void setAddress(uint32_t NewAddr)
   { uint32_t Addr = get4bytes(Address);
     Addr = (Addr&0xC000003F) | (NewAddr<<6);
     set4bytes(Address, Addr); }

   uint8_t  getAddrTable(void) const { return Address[0]&0x3F; }
    void    setAddrTable(uint8_t Table) { Address[0] = (Address[0]&0xC0) | Table; }

   uint8_t getAddrTypeOGN(void) const
   { uint8_t Table=getAddrTable();
     if(Table==0x05) return 1;         // ICAO
     if(Table==0x06) return 2;         // FLARM
     if(Table==0x07) return 3;         // OGN
     if(Table==0x08) return 2;         // FANET => FLARM ?
     return 0; }

   void setAddrTypeOGN(uint8_t AddrType)
   { if(AddrType==0) setAddrTable(0);
     else setAddrTable(AddrType+4); }

   void setAcftTypeOGN(uint8_t AcftType)                       // set OGN aircraft-type
   { const uint8_t Map[16] = { 0, 4, 1, 3,                     // unknown, glider, tow-plane, helicopter
                               8, 1, 7, 7,                     // sky-diver, drop plane, hang-glider, para-glider
                               1, 2, 0, 5,                     // motor airplane, jet, UFO, balloon
                               5,11, 0, 0 } ;                  // airship, UAV, ground vehicle, static object
     if(AcftType<16) AcftCat=Map[AcftType];
                else AcftCat=0; }
   uint8_t getAcftTypeOGN(void) const                          // get OGN aircraft-type
   { const uint8_t Map[32] = { 0, 8, 9, 3, 1,11, 2, 7,
                               4,13, 3,13,13,13, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0 } ;
     return Map[AcftCat]; }

   static uint8_t getAcftTypeADSB(uint8_t AcftCat)
   { uint8_t Cat=0; if(AcftCat>13) return Cat;
     const uint8_t Map[14] = { 0xA0, 0xA1, 0xA2, 0xA7, 0xB1, 0xB2, 0xB4, 0xB4, 0xB3, 0xA7, 0xA7, 0xB6, 0xB6, 0xB6 } ;
     return Map[AcftCat]; }

// aircraft-types
//                   ADS-L   FLARM/OGN/PilotAware  FANET  ADS-B  GDL90
// no info             0        0                          x0
// motor plane         1        8                          A1
// jet plane          1-2       9                          A2
// towing plane        1        2                          A1
// drop plane          1        5                          A1
// light fixed wing    1        2                          A1     1
// small fixed wing    2        2                          A2     2
//
// rotorcraft          3        3                          A7     7
// glider              4        1                          B1     9
// balloon             5        B                          B2    10
// airship             5        C                          B2    10
// Ultralight          6                                   B4    12
// hang-glider         7        6                          B4    12
// para-glider         7        7                          B4    12
// skydiver            8        4                          B3    11
// VTOL/UAM            9        3                          A7
// gyrocopter         10        3                          A7
// UAV                11-13     D                          B6    14
// space vehicle                                           B7    15
// ground vehicle               E                          C3 C1 17 18
// Fixed object                 F                          C4-C7 19

// --------------------------------------------------------------------------------------------------------

   uint32_t getTime(int16_t &msTime, uint32_t RefTime, int FwdMargin=3) const
   { msTime=250*(TimeStamp&3);
     if(TimeStamp>=60) return 0;
     int Sec=RefTime%15;
     int DiffSec=(TimeStamp>>2)-Sec;
     if(DiffSec>FwdMargin) DiffSec-=15;
     else if(DiffSec<=(-15+FwdMargin)) DiffSec+=15;
     return RefTime+DiffSec; }           // get out the correct position timestamp

   uint8_t getHorAccur(void) const
   { const uint8_t Map[8] = { 63, 63, 63, 63, 63, 30, 10, 3 } ;
     return Map[HorizAccuracy]; }
   void setHorAccur(uint8_t Prec)
   {      if(Prec<= 3) HorizAccuracy=7;
     else if(Prec<=10) HorizAccuracy=6;
     else if(Prec<=30) HorizAccuracy=5;
     else HorizAccuracy=4;
     VelAccuracy = HorizAccuracy-4; }

   uint8_t getVerAccur(void) const                // [m] vertical accuracy
   { const uint8_t Map[8] = { 63, 63, 45, 15 } ;
     return Map[VertAccuracy]; }
   void setVerAccur(uint8_t Prec)
   {      if(Prec<=15) VertAccuracy=3;
     else if(Prec<=45) VertAccuracy=2;
     else VertAccuracy=1; }

   static int32_t FNTtoOGN(int32_t Coord) { return ((int64_t)Coord*27000219 +(1<<28))>>29; }    // [FANET cordic] => [0.0001/60 deg]
   static int32_t OGNtoFNT(int32_t Coord) { return ((int64_t)Coord*83399317 +(1<<21))>>22; }    // [0.0001/60 deg] => [FANET cordic]
   static int32_t FNTtoUBX(int32_t Coord) { return ((int64_t)Coord*900007296+(1<<29))>>30; }    // [FANET-cordic ] => [1e-7 deg]
   static int32_t UBXtoFNT(int32_t Coord) { return ((int64_t)Coord*5003959  +(1<<21))>>22; }    // [1e-7 deg]      => [FANET cordic]
   static float   FNTtoFloat(int32_t Coord)                             // convert from FANET cordic units to float degrees
   { const float Conv = 90.0007295677/0x40000000;                       // FANET cordic conversion factor (not exactly cordic)
     return Conv*Coord; }

    int32_t getLatOGN(void) const { return FNTtoOGN(getLat()); }
    int32_t getLonOGN(void) const { return FNTtoOGN(getLon()); }

    int32_t getLatUBX(void) const { return FNTtoUBX(getLat()); }
    int32_t getLonUBX(void) const { return FNTtoUBX(getLon()); }

    int32_t getLat(void) const { int32_t Lat=get3bytes(Position  ); Lat<<=8; Lat>>=1; return Lat; } // FANET-cordic
    int32_t getLon(void) const { int32_t Lon=get3bytes(Position+3); Lon<<=8; return Lon; }          // FANET-cordic

    void setLatOGN(int32_t Lat)  { setLat(OGNtoFNT(Lat)); }
    void setLonOGN(int32_t Lon)  { setLon(OGNtoFNT(Lon)); }

    void    setLat(int32_t Lat)  { Lat = (Lat+0x40)>>7; set3bytes(Position  , Lat); }           // FANET-cordic
    void    setLon(int32_t Lon)  { Lon = (Lon+0x80)>>8; set3bytes(Position+3, Lon); }           // FANET-cordic

    uint16_t getSpeed(void) const { return UnsVRdecode<uint16_t,6>(Position[6]); }              // [0.25 m/s]
    void setSpeed(uint16_t Speed) { Position[6] = UnsVRencode<uint16_t,6>(Speed); }             // [0.25 m/s]

   int32_t getAlt(void) const                                                                  // [m]
   { int32_t Word=Position[8]&0x3F; Word<<=8; Word|=Position[7];
     return UnsVRdecode<int32_t,12>(Word)-320; }
   void setAlt(int32_t Alt)
   { Alt+=320; if(Alt<0) Alt=0;
     int32_t Word=UnsVRencode<uint32_t,12>(Alt);
     Position[7]=Word;
     Position[8] = (Position[8]&0xC0) | (Word>>8); }

   int16_t getClimbWord(void) const                                                             //
   { int16_t Word=Position[9]&0x7F; Word<<=2; Word|=Position[8]>>6; return Word; }
   int16_t getClimb(void) const                                                                 // [0.125 m/s]
   { return SignVRdecode<int16_t,6>(getClimbWord()); }
   void setClimb(int16_t Climb)                                                                 // [0.125 m/s]
   { setClimbWord(SignVRencode<int16_t,6>(Climb)); }
   void setClimbWord(int16_t Word)
   { Position[8] = (Position[8]&0x3F) | ((Word&0x03)<<6);
     Position[9] = (Position[9]&0x80) |  (Word>>2); }
   bool hasClimb(void) const { return getClimbWord()!=0x100; }                                        // climb-rate present or absent
   void clrClimb(void) { setClimbWord(0x100); }                                                 // declare climb-rate as absent

   uint16_t getTrack(void) const                                                                // 9-bit cordic
   { int16_t Word=Position[10]; Word<<=1; Word|=Position[9]>>7; return Word; }
   void setTrack(int16_t Word)
   { Position[9] = (Position[9]&0x7F) | ((Word&0x01)<<7);
     Position[10] = Word>>1; }

// --------------------------------------------------------------------------------------------------------

   // calculate distance vector [LatDist, LonDist] from a given reference [RefLat, Reflon]
   int calcDistanceVectorOGN(int32_t &LatDist, int32_t &LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000, int32_t MaxDist=0x7FFF)
   { LatDist = ((getLatOGN()-RefLat)*1517+0x1000)>>13;           // convert from 1/600000deg to meters (40000000m = 360deg) => x 5/27 = 1517/(1<<13)
     if(abs(LatDist)>MaxDist) return -1;
     LonDist = ((getLonOGN()-RefLon)*1517+0x1000)>>13;
     if(abs(LonDist)>(4*MaxDist)) return -1;
             LonDist = (LonDist*LatCos+0x800)>>12;
     if(abs(LonDist)>MaxDist) return -1;
     return 1; }

   // sets position [Lat, Lon] according to given distance vector [LatDist, LonDist] from a reference point [RefLat, RefLon]
   void setDistanceVectorOGN(int32_t LatDist, int32_t LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000)
   { setLatOGN(RefLat+(LatDist*27)/5);
     LonDist = (LonDist<<12)/LatCos;                                  // LonDist/=cosine(Latitude)
     setLonOGN(RefLon+(LonDist*27)/5); }

// --------------------------------------------------------------------------------------------------------

   int WriteSafeSky(char *Msg, const char *Call, uint32_t RxTime) const  // SafeSky message
   { int Len=0;
     uint32_t Address=getAddress();                               // address
     Msg[Len++]='\"';
     Len+=Format_Hex(Msg+Len, (uint8_t) (Address>>16));
     Len+=Format_Hex(Msg+Len, (uint16_t)(Address));
     Msg[Len++]='\"';
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, (uint32_t)getAddrTable());       // address-type
     Msg[Len++]=',';
      int16_t msTime=0;
     uint32_t Time=getTime(msTime, RxTime);                       // timestamp
     Len+=Format_UnsDec(Msg+Len, Time);
     // Msg[Len++]='.';
     // Len+=Format_UnsDec(Msg+Len, (uint32_t)msTime, 3);
     Msg[Len++]=',';
     Msg[Len++]='0'+FlightState;                                  // flight-state
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, (uint32_t)AcftCat);              // aircraft-category
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, (uint32_t)Emergency);            // emergency status
     Msg[Len++]=',';
     Len+=Format_SignDec(Msg+Len, getLatUBX(), 8, 7, 1);          // latitude
     Msg[Len++]=',';
     Len+=Format_SignDec(Msg+Len, getLonUBX(), 8, 7, 1);          // longitude
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, ((uint32_t)getSpeed()*10+2)>>2, 2, 1); // [m/s] ground speed
     Msg[Len++]=',';
     Len+=Format_SignDec(Msg+Len, getAlt(), 1, 0, 1);            // [m] HAE <= not MSL !
     Msg[Len++]=',';
     if(hasClimb()) Len+=Format_SignDec(Msg+Len, ((int32_t)getClimb()*10+4)>>3, 2, 1, 1); // [m/s] vertical rate
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, ((uint32_t)225*getTrack()+16)>>5, 2, 1); // [deg] ground track
     Msg[Len++]=',';
     Msg[Len++]='0'+SourceIntegrity;                             // Source Integrity
     Msg[Len++]=',';
     Msg[Len++]='0'+DesignAssurance;                             // Design Assurance
     Msg[Len++]=',';
     Len+=Format_UnsDec(Msg+Len, (uint32_t)NavigIntegrity);      // Navigation Integrity
     Msg[Len++]=',';
     Msg[Len++]='0'+HorizAccuracy;                               // Horizontal postion accuracy
     Msg[Len++]=',';
     Msg[Len++]='0'+VertAccuracy;                                // Vertical Position Accuracy
     Msg[Len++]=',';
     Msg[Len++]='0'+VelAccuracy;                                 // Velocity Accuracy
     // Msg[Len++]=',';
     // if(Call)                                                    // call-sign
     // { Msg[Len++]='\"';
     //   Len+=Format_String(Msg+Len, Call);
     //   Msg[Len++]='\"'; }
     Msg[Len]=0; return Len; }

   int WriteStxJSON(char *JSON) const                              // Stratux JSON message
   { int Len=0;
     uint32_t Address=getAddress();
     Len+=Format_String(JSON+Len, "\"addr\":\"");
     Len+=Format_Hex(JSON+Len, (uint8_t) (Address>>16));
     Len+=Format_Hex(JSON+Len, (uint16_t)(Address));
     JSON[Len++]='\"';
     JSON[Len++]=',';
     Len+=Format_String(JSON+Len, "\"addr_type\":");
     JSON[Len++] = HexDigit(getAddrTypeOGN());
     Len+=Format_String(JSON+Len, ",\"acft_type\":\"");
     JSON[Len++] = HexDigit(getAcftTypeOGN());
     // Len+=Format_String(JSON+Len, ",\"acft_cat\":\"");
     // Len+=Format_Hex(JSON+Len, AcftCat);
     JSON[Len++]='\"';

     Len+=Format_String(JSON+Len, ",\"lat_deg\":");
     Len+=Format_SignDec(JSON+Len, getLatUBX(), 8, 7, 1);
     Len+=Format_String(JSON+Len, ",\"lon_deg\":");
     Len+=Format_SignDec(JSON+Len, getLonUBX(), 8, 7, 1);

     Len+=Format_String(JSON+Len, ",\"track_deg\":");
     Len+=Format_UnsDec(JSON+Len, ((uint32_t)225*getTrack()+16)>>5, 2, 1);
     Len+=Format_String(JSON+Len, ",\"speed_mps\":");
     Len+=Format_UnsDec(JSON+Len, ((uint32_t)getSpeed()*10+2)>>2, 2, 1);
     Len+=Format_String(JSON+Len, ",\"climb_mps\":");
     Len+=Format_SignDec(JSON+Len, ((int32_t)getClimb()*10+4)>>3, 2, 1, 1);

     Len+=Format_String(JSON+Len, ",\"alt_hae_m\":");
     Len+=Format_SignDec(JSON+Len, getAlt(), 1, 0, 1);

     if(getRelay()) Len+=Format_String(JSON+Len, ",\"relay\":1");

     Len+=Format_String(JSON+Len, ",\"NACp\":");
     JSON[Len++]='0'+HorizAccuracy;
     Len+=Format_String(JSON+Len, ",\"NACv\":");
     JSON[Len++]='0'+VelAccuracy;
     // Len+=Format_String(JSON+Len, ",\"Emergency\":");
     // JSON[Len++]='0'+Emergency;
     // if(FlightState>0 && FlightState<3)
     // { Len+=Format_String(JSON+Len, ",\"airborne\":");
     //   Len+=Format_String(JSON+Len, FlightState==2?"true":"false"; }

     return Len; }

// --------------------------------------------------------------------------------------------------------

   void Scramble(void)
   { XXTEA_Encrypt_Key0(Word, 5, 6); }

   void Descramble(void)
   { XXTEA_Decrypt_Key0(Word, 5, 6); }

// --------------------------------------------------------------------------------------------------------

   static uint32_t PolyPass(uint32_t crc, uint8_t Byte)     // pass a single byte through the CRC polynomial
   { const uint32_t Poly = 0xFFFA0480;
     crc |= Byte;
     for(uint8_t Bit=0; Bit<8; Bit++)
     { if(crc&0x80000000) crc ^= Poly;
       crc<<=1; }
     return crc; }

   static uint32_t checkPI(const uint8_t *Byte, uint8_t Bytes) // run over data bytes and the three CRC bytes
   { uint32_t crc = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { crc = PolyPass(crc, Byte[Idx]); }
     return crc>>8; }                                          // should be all zero for a correct packet

   static uint32_t calcPI(const uint8_t *Byte, uint8_t Bytes)  // calculate PI for the given packet data excluding the three CRC bytes
   { uint32_t crc = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { crc = PolyPass(crc, Byte[Idx]); }
     crc=PolyPass(crc, 0); crc=PolyPass(crc, 0); crc=PolyPass(crc, 0);
     return crc>>8; }                                          //

    void setCRC(void)
    { uint32_t Word = calcPI((const uint8_t *)&Version, TxBytes-6);
      CRC24[0]=Word>>16; CRC24[1]=Word>>8; CRC24[2]=Word; }

    uint32_t checkCRC(void) const
    { return checkPI((const uint8_t *)&Version, TxBytes-3); }

    static int Correct(uint8_t *PktData, uint8_t *PktErr, const int MaxBadBits=6) // correct the manchester-decoded packet with dead/weak bits marked
    { const int Bytes=TxBytes-3;
      uint32_t crc = checkPI(PktData, Bytes); if(crc==0) return 0;
      uint8_t ErrBit=FindCRCsyndrome(crc);
      if(ErrBit!=0xFF) { FlipBit(PktData, ErrBit); return 1; }

      uint8_t BadBitIdx[MaxBadBits];                                    // bad bit index
      uint8_t BadBitMask[MaxBadBits];                                   // bad bit mask
      uint32_t Syndrome[MaxBadBits];                                    // bad bit mask
      uint8_t BadBits=0;                                                // count the bad bits
      for(uint8_t ByteIdx=0; ByteIdx<Bytes; ByteIdx++)                  // loop over bytes
      { uint8_t Byte=PktErr[ByteIdx];
        uint8_t Mask=0x80;
        for(uint8_t BitIdx=0; BitIdx<8; BitIdx++)                       // loop over bits
        { if(Byte&Mask)
          { if(BadBits<MaxBadBits)
            { BadBitIdx[BadBits]=ByteIdx;                               // store the bad bit index
              BadBitMask[BadBits]=Mask;
              Syndrome[BadBits]=CRCsyndrome(ByteIdx*8+BitIdx); }
            BadBits++;
          }
          Mask>>=1;
        }
        if(BadBits>MaxBadBits) break;
      }
      if(BadBits>MaxBadBits) return -1;                                 // return failure when too many bad bits

      uint8_t Loops = 1<<BadBits; uint8_t PrevGrayIdx=0;
      for(uint8_t Idx=1; Idx<Loops; Idx++)                              // loop through all combination of bad bit flips
      { uint8_t GrayIdx= Idx ^ (Idx>>1);                                // use Gray code to change flip just one bit at a time
        uint8_t BitExp = GrayIdx^PrevGrayIdx;
        uint8_t Bit=0; while(BitExp>>=1) Bit++;
        PktData[BadBitIdx[Bit]]^=BadBitMask[Bit];
        crc^=Syndrome[Bit]; if(crc==0) return Count1s(GrayIdx);
        uint8_t ErrBit=FindCRCsyndrome(crc);
        if(ErrBit!=0xFF)
        { FlipBit(PktData, ErrBit);
          return Count1s(GrayIdx)+1; }
        PrevGrayIdx=GrayIdx; }

      return -1; }

    static void FlipBit(uint8_t *Byte, int BitIdx)
    { int ByteIdx=BitIdx>>3;
      BitIdx&=7; BitIdx=7-BitIdx;
      uint8_t Mask=1; Mask<<=BitIdx;
      Byte[ByteIdx]^=Mask; }

    static uint32_t CRCsyndrome(uint8_t Bit)
    { const uint16_t PacketBytes = TxBytes-3;
      const uint16_t PacketBits = PacketBytes*8;
      const uint32_t Syndrome[PacketBits] = {
 0x7ABEE1, 0xC2A574, 0x6152BA, 0x30A95D, 0xE7AEAA, 0x73D755, 0xC611AE, 0x6308D7,
 0xCE7E6F, 0x98C533, 0xB3989D, 0xA6364A, 0x531B25, 0xD67796, 0x6B3BCB, 0xCA67E1,
 0x9AC9F4, 0x4D64FA, 0x26B27D, 0xECA33A, 0x76519D, 0xC4D2CA, 0x626965, 0xCECEB6,
 0x67675B, 0xCC49A9, 0x99DED0, 0x4CEF68, 0x2677B4, 0x133BDA, 0x099DED, 0xFB34F2,
 0x7D9A79, 0xC13738, 0x609B9C, 0x304DCE, 0x1826E7, 0xF3E977, 0x860EBF, 0xBCFD5B,
 0xA184A9, 0xAF3850, 0x579C28, 0x2BCE14, 0x15E70A, 0x0AF385, 0xFA83C6, 0x7D41E3,
 0xC15AF5, 0x9F577E, 0x4FABBF, 0xD82FDB, 0x93EDE9, 0xB60CF0, 0x5B0678, 0x2D833C,
 0x16C19E, 0x0B60CF, 0xFA4A63, 0x82DF35, 0xBE959E, 0x5F4ACF, 0xD05F63, 0x97D5B5,
 0xB410DE, 0x5A086F, 0xD2FE33, 0x96851D, 0xB4B88A, 0x5A5C45, 0xD2D426, 0x696A13,
 0xCB4F0D, 0x9A5D82, 0x4D2EC1, 0xD96D64, 0x6CB6B2, 0x365B59, 0xE4D7A8, 0x726BD4,
 0x3935EA, 0x1C9AF5, 0xF1B77E, 0x78DBBF, 0xC397DB, 0x9E31E9, 0xB0E2F0, 0x587178,
 0x2C38BC, 0x161C5E, 0x0B0E2F, 0xFA7D13, 0x82C48D, 0xBE9842, 0x5F4C21, 0xD05C14,
 0x682E0A, 0x341705, 0xE5F186, 0x72F8C3, 0xC68665, 0x9CB936, 0x4E5C9B, 0xD8D449,
 0x939020, 0x49C810, 0x24E408, 0x127204, 0x093902, 0x049C81, 0xFDB444, 0x7EDA22,
 0x3F6D11, 0xE04C8C, 0x702646, 0x381323, 0xE3F395, 0x8E03CE, 0x4701E7, 0xDC7AF7,
 0x91C77F, 0xB719BB, 0xA476D9, 0xADC168, 0x56E0B4, 0x2B705A, 0x15B82D, 0xF52612,
 0x7A9309, 0xC2B380, 0x6159C0, 0x30ACE0, 0x185670, 0x0C2B38, 0x06159C, 0x030ACE,
 0x018567, 0xFF38B7, 0x80665F, 0xBFC92B, 0xA01E91, 0xAFF54C, 0x57FAA6, 0x2BFD53,
 0xEA04AD, 0x8AF852, 0x457C29, 0xDD4410, 0x6EA208, 0x375104, 0x1BA882, 0x0DD441,
 0xF91024, 0x7C8812, 0x3E4409, 0xE0D800, 0x706C00, 0x383600, 0x1C1B00, 0x0E0D80,
 0x0706C0, 0x038360, 0x01C1B0, 0x00E0D8, 0x00706C, 0x003836, 0x001C1B, 0xFFF409,
 0x800000, 0x400000, 0x200000, 0x100000, 0x080000, 0x040000, 0x020000, 0x010000,
 0x008000, 0x004000, 0x002000, 0x001000, 0x000800, 0x000400, 0x000200, 0x000100,
 0x000080, 0x000040, 0x000020, 0x000010, 0x000008, 0x000004, 0x000002, 0x000001 } ;
      return Syndrome[Bit]; }

    static uint8_t FindCRCsyndrome(uint32_t Syndr)              // quick search for a single-bit CRC syndrome
    { const uint16_t PacketBytes = TxBytes-3;
      const uint16_t PacketBits = PacketBytes*8;
      const uint32_t Syndrome[PacketBits] = {
 0x000001BF, 0x000002BE, 0x000004BD, 0x000008BC, 0x000010BB, 0x000020BA, 0x000040B9, 0x000080B8,
 0x000100B7, 0x000200B6, 0x000400B5, 0x000800B4, 0x001000B3, 0x001C1BA6, 0x002000B2, 0x003836A5,
 0x004000B1, 0x00706CA4, 0x008000B0, 0x00E0D8A3, 0x010000AF, 0x01856788, 0x01C1B0A2, 0x020000AE,
 0x030ACE87, 0x038360A1, 0x040000AD, 0x049C816D, 0x06159C86, 0x0706C0A0, 0x080000AC, 0x0939026C,
 0x099DED1E, 0x0AF3852D, 0x0B0E2F5A, 0x0B60CF39, 0x0C2B3885, 0x0DD44197, 0x0E0D809F, 0x100000AB,
 0x1272046B, 0x133BDA1D, 0x15B82D7E, 0x15E70A2C, 0x161C5E59, 0x16C19E38, 0x1826E724, 0x18567084,
 0x1BA88296, 0x1C1B009E, 0x1C9AF551, 0x200000AA, 0x24E4086A, 0x2677B41C, 0x26B27D12, 0x2B705A7D,
 0x2BCE142B, 0x2BFD538F, 0x2C38BC58, 0x2D833C37, 0x304DCE23, 0x30A95D03, 0x30ACE083, 0x34170561,
 0x365B594D, 0x37510495, 0x38132373, 0x3836009D, 0x3935EA50, 0x3E44099A, 0x3F6D1170, 0x400000A9,
 0x457C2992, 0x4701E776, 0x49C81069, 0x4CEF681B, 0x4D2EC14A, 0x4D64FA11, 0x4E5C9B66, 0x4FABBF32,
 0x531B250C, 0x56E0B47C, 0x579C282A, 0x57FAA68E, 0x58717857, 0x5A086F41, 0x5A5C4545, 0x5B067836,
 0x5F4ACF3D, 0x5F4C215E, 0x609B9C22, 0x6152BA02, 0x6159C082, 0x62696516, 0x6308D707, 0x67675B18,
 0x682E0A60, 0x696A1347, 0x6B3BCB0E, 0x6CB6B24C, 0x6EA20894, 0x70264672, 0x706C009C, 0x726BD44F,
 0x72F8C363, 0x73D75505, 0x76519D14, 0x78DBBF53, 0x7A930980, 0x7ABEE100, 0x7C881299, 0x7D41E32F,
 0x7D9A7920, 0x7EDA226F, 0x800000A8, 0x80665F8A, 0x82C48D5C, 0x82DF353B, 0x860EBF26, 0x8AF85291,
 0x8E03CE75, 0x91C77F78, 0x93902068, 0x93EDE934, 0x96851D43, 0x97D5B53F, 0x98C53309, 0x99DED01A,
 0x9A5D8249, 0x9AC9F410, 0x9CB93665, 0x9E31E955, 0x9F577E31, 0xA01E918C, 0xA184A928, 0xA476D97A,
 0xA6364A0B, 0xADC1687B, 0xAF385029, 0xAFF54C8D, 0xB0E2F056, 0xB3989D0A, 0xB410DE40, 0xB4B88A44,
 0xB60CF035, 0xB719BB79, 0xBCFD5B27, 0xBE959E3C, 0xBE98425D, 0xBFC92B8B, 0xC1373821, 0xC15AF530,
 0xC2A57401, 0xC2B38081, 0xC397DB54, 0xC4D2CA15, 0xC611AE06, 0xC6866564, 0xCA67E10F, 0xCB4F0D48,
 0xCC49A919, 0xCE7E6F08, 0xCECEB617, 0xD05C145F, 0xD05F633E, 0xD2D42646, 0xD2FE3342, 0xD677960D,
 0xD82FDB33, 0xD8D44967, 0xD96D644B, 0xDC7AF777, 0xDD441093, 0xE04C8C71, 0xE0D8009B, 0xE3F39574,
 0xE4D7A84E, 0xE5F18662, 0xE7AEAA04, 0xEA04AD90, 0xECA33A13, 0xF1B77E52, 0xF3E97725, 0xF526127F,
 0xF9102498, 0xFA4A633A, 0xFA7D135B, 0xFA83C62E, 0xFB34F21F, 0xFDB4446E, 0xFF38B789, 0xFFF409A7 } ;

      uint16_t Bot=0;
      uint16_t Top=PacketBits;
      uint32_t MidSyndr=0;
      for( ; ; )
      { uint16_t Mid=(Bot+Top)>>1;
        MidSyndr = Syndrome[Mid]>>8;
        if(Syndr==MidSyndr) return (uint8_t)Syndrome[Mid];
        if(Mid==Bot) break;
        if(Syndr< MidSyndr) Top=Mid;
                       else Bot=Mid; }
      return 0xFF; }

} __attribute__((packed));

class ADSL_RxPacket: public ADSL_Packet
{ public:
   uint32_t  sTime;         // [ s] reception time
   uint16_t msTime;         // [ms]
    int8_t RSSI;            // [dBm]
   uint8_t BitErr;          // number of bit errors

  public:
   void setTime(double RxTime) { sTime=floor(RxTime); msTime=floor(1000.0*(RxTime-sTime)); }
   double getTime(void) const { return (double)sTime+0.001*msTime; }
   uint32_t SlotTime(void) const { uint32_t Slot=sTime; if(msTime<=300) Slot--; return Slot; }

} __attribute__((packed));

#endif // __ADSL_H__

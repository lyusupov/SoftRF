#ifndef __NMEA_H__
#define __NMEA_H__

#include <stdint.h>

uint8_t NMEA_Check(uint8_t *NMEA, uint8_t Len);
uint8_t NMEA_AppendCheck(uint8_t *NMEA, uint8_t Len);
inline uint8_t NMEA_AppendCheck(char *NMEA, uint8_t Len) { return NMEA_AppendCheck((uint8_t*)NMEA, Len); }
uint8_t NMEA_AppendCheckCRNL(uint8_t *NMEA, uint8_t Len);
inline uint8_t NMEA_AppendCheckCRNL(char *NMEA, uint8_t Len) { return NMEA_AppendCheckCRNL((uint8_t*)NMEA, Len); }

 class NMEA_RxMsg             // receiver for the NMEA sentences
{ public:
   static const uint8_t MaxLen=96;   // maximum length
   static const uint8_t MaxParms=24; // maximum number of parameters (commas)
   uint8_t Data[MaxLen];             // the message itself
   uint8_t Len;                      // number of bytes
   uint8_t Parms;                    // number of commas
   uint8_t Parm[MaxParms];           // offset to each comma
   uint8_t State;                    // bits: 0:loading, 1:complete, 2:locked,
   uint8_t Check;                    // check sum: should be a XOR of all bytes between '$' and '*'

  public:
   void Clear(void)                          // Clear the frame: discard all data, ready for next message
     { State=0; Len=0; Parms=0; }

   void Send(void (*SendByte)(char) ) const
   { for(uint8_t Idx=0; Idx<Len; Idx++)
     { (*SendByte)(Data[Idx]); }
     (*SendByte)('\r'); (*SendByte)('\n'); }

   void ProcessByte(uint8_t Byte)            // pass all bytes through this call and it will build the frame
     {
       if(isComplete()) return;              // if already a complete frame, ignore extra bytes
       if(Len==0)                            // if data is empty
       { if(Byte!='$') return;               // then ignore all bytes but '$'
         Data[Len++]=Byte;                   // start storing the frame
         setLoading(); Check=0x00; Parms=0;  // set state to "isLoading", clear checksum
       } else                                // if not empty (being loaded)
       { if((Byte=='\r')||(Byte=='\n'))      // if CR (or NL ?) then frame is complete
         { setComplete(); if(Len<MaxLen) Data[Len]=0;
           return; }
         else if(Byte<=' ')                  // other control bytes treat as errors
         { Clear(); return; }                // and drop the frame
         else if(Byte==',')                  // save comma positions to later get back to the fields
         { if(Parms<MaxParms) Parm[Parms++]=Len+1; }
         if(Len<MaxLen) { Data[Len++]=Byte; Check^=Byte; } // store data but if too much then treat as an error
                   else Clear();             // if too long, then drop the frame completely
       }
       return; }

   uint8_t isLoading(void) const  { return State &0x01; }
   void   setLoading(void)        {        State|=0x01; }

   uint8_t isComplete(void) const { return State &0x02; }
   void   setComplete(void)       {        State|=0x02; }

   uint8_t isLocked(void) const   { return State&0x04; }

   uint8_t isEmpty(void) const    { return Len==0; }

   uint8_t hasCheck(void) const
     { if(Len<4) return 0;
       return Data[Len-3]=='*'; }

   uint8_t isChecked(void) const    // is the NMEA checksum OK ?
     { if(!hasCheck()) return 0;
       uint8_t DataCheck = Check^Data[Len-3]^Data[Len-2]^Data[Len-1];
       int8_t HighDigit=HexValue(Data[Len-2]); if(HighDigit<0) return 0;
       int8_t LowDigit=HexValue(Data[Len-1]); if(LowDigit<0) return 0;
       uint8_t FrameCheck = (HighDigit<<4) | LowDigit;
       return DataCheck==FrameCheck; }
/*
   static int8_t DecValue(uint8_t Char)
     { if(Char<'0') return -1;
       if(Char<='9') return Char-'0';
       return -1; }
*/
   static int8_t HexValue(uint8_t Char)
     { if(Char<'0') return -1;
       if(Char<='9') return Char-'0';
       if(Char<'A') return -1;
       if(Char<='F') return Char-('A'-10);
       // if(Char<'a') return -1;                         // check-sum uses uppercase latters
       // if(Char<='f') return Char-('a'-10);
       return -1; }

   const uint8_t *ParmPtr(uint8_t Field) const            // get a pointer to given (comma separated) field
     { if(Field>=Parms) return 0;
       return Data+Parm[Field]; }
/*
   uint8_t ParmLen(uint8_t Field) const
     { if(Field>=Parms) return 0;
       if(Field==(Parms-1)) return Len-4-Comma[Field];
       return Parm[Field+1]-Parm[Field]-1; }
*/

   uint8_t isGP(void) const                    // GPS sentence ?
   //   { return isGP((const char *)Data); }
   // uint8_t static isGP(const char *Data)
     { if(Data[1]!='G') return 0;
       return Data[2]=='P'; }

   uint8_t isGN(void) const
   //  { return isGN((const char *)Data); }
   // uint8_t static isGN(const char *Data)
     { if(Data[1]!='G') return 0;
       return Data[2]=='N'; }

   uint8_t isGx(void) const                    // GPS or GLONASS sentence ?
     { return Data[1]=='G'; }

   uint8_t isGPRMC(void) const                  // GPS recomended minimum data
     { if(!isGP()) return 0;
       if(Data[3]!='R') return 0;
       if(Data[4]!='M') return 0;
       return Data[5]=='C'; }

   uint8_t isGNRMC(void) const                  // GPS recomended minimum data
     { if(!isGN()) return 0;
       if(Data[3]!='R') return 0;
       if(Data[4]!='M') return 0;
       return Data[5]=='C'; }

   uint8_t isGxRMC(void) const                  // GPS recomended minimum data
     { if(!isGx()) return 0;
       if(Data[3]!='R') return 0;
       if(Data[4]!='M') return 0;
       return Data[5]=='C'; }

   uint8_t isGPGGA(void) const                  // GPS 3-D fix data
     { if(!isGP()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='G') return 0;
       return Data[5]=='A'; }

   uint8_t isGNGGA(void) const                  // GPS 3-D fix data
     { if(!isGN()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='G') return 0;
       return Data[5]=='A'; }

   uint8_t isGxGGA(void) const                  // 
     { if(!isGx()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='G') return 0;
       return Data[5]=='A'; }

   uint8_t isGPGSA(void) const                   // GPS satellite data
     { if(!isGP()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='S') return 0;
       return Data[5]=='A'; }

   uint8_t isGNGSA(void) const                   // 
     { if(!isGN()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='S') return 0;
       return Data[5]=='A'; }

   uint8_t isGxGSA(void) const                   // 
     { if(!isGx()) return 0;
       if(Data[3]!='G') return 0;
       if(Data[4]!='S') return 0;
       return Data[5]=='A'; }

   uint8_t isGPTXT(void) const                   // GPS satellite data
     { if(!isGP()) return 0;
       if(Data[3]!='T') return 0;
       if(Data[4]!='X') return 0;
       return Data[5]=='T'; }

   uint8_t isP(void) const
   { return Data[1]=='P'; }

   uint8_t isPOGN(void) const                    // OGN dedicated NMEA sentence
     { if(Data[1]!='P') return 0;
       if(Data[2]!='O') return 0;
       if(Data[3]!='G') return 0;
       return Data[4]=='N'; }

   uint8_t isPOGNB(void)                         // barometric report from the OGN tracker
     { if(!isPOGN()) return 0;
       return Data[5]=='B'; }

   uint8_t isPOGNT(void)                         // other aircraft position (tracking) report from OGN trackers
     { if(!isPOGN()) return 0;
       return Data[5]=='T'; }

   uint8_t isPOGNS(void)                         // tracker parameters setup
     { if(!isPOGN()) return 0;
       return Data[5]=='S'; }

} ;

#endif // of __NMEA_H__

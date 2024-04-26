#include "format.h"

// ------------------------------------------------------------------------------------------

char HexDigit(uint8_t Val) { return Val+(Val<10?'0':'A'-10); }

// ------------------------------------------------------------------------------------------

void Format_Bytes( void (*Output)(char), const uint8_t *Bytes, uint8_t Len)
{ for( ; Len; Len--)
    (*Output)(*Bytes++);
}

void Format_String( void (*Output)(char), const char *String)
{ if(String==0) return;
  for( ; ; )
  { uint8_t ch = (*String++); if(ch==0) break;
#ifdef WITH_AUTOCR
    if(ch=='\n') (*Output)('\r');
#endif
    (*Output)(ch); }
}

uint8_t Format_String(char *Out, const char *String)
{ if(String==0) return 0;
  uint8_t OutLen=0;
  for( ; ; )
  { char ch = (*String++); if(ch==0) break;
#ifdef WITH_AUTOCR
    if(ch=='\n') Out[OutLen++]='\r';
#endif
    Out[OutLen++]=ch; }
  // Out[OutLen]=0;
  return OutLen; }

void Format_String( void (*Output)(char), const char *String, uint8_t MinLen, uint8_t MaxLen)
{ if(String==0) return;
  if(MaxLen<MinLen) MaxLen=MinLen;
  uint8_t Idx;
  for(Idx=0; Idx<MaxLen; Idx++)
  { char ch = String[Idx]; if(ch==0) break;
#ifdef WITH_AUTOCR
    if(ch=='\n') (*Output)('\r');
#endif
    (*Output)(ch); }
  for(    ; Idx<MinLen; Idx++)
    (*Output)(' ');
}

uint8_t Format_String(char *Out, const char *String, uint8_t MinLen, uint8_t MaxLen)
{ if(String==0) return 0;
  if(MaxLen<MinLen) MaxLen=MinLen;
  uint8_t OutLen=0;
  uint8_t Idx;
  for(Idx=0; Idx<MaxLen; Idx++)
  { char ch = String[Idx]; if(ch==0) break;
#ifdef WITH_AUTOCR
    if(ch=='\n') Out[OutLen++]='\r';
#endif
    Out[OutLen++]=ch; }
  for(    ; Idx<MinLen; Idx++)
    Out[OutLen++]=' ';
  // Out[OutLen++]=0;
  return OutLen; }

void Format_Hex( void (*Output)(char), uint8_t Byte )
{ (*Output)(HexDigit(Byte>>4)); (*Output)(HexDigit(Byte&0x0F)); }

void Format_HexBytes( void (*Output)(char), const uint8_t *Byte, uint8_t Bytes)
{ for(uint8_t Idx=0; Idx<Bytes; Idx++) Format_Hex(Output, Byte[Idx]); }

void Format_Hex( void (*Output)(char), uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output, (uint8_t)Word); }

void Format_Hex( void (*Output)(char), uint32_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>24)); Format_Hex(Output, (uint8_t)(Word>>16));
  Format_Hex(Output, (uint8_t)(Word>>8));  Format_Hex(Output, (uint8_t)Word); }

void Format_Hex( void (*Output)(char), uint64_t Word )
{ Format_Hex(Output, (uint32_t)(Word>>32));
  Format_Hex(Output, (uint32_t)(Word    )); }

void Format_MAC( void (*Output)(char), uint8_t *MAC, uint8_t Len)
{ for(uint8_t Idx=0; Idx<Len; Idx++)
  { if(Idx) (*Output)(':');
    Format_Hex(Output, MAC[Idx]); }
}

uint8_t Format_HHcMMcSS(char *Out, uint32_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  uint32_t HHMMSS = 1000000*Hour + 1000*Min + Sec;
  uint8_t Len=Format_UnsDec(Out, HHMMSS, 8);
  Out[2]=':'; Out[5]=':';
  return Len; }

uint8_t Format_HHMMSS(char *Out, uint32_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  uint32_t HHMMSS = 10000*Hour + 100*Min + Sec;
  return Format_UnsDec(Out, HHMMSS, 6); }

void Format_HHMMSS(void (*Output)(char), uint32_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  uint32_t HHMMSS = 10000*Hour + 100*Min + Sec;
  Format_UnsDec(Output, HHMMSS, 6); }

void Format_Period(void (*Output)(char), int32_t Time)
{ if(Time<0) { (*Output)('-'); Time=(-Time); }
        else { (*Output)(' '); }
  if(Time<60) { (*Output)(' '); Format_UnsDec(Output, (uint32_t)Time, 2); (*Output)('s'); return; }
  if(Time<3600) { Format_UnsDec(Output, (uint32_t)Time/60, 2); (*Output)('m'); Format_UnsDec(Output, (uint32_t)Time%60, 2); return; }
  if(Time<86400) { Format_UnsDec(Output, (uint32_t)Time/3600, 2); (*Output)('h'); Format_UnsDec(Output, ((uint32_t)Time%3600)/60, 2); return; }
  Format_UnsDec(Output, (uint32_t)Time/86400, 2); (*Output)('d'); Format_UnsDec(Output, ((uint32_t)Time%86400)/3600, 2); }

uint8_t Format_Period(char *Out, int32_t Time)
{ uint8_t Len=0;
  if(Time<0) { Out[Len++]='-'; Time=(-Time); }
        else { Out[Len++]=' '; }
  if(Time<60) { Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, (uint32_t)Time, 2); Out[Len++]='s'; return Len; }
  if(Time<3600) { Len+=Format_UnsDec(Out+Len, (uint32_t)Time/60, 2); Out[Len++]='m'; Len+=Format_UnsDec(Out+Len, (uint32_t)Time%60, 2); return Len; }
  if(Time<86400) { Len+=Format_UnsDec(Out+Len, (uint32_t)Time/3600, 2); Out[Len++]='h'; Len+=Format_UnsDec(Out+Len, ((uint32_t)Time%3600)/60, 2); return Len; }
  Len+=Format_UnsDec(Out+Len, (uint32_t)Time/86400, 2); Out[Len++]='d'; Len+=Format_UnsDec(Out+Len, ((uint32_t)Time%86400)/3600, 2);
  return Len; }

void Format_UnsDec( void (*Output)(char), uint16_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint16_t Base; uint8_t Pos;
  for( Pos=5, Base=10000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int16_t Value, uint8_t MinDigits, uint8_t DecPoint, uint8_t NoPlus)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else if(!NoPlus) { (*Output)('+'); }
  Format_UnsDec(Output, (uint16_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int32_t Value, uint8_t MinDigits, uint8_t DecPoint, uint8_t NoPlus)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else if(!NoPlus) { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint64_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint64_t Base; uint8_t Pos;
  for( Pos=20, Base=10000000000000000000llu; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int64_t Value, uint8_t MinDigits, uint8_t DecPoint, uint8_t NoPlus)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else if(!NoPlus) { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

// ------------------------------------------------------------------------------------------

uint8_t Format_UnsDec(char *Out, uint64_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint64_t Base; uint8_t Pos, Len=0;
  for( Pos=20, Base=10000000000000000000llu; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) { (*Out++)='.'; Len++; }
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Out++)='0'+Dig; Len++; MinDigits=Pos; }
    // (*Out)=0;
  }
  return Len; }

uint8_t Format_SignDec(char *Out, int64_t Value, uint8_t MinDigits, uint8_t DecPoint, uint8_t NoPlus)
{ uint8_t Len=0;
  if(Value<0) { (*Out++)='-'; Len++; Value=(-Value); }
         else if(!NoPlus) { (*Out++)='+'; Len++; }
  return Len+Format_UnsDec(Out, (uint64_t)Value, MinDigits, DecPoint); }

uint8_t Format_UnsDec(char *Out, uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos, Len=0;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) { (*Out++)='.'; Len++; }
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Out++)='0'+Dig; Len++; MinDigits=Pos; }
    // (*Out)=0;
  }
  return Len; }

uint8_t Format_SignDec(char *Out, int32_t Value, uint8_t MinDigits, uint8_t DecPoint, uint8_t NoPlus)
{ uint8_t Len=0;
  if(Value<0) { (*Out++)='-'; Len++; Value=(-Value); }
         else if(!NoPlus) { (*Out++)='+'; Len++; }
  return Len+Format_UnsDec(Out, (uint32_t)Value, MinDigits, DecPoint); }

uint8_t Format_Hex( char *Output, uint8_t Byte )
{ (*Output++) = HexDigit(Byte>>4); (*Output++)=HexDigit(Byte&0x0F); return 2; }

uint8_t Format_HexBytes(char *Output, const uint8_t *Byte, uint8_t Bytes)
{ uint8_t Len=0;
  for(uint8_t Idx=0; Idx<Bytes; Idx++)
    Len+=Format_Hex(Output+Len, Byte[Idx]);
  return Len;  }

uint8_t Format_Hex( char *Output, uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8));
  Format_Hex(Output+2, (uint8_t)Word);
  return 4; }

uint8_t Format_Hex( char *Output, uint32_t Word )
{ Format_Hex(Output  , (uint16_t)(Word>>16));
  Format_Hex(Output+4, (uint16_t)(Word    ));
  return 8; }

uint8_t Format_Hex( char *Output, uint64_t Word )
{ Format_Hex(Output  , (uint32_t)(Word>>32));
  Format_Hex(Output+8, (uint32_t)(Word    ));
  return 16; }

// uint8_t Format_Hex( char *Output, uint32_t Word, uint8_t Digits)
// { for(uint8_t Idx=Digits; Idx>0; )
//   { Output[--Idx]=HexDigit(Word&0x0F);
//     Word>>=4; }
//   return Digits; }

// ------------------------------------------------------------------------------------------

uint8_t Format_Latitude(char *Out, int32_t Lat)
{ uint8_t Len=0;
  char Sign='N';
  if(Lat<0) { Sign='S'; Lat=(-Lat); }
  uint32_t Deg=Lat/600000;
  Lat -= 600000*Deg;
  Len+=Format_UnsDec(Out+Len, Deg, 2, 0);
  Len+=Format_UnsDec(Out+Len, (uint32_t)Lat, 6, 4);
  Out[Len++]=Sign;
  return Len; }

uint8_t Format_Longitude(char *Out, int32_t Lon)
{ uint8_t Len=0;
  char Sign='E';
  if(Lon<0) { Sign='W'; Lon=(-Lon); }
  uint32_t Deg=Lon/600000;
  Lon -= 600000*Deg;
  Len+=Format_UnsDec(Out+Len, Deg, 3, 0);
  Len+=Format_UnsDec(Out+Len, (uint32_t)Lon, 6, 4);
  Out[Len++]=Sign;
  return Len; }

// ------------------------------------------------------------------------------------------

int8_t Read_Hex1(char Digit)
{ int8_t Val=Read_Dec1(Digit); if(Val>=0) return Val; 
  if( (Digit>='A') && (Digit<='F') ) return Digit-'A'+10;
  if( (Digit>='a') && (Digit<='f') ) return Digit-'a'+10;
  return -1; }

int8_t Read_Dec1(char Digit)                   // convert single digit into an integer
{ if(Digit<'0') return -1;                     // return -1 if not a decimal digit
  if(Digit>'9') return -1;
  return Digit-'0'; }

int8_t Read_Dec2(const char *Inp)              // convert two digit decimal number into an integer
{ int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
  int8_t Low =Read_Dec1(Inp[1]); if(Low<0)  return -1;
  return Low+10*High; }

int16_t Read_Dec3(const char *Inp)             // convert three digit decimal number into an integer
{ int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
  int8_t Mid=Read_Dec1(Inp[1]);  if(Mid<0) return -1;
  int8_t Low=Read_Dec1(Inp[2]);  if(Low<0) return -1;
  return (int16_t)Low + (int16_t)10*(int16_t)Mid + (int16_t)100*(int16_t)High; }

int16_t Read_Dec4(const char *Inp)             // convert four digit decimal number into an integer
{ int16_t High=Read_Dec2(Inp  ); if(High<0) return -1;
  int16_t Low =Read_Dec2(Inp+2); if(Low<0) return -1;
  return Low + (int16_t)100*(int16_t)High; }

int32_t Read_Dec5(const char *Inp)             // convert four digit decimal number into an integer
{ int16_t High=Read_Dec2(Inp  ); if(High<0) return -1;
  int16_t Low =Read_Dec3(Inp+2); if(Low<0) return -1;
  return (int32_t)Low + (int32_t)1000*(int32_t)High; }

// ------------------------------------------------------------------------------------------

int8_t Read_Coord(int32_t &Lat, const char *Inp)
{ uint16_t Deg; int8_t Min, Sec;
  Lat=0;
  const char *Start=Inp;
  int8_t Len=Read_UnsDec(Deg, Inp); if(Len<0) return -1;
  Inp+=Len;
  Lat=(uint32_t)Deg*36000;
  if(Inp[0]!=(char)0xC2) return -1;
  if(Inp[1]!=(char)0xB0) return -1;
  Inp+=2;
  Min=Read_Dec2(Inp); if(Min<0) return -1;
  Inp+=2;
  Lat+=(uint32_t)Min*600;
  if(Inp[0]!=(char)'\'') return -1;
  Inp++;
  Sec=Read_Dec2(Inp); if(Sec<0) return -1;
  Inp+=2;
  Lat+=(uint32_t)Sec*10;
  if(Inp[0]=='.')
  { Sec=Read_Dec1(Inp+1); if(Sec<0) return -1;
    Inp+=2; Lat+=Sec; }
  if(Inp[0]==(char)'\"') { Inp++; }
  else if( (Inp[0]==(char)'\'') && (Inp[1]==(char)'\'') ) { Inp+=2; }
  else return -1;
  return Inp-Start; }

int8_t Read_LatDDMMSS(int32_t &Lat, const char *Inp)
{ Lat=0;
  const char *Start=Inp;
  int8_t Sign=0;
       if(Inp[0]=='N') { Sign=  1 ; Inp++; }
  else if(Inp[0]=='S') { Sign=(-1); Inp++; }
  int8_t Len=Read_Coord(Lat, Inp); if(Len<0) return -1;
  Inp+=Len;
  if(Sign==0)
  {      if(Inp[0]=='N') { Sign=  1 ; Inp++; }
    else if(Inp[0]=='S') { Sign=(-1); Inp++; }
  }
  if(Sign==0) return -1;
  if(Sign<0) Lat=(-Lat);
  return Inp-Start; }

int8_t Read_LonDDMMSS(int32_t &Lon, const char *Inp)
{ Lon=0;
  const char *Start=Inp;
  int8_t Sign=0;
       if(Inp[0]=='E') { Sign=  1 ; Inp++; }
  else if(Inp[0]=='W') { Sign=(-1); Inp++; }
  int8_t Len=Read_Coord(Lon, Inp); if(Len<0) return -1;
  Inp+=Len;
  if(Sign==0)
  {      if(Inp[0]=='E') { Sign=  1 ; Inp++; }
    else if(Inp[0]=='W') { Sign=(-1); Inp++; }
  }
  if(Sign==0) return -1;
  if(Sign<0) Lon=(-Lon);
  return Inp-Start; }

int8_t Read_QuotedString(char *String, uint8_t MaxStrLen, const char *Inp)
{ if(Inp[0]!='\"') return 0;
  uint8_t StrLen=0;
  uint8_t InpIdx=1;
  for( ; ; )
  { char C=Inp[InpIdx]; if(C<' ') break;
    InpIdx++; if(C=='\"') break;
    if(StrLen<MaxStrLen) String[StrLen++]=C; }
  String[StrLen]=0; return InpIdx; }            // returns number of characters read form the Input

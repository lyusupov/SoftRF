#include <stdint.h>
#include <string.h>
#include <math.h>

#include "format.h"
#include "ognconv.h"

// ==============================================================================================
// Coordinate scales:
// - uBlox GPS and FLARM:       LSB = 1e-7 deg
// - OGN-Tracker:               LSB = 0.0001/60 deg
// - FANET/ADS-L pseudo-cordic: LSB =
// - True cordic:               2^32 = 360 deg

int32_t Coord_FNTtoOGN(int32_t Coord) { return ((int64_t)Coord*27000219 +(1<<28))>>29; }    // [FANET cordic] => [0.0001/60 deg]
int32_t Coord_OGNtoFNT(int32_t Coord) { return ((int64_t)Coord*83399317 +(1<<21))>>22; }    // [0.0001/60 deg] => [FANET cordic]

int32_t Coord_FNTtoUBX(int32_t Coord) { return ((int64_t)Coord*900007296+(1<<29))>>30; }    // [FANET cordic ] => [1e-7 deg]
int32_t Coord_UBXtoFNT(int32_t Coord) { return ((int64_t)Coord*5003959  +(1<<21))>>22; }    // [1e-7 deg]      => [FANET cordic]

int32_t Coord_CRDtoOGN(int32_t Coord) { return ((int64_t)Coord*421875   +(1<<22))>>23; }    // [32-bit cordic] => [0.0001/60 deg]
int32_t Coord_OGNtoCRD(int32_t Coord) { return ((int64_t)Coord*83399993 +(1<<21))>>22; }    // [0.0001/60 deg] => [32-bit cordic]

int32_t Coord_UBXtoCRD(int32_t Coord) { return ((int64_t)Coord*640511947+(1<<28))>>29; }
int32_t Coord_CRDtoUBX(int32_t Coord) { return ((int64_t)Coord*78125+(1<<24))>>25; }

// ==============================================================================================

int32_t FeetToMeters(int32_t Altitude) { return (Altitude*2497+4096)>>13; } // [feet] => [m]  3 m error at 300'000 ft
int32_t MetersToFeet(int32_t Altitude) { return (Altitude*6719+1024)>>11; } // [m] => [feet]  8 ft error at 100'000 m

// ==============================================================================================

uint8_t AcftType_OGNtoADSB(uint8_t AcftType)
                         // no-inf0, glider, tow, heli, parachute, drop-plane, hang-glider, para-glider, powered, jet, UFO, balloon, Zeppelin, UAV, ground vehicle, fixed object
{ const uint8_t AcftCat[16] = { 0x00, 0xB1, 0xA1, 0xA7, 0xB3,      0xA1,       0xB4,        0xB4,        0xA1,   0xA2, 0x00, 0xB2,    0xB2,    0xB6, 0xC3, 0xC4 };
  return AcftCat[AcftType]; }

uint8_t AcftType_FNTtoADSB(uint8_t AcftType)
                            // no-info, para-glider, hang-glider, balloon, glider, powered, heli, UAV
{ const uint8_t AcftCat[8] = { 0,          0xB4,         0xB4,      0xB2,    0xB1,   0xA1,  0xA7, 0xB6 } ;
  return AcftCat[AcftType]; }

uint8_t AcftType_ADSBtoOGN(uint8_t AcftCat)
{ // if(AcftCat&0x38) return 0;
  uint8_t Upp = AcftCat>>4;
  uint8_t Low = AcftCat&7;
  if(Upp==0xA)
  { if(Low==1) return 8;
    if(Low==7) return 3;
    return 9; }
  if(Upp==0xB)
  { const uint8_t Map[8] = { 0, 0xB, 1, 4, 7, 0, 0xD, 0 };
    return Map[Low]; }
  if(Upp==0xC)
  { if(Low>=4) return 0xF;
    if(Low==3) return 0xE;
    return 0; }
  return 0; }

uint8_t AcftType_OGNtoGDL(uint8_t AcftType)
                         // no-info, glider, tow, heli, parachute, drop-plane, hang-glider, para-glider, powered, jet, UFO, balloon, Zeppelin, UAV, ground vehicle, static-object
{ const uint8_t AcftCat[16] = { 0,      9,   1,    7,        11,          1,          12,          12,       1,   2,   0,      10,       10,   14,  18,     19 } ;
  return AcftCat[AcftType]; }

uint8_t AcftType_OGNtoADSL(uint8_t AcftType)                // OGN to ADS-L aircraft-type
{ const uint8_t Map[16] = { 0, 4, 1, 3,                     // unknown, glider, tow-plane, helicopter
                            8, 1, 7, 7,                     // sky-diver, drop plane, hang-glider, para-glider
                            1, 2, 0, 5,                     // motor airplane, jet, UFO, balloon
                            5,11, 0, 0 } ;                  // airship, UAV, ground vehicle, static object
  return Map[AcftType]; }

uint8_t AcftType_ADSLtoOGN(uint8_t AcftCat)                 // ADS-L to OGN aircraft-type
{ const uint8_t Map[32] = { 0, 8, 9, 3, 1,12, 2, 7,
                            4,13, 3,13,13,13, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0 } ;
  return Map[AcftCat]; }

uint8_t AcftType_FNTtoOGN(uint8_t AcftType)
                              // no-info, para-glider, hang-glider, balloon, glider, powered, heli, UAV
{ const uint8_t OGNtype[8] = {   0,       7,           6,           0xB,     1,      8,       3,    0xD } ;
  return OGNtype[AcftType]; }

uint8_t AcftType_FNTtoADSL(uint8_t AcftType)
                            // no-info, para-glider, hang-glider, balloon, glider, powered, heli, UAV
{ const uint8_t AcftCat[8] = { 0,          12,          12,       10,      9,      1,       7,    14 } ;
  return AcftCat[AcftType]; }

// ==============================================================================================

uint16_t EncodeUR2V8(uint16_t Value)                                 // Encode unsigned 12bit (0..3832) as 10bit
{      if(Value<0x100) { }
  else if(Value<0x300) Value = 0x100 | ((Value-0x100)>>1);
  else if(Value<0x700) Value = 0x200 | ((Value-0x300)>>2);
  else if(Value<0xF00) Value = 0x300 | ((Value-0x700)>>3);
  else                 Value = 0x3FF;
  return Value; }

uint16_t DecodeUR2V8(uint16_t Value)                                 // Decode 10bit 0..0x3FF
{ uint16_t  Range = Value>>8;
  Value &= 0x0FF;
  if(Range==0) return Value;              // 000..0FF
  if(Range==1) return 0x101+(Value<<1);   // 100..2FE
  if(Range==2) return 0x302+(Value<<2);   // 300..6FC
               return 0x704+(Value<<3); } // 700..EF8                       // in 12bit (0..3832)


uint8_t EncodeUR2V5(uint16_t Value)                                  // Encode unsigned 9bit (0..472) as 7bit
{      if(Value<0x020) { }
  else if(Value<0x060) Value = 0x020 | ((Value-0x020)>>1);
  else if(Value<0x0E0) Value = 0x040 | ((Value-0x060)>>2);
  else if(Value<0x1E0) Value = 0x060 | ((Value-0x0E0)>>3);
  else                 Value = 0x07F;
  return Value; }

uint16_t DecodeUR2V5(uint16_t Value)                                 // Decode 7bit as unsigned 9bit (0..472)
{ uint8_t Range = (Value>>5)&0x03;
          Value &= 0x1F;
       if(Range==0) { }                            // 000..01F
  else if(Range==1) { Value = 0x021+(Value<<1); }  // 020..05E
  else if(Range==2) { Value = 0x062+(Value<<2); }  // 060..0DC
  else              { Value = 0x0E4+(Value<<3); }  // 0E0..1D8 => max. Value = 472
  return Value; }

uint8_t EncodeSR2V5(int16_t Value)                                  // Encode signed 10bit (-472..+472) as 8bit
{ uint8_t Sign=0; if(Value<0) { Value=(-Value); Sign=0x80; }
  Value = EncodeUR2V5(Value);
  return Value | Sign; }

int16_t DecodeSR2V5( int16_t Value)                                // Decode
{ int16_t Sign =  Value&0x80;
  Value = DecodeUR2V5(Value&0x7F);
  return Sign ? -Value: Value; }

uint16_t EncodeUR2V6(uint16_t Value)                                // Encode unsigned 10bit (0..952) as 8 bit
{      if(Value<0x040) { }
  else if(Value<0x0C0) Value = 0x040 | ((Value-0x040)>>1);
  else if(Value<0x1C0) Value = 0x080 | ((Value-0x0C0)>>2);
  else if(Value<0x3C0) Value = 0x0C0 | ((Value-0x1C0)>>3);
  else                 Value = 0x0FF;
  return Value; }

uint16_t DecodeUR2V6(uint16_t Value)                                // Decode 8bit as unsigned 10bit (0..952)
{ uint16_t Range  = (Value>>6)&0x03;
           Value &= 0x3F;
       if(Range==0) { }                            // 000..03F
  else if(Range==1) { Value = 0x041+(Value<<1); }  // 040..0BE
  else if(Range==2) { Value = 0x0C2+(Value<<2); }  // 0C0..1BC
  else              { Value = 0x1C4+(Value<<3); }  // 1C0..3B8 => max. Value = 952
  return Value; }

uint16_t EncodeSR2V6(int16_t Value)                                 // Encode signed 11bit (-952..+952) as 9bit
{ uint16_t Sign=0; if(Value<0) { Value=(-Value); Sign=0x100; }
  Value = EncodeUR2V6(Value);
  return Value | Sign; }

 int16_t DecodeSR2V6( int16_t Value)                                // Decode 9bit as signed 11bit (-952..+952)
{ int16_t Sign =  Value&0x100;
  Value = DecodeUR2V6(Value&0x00FF);
  return Sign ? -Value: Value; }

uint8_t EncodeUR2V4(uint8_t DOP)
{      if(DOP<0x10) { }
  else if(DOP<0x30) DOP = 0x10 | ((DOP-0x10)>>1);
  else if(DOP<0x70) DOP = 0x20 | ((DOP-0x30)>>2);
  else if(DOP<0xF0) DOP = 0x30 | ((DOP-0x70)>>3);
  else              DOP = 0x3F;
  return DOP; }

uint8_t DecodeUR2V4(uint8_t DOP)
{ uint8_t Range = DOP>>4;
  DOP &= 0x0F;
  if(Range==0) return       DOP;              // 00..0F
  if(Range==1) return 0x11+(DOP<<1);          // 10..2E
  if(Range==2) return 0x32+(DOP<<2);          // 30..6C
               return 0x74+(DOP<<3); }        // 70..E8 => max. DOP = 232*0.1=23.2

uint16_t EncodeUR2V12(uint16_t Value)                        // encode unsigned 16-bit (0..61432) as 14-bit
{      if(Value<0x1000) { }
  else if(Value<0x3000) Value = 0x1000 | ((Value-0x1000)>>1);
  else if(Value<0x7000) Value = 0x2000 | ((Value-0x3000)>>2);
  else if(Value<0xF000) Value = 0x3000 | ((Value-0x7000)>>3);
  else                  Value = 0x3FFF;
  return Value; }

uint16_t DecodeUR2V12(uint16_t Value)
{ uint16_t Range = Value>>12;
           Value &=0x0FFF;
  if(Range==0) return         Value;       // 0000..0FFF
  if(Range==1) return 0x1001+(Value<<1);   // 1000..2FFE
  if(Range==2) return 0x3002+(Value<<2);   // 3000..6FFC
               return 0x7004+(Value<<3); } // 7000..EFF8 => max: 61432

// ==============================================================================================

uint8_t EncodeGray(uint8_t Binary)
{ return Binary ^ (Binary>>1); }

uint8_t DecodeGray(uint8_t Gray)
{ Gray ^= (Gray >> 4);
  Gray ^= (Gray >> 2);
  Gray ^= (Gray >> 1);
  return Gray; }

uint16_t EncodeGray(uint16_t Binary)
{ return Binary ^ (Binary>>1); }

uint16_t DecodeGray(uint16_t Gray)
{ Gray ^= (Gray >> 8);
  Gray ^= (Gray >> 4);
  Gray ^= (Gray >> 2);
  Gray ^= (Gray >> 1);
  return Gray; }

uint32_t EncodeGray(uint32_t Binary)
{ return Binary ^ (Binary>>1); }

uint32_t DecodeGray(uint32_t Gray)
{ Gray ^= (Gray >>16);
  Gray ^= (Gray >> 8);
  Gray ^= (Gray >> 4);
  Gray ^= (Gray >> 2);
  Gray ^= (Gray >> 1);
  return Gray; }

// ==============================================================================================
// TEA encryption/decryption
// Data is 2 x 32-bit word
// Key  is 4 x 32-bit word

void TEA_Encrypt (uint32_t* Data, const uint32_t *Key, int Loops)
{ uint32_t v0=Data[0], v1=Data[1];                         // set up
  const uint32_t delta=0x9e3779b9; uint32_t sum=0;         // a key schedule constant
  uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];     // cache key
  for (int i=0; i < Loops; i++)                            // basic cycle start
  { sum += delta;
    v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
    v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3); }  // end cycle
  Data[0]=v0; Data[1]=v1;
}

void TEA_Decrypt (uint32_t* Data, const uint32_t *Key, int Loops)
{ uint32_t v0=Data[0], v1=Data[1];                           // set up
  const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
  uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];       // cache key
  for (int i=0; i < Loops; i++)                              // basic cycle start */
  { v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
    v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
    sum -= delta; }                                          // end cycle
  Data[0]=v0; Data[1]=v1;
}

void TEA_Encrypt_Key0 (uint32_t* Data, int Loops)
{ uint32_t v0=Data[0], v1=Data[1];                          // set up
  const uint32_t delta=0x9e3779b9; uint32_t sum=0;          // a key schedule constant
  for (int i=0; i < Loops; i++)                             // basic cycle start
  { sum += delta;
    v0 += (v1<<4) ^ (v1 + sum) ^ (v1>>5);
    v1 += (v0<<4) ^ (v0 + sum) ^ (v0>>5); }                 // end cycle
  Data[0]=v0; Data[1]=v1;
}

void TEA_Decrypt_Key0 (uint32_t* Data, int Loops)
{ uint32_t v0=Data[0], v1=Data[1];                           // set up
  const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
  for (int i=0; i < Loops; i++)                              // basic cycle start
  { v1 -= (v0<<4) ^ (v0 + sum) ^ (v0>>5);
    v0 -= (v1<<4) ^ (v1 + sum) ^ (v1>>5);
    sum -= delta; }                                          // end cycle
  Data[0]=v0; Data[1]=v1;
}

// ==============================================================================================
// XXTEA encryption/decryption

static uint32_t XXTEA_MX(uint8_t E, uint32_t Y, uint32_t Z, uint8_t P, uint32_t Sum, const uint32_t Key[4])
{ return ((((Z>>5) ^ (Y<<2)) + ((Y>>3) ^ (Z<<4))) ^ ((Sum^Y) + (Key[(P&3)^E] ^ Z))); }

void XXTEA_Encrypt(uint32_t *Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops)
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

void XXTEA_Decrypt(uint32_t *Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops)
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

static uint32_t XXTEA_MX_KEY0(uint32_t Y, uint32_t Z, uint32_t Sum)
{ return ((((Z>>5) ^ (Y<<2)) + ((Y>>3) ^ (Z<<4))) ^ ((Sum^Y) + Z)); }

void XXTEA_Encrypt_Key0(uint32_t *Data, uint8_t Words, uint8_t Loops)
{ const uint32_t Delta = 0x9e3779b9;
  uint32_t Sum = 0;
  uint32_t Z = Data[Words-1]; uint32_t Y;
  for( ; Loops; Loops--)
  { Sum += Delta;
    for (uint8_t P=0; P<(Words-1); P++)
    { Y = Data[P+1];
      Z = Data[P] += XXTEA_MX_KEY0(Y, Z, Sum); }
    Y = Data[0];
    Z = Data[Words-1] += XXTEA_MX_KEY0(Y, Z, Sum); }
}

void XXTEA_Decrypt_Key0(uint32_t *Data, uint8_t Words, uint8_t Loops)
{ const uint32_t Delta = 0x9e3779b9;
  uint32_t Sum = Loops*Delta;
  uint32_t Y = Data[0]; uint32_t Z;
  for( ; Loops; Loops--)
  { for (uint8_t P=Words-1; P; P--)
    { Z = Data[P-1];
      Y = Data[P] -= XXTEA_MX_KEY0(Y, Z, Sum); }
    Z = Data[Words-1];
    Y = Data[0] -= XXTEA_MX_KEY0(Y, Z, Sum);
    Sum -= Delta; }
}

// ==============================================================================================

void XorShift32(uint32_t &Seed)      // simple random number generator
{ Seed ^= Seed << 13;
  Seed ^= Seed >> 17;
  Seed ^= Seed << 5; }

void XorShift64(uint64_t &Seed)
{ Seed ^= Seed >> 12;
  Seed ^= Seed << 25;
  Seed ^= Seed >> 27; }

// ==============================================================================================

const static unsigned char MapAscii85[86] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz!#$%&()*+-;<=>?@^_`{|}~";

const static uint8_t UnmapAscii85[128] =
{ 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85,
  85, 62, 85, 63, 64, 65, 66, 85, 67, 68, 69, 70, 85, 71, 85, 85,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 85, 72, 73, 74, 75, 76,
  77, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 85, 85, 85, 78, 79,
  80, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 81, 82, 83, 84, 85  };

uint8_t EncodeAscii85(char *Ascii, uint32_t Word)
{ for( uint8_t Idx=5; Idx; )
  { uint32_t Div = Word/85;
    Idx--;
    Ascii[Idx]=MapAscii85[Word-Div*85];
    Word=Div; }
  Ascii[5]=0;
  return 5; }

uint8_t DecodeAscii85(uint32_t &Word, const char *Ascii)
{ Word=0;
  for( uint8_t Idx=0; Idx<5; Idx++)
  { char Char = Ascii[Idx]; if(Char<=0) return 0;
    uint8_t Dig = UnmapAscii85[(uint8_t)Char];
    if(Dig>=85) return 0;
    Word = Word*85+Dig; }
  return 5; }

// ==============================================================================================

int APRS2IGC(char *Out, const char *Inp, int GeoidSepar)             // convert APRS positon message into IGC B-record
{ int Len=0;
  const char *Msg = strchr(Inp, ':'); if(Msg==0) return 0;           // colon: separates header and message
  Msg++;                                                             // where message starts
  if(Msg[0]!='/' || Msg[7]!='h') return 0;
  const char *Pos = Msg+8; if(Pos[4]!='.' || Pos[14]!='.') return 0; // where position starts
  const char *ExtPos = strstr(Pos+18, " !W"); if(ExtPos[5]=='!') ExtPos+=3; else ExtPos=0;
  Out[Len++]='B';                                                    // B-record
  memcpy(Out+Len, Msg+1, 6); Len+=6;                                 // copy UTC time
  memcpy(Out+Len, Pos, 4); Len+=4;                                   // copy DDMM
  memcpy(Out+Len, Pos+5, 2); Len+=2;                                 // copy fractional MM
  Out[Len++] = ExtPos?ExtPos[0]:'0';                                 // extended precision
  Out[Len++] = Pos[7];                                               // copy N/S sign
  memcpy(Out+Len, Pos+9, 5); Len+=5;                                 // copy DDMM
  memcpy(Out+Len, Pos+15,2); Len+=2;                                 // copy fractional MM
  Out[Len++] = ExtPos?ExtPos[1]:'0';                                 // extended precision
  Out[Len++] = Pos[17];                                              // copy E/W sign
  Out[Len++] = 'A';                                                  // GPS-valid flag
  memcpy(Out+Len, "          ", 10);                                 // prefill pressure and GNSS altitude with spaces
  const char *FL = strstr(Pos+18, " FL");                            // search pressure altitude
  int32_t AltH=0; int32_t AltL=0;
  if(FL && FL[6]=='.' && Read_Int(AltH, FL+3)==3 && Read_Int(AltL, FL+7)==2) // pressure altitude
  { int Alt = AltH*100+AltL; Alt=FeetToMeters(Alt);
    if(Alt<0) { Alt = (-Alt); Out[Len] = '-'; Format_UnsDec(Out+Len+1, (uint32_t)Alt, 4); }
         else { Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }
  }
  Len+=5;
  int32_t Alt=0;                                                     //
  if(Pos[27]=='A' && Pos[28]=='=' && Read_Int(Alt, Pos+29)==6)       // geometrical altitude
  { Alt=FeetToMeters(Alt); Alt+=GeoidSepar;                          // convert to meters and add GeoidSepar for HAE
    if(Alt<0) { Alt = (-Alt); Out[Len] = '-'; Format_UnsDec(Out+Len+1, (uint32_t)Alt, 4); }
         else { Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }
  }
  Len+=5;
  Out[Len++]='\n'; Out[Len]=0; return Len; } // add NL, terminator and return length og the string

// ==============================================================================================

// https://en.wikipedia.org/wiki/Barometric_formula
// https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html

const float g0 = 9.80665;
const float M  = 0.0289644;
const float R  = 8.3144598;

static float BaroAlt(float P, float hb, float Pb, float Tb, float Lb, float gb=g0)
{ if(Lb==0) return hb - logf(P/Pb)*(R*Tb)/(gb*M);
  return hb + Tb/Lb*(1-powf(P/Pb, (R*Lb)/(gb*M))); }

static float BaroPress(float h, float hb, float Pb, float Tb, float Lb, float gb=g0)
{ if(Lb==0) return Pb*expf(-(gb*M/R)*(h-hb)/Tb);
  return Pb*powf( (Tb-(h-hb)*Lb)/Tb, (gb*M)/(R*Lb)); }

static class BaroRef
{ public:
   float hb, Pb, Tb, Lb, gb;
} BaroRefTable[7] =
{ //  [m]    [Pa]     [K]      [K/m]  [m/s^2]
  {     0, 101325.00, 288.15,  0.0065, 9.807 },
  { 11000,  22632.10, 216.65,  0     , 9.776 },
  { 20000,   5474.89, 216.65, -0.0010, 9.745 },
  { 32000,    868.02, 228.65, -0.0028, 9.715 },
  { 47000,    110.91, 270.65,  0     , 9.654 },
  { 51000,     66.94, 270.65,  0.0028, 9.654 },
  { 71000,      3.96, 214.65,  0.0020, 9.594 }
} ;

float BaroTemp(float h)                     // temperature [K] at given altitude [m]
{ int Idx=0;
  for( ; Idx<6; Idx++)
  { if(h<BaroRefTable[Idx+1].hb) break; }
  BaroRef &Ref = BaroRefTable[Idx];
  return Ref.Tb - (h-Ref.hb)*Ref.Lb; }      // [K]

float BaroPress(float h)                   // pressure [Pa] at given altitude [m]
{ int Idx=0;
  for( ; Idx<6; Idx++)
  { if(h<BaroRefTable[Idx+1].hb) break; }
  BaroRef &Ref = BaroRefTable[Idx];
  return BaroPress(h, Ref.hb, Ref.Pb, Ref.Tb, Ref.Lb /* , Ref.gb */ ); }

float BaroAlt(float P)                     // altitude [m] for given pressure [Pa]
{ int Idx=0;
  for( ; Idx<6; Idx++)
  { if(P>BaroRefTable[Idx+1].Pb) break; }
  BaroRef &Ref = BaroRefTable[Idx];
  return BaroAlt(P, Ref.hb, Ref.Pb, Ref.Tb, Ref.Lb /* , Ref.gb */ ); }

// ==============================================================================================

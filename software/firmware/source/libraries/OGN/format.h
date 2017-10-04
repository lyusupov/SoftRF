#ifndef  __FORMAT_H__
#define  __FORMAT_H__

#include <stdint.h>

char HexDigit(uint8_t Val);

       void Format_Bytes ( void (*Output)(char), const uint8_t *Bytes,  uint8_t Len);
inline void Format_Bytes ( void (*Output)(char), const    char *Bytes,  uint8_t Len) { Format_Bytes(Output, (const uint8_t *)Bytes,  Len); }

       void Format_String( void (*Output)(char), const    char *String);
inline void Format_String( void (*Output)(char), const    char *String, uint8_t Len) { Format_Bytes(Output, (const uint8_t *)String, Len); }

void Format_Hex( void (*Output)(char), uint8_t  Byte );
void Format_Hex( void (*Output)(char), uint16_t Word );
void Format_Hex( void (*Output)(char), uint32_t Word );

void Format_UnsDec ( void (*Output)(char), uint16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
void Format_SignDec( void (*Output)(char),  int16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);

void Format_UnsDec ( void (*Output)(char), uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
void Format_SignDec( void (*Output)(char),  int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);

uint8_t Format_String(char *Str, const char *String);
uint8_t Format_String(char *Str, const char *String, uint8_t Len);

uint8_t Format_UnsDec (char *Str, uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
uint8_t Format_SignDec(char *Str,  int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);

uint8_t Format_Hex( char *Output, uint8_t  Byte );
uint8_t Format_Hex( char *Output, uint16_t Word );
uint8_t Format_Hex( char *Output, uint32_t Word );
uint8_t Format_Hex( char *Output, uint32_t Word, uint8_t Digits);


   int8_t  Read_Hex1(char Digit);

   int8_t  Read_Dec1(char Digit);                  // convert single digit into an integer
   inline int8_t Read_Dec1(const char *Inp) { return Read_Dec1(Inp[0]); }
   int8_t  Read_Dec2(const char *Inp);             // convert two digit decimal number into an integer
   int16_t Read_Dec3(const char *Inp);             // convert three digit decimal number into an integer
   int16_t Read_Dec4(const char *Inp);             // convert three digit decimal number into an integer

  template <class Type>
   int8_t Read_Hex(Type &Int, const char *Inp)            // convert variable number of digits hexadecimal number into an integer
   { Int=0; int8_t Len=0;
     if(Inp==0) return 0;
     for( ; ; )
     { int8_t Dig=Read_Hex1(Inp[Len]); if(Dig<0) break;
       Int = (Int<<4) + Dig; Len++; }
     return Len; }                                        // return number of characters read

  template <class Type>
   int8_t Read_UnsDec(Type &Int, const char *Inp)         // convert variable number of digits unsigned decimal number into an integer
   { Int=0; int8_t Len=0;
     if(Inp==0) return 0;
     for( ; ; )
     { int8_t Dig=Read_Dec1(Inp[Len]); if(Dig<0) break;
       Int = 10*Int + Dig; Len++; }
     return Len; }                                        // return number of characters read

  template <class Type>
   int8_t Read_SignDec(Type &Int, const char *Inp)        // convert signed decimal number into in16_t or int32_t
   { Int=0; int8_t Len=0;
     if(Inp==0) return 0;
     char Sign=Inp[0];
     if((Sign=='+')||(Sign=='-')) Len++;
     Len+=Read_UnsDec(Int, Inp); if(Sign=='-') Int=(-Int);
     return Len; }                                        // return number of characters read

  template <class Type>
   int8_t Read_Float1(Type &Value, const char *Inp)       // read floating point, take just one digit after decimal point
   { Value=0; int8_t Len=0;
     if(Inp==0) return 0;
     char Sign=Inp[0]; int8_t Dig;
     if((Sign=='+')||(Sign=='-')) Len++;
     Len+=Read_UnsDec(Value, Inp+Len); Value*=10;
     if(Inp[Len]!='.') goto Ret;
     Len++;
     Dig=Read_Dec1(Inp[Len]); if(Dig<0) goto Ret;
     Value+=Dig; Len++;
     Dig=Read_Dec1(Inp[Len]); if(Dig>=5) Value++;
     Ret: if(Sign=='-') Value=(-Value); return Len; }


#endif //  __FORMAT_H__

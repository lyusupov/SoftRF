#include <math.h>
#include <stdint.h>

#ifndef __INTMATH_H__
#define __INTMATH_H__

const uint32_t IntSine_Scale=0x80000000;

// get Sine from the SineTable
// Angle is 0..255 which corresponds to <0..2*PI)
int32_t IntSine(uint8_t Angle);

// precise Sine with for 16-bit angles 2nd derivative interpolation
// max. result error is about 2.3e-7
int32_t IntSine(uint16_t Angle);

// precise Sine for 32-bit angles with 2nd derivative interpolation
// max. result error is about 2.3e-7
int32_t IntSine(uint32_t Angle);

// less precise sine for 16-bit angles
const int16_t Isine_Scale=0x4000;
int16_t        Isin(int16_t Angle);
int16_t inline Icos(int16_t Angle) { return Isin(Angle+0x4000); }

// atan2(Y, X)
// max. result error is 1/6 degree
int16_t IntAtan2(int16_t Y, int16_t X);

// integer square root
// uint32_t IntSqrt(uint32_t Inp);
// uint64_t IntSqrt(uint64_t Inp);

template<class Type>                                 // integer square root for 16-bit or 32-bit
 Type IntSqrt(Type Inp)                              // must be made with unsigned type or signed types with _positive_ values
{ Type Out  = 0;
  Type Mask = 1; Mask<<=(sizeof(Type)*8-2);

  while(Mask>Inp) Mask>>=2;
  while(Mask)
  { if(Inp >= (Out+Mask))
    { Inp -= Out+Mask; Out += Mask<<1; }
    Out>>=1; Mask>>=2; }
  if(Inp>Out) Out++;

  return Out; }

// Distance = sqrt(dX*dX+dY*dY)

inline uint32_t IntDistance(int32_t dX, int32_t dY) { return IntSqrt((uint64_t)((int64_t)dX*dX + (int64_t)dY*dY)); }
inline uint16_t IntDistance(int16_t dX, int16_t dY) { return IntSqrt((uint32_t)((int32_t)dX*dX + (int32_t)dY*dY)); }

template <class IntType>
 IntType IntFastDistance(IntType dX, IntType dY) // after: http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
 { IntType min, max, approx;

   if(dX<0) dX = -dX;
   if(dY<0) dY = -dY;

   if(dX<dY) { min = dX; max = dY; }
        else { min = dY; max = dX; }

   approx = max*1007 + min*441;
   if( max < (min<<4) ) approx -= max*40;

   return (approx+512)>>10; }

#endif // of __INTMATH_H__

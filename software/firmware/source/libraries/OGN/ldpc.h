#ifndef __LDPC_H__
#define __LDPC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <limits>

#include "bitcount.h"

#ifndef __AVR__
// #include <stdio.h>
#include <math.h>
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

// extern const uint32_t LDPC_ParityGen_n208k160[48][5];
// extern const uint32_t LDPC_ParityCheck_n208k160[48][7];
// extern const uint8_t  LDPC_ParityCheckIndex_n208k160[48][24];
// extern const uint8_t  LDPC_BitWeight_n208k160[208];
#ifdef WITH_PPM
extern const uint32_t LDPC_ParityGen_n354k160[194][5];
extern const uint32_t LDPC_ParityCheck_n354k160[194][12];
#endif

#ifdef __AVR__

// encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity);

// encode Parity from Data: Data is 20 bytes = 160 bits, Parity is 6 bytes = 48 bits
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity);
void LDPC_Encode(      uint8_t *Data);
                                         // check Data against Parity (run 48 parity checks) - return number of failed checks
uint8_t LDPC_Check(const uint8_t  *Data); // 20 data bytes followed by 6 parity bytes
uint8_t LDPC_Check(const uint32_t *Packet);

#else // if not 8-bit AVR

void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity);
void LDPC_Encode(      uint8_t *Data);
                                                                 // encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
// void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5]);

// void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, uint8_t DataWords,  uint8_t Checks, const uint32_t *ParityGen);
// inline void LDPC_Encode(const uint32_t *Data, uint32_t *Parity) { LDPC_Encode(Data, Parity, 5, 48, (uint32_t *)LDPC_ParityGen_n208k160); }
// inline void LDPC_Encode(      uint32_t *Data)                   { LDPC_Encode(Data, Data+5, 5, 48, (uint32_t *)LDPC_ParityGen_n208k160); }
// inline void LDPC_Encode_n394k160(const uint32_t *Data, uint32_t *Parity) { LDPC_Encode(Data, Parity, 5, 194, (uint32_t *)LDPC_ParityGen_n354k160); }
// inline void LDPC_Encode_n394k160(      uint32_t *Data)                   { LDPC_Encode(Data, Data+5, 5, 194, (uint32_t *)LDPC_ParityGen_n354k160); }

void LDPC_Encode(const uint32_t *Data, uint32_t *Parity);
void LDPC_Encode(      uint32_t *Data);
#ifdef WITH_PPM
void LDPC_Encode_n354k160(const uint32_t *Data, uint32_t *Parity);
void LDPC_Encode_n354k160(      uint32_t *Data);
#endif
                                                                  // check Data against Parity (run 48 parity checks) - return number of failed checks
uint8_t LDPC_Check(const uint32_t *Data, const uint32_t *Parity); // Data and Parity are 32-bit words
uint8_t LDPC_Check(const uint32_t *Data);
uint8_t LDPC_Check(const uint8_t  *Data);                         // 20 data bytes followed by 6 parity bytes
#ifdef WITH_PPM
uint8_t LDPC_Check_n354k160(const uint32_t *Data, const uint32_t *Parity); // Data and Parity are 32-bit words
uint8_t LDPC_Check_n354k160(const uint32_t *Data);
#endif

#endif // __AVR__

#ifndef __AVR__

extern const uint8_t LDPC_ParityCheckIndex_n208k160[48][24];

class LDPC_Decoder
{ public:
   const static uint8_t UserBits   = 160;                 // 5 32-bit bits = 20 bytes
   const static uint8_t UserWords  = UserBits/32;
   const static uint8_t ParityBits =  48;                 // 6 bytes (total packet is 26 bytes)
   const static uint8_t CodeBits   = UserBits+ParityBits; // 160+48 = 208 code bits = 26 bytes
   const static uint8_t CodeBytes  = (CodeBits+ 7)/ 8;    //
   const static uint8_t CodeWords  = (CodeBits+31)/32;    //
   const static uint8_t MaxCheckWeight = 24;
   // const static uint8_t MaxBitWeight   =  8;

  public:

   int16_t  InpBit[CodeBits]; // a-priori bits
   int16_t  ExtBit[CodeBits]; // extrinsic inf.
   int16_t  OutBit[CodeBits]; // a-posteriori bits

   void Input(const uint8_t *Data, uint8_t *Err)
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t DataByte=0; uint8_t ErrByte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(Mask==1) { DataByte=Data[Idx];  ErrByte=Err[Idx]; }
       int16_t Inp;
       if(ErrByte&Mask) Inp=0;
                   else Inp=(DataByte&Mask) ? +128:-128;
       OutBit[Bit] = InpBit[Bit] = Inp; ExtBit[Bit]=0;
       Mask<<=1; if(Mask==0) { Idx++; Mask=1; }
     }
   }

   void Input(const uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=Data[Idx];
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { OutBit[Bit] = InpBit[Bit] = (Word&Mask) ? +128:-128;
       ExtBit[Bit]=0;
       Mask<<=1; if(Mask==0) { Word=Data[++Idx]; Mask=1; }
     }
   }

   void Input(const float *Data, float RefAmpl=1.0)
   { for(int Bit=0; Bit<CodeBits; Bit++)
     { int Inp = floor(128*Data[Bit^7]/RefAmpl+0.5);
       if(Inp>32767) Inp=32767; else if(Inp<(-32767)) Inp=(-32767);
       OutBit[Bit] = InpBit[Bit] = Inp;
       ExtBit[Bit]=0; }
   }

   void Output(uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(OutBit[Bit]>0) Word|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Word; Word=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Word;
   }

   void Output(uint8_t Data[CodeBytes])
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t Byte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(OutBit[Bit]>0) Byte|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Byte; Byte=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Byte;
   }

   int8_t ProcessChecks(void)
   { for(uint8_t Bit=0; Bit<CodeBits; Bit++)
       ExtBit[Bit]=0;
     uint8_t Count=0;
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { int16_t Ret=ProcessCheck(Row);
       if(Ret<=0) Count++; }
     // printf("%d parity checks fail\n", Count);
     if(Count==0) return 0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { OutBit[Bit] = InpBit[Bit] + (ExtBit[Bit]>>1); }
     return Count; }

   int16_t ProcessCheck(uint8_t Row)
   { int16_t MinAmpl=32767; uint8_t MinBit=0; int16_t MinAmpl2=MinAmpl;
     uint32_t Word=0; uint32_t Mask=1;
     const uint8_t *CheckIndex = LDPC_ParityCheckIndex_n208k160[Row];
     uint8_t CheckWeight = *CheckIndex++;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit];
       int16_t Ampl=OutBit[BitIdx];
       if(Ampl>0) Word|=Mask;
       Mask<<=1;
       if(Ampl<0) Ampl=(-Ampl);
       if(Ampl<MinAmpl) { MinAmpl2=MinAmpl; MinAmpl=Ampl; MinBit=Bit; }
       else if(Ampl<MinAmpl2) { MinAmpl2=Ampl; }
     }
     uint8_t CheckFails = Count1s(Word)&1;
     Mask=1;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit];
       int16_t Ampl = Bit==MinBit ? MinAmpl2 : MinAmpl;
       if(CheckFails) Ampl=(-Ampl);
       ExtBit[BitIdx] += (Word&Mask) ? Ampl:-Ampl;
       Mask<<=1; }
     return CheckFails?-MinAmpl:MinAmpl; }

} ;

template <class Float=float>
 class LDPC_FloatDecoder
{ public:

   const static int MaxCodeBits=512;
   const static int MaxParityBits=256;
   const static int MaxParityWeight=32;                       //
   int CodeBits;                                              // number of code bits
   int ParityBits;                                            // number of parity bits
   uint16_t ParityCheckIndex[MaxParityBits][MaxParityWeight]; // list of 1's in the ParityCheck matrix
   uint8_t  ParityCheckRowWeight[MaxParityBits];              // number of 1's in ParityCheck rows
   uint8_t  ParityCheckColWeight[MaxCodeBits];                // number of 1's in ParityCheck columns

   Float InpBit[MaxCodeBits];                                 // a-priori bits
   Float ExtBit[MaxCodeBits];                                 // extrinsic inf.
   Float OutBit[MaxCodeBits];                                 // a-posteriori bits
   Float Feedback;

  public:

   LDPC_FloatDecoder()
   { CodeBits=0; ParityBits=0; Feedback=0.33; }

   void Clear(void)
   { for(int Bit=0; Bit<CodeBits; Bit++)
     { OutBit[Bit] = InpBit[Bit] = ExtBit[Bit]=0; }
   }

   int Configure(int NewCodeBits, int NewParityBits, const uint32_t *PackedParityCheck )
   { if(CodeBits>MaxCodeBits) return -1;
     CodeBits=NewCodeBits;
     if(ParityBits>MaxParityBits) return -1;
     ParityBits=NewParityBits;
     for(int Bit=0; Bit<CodeBits; Bit++)
       ParityCheckColWeight[Bit]=0;
     const uint32_t *Check=PackedParityCheck;
     for(int ParBit=0; ParBit<ParityBits; ParBit++)
     { int RowWeight=0;
       uint32_t Word=0; uint32_t Mask=0;
       for(int Bit=0; Bit<CodeBits; Bit++)
       { if(Mask==0) { Mask=1; Word=(*Check++); }
         if(Word&Mask)
         { ParityCheckIndex[ParBit][RowWeight++]=Bit;
           ParityCheckColWeight[Bit]++; }
         Mask<<=1;
       }
       ParityCheckRowWeight[ParBit]=RowWeight;
     }
     return 1; }

   void PrintConfig(void) const
   { printf("LDPC_FloatDecoder[%d,%d] Check index table:\n", CodeBits, ParityBits);
     for(int ParBit=0; ParBit<ParityBits; ParBit++)
     { printf("Check[%3d]:", ParityCheckRowWeight[ParBit]);
       for(int Bit=0; Bit<ParityCheckRowWeight[ParBit]; Bit++)
       { printf(" %3d", ParityCheckIndex[ParBit][Bit]); }
       printf("\n");
     }
     printf("ColWeight[%d]:\n", CodeBits);
     int Bit;
     for(Bit=0; Bit<CodeBits; Bit++)
     { if((Bit&0x1F)==0x00) printf("%03d:", Bit);
       printf(" %d", ParityCheckColWeight[Bit]);
       if((Bit&0x1F)==0x1F) printf("\n"); }
     if((Bit&0x1F)!=0x00) printf("\n");
   }

   void PrintOutBits(void)
   { printf("OutBit[%d]\n", CodeBits);
     for(int Bit=0; Bit<CodeBits; Bit++)
     { if((Bit&0xF)==0x0) printf("%03d:", Bit);
       printf(" %+6.3f", OutBit[Bit]);
       if((Bit&0xF)==0xF) printf("\n"); }
   }

   void addInput(int Bit, Float Ampl)
   { InpBit[Bit]+=Ampl; OutBit[Bit] = InpBit[Bit]; }

   void Input(const uint8_t *Data, const uint8_t *Err, Float Ampl=1.0)   // get bits from series of bytes and the error pattern (from Manchester decoder)
   { uint8_t Mask=1; int Idx=0; uint8_t DataByte=0; uint8_t ErrByte=0;
     for(int Bit=0; Bit<CodeBits; Bit++)
     { if(Mask==1) { DataByte=Data[Idx];  ErrByte=Err[Idx]; }
       Float Inp;
       if(ErrByte&Mask) Inp=0;
                   else Inp=(DataByte&Mask) ? +Ampl:-Ampl;
       OutBit[Bit] = InpBit[Bit] = Inp; ExtBit[Bit]=0;
       Mask<<=1; if(Mask==0) { Idx++; Mask=1; }
     }
   }

   void Input(const uint32_t *Data, Float Ampl=1.0)                      // get bits from a series of 32-bit words
   { uint32_t Mask=0; int Idx=0; uint32_t Word=0;
     for(int Bit=0; Bit<CodeBits; Bit++)
     { if(Mask==0) { Word=Data[Idx++]; Mask=1; }
       OutBit[Bit] = InpBit[Bit] = (Word&Mask) ? +Ampl:-Ampl; ExtBit[Bit]=0;
       Mask<<=1;
     }
   }

   void Output(uint32_t *Data)                                           // format decoded bits as a series of 32-bit words
   { uint32_t Mask=1; int Idx=0; uint32_t Word=0;
     for(int Bit=0; Bit<CodeBits; Bit++)
     { if(OutBit[Bit]>0) Word|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Word; Word=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Word;
   }

   void Output(uint8_t *Data)                                            // format decoded bits as a series of bytes
   { uint8_t Mask=1; int Idx=0; uint8_t Byte=0;
     for(int Bit=0; Bit<CodeBits; Bit++)
     { if(OutBit[Bit]>0) Byte|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Byte; Byte=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Byte;
   }

   int ProcessChecks(void)
   { for(int Bit=0; Bit<CodeBits; Bit++)                                 // clear the extrinsic inf. for bits
       ExtBit[Bit]=0;
     int Count=0;
     for(int Row=0; Row<ParityBits; Row++)                               // process all parity checks and count how many have failed
     { Float Ret=ProcessCheck(Row);
       if(Ret<=0) Count++; }
     // printf("%d parity checks fail\n", Count);
     if(Count==0) return 0;                                              // if all passed, then return
     for(int Bit=0; Bit<CodeBits; Bit++)                                 // add Input+Extrinsic and store in Output
     { OutBit[Bit] = InpBit[Bit] + Feedback*ExtBit[Bit]; }
     return Count; }

   Float ProcessCheck(uint8_t Row)
   { Float MinAmpl=std::numeric_limits<Float>::max(); int MinBit=0; Float MinAmpl2=MinAmpl;               // look for 1st and 2nd smallest LL
     uint32_t Word=0; uint32_t Mask=1;
     const uint16_t *CheckIndex = ParityCheckIndex[Row];                 // indeces of bits in this parity check
     int CheckWeight = ParityCheckRowWeight[Row];                    // number of bits in this parity check
     for(int Bit=0; Bit<CheckWeight; Bit++)                              // loop over bits in the parity check
     { int BitIdx=CheckIndex[Bit];                                       // index of the bit
       Float Ampl=OutBit[BitIdx];                                       // LL of the bit
       if(Ampl>0) Word|=Mask;                                            // store hard bits in the Word
       Mask<<=1;
       if(Ampl<0) Ampl=(-Ampl);                                          // strip the LL sign
       if(Ampl<MinAmpl) { MinAmpl2=MinAmpl; MinAmpl=Ampl; MinBit=Bit; }  // find 1st and 2nd smallest
       else if(Ampl<MinAmpl2) { MinAmpl2=Ampl; }
     }
     int CheckFails = __builtin_parityl(Word);                           // tell if this parity check failed
     Mask=1;
     for(int Bit=0; Bit<CheckWeight; Bit++)                              // loop over bits in this parity check
     { int BitIdx=CheckIndex[Bit];                                       // inndex of the bit
       Float Ampl = Bit==MinBit ? MinAmpl2 : MinAmpl;                   // if this is the weakest bit, then use 2nd smallest LL, otherwise 1st
       if(CheckFails) Ampl=(-Ampl);
       ExtBit[BitIdx] += (Word&Mask) ? Ampl:-Ampl;                       // add to the extrinsic inf. with the correct sign
       Mask<<=1; }
     return CheckFails?-MinAmpl:MinAmpl; }

   int CountErrors(void)
   { int Count=0;
     for(int Idx=0; Idx<CodeBits; Idx++)
     { bool Inp=InpBit[Idx]>0;
       bool Out=OutBit[Idx]>0;
       if(Inp!=Out) Count++; }
     return Count; }

} ;

#ifdef WITH_PPM
template <class Float>
 class OGN_PPM_Decoder
{ public:
   static const int DataBits = 32*5;                      // 5 words = 160 data bits = OGN packet
   static const int ParityBits = 194;                     // 194 parity bits (Gallager code)
   static const int CodeBits = DataBits+ParityBits;       // 354 total bits per Gallager code block
   static const int BitsPerSymbol = 6;                    // 6 bits per symbol for PPM modulation
   static const int PulsesPerSlot = 1<<BitsPerSymbol;     // 64 (possible) pulses per time slot = 1 symbol = 6 bits
   static const int CodeSymbols = CodeBits/BitsPerSymbol; // 59 time slots to form complete packet

   LDPC_FloatDecoder<Float> LDPC_Decoder;                 // inner LDPC code decoder

   Float InpSymb[CodeSymbols][PulsesPerSlot];             // input from the demodulator
   Float ExtSymb[CodeSymbols][PulsesPerSlot];             // output from the LDPC decoder
   Float OutSymb[CodeSymbols][PulsesPerSlot];             // input x extrinsic inf.

  public:
   OGN_PPM_Decoder()
   { LDPC_Decoder.Configure(CodeBits, ParityBits, (uint32_t *)LDPC_ParityCheck_n354k160);
     Clear(); }

   void Clear(void)
   { for(int Symb=0; Symb<CodeSymbols; Symb++)
     { Float Ext=1.0/PulsesPerSlot;
       for(int Pulse=0; Pulse<PulsesPerSlot; Pulse++)
       { InpSymb[Symb][Pulse]=0; ExtSymb[Symb][Pulse]=Ext; OutSymb[Symb][Pulse]=0; }
     }
   }

   void addSymbol(unsigned int Slot, unsigned int Symbol, Float Power=1.0)
   { if( (Slot>=CodeSymbols) || (Symbol>=PulsesPerSlot) ) return;
     InpSymb[Slot][Symbol]+=Power; }

   int Process(int Loops=48)
   { LDPC_Decoder.Clear();

     for(int Symb=0; Symb<CodeSymbols; Symb++)
     { for(int Pulse=0; Pulse<PulsesPerSlot; Pulse++)
       { Float Pwr=InpSymb[Symb][Pulse]*ExtSymb[Symb][Pulse];
         if(Pwr==0) continue;
         Pwr=Pwr*Pwr;
         int Bin=Binary(Pulse);
         int Idx=Symb;
         for(int Bit=0; Bit<BitsPerSymbol; Bit++, Idx+=CodeSymbols)
         { LDPC_Decoder.addInput(Idx, (Bin&1) ? +Pwr:-Pwr);
           Bin>>=1; }
       }
     }
     int CheckErr=0;
     for( int Loop=0; Loop<Loops; Loop++)
     { CheckErr=LDPC_Decoder.ProcessChecks();
       printf("%3d: OGN_PPM_Decoder.Process() => %3d\n", Loop, CheckErr);
       if(CheckErr==0) break; }
     return CheckErr; }

   static uint8_t Gray(uint8_t Binary) { return Binary ^ (Binary>>1); }

   static uint8_t Binary(uint8_t Gray)
   { Gray = Gray ^ (Gray >> 4);
     Gray = Gray ^ (Gray >> 2);
     Gray = Gray ^ (Gray >> 1);
     return Gray; }

   void NormExtSymb(Float Norm=1.0)
   { for(int Symb=0; Symb<CodeSymbols; Symb++)
     { NormExtSymb(Symb, Norm); }
   }

   void NormExtSymp(int Symb, Float Norm=1.0)
   { Float Sum=0;
     for(int Pulse=0; Pulse<PulsesPerSlot; Pulse++)
     { Sum+=ExtSymb[Symb][Pulse]; }
     Sum=Norm/Sum;
     for(int Pulse=0; Pulse<PulsesPerSlot; Pulse++)
     { ExtSymb[Symb][Pulse]*=Sum; }    
   }

} ;
#endif // WITH_PPM

#endif // __AVR__

#endif // of __LDPC_H__

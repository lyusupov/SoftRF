
#include "si4032.h"

#include "manchester.h"

class RFM_TRX
{ public:                             // hardware access functions


#ifdef USE_BLOCK_SPI                                                    // SPI transfers in blocks, implicit control of the SPI-select
   void (*TransferBlock)(uint8_t *Data, uint8_t Len);
   static const size_t MaxBlockLen = 64;
   uint8_t Block_Buffer[MaxBlockLen];

   uint8_t *Block_Read(uint8_t Len, uint8_t Addr)                       // read given number of bytes from given Addr
   { Block_Buffer[0]=Addr; memset(Block_Buffer+1, 0, Len);
     (*TransferBlock) (Block_Buffer, Len+1);
     return  Block_Buffer+1; }                                          // return the pointer to the data read from the given Addr

   uint8_t *Block_Write(const uint8_t *Data, uint8_t Len, uint8_t Addr) // write given number of bytes to given Addr
   { Block_Buffer[0] = Addr | 0x80; memcpy(Block_Buffer+1, Data, Len);
     // printf("Block_Write( [0x%02X, .. ], %d, 0x%02X) .. [0x%02X, 0x%02X, ...]\n", Data[0], Len, Addr, Block_Buffer[0], Block_Buffe$
     (*TransferBlock) (Block_Buffer, Len+1);
     return  Block_Buffer+1; }
#else                                                                   // SPI transfers as single bytes, explicit control of the SPI$
   void (*Select)(void);                                                // activate SPI select
   void (*Deselect)(void);                                              // desactivate SPI select
   uint8_t (*TransferByte)(uint8_t);                                    // exchange one byte through SPI
#endif

   bool (*DIO0_isOn)(void);                                              // read DIO0 = packet is ready
   void (*RESET)(uint8_t On);                                            // activate or desactivate the RF chip reset

                                      // the following are in units of the synthesizer with 8 extra bits of precision
   uint32_t BaseFrequency;            // [Hz] base frequency = channel #0
   uint32_t ChannelSpacing;           // [Hz] spacing between channels
    int16_t FreqCorr;                 // [0.1ppm]
    int16_t Channel;                  // [int] channel being used

   void setBaseFrequency(uint32_t Frequency=868200000) { BaseFrequency=Frequency; } // [Hz]
   void setChannelSpacing(uint32_t  Spacing=   200000) { ChannelSpacing=Spacing; }  // [Hz]
   void setFrequencyCorrection(int16_t ppmFreqCorr=0)  { FreqCorr = ppmFreqCorr; }  // [0.1ppm]

   void setChannel(int16_t newChannel)
   { Channel=newChannel;
     uint32_t Freq = BaseFrequency+ChannelSpacing*Channel;
      int32_t Corr = ((Freq>>7)*FreqCorr)/78125;
              Freq += Corr;
     WriteCarrFreq(Freq); }
   uint8_t getChannel(void) const { return Channel; }

#ifdef USE_BLOCK_SPI

   static uint16_t SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }

   uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0) // write Byte
   { // printf("WriteByte(0x%02X, 0x%02X)\n", Byte, Addr);
     uint8_t *Ret = Block_Write(&Byte, 1, Addr); return *Ret; }

   void WriteWord(uint16_t Word, uint8_t Addr=0) // write Word => two bytes
   { // printf("WriteWord(0x%04X, 0x%02X)\n", Word, Addr);
     uint16_t Swapped = SwapBytes(Word); Block_Write((uint8_t *)&Swapped, 2, Addr); }

   uint8_t ReadByte (uint8_t Addr=0)
   { uint8_t *Ret = Block_Read(1, Addr);
     // printf("ReadByte(0x%02X) => 0x%02X\n", Addr, *Ret );
     return *Ret; }

   uint16_t ReadWord (uint8_t Addr=0)
   { uint16_t *Ret = (uint16_t *)Block_Read(2, Addr);
     // printf("ReadWord(0x%02X) => 0x%04X\n", Addr, SwapBytes(*Ret) );
     return SwapBytes(*Ret); }

   void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0)
   { Block_Write(Data, Len, Addr); }
/*
   void WritePacket(const uint8_t *Data, uint8_t Len=26)         // write the packet data (26 bytes)
   { uint8_t Packet[2*Len];
     uint8_t PktIdx=0;
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { uint8_t Byte=Data[Idx];
       Packet[PktIdx++]=ManchesterEncode[Byte>>4];                               // software manchester encode every byte
       Packet[PktIdx++]=ManchesterEncode[Byte&0x0F];
     }
     Block_Write(Packet, 2*Len, REG_FIFO);
   }
*/
/*
   void ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26)             // read packet data from FIFO
   { uint8_t *Packet = Block_Read(2*Len, REG_FIFO);                         // read 2x26 bytes from the RF chip RxFIFO
     uint8_t PktIdx=0;
     for(uint8_t Idx=0; Idx<Len; Idx++)                                     // loop over packet bytes
     { uint8_t ByteH = Packet[PktIdx++];
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
       uint8_t ByteL = Packet[PktIdx++];
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
       Data[Idx]=(ByteH<<4) | ByteL;
       Err [Idx]=(ErrH <<4) | ErrL ;
     }
   }
*/
#else // single Byte transfer SPI

   uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0) const  // write Byte
   { Select();
     TransferByte(Addr | 0x80);
     uint8_t Old=TransferByte(Byte);
     Deselect();
     return Old; }

   uint16_t WriteWord(uint16_t Word, uint8_t Addr=0) const // write Word => two bytes
   { Select();
     TransferByte(Addr | 0x80);
     uint16_t Old=TransferByte(Word>>8);             // upper byte first
     Old = (Old<<8) | TransferByte(Word&0xFF);       // lower byte second
     Deselect();
     return Old; }

   void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0) const
   { Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { TransferByte(Data[Idx]); }
     Deselect(); }

   uint8_t ReadByte (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint8_t Byte=TransferByte(0);
     Deselect();
     return Byte; }

   uint16_t ReadWord (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint16_t Word=TransferByte(0);
     Word = (Word<<8) | TransferByte(0);
     Deselect();
     return Word; }

   void WritePacket(const uint8_t *Data, uint8_t Len=26) const   // write the packet data (26 bytes)
   { const uint8_t Addr=REG_FIFO;                                // write to FIFO
     Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { TransferByte(Data[Idx]);
       // uint8_t Byte=Data[Idx];
       // TransferByte(ManchesterEncode[Byte>>4]);                  // software manchester encode every byte
       // TransferByte(ManchesterEncode[Byte&0x0F]);
     }
     Deselect();
   }
/*
   void ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26) const       // read packet data from FIFO
   { const uint8_t Addr=REG_FIFO;
     Select();                                                              // select the RF chip: start SPI transfer
     TransferByte(Addr);                                                    // trasnfer the address/read: FIFO
     for(uint8_t Idx=0; Idx<Len; Idx++)                                     // loop over packet byte
     { uint8_t ByteH = 0;
       ByteH = TransferByte(ByteH);
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
       uint8_t ByteL = 0;
       ByteL = TransferByte(ByteL);
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
       Data[Idx]=(ByteH<<4) | ByteL;
       Err [Idx]=(ErrH <<4) | ErrL ;
     }
     Deselect();                                                            // de-select RF chip: end of SPI transfer
   }
*/
#endif // USE_BLOCK_SPI

   int Configure(int16_t Channel, const uint32_t SYNC)
   { WriteTxPower(10);                        // Tx power: 10dBm
     WriteWord(0x0000, REG_FREQOFS0);         // clear frequency offset
     setChannel(Channel);                     // set frequency to the given channel
     WriteFreqDev(50000);                     // GFSK deviation
     WriteDataRate(50000);                    // Data rate 50 or 100kbps ?
     WriteByte(0x08, REG_DATACTRL);           // packet handling enable, no CRC, MSB first
     WriteByte(0x0E, REG_HEADCTRL2);          // no header, 4 SYNC bytes, fixed packet length
     WriteByte(0x00, REG_PREALEN);            // minimal preamble: 1 nibble
     WriteByte(26, REG_PKTLEN);               // packets of 26 bytes
     WriteByte(SYNC>>24, REG_SYNC3);
     WriteByte(SYNC>>16, REG_SYNC2);
     WriteByte(SYNC>> 8, REG_SYNC1);
     WriteByte(SYNC    , REG_SYNC0);
     ClearTxFIFO();
     return 0; }

   void RegDump(void (*CONS_UART_Write)(char))
   { Format_String(CONS_UART_Write, "Si4032 reg[0x00..0x7F]:\n");
     for(uint8_t Addr=0x00; Addr<0x80; Addr++)
     { if((Addr&0xF)==0x00)
       { Format_Hex(CONS_UART_Write, Addr); CONS_UART_Write(':'); }
       CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, ReadByte(Addr));
       if((Addr&0xF)==0x0F)
       { Format_String(CONS_UART_Write, "\n"); }
     }
   }

   void    SoftReset(void)   { WriteByte(0x80, REG_OPMODE1); }
   void    Transmit (void)   { WriteByte(0x0B, REG_OPMODE1); }

   void ClearTxFIFO(void)
   { WriteByte(0x01, REG_OPMODE2);
     WriteByte(0x00, REG_OPMODE2); }

   void WriteTxPower(int8_t TxPower) // [dBm]
   { TxPower = (TxPower+2)/3;
     if(TxPower<0) TxPower=0;
     else if(TxPower>7) TxPower=7;
     WriteByte(TxPower, REG_TXPOWER); }

   void WriteCarrFreq(uint32_t Freq)                                   // [Hz]
   {
#ifdef WITH_XTAL26MHZ                                                  // if Xtal 26MHz (nominal 30MHz)
     WriteCarrFreq_Xtal30MHz((((Freq/4)*15+6)/13)*4);                  // convert the frequency factor
#else
     WriteCarrFreq_Xtal30MHz(Freq);
#endif
   }

   void WriteCarrFreq_Xtal30MHz(uint32_t Freq)                         // [Hz]
   { uint8_t  HBsel=0; if(Freq>=480000000) { HBsel=0x20; Freq/=2; }    // high-band when >=480MHz
     uint8_t  Fb = Freq/10000000; Freq-=(uint32_t)Fb*10000000; Fb-=24; // band-select in 10MHz steps
     uint16_t Fc = (Freq*64)/10000;
     WriteByte( 0x40 | HBsel | Fb, REG_FREQ2);
     WriteByte( Fc>>8,   REG_FREQ1);
     WriteByte( Fc&0xFF, REG_FREQ0);
   }

   void WriteFreqDev(uint32_t FreqDev)                                 // [Hz]
   {
#ifdef WITH_XTAL26MHZ
     WriteFreqDev_Xtal30MHz(((FreqDev*15+6)/13));
#else
     WriteFreqDev_Xtal30MHz(FreqDev);
#endif
   }

   void WriteFreqDev_Xtal30MHz(uint32_t FreqDev)                       // [Hz]
   { uint16_t Fd = FreqDev/625;
     WriteByte(Fd&0x0FF, REG_DEV);
     WriteByte( 0x23 | ((Fd&0x100)>>6) , REG_MOD2);                    // FIFO mode and GFSK modulation
     WriteByte( 0x02, REG_MOD1); }                                     // Manchester encoding

   void WriteDataRate(uint32_t DataRate)                               // [Hz]
   {
#ifdef WITH_XTAL26MHZ
     WriteDataRate_Xtal30MHz(((DataRate*15+6)/13));
#else
     WriteDataRate_Xtal30MHz(DataRate);
#endif
   }

   void WriteDataRate_Xtal30MHz(uint32_t DataRate)                     // [Hz]
   { uint16_t DR_word = ((DataRate<<10)+7812)/15625;
     WriteByte(DR_word>>8, REG_RATE1);
     WriteByte(DR_word&0xFF, REG_RATE0); }

   uint8_t ReadVersion(void) { return ReadByte(REG_VERSION); }         // normally returns: 0x24
   uint8_t ReadLowBat(void)  { return ReadByte(REG_LOWBAT ); }         // read low battery status

   int16_t ReadChipTemp(void)
   { WriteByte(0x80, REG_ADCCONF);                                     // trigger the ADC conversion with temperature sensor
     vTaskDelay(1);
     int16_t Temp=ReadByte(REG_ADC);
     return Temp*5-640; }                                              // [0.1degC]

   uint16_t ReadBatVolt(void)
   { return 170+5*(uint16_t)ReadByte(REG_BATVOLT); }                   // [0.01V]
} ;


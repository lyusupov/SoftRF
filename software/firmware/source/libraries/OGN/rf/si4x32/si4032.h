#define REG_VERSION                                 0x01
#define REG_STATUS                                  0x02 // OUrrrrSS Overflow, Underflow, State: 00=Idle, 01=TX
#define REG_OPMODE1                                 0x07 // RLWXTrPR Reset, LowBat, WakeUp, Xtal32kHz, Transmit, PLL. Ready
#define REG_OPMODE2                                 0x08 // rrrrArrF AutoTx, FIFO reset
#define REG_XTAL                                    0x09 // Xtal load capacitance

#define REG_GPIO0                                   0x0B // 
#define REG_GPIO1                                   0x0C
#define REG_GPIO2                                   0x0D
#define REG_IOPORT                                  0x0E

#define REG_ADCCONF                                 0x0F // ADC configuration: TSSSRRGG Trigger, Source, Ref, Gain
#define REG_ADCOFS                                  0x10
#define REG_ADC                                     0x11 // ADC output value
#define REG_TEMPCTRL                                0x12 // temparature: RROTVVVV Range, Offset, Trim, Value
#define REG_TEMPOFS                                 0x13 // temperature offset

#define REG_LOWBAT                                  0x1A // [50mV, 5-bit] low-bat threshold, p.44
#define REG_BATVOLT                                 0x1B // [50mV, 5-bit] battery level = 1.7V + 50mV*ADC

#define REG_DATACTRL                                0x30 //
#define REG_PKTSTAT                                 0x31 // xxxxxxTS, T=packet being transmitting, S=packet has been sent
#define REG_HEADCTRL2                               0x33 //

#define REG_PREALEN                                 0x34 // preamble lenght in nibbles (half-bytes)
#define REG_SYNC3                                   0x36 //
#define REG_SYNC2                                   0x37 //
#define REG_SYNC1                                   0x38 //
#define REG_SYNC0                                   0x39 //
#define REG_HEAD3                                   0x3A //
#define REG_HEAD2                                   0x3B //
#define REG_HEAD1                                   0x3C //
#define REG_HEAD0                                   0x3D //
#define REG_PKTLEN                                  0x3E // [bytes] packet length

#define REG_TXPOWER                                 0x6D // [3dBm] transmitter power, 7=20dBm, p.32

#define REG_RATE1                                   0x6E // [1Msps/2^8 ] date rate MSB, p.26
#define REG_RATE0                                   0x6F // [1Msps/2^16] date rate LSB
#define REG_MOD1                                    0x70 // modulation control
#define REG_MOD2                                    0x71
#define REG_DEV                                     0x72 // [625Hz] frequency deviation

#define REG_FREQOFS0                                0x73 //
#define REG_FREQOFS1                                0x74 //
#define REG_FREQ2                                   0x75 // [10MHz] band select, p.22
#define REG_FREQ1                                   0x76 // [10MHz/64000] carrier freq. MSB
#define REG_FREQ0                                   0x77 // [10MHz/64000] carrier freq. LSB

#define REG_HOPCHAN                                 0x79 // hopping channel select
#define REG_HOPSTEP                                 0x7A // [10kHz] hopping step

#define REG_FIFO                                    0x7F

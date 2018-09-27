#ifndef __FREQPLAN_H__
#define __FREQPLAN_H__

#include <stdint.h>

//#define TEST_PAW_ON_NICERF_SV610_FW466

enum
{
  RF_BAND_AUTO = 0,
  RF_BAND_EU   = 1, /* 868.4 MHz band */
  RF_BAND_US   = 2, /* 915 MHz band */
  RF_BAND_AU   = 3, /* 921 MHz band */
  RF_BAND_NZ   = 4, /* 869.250 MHz band */
  RF_BAND_RU   = 5, /* 868.8 MHz band */
  RF_BAND_CN   = 6, /* 470 MHz band */
  RF_BAND_UK   = 7, /* 869.52 MHz band */
  RF_BAND_IN   = 8  /* 866.0 MHz band */
};

class FreqPlan
{ public:
   uint8_t  Plan;        // 1=Europe, 2=USA/Canada, 3=Australia/Chile, 4=New Zealand
   uint8_t  Channels;    // number of channels
   uint32_t BaseFreq;    // [Hz] base channel (#0) frequency
   uint32_t ChanSepar;   // [Hz] channel spacing
   uint8_t  MaxTxPower;  // max. EIRP in dBm
   static const uint8_t MaxChannels=65;

  public:
   void setPlan(uint8_t NewPlan=0) // preset for a given frequency plan
   {  Plan=NewPlan;

      switch (Plan)
      {
        case RF_BAND_US:
          { BaseFreq=902200000; ChanSepar=400000; Channels=65; MaxTxPower = 30; } // USA, 902-928 MHz
          break;
        case RF_BAND_AU:
          { BaseFreq=917000000; ChanSepar=400000; Channels=24; MaxTxPower = 30; } // Australia and South America
          break;
        case RF_BAND_NZ:
          { BaseFreq=869250000; ChanSepar=200000; Channels= 1; MaxTxPower = 10; } // New Zealand
          break;
        case RF_BAND_RU:
          { BaseFreq=868800000; ChanSepar=200000; Channels= 1; MaxTxPower = 14; } // Russia
          break;
        case RF_BAND_CN:
          { BaseFreq=470100000; ChanSepar=200000; Channels= 1 /* 18 */; MaxTxPower = 17; } // China, 470-473.6 MHz
          break;
        case RF_BAND_UK:
#if !defined(TEST_PAW_ON_NICERF_SV610_FW466)
          { BaseFreq=869525000; ChanSepar=200000; Channels= 1; MaxTxPower = 27; } // PilotAware (UK)0
#else
          { BaseFreq=869920000; ChanSepar=200000; Channels= 1; MaxTxPower = 27; } // Test PAW on NiceRF SV6X0
#endif
          break;
        case RF_BAND_IN:
          { BaseFreq=866000000; ChanSepar=200000; Channels= 1; MaxTxPower = 30; } // India
          break;
        case RF_BAND_EU:
        default:
          { BaseFreq=868200000; ChanSepar=200000; Channels= 2; MaxTxPower = 14; } // Europe
          break;
      }
   }

   void setPlan(int32_t Latitude, int32_t Longitude)
   { setPlan(calcPlan(Latitude, Longitude)); }

   const char *getPlanName(void) { return getPlanName(Plan); }

   static const char *getPlanName(uint8_t Plan)
   { static const char *Name[9] = { "Default", "Europe/Africa",
       "USA/Canada", "Australia/South America", "New Zealand",
       "Russia", "China", "PilotAware (UK)", "India" } ;
     if(Plan>RF_BAND_IN) return 0;
     return Name[Plan]; }

   uint8_t getChannel  (uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const // OGN-tracker or FLARM, UTC time, slot: 0 or 1
   { if(Channels<=1) return 0;                                         // if single channel (New Zealand) return channel #0
     if(Plan>=2)                                                       // if USA/Canada or Australia/South America
     { uint8_t Channel = FreqHopHash((Time<<1)+Slot) % Channels;       // Flarm hopping channel
       if(OGN)                                                         // for OGN tracker
       { if(Slot) { uint8_t Channel1=FreqHopHash((Time<<1)) % Channels; // for 2nd slot choose a channel close to the 1st slot
                            Channel1++; if(Channel1>=Channels) Channel1-=2; // 
                    uint8_t Channel2=Channel1+1; if(Channel2>=Channels) Channel2-=2;
                    if(Channel2==Channel) Channel=Channel1;            // avoid being on same chanel as Flarm
                                    else  Channel=Channel2; }
             else { Channel++; if(Channel>=Channels) Channel-=2; }     // for 1st slot choose a higher channel (unless already highest, then choose a lower one)
       }
       return Channel; }                                               // return 0..Channels-1 for USA/CA or Australia.
     return Slot^OGN; }                                                // if Europe/South Africa: return 0 or 1 for EU freq. plan

   uint32_t getChanFrequency(int Channel) const { return BaseFreq+ChanSepar*Channel; }

   uint32_t getFrequency(uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const
   { uint8_t Channel=getChannel(Time, Slot, OGN); return BaseFreq+ChanSepar*Channel; } // return frequency [Hz] for given UTC time and slot

   uint8_t static calcPlan(int32_t Latitude, int32_t Longitude) // get the frequency plan from Lat/Lon: 1 = Europe + Africa, 2 = USA/CAnada, 3 = Australia + South America, 4$
   { if( (Longitude>=(-20*600000)) && (Longitude<=(60*600000)) ) return RF_BAND_EU; // between -20 and 60 deg Lat => Europe + Africa: 868MHz band
     if( Latitude<(20*600000) )                                            // below 20deg latitude
     { if( ( Longitude>(164*600000)) && (Latitude<(-30*600000)) && (Latitude>(-48*600000)) ) return RF_BAND_NZ;  // => New Zealand
       return RF_BAND_AU; }                                                // => Australia + South America: upper half of 915MHz band
     return RF_BAND_US; }                                                  // => USA/Canada: full 915MHz band

  private:
   static uint32_t FreqHopHash(uint32_t Time)
   { Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16); }

} ;

#endif // __FREQPLAN_H__

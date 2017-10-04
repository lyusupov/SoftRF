#ifndef __FREQPLAN_H__
#define __FREQPLAN_H__

#include <stdint.h>

class FreqPlan
{ public:
   uint8_t  Plan;        // 1=Europe, 2=USA/Canada, 3=Australia/Chile, 4=New Zeeland
   uint8_t  Channels;    // number of channels
   uint32_t BaseFreq;    // [Hz] base channel (#0) frequency
   uint32_t ChanSepar;   // [Hz] channel spacing
   static const uint8_t MaxChannels=65;

  public:
   void setPlan(uint8_t NewPlan=0) // preset for a given frequency plan
   { Plan=NewPlan;
          if(Plan==2) { BaseFreq=902200000; ChanSepar=400000; Channels=65; } // USA
     else if(Plan==3) { BaseFreq=917000000; ChanSepar=400000; Channels=24; } // Australia and South America
     else if(Plan==4) { BaseFreq=869250000; ChanSepar=200000; Channels= 1; } // New Zeeland
     else             { BaseFreq=868200000; ChanSepar=200000; Channels= 2; } // Europe
   }

   void setPlan(int32_t Latitude, int32_t Longitude)
   { setPlan(calcPlan(Latitude, Longitude)); }

   const char *getPlanName(void) { return getPlanName(Plan); }

   static const char *getPlanName(uint8_t Plan)
   { static const char *Name[5] = { "Default", "Europe/Africa", "USA/Canada", "Australia/South America", "New Zeeland" } ;
     if(Plan>4) return 0;
     return Name[Plan]; }

   uint8_t getChannel  (uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const // OGN-tracker or FLARM, UTC time, slot: 0 or 1
   { if(Channels<=1) return 0;                                         // if single channel (New Zeeland) return channel #0
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
   { if( (Longitude>=(-20*600000)) && (Longitude<=(60*600000)) ) return 1; // between -20 and 60 deg Lat => Europe + Africa: 868MHz band
     if( Latitude<(20*600000) )                                            // below 20deg latitude
     { if( ( Longitude>(164*600000)) && (Latitude<(-30*600000)) && (Latitude>(-48*600000)) ) return 4;  // => New Zeeland
       return 3; }                                                         // => Australia + South America: upper half of 915MHz band
     return 2; }                                                           // => USA/Canada: full 915MHz band

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

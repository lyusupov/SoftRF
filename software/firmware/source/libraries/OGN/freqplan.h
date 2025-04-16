#ifndef __FREQPLAN_H__
#define __FREQPLAN_H__

#include <stdint.h>
#include <protocol.h>

//#define TEST_PAW_ON_NICERF_SV610_FW466

enum
{
  RF_BAND_AUTO = 0,  /* Deprecated - treated as EU */
  RF_BAND_EU   = 1,  /* 868.2 MHz band */
  RF_BAND_US   = 2,  /* 915 MHz band */
  RF_BAND_AU   = 3,  /* 921 MHz band */
  RF_BAND_NZ   = 4,  /* 869.250 MHz band */
  RF_BAND_RU   = 5,  /* 868.8 MHz band */
  RF_BAND_CN   = 6,  /* 470 MHz band */
  RF_BAND_UK   = 7,  /* 869.52 MHz band. Deprecated - treated as EU */
  RF_BAND_IN   = 8,  /* 866.0 MHz band */
  RF_BAND_IL   = 9,  /* 916.2 MHz band */
  RF_BAND_KR   = 10, /* 920.9 MHz band */
  RF_BAND_RSVD = 11, /* reserved */
  RF_BAND_COUNT
};

class FreqPlan
{ public:
   uint8_t  Protocol;    // 0=Legacy, 1=OGNTP, 2=P3I, 3=ADS-B (1090ES), 4=UAT, 5=FANET, 6=APRS, 7=ADS-L (SRD860)
   uint8_t  Plan;        // 1=Europe, 2=USA/Canada, 3=Australia/Chile, 4=New Zealand, ...
   uint8_t  Channels;    // number of channels
   uint32_t BaseFreq;    // [Hz] base channel (#0) frequency
   uint32_t ChanSepar;   // [Hz] channel spacing
   int8_t   MaxTxPower;  // max. EIRP in dBm
   uint8_t  Bandwidth;   // FANET only
   static const uint8_t MaxChannels=65;

  public:
   void setPlan(uint8_t NewPlan=0, uint8_t NewProto=RF_PROTOCOL_OGNTP) // preset for a given frequency plan
   { Protocol=NewProto; Plan=NewPlan;

     switch (Protocol)
     {
      case RF_PROTOCOL_P3I:
        switch (Plan)
        {
          case RF_BAND_RSVD:
            { BaseFreq=2450000000; ChanSepar= 200000; Channels= 1; MaxTxPower = 30; } // reserved
//            { BaseFreq=2000000000; ChanSepar= 200000; Channels= 1; MaxTxPower = 30; } /* S-band */
            break;
          case RF_BAND_EU:
          default:
            { BaseFreq= 869525000; ChanSepar= 200000; Channels= 1; MaxTxPower = 27; } // Europe
#if defined(TEST_PAW_ON_NICERF_SV610_FW466)
              BaseFreq= 869920000; // Test PAW on NiceRF SV6X0
#endif
            break;
        }
        break;
      case RF_PROTOCOL_ADSB_1090:
        { BaseFreq=1090000000; ChanSepar=2000000; Channels= 1; MaxTxPower = -10; }
        break;
      case RF_PROTOCOL_ADSB_UAT:
        { BaseFreq= 978000000; ChanSepar=2000000; Channels= 1; MaxTxPower = -10; }
        break;
      case RF_PROTOCOL_FANET:
        { ChanSepar=400000; Channels= 1; }
        switch (Plan)
        {
          case RF_BAND_US:
          case RF_BAND_AU:
          case RF_BAND_NZ: /* ISM 915-928 MHz, https://www.rsm.govt.nz/assets/Uploads/documents/pibs/table-of-radio-spectrum-usage-in-new-zealand-pib-21.pdf */
          case RF_BAND_CN: /* ? */
            BaseFreq   = 920800000;
            Bandwidth  = RF_RX_BANDWIDTH_SS_250KHZ; // BW500
            MaxTxPower = 15 /* LoRaWAN: 30 */;
            break;
          case RF_BAND_IN:
            BaseFreq   = 866200000;
            Bandwidth  = RF_RX_BANDWIDTH_SS_125KHZ; // BW250
            MaxTxPower = 14 /* LoRaWAN: 30 */;
            break;
          case RF_BAND_IL:
            BaseFreq   = 918500000;
            Bandwidth  = RF_RX_BANDWIDTH_SS_62KHZ;  // BW125
            MaxTxPower = 15;
            break;
          case RF_BAND_KR:
            BaseFreq   = 923200000;
            Bandwidth  = RF_RX_BANDWIDTH_SS_62KHZ;  // BW125
            MaxTxPower = 15 /* LoRaWAN: 23 */;
            break;
          case RF_BAND_RSVD:
            BaseFreq   = 2450000000;
//            BaseFreq   = 2000000000; /* S-band */
            Bandwidth  = RF_RX_BANDWIDTH_SS_250KHZ; // BW500
            MaxTxPower = 30;
            break;
          case RF_BAND_EU:
          case RF_BAND_RU:
          default:
            BaseFreq   = 868200000;
            Bandwidth  = RF_RX_BANDWIDTH_SS_125KHZ; // BW250
            MaxTxPower = 14;
            break;
        }
        break;
      case RF_PROTOCOL_APRS: /* VHF */
        { ChanSepar = 10000; Channels = 1; MaxTxPower = 30; }
        switch (Plan)
        {
          case RF_BAND_CN: /* + Hong Kong, Taiwan */
            { BaseFreq = 144640000; }
            break;
          case RF_BAND_KR:
            { BaseFreq = 144620000; }
            break;
          case RF_BAND_AU: /* + Tasmania */
            { BaseFreq = 145175000; }
            break;
          case RF_BAND_NZ:
            { BaseFreq = 144575000; }
            break;
          case RF_BAND_US: /* + Canada, Chile, Indonesia, Singapore, Malaysia,
                            * Mexico, Dominican Republic, Puerto Rico,
                            * Trinidad & Tabago, Columbia */
            { BaseFreq = 144390000; }
            break;
          case RF_BAND_EU: /* + UK, Ireland, Iceland, South Africa,
                            * Azores, Costa Rica, Lebanon, Senegal */
          case RF_BAND_RU:
          case RF_BAND_IN: /* TBD */
          case RF_BAND_IL:
          default:
            { BaseFreq = 144800000; }
            break;
        /*
         * Japan:       144.660 MHz
         */
        }
        /*
         * UHF
         * ------------------------
         * Europe:      433.800 MHz
         * Australia:   439.100 MHz
         * US:          445.925 MHz
         * New Zealand: 432.575 MHz
         */
        break;
      case RF_PROTOCOL_LEGACY:
      case RF_PROTOCOL_OGNTP:
      case RF_PROTOCOL_ADSL_860:
      default:
        switch (Plan)
        {
          case RF_BAND_US:
            { BaseFreq= 902200000; ChanSepar=400000; Channels=65; MaxTxPower = 30; } // USA, 902-928 MHz
            break;
          case RF_BAND_AU:
            { BaseFreq= 917000000; ChanSepar=400000; Channels=24; MaxTxPower = 30; } // Australia and South America
            break;
          case RF_BAND_NZ: /* SRD 868-870 MHz, https://www.rsm.govt.nz/assets/Uploads/documents/pibs/table-of-radio-spectrum-usage-in-new-zealand-pib-21.pdf */
            { BaseFreq= 869250000; ChanSepar=200000; Channels= 1; MaxTxPower = 10; } // New Zealand
            break;
          case RF_BAND_RU:
            { BaseFreq= 868800000; ChanSepar=200000; Channels= 1; MaxTxPower = 20; } // Russia
            break;
          case RF_BAND_CN:
            { BaseFreq= 470100000; ChanSepar=200000; Channels= 1 /* 18 */; MaxTxPower = 17; } // China, 470-473.6 MHz
            break;
          case RF_BAND_IN:
            { BaseFreq= 866000000; ChanSepar=200000; Channels= 1; MaxTxPower = 30; } // India
            break;
          case RF_BAND_IL:
            { BaseFreq= 916200000; ChanSepar=200000; Channels= 1; MaxTxPower = 30; } // Israel
            break;
          case RF_BAND_KR:
            { BaseFreq= 920900000; ChanSepar=200000; Channels= 1; MaxTxPower = 23; } // South Korea
            break;
          case RF_BAND_RSVD:
            { BaseFreq=2450000000; ChanSepar=200000; Channels= 1; MaxTxPower = 30; } // reserved
//            { BaseFreq=2000000000; ChanSepar=200000; Channels= 1; MaxTxPower = 30; } /* S-band */
            break;
          case RF_BAND_EU:
          default: /* AUTO, UK */
            { BaseFreq= 868200000; ChanSepar=200000; Channels= 2; MaxTxPower = 14; } // Europe
            break;
        }
        break;
      }
   }

   void setPlan(int32_t Latitude, int32_t Longitude, uint8_t NewProto=RF_PROTOCOL_OGNTP)
   { setPlan(calcPlan(Latitude, Longitude), NewProto); }

   const char *getPlanName(void) { return getPlanName(Plan); }

   static const char *getPlanName(uint8_t Plan)
   { static const char *Name[RF_BAND_COUNT] = { "Default", "Europe/Africa",
       "USA/Canada", "Australia/South America", "New Zealand",
       "Russia", "China", "PilotAware (UK)", "India", "Israel",
       "South Korea", "Reserved" } ;
     if(Plan >= RF_BAND_COUNT) return 0;
     return Name[Plan]; }

   uint8_t getChannel  (uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const // OGN-tracker or FLARM, UTC time, slot: 0 or 1
   { if(Channels<=1) return 0;                                         // if single channel (New Zealand) return channel #0
     if(Plan>=2)                                                       // if USA/Canada or Australia/South America
     { uint8_t Channel = FreqHopHash((Time<<1)+Slot) % Channels;       // Flarm hopping channel
       if(OGN)                                                              // OGN Tracker
       { if(Slot)                                                           // for 2nd slot
         { uint8_t Channel2 = FreqHopHash((Time<<1)) % Channels;            // use same as Flarm in the 1st slot
           if(Channel2==Channel) { Channel++; if(Channel>=Channels) Channel-=2; } // but if same then Flarm in the 2nd slot
                            else Channel=Channel2;
         }
         else { Channel++; if(Channel>=Channels) Channel-=2; }              // for 1st slot choose a higher channel (unless already highest, then choose a lower one)
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

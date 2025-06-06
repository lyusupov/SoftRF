#if !defined(_RADIOLIB_EXTERNAL_RADIO_H)
#define _RADIOLIB_EXTERNAL_RADIO_H

#include "../../TypeDef.h"
#include "../../Module.h"

#include "../PhysicalLayer/PhysicalLayer.h"

/*!
  \class ExternalRadio
  \brief Class to interface with external radio hardware.
*/
class ExternalRadio: public PhysicalLayer {
  public:
    #if defined(RADIOLIB_BUILD_ARDUINO)
    /*!
      \brief Default constructor.
      \param pin Output pin when using direct transmission, defaults to unused pin.
    */
    ExternalRadio(uint32_t pin = RADIOLIB_NC); // cppcheck-suppress noExplicitConstructor
    #endif
    
    /*!
      \brief Default constructor.
      \param hal Pointer to the hardware abstraction layer to use.
      \param pin Output pin when using direct transmission, defaults to unused pin.
    */
    ExternalRadio(RadioLibHal *hal, uint32_t pin = RADIOLIB_NC); // cppcheck-suppress noExplicitConstructor

    /*!
      \brief Copy constructor.
      \param ext ExternalRadio instance to copy.
    */
    ExternalRadio(const ExternalRadio& ext);
    
    /*!
      \brief Overload for assignment operator.
      \param ext rvalue ExternalRadio.
    */
    ExternalRadio& operator=(const ExternalRadio& ext);

    /*!
      \brief Default destructor.
    */
    ~ExternalRadio();

    /*!
      \brief Method to retrieve pointer to the underlying Module instance.
      \returns Pointer to the Module instance.
    */
    Module* getMod() override;

    /*!
      \brief Dummy implementation overriding PhysicalLayer.
      \param freqDev Ignored.
      \returns \ref status_codes
    */
    int16_t setFrequencyDeviation(float freqDev) override;

    /*!
      \brief Dummy implementation overriding PhysicalLayer.
      \param sh Ignored.
      \returns \ref status_codes
    */
    int16_t setDataShaping(uint8_t sh) override;

    /*!
      \brief Dummy implementation overriding PhysicalLayer.
      \param encoding Ignored.
      \returns \ref status_codes
    */
    int16_t setEncoding(uint8_t encoding) override;

    /*!
      \brief Direct transmission to drive external radio.
      \param frf "Frequency" to control the output pin. If the frequency is higher than the one sent previously,
      the output pin will be set to logic high. Otherwise it will be set to logic low.
      \returns \ref status_codes
    */
    int16_t transmitDirect(uint32_t frf = 0) override;
  
  private:
    Module* mod;
    uint32_t prevFrf;
};

#endif
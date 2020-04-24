// project-specific definitions for otaa sensor

// even if you have lora_project_config.h in your sketch directory.
// some define are not used in all stack at compilation time, 
// even adding #include "lora_project_config.h" at top of the sketch
// so put all in this file and it works all time

//#define CFG_us915 1
#define CFG_eu868 1
#define CFG_sx1276_radio 1

// Use real interrupts
//#define LMIC_USE_INTERRUPTS

// Set SPI 8MHz
//#define LMIC_SPI_FREQ 8000000

// Enable Debug (uncomment both lines)
//#define LMIC_DEBUG_LEVEL 2
//#define LMIC_PRINTF_TO Serial

// Use Original AES (More space but quicker)
//#define USE_ORIGINAL_AES




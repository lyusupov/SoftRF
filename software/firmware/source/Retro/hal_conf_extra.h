/*
 * Override NUCLEO-L073RZ default Serial (2) assignment with S76G one's (1)
 */
#if defined(ARDUINO_NUCLEO_L073RZ)
#if SERIAL_UART_INSTANCE == 2

#undef  SERIAL_UART_INSTANCE
#undef  PIN_SERIAL_RX
#undef  PIN_SERIAL_TX

#define SERIAL_UART_INSTANCE    1
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

#endif /* SERIAL_UART_INSTANCE */
#endif /* ARDUINO_NUCLEO_L073RZ */

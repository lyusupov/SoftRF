// simulator.h
// Lets Arduino RadioHead sketches run within a simulator on Linux as a single process
// Copyright (C) 2014 Mike McCauley
// $Id: simulator.h,v 1.4 2015/08/13 02:45:47 mikem Exp $

#ifndef simulator_h
#define simulator_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Equivalent types for common Arduino types like uint8_t are in stdint.h

// Access to some globals
// Command line args passed to the process.
extern int    _simulator_argc;
extern char** _simulator_argv;

// Definitions for various Arduino functions
extern void delay(unsigned long ms);
extern unsigned long millis();
extern long random(long to);
extern long random(long from, long to);

// Equavalent to HardwareSerial in Arduino
// but outputs to stdout
class SerialSimulator
{
public:
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

    // TODO: move these from being inlined
    void begin(int baud) {}

    size_t println(const char* s)
    {
	print(s);
	return printf("\n");
    }
    size_t print(const char* s)
    {
	return printf("%s", s); // This style prevent warnings from [-Wformat-security]
    }
    size_t print(unsigned int n, int base = DEC)
    {
	if (base == DEC)
	    return printf("%d", n);
	else if (base == HEX)
	    return printf("%02x", n);
	else if (base == OCT)
	    return printf("%o", n);
	// TODO: BIN
	else
	    return 0;
    }
    size_t print(char ch)
    {
        return printf("%c", ch);
    }
    size_t println(char ch)
    {
        return printf("%c\n", ch);
    }
    size_t print(unsigned char ch, int base = DEC)
    {
	return print((unsigned int)ch, base);
    }
    size_t println(unsigned char ch, int base = DEC)
    {
	print((unsigned int)ch, base);
	return printf("\n");
    }

};

// Global instance of the Serial output
extern SerialSimulator Serial;

#endif

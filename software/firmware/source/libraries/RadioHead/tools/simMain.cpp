// main.cpp
// Lets Arduino RadioHead sketches run within a simulator on Linux as a single process
// Copyright (C) 2014 Mike McCauley
// $Id: simMain.cpp,v 1.3 2020/08/05 04:32:19 mikem Exp mikem $

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_UNIX) 

#include <stdio.h>
#include <RHutil/simulator.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>

SerialSimulator Serial;

// Functions we expect to find in the sketch
extern void setup();
extern void loop();

// Millis at the start of the process
unsigned long start_millis;

int    _simulator_argc;
char** _simulator_argv;

// Returns milliseconds since beginning of day
unsigned long time_in_millis()
{    
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    unsigned long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caclulate milliseconds
    return milliseconds;
}

// Run the Arduino standard functions in the main loop
int main(int argc, char** argv)
{
    // Let simulated program have access to argc and argv
    _simulator_argc = argc;
    _simulator_argv = argv;
    start_millis = time_in_millis();
    // Seed the random number generator
    srand(getpid() ^ (unsigned) time(NULL)/2);
    setup();
    while (1)
	loop();
}

void delay(unsigned long ms)
{
    usleep(ms * 1000);
}

// Arduino equivalent, milliseconds since process start
unsigned long millis()
{
    return time_in_millis() - start_millis;
}

long random(long from, long to)
{
    return from + (random() % (to - from));
}

long random(long to)
{
    return random(0, to);
}

#endif

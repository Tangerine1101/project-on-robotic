#ifndef SERIAL_COMMANDS
#define POSITION    1
#define NONE    0
#include <Arduino.h>

#define n  4
double argument[n];
//char commandIndexxxs[4];
unsigned int command();
void checkCommand();
void clearArgument();
#endif
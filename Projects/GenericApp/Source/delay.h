#ifndef __dalay_H
#define __dalay_H
#include <ioCC2530.h>
#define NOP()  asm("NOP")

extern void halMcuWaitUs(unsigned int usec);
extern void halMcuWaitMs(unsigned int msec);
#endif
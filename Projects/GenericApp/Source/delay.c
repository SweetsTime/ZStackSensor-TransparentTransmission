#include "delay.h"
void halMcuWaitUs(unsigned int usec);
void halMcuWaitMs(unsigned int msec);
void halMcuWaitUs(unsigned int usec)
{ usec>>= 1;
    while(usec--)
    {
        NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
        NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
    }
}
void halMcuWaitMs(unsigned int msec)
{ while(msec--)
        halMcuWaitUs(1000);
}
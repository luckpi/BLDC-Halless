#include "debug.h"
#include "uart.h"
#include "common.h"
void Debug()
{
    switch (error_code)
    {
    case normal:
        Ps("\nNomal", NONE);
        break;
    case overcurrent:
        Ps("\nOver Current", NONE);
        break;
    case overvoltage:
        Ps("\nOver Voltage", NONE);
        break;
    case undervoltage:
        Ps("\nUnder Voltage", NONE);
        break;
    case hallerror:
        Ps("\nHall Error", NONE);
        break;
    case stall:
        Ps("\nStall", NONE);
        break;
    default:
        Ps("\nNomal", NONE);
        break;
    }
}
#include "mbed.h"

#ifndef _REMOTE_H
#define _REMOTE_H

class Remote {
public:
    Remote(PinName pin, int channels);
    int *get();

    int bufferLength;
    volatile int position;
    volatile int *data;

private:
    void trigger();

    InterruptIn interrupt;
    Timer timer;
};

#endif

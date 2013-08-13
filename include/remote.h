#include "mbed.h"

#ifndef _REMOTE_H
#define _REMOTE_H

class Remote {
public:
    Remote(PinName pin, int ch);
    int *get();

    int channels;
    int bufferLength;
    volatile int position;
    volatile int *data;

    static const int min = 1100;
    static const int max = 1900;
    static const int fail = 900;

private:
    void trigger();

    InterruptIn interrupt;
    Timer timer;
};

#endif

#include "remote.h"
#include <stdlib.h>

Remote::Remote(PinName pin, int ch) : interrupt(pin) {
    channels = ch;
    bufferLength = channels + 1;
    position = 0;
    data = (int *)calloc(bufferLength, sizeof(int));

    interrupt.rise(this, &Remote::trigger);
    timer.reset();
    timer.start();
}

int *Remote::get() {
    int *buff = (int *)calloc(channels, sizeof(int));

    // find maximum --> signal pause
    int max = 0, pos = 0;
    for (int i = 0; i < bufferLength; i++) {
        if (data[i] > max) {
            max = data[i];
            pos = i;
        }
    }

    int source = pos + 1, target = 0;
    if (source >= bufferLength)
            source = 0;

    for (; target < channels; target++) {
        buff[target] = data[source++];
        if (source >= bufferLength)
            source = 0;
    }

    return buff;
}

void Remote::trigger() {
    timer.stop();
    data[position++] = timer.read_us();
    if (position >= bufferLength)
        position = 0;
    timer.reset();
    timer.start();
}

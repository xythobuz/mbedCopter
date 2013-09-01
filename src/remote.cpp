/*
 * remote.cpp
 *
 * Copyright (c) 2013, Thomas Buck <xythobuz@xythobuz.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
    if (buff == NULL)
        return NULL;

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

/*
 * sensors.h
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
#include "mbed.h"

#ifndef _SENSORS_H
#define _SENSORS_H

class Gyro {
public:
    enum Range {
        r250DPS,
        r500DPS,
        r2000DPS,
    };

    Gyro(I2C *i);
    int init(Range r); // 0 on success
    int read(float *v); // 0 on success

    static const uint8_t address = 0xD6;
    static const uint8_t register1 = 0x20;
    static const uint8_t register4 = 0x23;
    static const uint8_t registerOut = 0x28;

private:
    I2C *i2c;
    Range range;
};


class Acc {
public:
    enum Range {
        r2G,
        r4G,
        r8G,
        r16G,
    };

    Acc(I2C *i);
    int init(Range r); // 0 on success
    int read(float *v); // 0 on success

    static const uint8_t address = 0x32;
    static const uint8_t register1 = 0x20;
    static const uint8_t register4 = 0x23;
    static const uint8_t registerOut = 0x28;

private:
    I2C *i2c;
    Range range;
};

#endif
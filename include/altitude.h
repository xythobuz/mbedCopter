/*
 * altitude.h
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

#ifndef _ALTITUDE_H
#define _ALTITUDE_H

class Altitude {
public:
    Altitude(I2C *i);
    int init(); // 0 on success

    // temperature in units of 0.1 deg C
    // pressure in units of Pa
    int read(uint16_t *temperature, int32_t *pressure); // 0 on success

    float calculateAltitude(int32_t pressure);

    static const uint8_t address = 0xEE;

    #define ALT_ULTRALOWPOW 0
    #define ALT_STANDARD 1
    #define ALT_HIGHRES 2
    #define ALT_ULTRAHIGHRES 3
    static const uint8_t OSS = ALT_STANDARD;

private:
    int read(uint8_t add, int16_t *data); // 0 on success
    int readCalibration(); // 0 on success
    int readUT(uint16_t *data); // 0 on success
    int readUP(uint32_t *data); // 0 on success
    int readTemperature(uint16_t *data); // 0 on success
    int readPressure(int32_t *data); // 0 on success

    I2C *i2c;
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    int32_t b5;
};

#endif
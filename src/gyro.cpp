/*
 * gyro.cpp
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
#include "sensors.h"
#include "errors.h"

Gyro::Gyro(I2C *i) {
    i2c = i;
}

int Gyro::init(Gyro::Range r) {
    char data[2];
    uint8_t v;

    range = r;
    switch (range) {
        case r250DPS:
            v = 0x00;
            break;
        case r500DPS:
            v = 0x10;
            break;
        case r2000DPS:
            v = 0x20;
            break;
        default:
            return ERR_ARGUMENT;
    }
    data[0] = register1; data[1] = 0x0F;
    if (i2c->write(address, data, 2, false))
        return ERR_GYRO_WRITE;

    data[0] = register4; data[1] = v;
    if (i2c->write(address, data, 2, false))
        return ERR_GYRO_WRITE;
    return SUCCESS;
}

int Gyro::read(float *v) {
    char data[6];

    if (v == NULL)
        return ERR_ARGUMENT;

    data[0] = registerOut | 0x80; // Auto-Increment
    if (i2c->write(address, data, 1, true))
        return ERR_GYRO_WRITE;

    if (i2c->read(address, data, 6, false))
        return ERR_GYRO_READ;

    uint8_t xl = data[0];
    uint8_t xh = data[1];
    uint8_t yl = data[2];
    uint8_t yh = data[3];
    uint8_t zl = data[4];
    uint8_t zh = data[5];
    int16_t x = *(int8_t *)(&xh);
    x *= (1 << 8);
    x |= xl;
    int16_t y = *(int8_t *)(&yh);
    y *= (1 << 8);
    y |= yl;
    int16_t z = *(int8_t *)(&zh);
    z *= (1 << 8);
    z |= zl;

    switch (range) {
        case r250DPS:
            v[0] = (((float)x) * 250 / 0x8000);
            v[1] = (((float)y) * 250 / 0x8000);
            v[2] = (((float)z) * 250 / 0x8000);
            break;
        case r500DPS:
            v[0] = (((float)x) * 500 / 0x8000);
            v[1] = (((float)y) * 500 / 0x8000);
            v[2] = (((float)z) * 500 / 0x8000);
            break;
        case r2000DPS:
            v[0] = (((float)x) * 2000 / 0x8000);
            v[1] = (((float)y) * 2000 / 0x8000);
            v[2] = (((float)z) * 2000 / 0x8000);
            break;
        default:
            return ERR_ARGUMENT;
    }

    return SUCCESS;
}
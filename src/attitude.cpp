/*
 * attitude.cpp
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
#include "attitude.h"
#include "errors.h"

#include <math.h>

#define TODEG(x) (((x) * 180.0f) / 3.14159f) /**< Convert Radians to Degrees */

#define GYRO 0
#define ACC 1
#define X 0
#define Y 1
#define Z 2

Attitude::Attitude(Gyro *g, Acc *a, int frequency) : pitchFilter(frequency), rollFilter(frequency) {
    gyro = g;
    acc = a;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
}

error_t Attitude::calculate() {
    float data[2][3];

    if (error_t error = gyro->read(data[GYRO]))
        return error;

    if (error_t error = acc->read(data[ACC]))
        return error;

    float accPitch = TODEG( atanf(data[ACC][Y] / hypotf(data[ACC][X], data[ACC][Z])) );
    float accRoll = TODEG( atanf(data[ACC][X] / hypotf(data[ACC][Y], data[ACC][Z])) );

    pitchFilter.innovate(accPitch, data[GYRO][X]);
    rollFilter.innovate(accRoll, data[GYRO][Y]);

    pitch = pitchFilter.x1;
    roll = rollFilter.x1;

    return SUCCESS;
}

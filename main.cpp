/*
 * main.cpp
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
#include "sensors.h"
#include "attitude.h"
#include "remote.h"
#include "altitude.h"
#include "errors.h"
#include "pid.h"
#include "motor.h"

#include <stdlib.h>

#define ERROR_ANIMATION_x 150 // IOOI <-> OIIO --> Used by Bootloader + Runtime
#define ERROR_ANIMATION_1 165 // IOIO <-> OIOI --> Gyro Init Error
#define ERROR_ANIMATION_2 135 // IOOO <-> OIII --> Acc Init Error
#define ERROR_ANIMATION_3 75  // OIOO <-> IOII --> Altitude Init Error
#define ERROR_ANIMATION_4 45  // OOIO <-> IIOI --> Attitude Handler Error
#define ERROR_ANIMATION_5 30  // OOOI <-> IIIO --> Too slow to keep up with calculations

const static int remoteChannels = 6;
const static float defaultP = 5.0;
const static float defaultI = 0.0015;
const static float defaultD = -13.0;
const static float defaultMin = -255.0;
const static float defaultMax = 255.0;
const static int attitudeFrequency = 100; // Attitude and Control Frequency in Hz

int attitudeFlag = 0; // Attitude Execution Flag

DigitalOut statusLED[] = { DigitalOut(LED1), DigitalOut(LED2), DigitalOut(LED3), DigitalOut(LED4) };
Serial pc(USBTX, USBRX);
Ticker ticker;
I2C i2c(p28, p27);

Gyro gyro(&i2c);
Acc acc(&i2c);
Attitude attitude(&gyro, &acc, attitudeFrequency);
Altitude altitude(&i2c);
Remote remote(p6, remoteChannels);
PID rollPID(defaultP, defaultI, defaultD, defaultMin, defaultMax);
PID pitchPID(defaultP, defaultI, defaultD, defaultMin, defaultMax);
Motor motor(&i2c);

void display(uint8_t anim) {
    statusLED[0] = ((anim & 0x08) >> 3);
    statusLED[1] = ((anim & 0x04) >> 2);
    statusLED[2] = ((anim & 0x02) >> 1);
    statusLED[3] = (anim & 0x01);
}

void errorLoop(uint8_t anim) {
    while(1) {
        display(anim >> 4);
        wait(0.25);
        display(anim);
        wait(0.25);
    }
}

void attitudeHandler() {
    attitudeFlag++;
}

int main() {
    display(0x0F); // All LEDs on

    i2c.frequency(400 * 1000);
    pc.baud(38400);

    if (int error = gyro.init(Gyro::r2000DPS)) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_1);
    }

    if (int error = acc.init(Acc::r8G)) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_2);
    }

    if (int error = altitude.init()) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_3);
    }

    pc.printf("mbedCopter initialized!\n");
    ticker.attach_us(&attitudeHandler, (1000000 / attitudeFrequency));
    display(0);

    while(1) {
        // Flight Control Code
        if (attitudeFlag > 0) {
            if (attitudeFlag > 1) {
                pc.printf("!! Too slow to keep up (%d) !!\n", attitudeFlag - 1);
                //errorLoop(ERROR_ANIMATION_5);
            }
            attitudeFlag = 0;

            if (int error = attitude.calculate()) {
                pc.printf("%s\n", getErrorString(error));
                errorLoop(ERROR_ANIMATION_4);
            }

            // To-Do: Read Remote and set target angles
            float rollDiff = rollPID.execute(0, attitude.roll);
            float pitchDiff = pitchPID.execute(0, attitude.pitch);
        }

        // UART Menu
        if(pc.readable()) {
            switch (pc.getc()) {
            case 'a':
                pc.printf("R %.2f\nP %.2f\nY %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
                break;

            case 'r':
                int *data;
                data = remote.get();
                for (int i = 0; i < remote.channels; i++) {
                    pc.printf("%d: %d --> ", i + 1, data[i]);
                    if (data[i] < remote.fail)
                        pc.printf("Fail!\n");
                    else
                        pc.printf("%d%%\n", (data[i] - remote.min) * 100 / (remote.max - remote.min));
                }
                free(data);
                break;

            case 'p':
                uint16_t temperature;
                int32_t pressure;
                altitude.read(&temperature, &pressure);
                pc.printf("Temp: %.1fC\n", temperature * 0.1f);
                pc.printf("Pres: %dPa\n", pressure);
                pc.printf("Alti: %.2fm\n", altitude.calculateAltitude(pressure));
                break;

            default:
                pc.printf("??\n");
                break;
            }
        }
    }

    return 0;
}

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

#define  MOTOR_PITCH_PLUS 0
#define MOTOR_PITCH_MINUS 2
#define   MOTOR_ROLL_PLUS 1
#define  MOTOR_ROLL_MINUS 3

#define    CHANNEL_SPEED 2
#define     CHANNEL_ROLL 0
#define    CHANNEL_PITCH 1
#define      CHANNEL_YAW 3
#define  CHANNEL_POTLEFT 5
#define CHANNEL_POTRIGHT 4

#define     STICK_POS_MIN -40
#define     STICK_POS_MAX 40
#define YAW_INFLUENCE_MIN -20
#define YAW_INFLUENCE_MAX 20

#define ERROR_ANIMATION_x 150 // IOOI <-> OIIO --> Used by Bootloader + Runtime
#define ERROR_ANIMATION_1 165 // IOIO <-> OIOI --> Gyro Init Error
#define ERROR_ANIMATION_2 135 // IOOO <-> OIII --> Acc Init Error
#define ERROR_ANIMATION_3 75  // OIOO <-> IOII --> Altitude Init Error
#define ERROR_ANIMATION_4 45  // OOIO <-> IIOI --> Attitude Handler Error
#define ERROR_ANIMATION_5 30  // OOOI <-> IIIO --> Too slow to keep up with calculations
#define ERROR_ANIMATION_6 240 // IIII <-> OOOO --> Ran out of Memory
#define ERROR_ANIMATION_7 195 // IIOO <-> OOII --> Motor Send Error

const static int remoteChannels = 6;
const static float defaultP = 0.0;
const static float defaultI = 0.0;
const static float defaultD = 0.0;
const static float defaultMin = -128.0;
const static float defaultMax = 128.0;
const static int attitudeFrequency = 100; // Attitude and Control Frequency in Hz
const static int remoteFrequency = 10; // Remote Read Frequency in Hz

volatile int attitudeFlag = 0; // Attitude Execution Flag
volatile int remoteFlag = 0;

int remoteError = 0;
int baseSpeed = 0;
int rollTarget = 0;
int pitchTarget = 0;
int yawTarget = 0;
int potLeft = 0;
int potRight = 0;

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

template <typename T>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

        // Just to be sure
        motor.set(4, 0);
        motor.send();
    }
}

void attitudeHandler() {
    attitudeFlag++;
}

void remoteHandler() {
    remoteFlag++;
}

template <typename T>
void atomicSet(volatile T *var, T content) {
    __disable_irq();
    *var = content;
    __enable_irq();
}

template <typename T>
T atomicGet(volatile T *var) {
    __disable_irq();
    T tmp = *var;
    __enable_irq();
    return tmp;
}

int main() {
    display(0x0F); // All LEDs on

    i2c.frequency(400000);
    pc.baud(38400);

    if (error_t error = gyro.init(Gyro::r2000DPS)) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_1);
    }

    if (error_t error = acc.init(Acc::r8G)) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_2);
    }

    if (error_t error = altitude.init()) {
        pc.printf("%s\n", getErrorString(error));
        errorLoop(ERROR_ANIMATION_3);
    }

    pc.printf("Init!\n");
    ticker.attach_us(&attitudeHandler, (1000000 / attitudeFrequency));
    ticker.attach_us(&remoteHandler, (1000000 / remoteFrequency));
    display(0);

    while(1) {
        // Remote Control Reading
        if (atomicGet(&remoteFlag) > 0) {
            if (atomicGet(&remoteFlag) > 1) {
                pc.printf("!! Too slow for Remote (%d) !!\n", atomicGet(&remoteFlag) - 1);
                //errorLoop(ERROR_ANIMATION_5);
            }
            atomicSet(&remoteFlag, 0);
            remoteError = 0;
            int *remoteData = remote.get();
            if (remoteData == NULL) {
                pc.printf("No Memory!\n");
                errorLoop(ERROR_ANIMATION_6);
            }
            for (int i = 0; i < remote.channels; i++) {
                if (remoteData[i] < remote.fail)
                    remoteError = 1;
            }
            if (remoteError == 0) {
                // Convert Stick Positions - 1 Degree Resolution!
                baseSpeed = map<int>(remoteData[CHANNEL_SPEED], remote.min, remote.max, 0, 255);
                rollTarget = map<int>(remoteData[CHANNEL_ROLL], remote.min, remote.max, STICK_POS_MIN, STICK_POS_MAX);
                pitchTarget = map<int>(remoteData[CHANNEL_PITCH], remote.min, remote.max, STICK_POS_MIN, STICK_POS_MAX);
                yawTarget = map<int>(remoteData[CHANNEL_YAW], remote.min, remote.max, YAW_INFLUENCE_MIN, YAW_INFLUENCE_MAX);
                potLeft = map<int>(remoteData[CHANNEL_POTLEFT], remote.min, remote.max, 0, 255);
                potRight = map<int>(remoteData[CHANNEL_POTRIGHT], remote.min, remote.max, 0, 255);
            }
            free(remoteData);
        }

        // Flight Control Code
        if (atomicGet(&attitudeFlag) > 0) {
            if (atomicGet(&attitudeFlag) > 1) {
                pc.printf("!! Too slow for Attitude (%d) !!\n", atomicGet(&attitudeFlag) - 1);
                //errorLoop(ERROR_ANIMATION_5);
            }
            atomicSet(&attitudeFlag, 0);

            if (error_t error = attitude.calculate()) {
                pc.printf("%s\n", getErrorString(error));
                errorLoop(ERROR_ANIMATION_4);
            }

            int rollDiff, pitchDiff;
            if (remoteError) {
                // Fail-Safe
                baseSpeed = 0;
                rollDiff = 0;
                pitchDiff = 0;
            } else {
                // Execute PID
                rollDiff = rollPID.execute(rollTarget, attitude.roll);
                pitchDiff = pitchPID.execute(pitchTarget, attitude.pitch);
            }

            // Mix it!
            motor.set(MOTOR_ROLL_PLUS, (baseSpeed + rollDiff) + (yawTarget / 2));
            motor.set(MOTOR_ROLL_MINUS, (baseSpeed - rollDiff) + (yawTarget / 2));
            motor.set(MOTOR_PITCH_PLUS, (baseSpeed + pitchDiff) - (yawTarget / 2));
            motor.set(MOTOR_PITCH_MINUS, (baseSpeed - pitchDiff) - (yawTarget / 2));
            if (error_t error = motor.send()) {
                pc.printf("%s\n", getErrorString(error));
                errorLoop(ERROR_ANIMATION_7);
            }
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
                if (data == NULL) {
                    pc.printf("No Memory!\n");
                    break;
                }
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

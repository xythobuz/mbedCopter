#include "mbed.h"
#include "sensors.h"
#include "attitude.h"
#include "remote.h"

#include <stdlib.h>

DigitalOut statusLED[] = { DigitalOut(LED1), DigitalOut(LED2), DigitalOut(LED3), DigitalOut(LED4) };
Serial pc(USBTX, USBRX);
Ticker ticker;
I2C i2c(p28, p27);
Gyro gyro(&i2c);
Acc acc(&i2c);
Attitude attitude(&gyro, &acc);
Remote remote(p6, 6);

#define ERROR_ANIMATION_1 150 // IOOI <-> OIIO
#define ERROR_ANIMATION_2 165 // IOIO <-> OIOI

#define ERROR_ANIMATION_3 135 // IOOO <-> OIII
#define ERROR_ANIMATION_4 75  // OIOO <-> IOII
#define ERROR_ANIMATION_5 45  // OOIO <-> IIOI
#define ERROR_ANIMATION_6 30  // OOOI <-> IIIO

void display(uint8_t anim) {
    statusLED[0] = ((anim & 0x08) >> 3);
    statusLED[1] = ((anim & 0x04) >> 2);
    statusLED[2] = ((anim & 0x02) >> 1);
    statusLED[3] = (anim & 0x01);
}

void errorLoop(uint8_t anim) {
    while(1) {
        display(anim >> 4);
        wait(0.2);
        display(anim);
        wait(0.2);
    }
}

void attitudeHandler() {
    if (int error = attitude.calculate()) {
        pc.printf("Attitude Calculation Error %d!\n", error);
        errorLoop(ERROR_ANIMATION_3);
    }
}

int main() {
    display(0x0F);

    i2c.frequency(400 * 1000);
    pc.baud(38400);

    if (int error = gyro.init(Gyro::r2000DPS)) {
        pc.printf("Gyro Init Error %d!\n", error);
        errorLoop(ERROR_ANIMATION_1);
    }

    if (int error = acc.init(Acc::r8G)) {
        pc.printf("Acc Init Error %d!\n", error);
        errorLoop(ERROR_ANIMATION_2);
    }

    display(0);
    ticker.attach_us(&attitudeHandler, (10 * 1000)); // 10ms --> 100Hz

    while(1) {
        if(pc.readable()) {
            switch (pc.getc()) {
            case 'a':
                pc.printf("Roll: %.2f\nPitch: %.2f\nYaw: %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
                break;

            case 'r':
                int *data;
                data = remote.get();
                for (int i = 0; i < remote.bufferLength; i++) {
                    pc.printf("%d: %d\n", i, data[i]);
                }
                free(data);
                break;

            default:
                pc.printf("Command unknown!\n");
                break;
            }
        }
    }

    return 0;
}

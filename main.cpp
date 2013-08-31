#include "mbed.h"
#include "sensors.h"
#include "attitude.h"
#include "remote.h"
#include "altitude.h"
#include "errors.h"
#include "pid.h"

#include <stdlib.h>

#define ERROR_ANIMATION_x 150 // IOOI <-> OIIO --> Used by Bootloader + Runtime
#define ERROR_ANIMATION_1 165 // IOIO <-> OIOI --> Gyro Init Error
#define ERROR_ANIMATION_2 135 // IOOO <-> OIII --> Acc Init Error
#define ERROR_ANIMATION_3 75  // OIOO <-> IOII --> Altitude Init Error
#define ERROR_ANIMATION_4 45  // OOIO <-> IIOI --> Attitude Handler Error
#define ERROR_ANIMATION_5 30  // OOOI <-> IIIO --> Too slow to keep up with calculations

const static int remoteChannels = 6;

int attitudeFrequency = 100;
int attitudeFlag = 0;

DigitalOut statusLED[] = { DigitalOut(LED1), DigitalOut(LED2), DigitalOut(LED3), DigitalOut(LED4) };
Serial pc(USBTX, USBRX);
Ticker ticker;
I2C i2c(p28, p27);

Gyro gyro(&i2c);
Acc acc(&i2c);
Attitude attitude(&gyro, &acc, attitudeFrequency);
Altitude altitude(&i2c);
Remote remote(p6, remoteChannels);
PID  rollPID(5.0, 0.0015, -13.0, -255, 255);
PID pitchPID(5.0, 0.0015, -13.0, -255, 255);

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
        if (attitudeFlag > 0) {
            attitudeFlag--;
            if (int error = attitude.calculate()) {
                pc.printf("%s\n", getErrorString(error));
                errorLoop(ERROR_ANIMATION_4);
            }
            if (attitudeFlag > 0) {
                pc.printf("!! Too slow to keep up !!\n");
                //errorLoop(ERROR_ANIMATION_5);
            }
        }

        if(pc.readable()) {
            switch (pc.getc()) {
            case 'a':
                pc.printf("Roll: %.2f\nPitch: %.2f\nYaw: %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
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
                pc.printf("Temperature: %.1f Celsius\n", temperature * 0.1f);
                pc.printf("Pressure: %d Pascal\n", pressure);
                pc.printf("Altitude: %.5f\n", altitude.calculateAltitude(pressure));
                break;

            default:
                pc.printf("Command unknown!\n");
                break;
            }
        }
    }

    return 0;
}

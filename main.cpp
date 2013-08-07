#include "mbed.h"
#include "sensors.h"

DigitalOut statusLED[] = { DigitalOut(LED1), DigitalOut(LED2), DigitalOut(LED3), DigitalOut(LED4) };
Serial pc(USBTX, USBRX);

#define ERROR_ANIMATION_1 150
#define ERROR_ANIMATION_2 165

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

int main() {
    I2C i2c(p28, p27);
    Gyro gyro(&i2c);
    Acc acc(&i2c);

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
    pc.puts("mbed Ready!\n");

    while(1) {
        if(pc.readable()) {
            char c = pc.getc();
            float v[3];
            switch (c) {
            case 'g':
                gyro.read(v);
                pc.printf("Gyro: %.3f %.3f %.3f\n", v[0], v[1], v[2]);
                break;

            case 'a':
                acc.read(v);
                pc.printf("Acc: %.3f %.3f %.3f\n", v[0], v[1], v[2]);
                break;

            default:
                pc.printf("Command unknown!\n");
                break;
            }
        }
    }

    return 0;
}

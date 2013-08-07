#include "mbed.h"


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
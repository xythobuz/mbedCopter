#include "mbed.h"

#ifndef _ALTITUDE_H
#define _ALTITUDE_H

class Altitude {
public:
    Altitude(I2C *i);
    int init(); // 0 on success

    // Call readTemperature() before readPressure()!!
    int readTemperature(uint16_t *data); // 0 on succes, units of 0.1 deg C
    int readPressure(int32_t *data); // 0 on success, units of Pa

    static const uint8_t address = 0x77;
    static const uint8_t OSS = 0; // Oversampling (0 to 3)

private:
    int read(uint8_t add, int16_t *data); // 0 on success
    int readCalibration(); // 0 on success
    int readUT(uint16_t *data); // 0 on success
    int readUP(uint32_t *data); // 0 on success

    I2C *i2c;
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    int32_t b5;
};

#endif
#include "sensors.h"

Acc::Acc(I2C *i) {
    i2c = i;
}

int Acc::init(Acc::Range r) {
    char data[2];
    uint8_t v;

    range = r;
    switch (range) {
        case r2G:
            v = 0x00;
            break;
        case r4G:
            v = 0x10;
            break;
        case r8G:
            v = 0x20;
            break;
        case r16G:
            v = 0x30;
            break;
        default:
            return 1;
    }
    data[0] = register1; data[1] = 0x57;
    if (i2c->write(address, data, 2, false))
        return 2;

    data[0] = register4; data[1] = v;
    return i2c->write(address, data, 2, false);
}

int Acc::read(float *v) {
    char data[6];

    if (v == NULL)
        return 3;

    data[0] = registerOut | 0x80; // Auto-Increment
    if (i2c->write(address, data, 1, true))
        return 2;

    if (i2c->read(address, data, 6, false))
        return 2;

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
        case r2G:
            v[0] = (((double)x) * 2 / 0x8000);
            v[1] = (((double)y) * 2 / 0x8000);
            v[2] = (((double)z) * 2 / 0x8000);
            break;
        case r4G:
            v[0] = (((double)x) * 4 / 0x8000);
            v[1] = (((double)y) * 4 / 0x8000);
            v[2] = (((double)z) * 4 / 0x8000);
            break;
        case r8G:
            v[0] = (((double)x) * 8 / 0x8000);
            v[1] = (((double)y) * 8 / 0x8000);
            v[2] = (((double)z) * 8 / 0x8000);
            break;
        case r16G:
            v[0] = (((double)x) * 16 / 0x8000);
            v[1] = (((double)y) * 16 / 0x8000);
            v[2] = (((double)z) * 16 / 0x8000);
            break;
        default:
            return 1;
    }

    return 0;
}
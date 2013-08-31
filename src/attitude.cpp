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

int Attitude::calculate() {
    float data[2][3];

    if (int error = gyro->read(data[GYRO]))
        return error;

    if (int error = acc->read(data[ACC]))
        return error;

    float accPitch = TODEG( atanf(data[ACC][Y] / hypotf(data[ACC][X], data[ACC][Z])) );
    float accRoll = TODEG( atanf(data[ACC][X] / hypotf(data[ACC][Y], data[ACC][Z])) );

    pitchFilter.innovate(accPitch, data[GYRO][X]);
    rollFilter.innovate(accRoll, data[GYRO][Y]);

    pitch = pitchFilter.x1;
    roll = rollFilter.x1;

    return SUCCESS;
}

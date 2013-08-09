#include "attitude.h"
#include <math.h>

#define TODEG(x) (((x) * 180) / M_PI) /**< Convert Radians to Degrees */

#define GYRO 0
#define ACC 1
#define X 0
#define Y 1
#define Z 2

Attitude::Attitude(Gyro *g, Acc *a) {
    gyro = g;
    acc = a;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    pitchFilter = Kalman();
    rollFilter = Kalman();
}

int Attitude::calculate() {
    double data[2][3];

    if (gyro->read(data[GYRO]))
        return 1;

    if (acc->read(data[ACC]))
        return 2;

    double accPitch = TODEG( atan(data[ACC][Y] / hypot(data[ACC][X], data[ACC][Z])) );
    double accRoll = TODEG( atan(data[ACC][X] / hypot(data[ACC][Y], data[ACC][Z])) );

    pitchFilter.innovate(accPitch, data[GYRO][X]);
    rollFilter.innovate(accRoll, data[GYRO][Y]);

    pitch = pitchFilter.x1;
    roll = rollFilter.x1;

    return 0;
}

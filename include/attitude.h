#include "kalman.h"
#include "sensors.h"

#ifndef _ATTITUDE_H
#define _ATTITUDE_H

class Attitude {
public:
    Attitude(Gyro *g, Acc *a, int frequency);
    int calculate(); // 0 on success

    float roll, pitch, yaw;

private:
    Kalman pitchFilter, rollFilter;
    Gyro *gyro;
    Acc *acc;
};

#endif
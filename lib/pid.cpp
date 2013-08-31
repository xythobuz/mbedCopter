#include "pid.h"

PID::PID(float p, float i, float d, float min, float max) {
    sum = 0;
    last = 0;
    lastTime = clock();
    set(p, i, d, min, max);
}

void PID::set(float p, float i, float d, float min, float max) {
    kp = p;
    ki = i;
    kd = d;
    PID::min = min;
    PID::max = max;
}

float PID::execute(float should, float is) {
    clock_t startTime = clock();
    float dT = (float)(startTime - lastTime);
    float error = should - is;
    sum = sum + (error * dT);
    float dError = (error - last) / dT;
    float output = (kp * error) + (ki * sum) + (kd * dError);
    last = error;
    lastTime = startTime;
    if (output > max) {
        output = max;
    }
    if (output < min) {
        output = min;
    }
    return output;
}
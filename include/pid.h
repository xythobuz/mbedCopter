#ifndef _PID_H
#define _PID_H

#include <time.h>

class PID {
public:
    PID(float p, float i, float d, float min, float max);
    void set(float p, float i, float d, float min, float max);
    float execute(float should, float is);

private:
    float kp, ki, kd; // Factors
    float min, max; // Output range
    float sum, last; // Integral & Derivative State
    clock_t lastTime; // Last Execution Time
};

#endif
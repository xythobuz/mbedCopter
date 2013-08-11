#include "kalman.h"

Kalman::Kalman(int frequency) {
    x1 = 0.0;
    x2 = 0.0;
    x3 = 0.0;

    // Init P to diagonal matrix with large values since
    // the initial state is not known
    q11 = 1000.0;
    q12 = 0.0;
    q13 = 0.0;
    q21 = 0.0;
    q22 = 1000.0;
    q23 = 0.0;
    q31 = 0.0;
    q32 = 0.0;
    q33 = 1000.0;

    DT = 1.0 / (double)frequency;
}

void Kalman::innovate(double z1, double z2) {
    double y1, y2;
    double a, b, c;
    double sDet;
    double s11, s12, s21, s22;
    double k11, k12, k21, k22, k31, k32;
    double p11, p12, p13, p21, p22, p23, p31, p32, p33;

    // Step 1
    // x(k) = Fx(k-1) + Bu + w:
    x1 = x1 + DT*x2 - DT*x3;
    //x2 = x2;
    //x3 = x3;

    // Step 2
    // P = FPF'+Q
    a = q11 + q21*DT - q31*DT;
    b = q12 + q22*DT - q32*DT;
    c = q13 + q23*DT - q33*DT;
    q11 = a + b*DT - c*DT + Q1;
    q12 = b;
    q13 = c;
    q21 = q21 + q22*DT - q23*DT;
    q22 = q22 + Q2;
    //p23 = p23;
    q31 = q31 + q32*DT - q33*DT;
    //p32 = p32;
    q33 = q33 + Q3;

    // Step 3
    // y = z(k) - Hx(k)
    y1 = z1-x1;
    y2 = z2-x2;

    // Step 4
    // S = HPT' + R
    s11 = q11 + R1;
    s12 = q12;
    s21 = q21;
    s22 = q22 + R2;

    // Step 5
    // K = PH*inv(S)
    sDet = 1/(s11*s22 - s12*s21);
    k11 = (q11*s22 - q12*s21)*sDet;
    k12 = (q12*s11 - q11*s12)*sDet;
    k21 = (q21*s22 - q22*s21)*sDet;
    k22 = (q22*s11 - q21*s12)*sDet;
    k31 = (q31*s22 - q32*s21)*sDet;
    k32 = (q32*s11 - q31*s12)*sDet;

    // Step 6
    // x = x + Ky
    x1 = x1 + k11*y1 + k12*y2;
    x2 = x2 + k21*y1 + k22*y2;
    x3 = x3 + k31*y1 + k32*y2;

    // Step 7
    // P = (I-KH)P
    p11 = q11*(1.0f - k11) - q21*k12;
    p12 = q12*(1.0f - k11) - q22*k12;
    p13 = q13*(1.0f - k11) - q23*k12;
    p21 = q21*(1.0f - k22) - q11*k21;
    p22 = q22*(1.0f - k22) - q12*k21;
    p23 = q23*(1.0f - k22) - q13*k21;
    p31 = q31 - q21*k32 - q11*k31;
    p32 = q32 - q22*k32 - q12*k31;
    p33 = q33 - q22*k32 - q13*k31;
    q11 = p11; q12 = p12; q13 = p13;
    q21 = p21; q22 = p22; q23 = p23;
    q31 = p31; q32 = p32; q33 = p33;
}

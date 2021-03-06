/*
 * kalman.cpp
 *
 * Copyright Linus Helgesson
 * http://www.linushelgesson.se/2012/04/pitch-and-roll-estimating-kalman-filter-for-stabilizing-quadrocopters/
 *
 * Copyright (c) 2013, Thomas Buck <xythobuz@xythobuz.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "kalman.h"

Kalman::Kalman(int frequency) {
    x1 = 0.0f;
    x2 = 0.0f;
    x3 = 0.0f;

    // Init P to diagonal matrix with large values since
    // the initial state is not known
    q11 = 1000.0f;
    q12 = 0.0f;
    q13 = 0.0f;
    q21 = 0.0f;
    q22 = 1000.0f;
    q23 = 0.0f;
    q31 = 0.0f;
    q32 = 0.0f;
    q33 = 1000.0f;

    DT = 1.0f / (float)frequency;
}

void Kalman::innovate(float z1, float z2) {
    float y1, y2;
    float a, b, c;
    float sDet;
    float s11, s12, s21, s22;
    float k11, k12, k21, k22, k31, k32;
    float p11, p12, p13, p21, p22, p23, p31, p32, p33;

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
    sDet = 1.0f/(s11*s22 - s12*s21);
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

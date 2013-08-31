/*
 * kalman.h
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
#ifndef _KALMAN_H
#define _KALMAN_H

class Kalman {
public:
    Kalman(int frequency);
    void innovate(float z1, float z2);

    float x1, x2, x3;

private:
    // Q (3x3 Matrix) with these elements on diagonal
    static const float Q1 = 5.0f; /**< Q Matrix Diagonal Element 1 */
    static const float Q2 = 100.0f; /**< Q Matrix Diagonal Element 2 */
    static const float Q3 = 0.01f; /**< Q Matrix Diagonal Element 3 */

    // R (2x2 Matrix) with these elements on diagonal
    static const float R1 = 1000.0f; /**< R Matrix Diagonal Element 1 */
    static const float R2 = 1000.0f; /**< R Matrix Diagonal Element 2 */

    float q11, q12, q13, q21, q22, q23, q31, q32, q33;
    float DT;
};

#endif
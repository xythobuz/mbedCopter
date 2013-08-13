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
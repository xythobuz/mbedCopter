#ifndef _KALMAN_H
#define _KALMAN_H

class Kalman {
public:
    Kalman();
    void innovate(double z1, double z2);

    double x1, x2, x3;

private:
    static const double DT = 0.01; // 100Hz

    // Q (3x3 Matrix) with these elements on diagonal
    static const double Q1 = 5.0; /**< Q Matrix Diagonal Element 1 */
    static const double Q2 = 100.0; /**< Q Matrix Diagonal Element 2 */
    static const double Q3 = 0.01; /**< Q Matrix Diagonal Element 3 */

    // R (2x2 Matrix) with these elements on diagonal
    static const double R1 = 1000.0; /**< R Matrix Diagonal Element 1 */
    static const double R2 = 1000.0; /**< R Matrix Diagonal Element 2 */

    double q11, q12, q13, q21, q22, q23, q31, q32, q33;
};

#endif
/**
 * Implementation of a 1D Kalman filter.
 */
#ifndef KALMANFILTER1D_H
#define KALMANFILTER1D_H


#include <iostream>

class KalmanFilter1D
{
private:
    double x;
    double P;
    double R;
    double Q;

public:
    KalmanFilter1D(double x0, double P, double R, double Q);
    ~KalmanFilter1D();

    void update(double z);

    void predict(double u=0.0);

    double get_x() { return x; }
    double get_P() { return P; }
};


#endif

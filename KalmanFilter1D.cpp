#include "KalmanFilter1D.hpp"

KalmanFilter1D::KalmanFilter1D(double x0, double P, double R, double Q)
{
    this->x = x0;
    this->P = P;
    this->R = R;
    this->Q = Q;
}

KalmanFilter1D::~KalmanFilter1D()
{
}

void KalmanFilter1D::update(double z)
{
    x = (P*z + x*R) / (P + R);
    P = 1. / (1./P + 1./R);
}

void KalmanFilter1D::predict(double u)
{
    x += u;
    P += Q;
}

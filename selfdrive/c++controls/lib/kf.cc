#include "kf.h"

KF1D::KF1D(double x0[2][1], double A[2][2], double C[2], double K[2][1])
{
  x0_0 = x0[0][0];
  x1_0 = x0[1][0];
  A0_0 = A[0][0];
  A0_1 = A[0][1];
  A1_0 = A[1][0];
  A1_1 = A[1][1];
  C0_0 = C[0];
  C0_1 = C[1];
  K0_0 = K[0][0];
  K1_0 = K[1][0];

  A_K_0 = A0_0 - K0_0 * C0_0;
  A_K_1 = A0_1 - K0_0 * C0_1;
  A_K_2 = A1_0 - K1_0 * C0_0;
  A_K_3 = A1_1 - K1_0 * C0_1;
}

KF1D::~KF1D()
{}

void KF1D::getX(double x0[2][1])
{
  x0[0][0] = x0_0;
  x0[1][0] = x1_0;
}

void KF1D::setX(double x0[2][1])
{
  x0_0 = x0[0][0];
  x1_0 = x0[1][0];
}

void KF1D::update(double meas, double x0[2][1])
{
  x0[0][0] = A_K_0 * x0_0 + A_K_1 * x1_0 + K0_0 * meas;
  x0[1][0] = A_K_2 * x0_0 + A_K_3 * x1_0 + K1_0 * meas;
  x0_0 = x0[0][0];
  x1_0 = x0[1][0];
}
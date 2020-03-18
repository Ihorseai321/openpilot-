#ifndef KF_H_
#define KF_H_

class KF1D
{
public:
  KF1D(double x0[2][1], double A[2][2], double C[2], double K[2][1]);
  ~KF1D();
  void setX(double x0[2][1]);
  void getX(double x0[2][1]);
  void update(double meas, double x0[2][1]);
private:
  double x0_0;
  double x1_0;
  double A0_0;
  double A0_1;
  double A1_0;
  double A1_1;
  double C0_0;
  double C0_1;
  double K0_0;
  double K1_0;

  double A_K_0;
  double A_K_1;
  double A_K_2;
  double A_K_3;

};
#endif
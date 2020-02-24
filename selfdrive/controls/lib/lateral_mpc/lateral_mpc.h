#ifndef LATERAL_MPC_H_
#define LATERAL_MPC_H_
#define N 20


typedef struct {
  double x, y, psi, delta, t;
} state_t;


typedef struct {
  double x[21];
  double y[21];
  double psi[21];
  double delta[21];
  double rate[20];
  double cost;
} log_t;

#endif //LATERAL_MPC_H_
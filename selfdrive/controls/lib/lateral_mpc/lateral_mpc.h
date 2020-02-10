#ifndef LATERAL_MPC_H_
#define LATERAL_MPC_H_
#define N 20


typedef struct {
  double x, y, psi, delta, t;
} state_t;


typedef struct {
  double x[N+1];
  double y[N+1];
  double psi[N+1];
  double delta[N+1];
  double rate[N];
  double cost;
} log_t;

void init_weights(double pathCost, double laneCost, double headingCost, double steerRateCost);

void init(double pathCost, double laneCost, double headingCost, double steerRateCost);

int run_mpc(state_t * x0, log_t * solution,
             double l_poly[4], double r_poly[4], double d_poly[4],
             double l_prob, double r_prob, double curvature_factor, double v_ref, double lane_width);

#endif //LATERAL_MPC_H_
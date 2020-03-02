#ifndef MPC_STRUCTS_H_
#define MPC_STRUCTS_H_

typedef struct {
    float x_ego, v_ego, a_ego, x_l, v_l, a_l;
} long_state_t;


typedef struct {
    float x_ego[21];
    float v_ego[21];
    float a_ego[21];
    float j_ego[20];
    float x_l[21];
    float v_l[21];
    float a_l[21];
    float t[21];
    float cost;
} long_log_t;

typedef struct {
    float x, y, psi, delta, t;
} lateral_state_t;

typedef struct {
    float x[21];
    float y[21];
    float psi[21];
    float delta[21];
    float rate[20];
    float cost;
} lateral_log_t;

#endif // MPC_STRUCTS_H_
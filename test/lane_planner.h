#ifndef LANE_PLANNER_H_
#define LANE_PLANNER_H_
#include "handler.h"

const float CAMERA_OFFSET = 0.06;

int compute_path_pinv(float pinv1[4][50]);
float interp(float x, float xp[], float fp[], int n);
void model_polyfit(float points[], float path_pinv[4][50], float ret[]);
void calc_d_poly(float l_poly[], float r_poly[], float p_poly[], float l_prob, float r_prob, float lane_width, float d_poly[]);

class LanePlanner
{
public:
    LanePlanner();
    virtual ~LanePlanner();
    void parse_model(Handler handler);
    void update_d_poly(float v_ego);
    void update(Handler handler, float v_ego);

    float l_poly[4];
    float r_poly[4];
    float p_poly[4];
    float d_poly[4];

    float lane_width_estimate;
    float lane_width_certainty;
    float lane_width;

    float l_prob;
    float r_prob;

    float l_lane_change_prob;
    float r_lane_change_prob;

    float _path_pinv[4][50];
    float x_points[50];
};

#endif // LANE_PLANNER_H_
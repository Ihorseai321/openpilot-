#ifndef FCW_H_
#define FCW_H_

typedef struct {
  double x_ego[21];
  double v_ego[21];
  double a_ego[21];
  double j_ego[20];
  double x_l[21];
  double v_l[21];
  double a_l[21];
  double t[21];
  double cost;
} log_t;

class FCWChecker
{
public:
    FCWChecker();
    virtual ~FCWChecker();
    void reset_lead(float cur_time);
    float calc_ttc(float v_ego, float a_ego, float x_lead, float v_lead, float a_lead);
    bool update(log_t mpc_solution, float cur_time, bool active, float v_ego, float a_ego, float x_lead, float v_lead, float a_lead, float y_lead, float vlat_lead, bool fcw_lead, bool blinkers);

private:
    float last_fcw_a;
    float v_lead_max;
    float lead_seen_t;
    float last_fcw_time;
    float last_min_a;

    float common_counters[2]; //0 blinkers, 1 v_ego
    float counters[6]; //0 ttc, 1 v_lead_max, 2 v_ego_lead, 3 lead_seen, 4 y_lead, 5 vlat_lead
};

#endif // FCW_H_
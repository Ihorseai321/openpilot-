#ifndef CARSTATE_H_
#define CARSTATE_H_

class CarState
{
public:
  CarState();
  ~CarState();

  int left_blinker_on;
  int right_blinker_on;
  int prev_left_blinker_on;
  int prev_right_blinker_on; 

  float v_ego_kf;
  float v_ego;
private:
  float dt;
};

#endif // CARSTATE_H_
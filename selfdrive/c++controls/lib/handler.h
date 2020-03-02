#ifndef HANDLER_H_
#define HANDLER_H_

#include "cereal/gen/cpp/log.capnp.h"
#include "utils.h"

#define DEGREES_TO_RADIANS 0.017453292519943295

class Handler
{
private:
  void handle_car_state(cereal::CarState::Reader car_state, unsigned long current_time);
  void handle_controls_state(cereal::ControlsState::Reader controls_state, unsigned long current_time);
  void handle_radar_state(cereal::RadarState::Reader radar_state, unsigned long current_time, unsigned long rcv_time);
  void handle_model(cereal::ModelData::Reader model, unsigned long current_time);
  void handle_live_parameters(cereal::LiveParametersData::Reader live_parameters, unsigned long current_time);

public:
  float prev_update_time;
  unsigned long car_state_time;
  unsigned long controls_state_time;
  unsigned long radar_state_time;
  unsigned long model_time;
  unsigned long liveParameters_time;
  unsigned long radar_rcv_time;

  // carState
  float vEgo;
  float steeringAngle;
  float aEgo;
  bool leftBlinker;
  bool rightBlinker;
  bool brakePressed;
  bool steeringPressed;
  float steeringTorque;

  // controlsState
  bool active;
  cereal::ControlsState::LongControlState long_control_state;
  float v_cruise_kph;
  bool force_slow_decel;
  float curvature;

  // radarState
  struct LeadData lead_1;
  struct LeadData lead_2;

  // model
  bool l_has_poly;
  bool r_has_poly;
  bool p_has_poly;
  bool l_has_points;
  bool r_has_points;
  bool p_has_points;
  bool has_meta_desirePrediction;
  float l_poly[4];
  float r_poly[4];
  float p_poly[4];
  float l_points[50];
  float r_points[50];
  float p_points[50];
  float l_prob;
  float r_prob;
  float meta_desirePrediction[32];

  // liveParameters
  float angleOffset;
  float stiffnessFactor;
  float steerRatio;
  float angleOffsetAverage;
  bool paramsValid;
  bool sensorValid;
  bool posenetValid;

  Handler();
  virtual ~Handler();
  void handle_log(cereal::Event::Reader event);

};

#endif // HANDLER_H_
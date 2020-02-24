#ifndef HANDLER_H_
#define HANDLER_H_

#include "cereal/gen/cpp/log.capnp.h"

#define DEGREES_TO_RADIANS 0.017453292519943295

struct LeadData {
    float dRel;
    float yRel;
    float vRel;
    float aRel;
    float vLead;
    float aLeadDEPRECATED;
    float dPath;
    float vLat;
    float vLeadK;
    float aLeadK;
    bool fcw;
    bool status;
    float aLeadTau;
    float modelProb;
    bool radar;
}

class Handler
{
  void handle_car_state(cereal::CarState::Reader car_state, float current_time);
  void handle_controls_state(cereal::ControlsState::Reader controls_state, float current_time);
  void handle_radar_state(cereal::RadarState::Reader radar_state, float current_time);
  void handle_model(cereal::ModelData::Reader model, float current_time);
  void handle_live_parameters(cereal::LiveParametersData::Reader live_parameters, float current_time);


public:
  float prev_update_time;
  float car_state_time;
  float controls_state_time;
  float radar_state_time;
  float model_time;
  float liveParameters_time;
  

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
  int long_control_state;
  float v_cruise_kph;
  bool force_slow_decel;

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
  float angle_offset;
  float stiffnessFactor;
  float steerRatio;
  float angleOffset;
  bool paramsValid;
  bool sensorValid;
  bool posenetValid;

  Handler();
  virtual ~Handler();
  void handle_log(cereal::Event::Reader event);

};

#endif // HANDLER_H_
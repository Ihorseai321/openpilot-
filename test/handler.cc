#include <iostream>
#include <cmath>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <eigen3/Eigen/Dense>

#include "cereal/gen/cpp/log.capnp.h"
#include "handler.h"

Handler::Handler()
{
  radarErrors[0] = 0;
  radarErrors[1] = 0;
  radarErrors[2] = 0;

  prev_update_time = -1;
  car_state_time = -1;
  controls_state_time = -1;
  radar_state_time = -1;
  model_time = -1;
  liveParameters_time = -1;
  

  // carState
  vEgo = 0.0;
  steeringAngle = 0.0;
  aEgo = 0.0;
  leftBlinker = false;
  rightBlinker = false;
  brakePressed = false;
  steeringPressed = false;
  steeringTorque = 0.0;

  // controlsState
  active = false;
  long_control_state = 0;
  v_cruise_kph = 0.0;
  force_slow_decel = false;

  // radarState
  lead_1.dRel = 0.0;
  lead_1.yRel = 0.0;
  lead_1.vRel = 0.0;
  lead_1.aRel = 0.0;
  lead_1.vLead = 0.0;
  lead_1.aLeadDEPRECATED = 0.0;
  lead_1.dPath = 0.0;
  lead_1.vLat = 0.0;
  lead_1.vLeadK = 0.0;
  lead_1.aLeadK = 0.0;
  lead_1.fcw = false;
  lead_1.status = false;
  lead_1.aLeadTau = 0.0;
  lead_1.modelProb = 0.0;
  lead_1.radar = false;
  
  lead_2.dRel = 0.0;
  lead_2.yRel = 0.0;
  lead_2.vRel = 0.0;
  lead_2.aRel = 0.0;
  lead_2.vLead = 0.0;
  lead_2.aLeadDEPRECATED = 0.0;
  lead_2.dPath = 0.0;
  lead_2.vLat = 0.0;
  lead_2.vLeadK = 0.0;
  lead_2.aLeadK = 0.0;
  lead_2.fcw = false;
  lead_2.status = false;
  lead_2.aLeadTau = 0.0;
  lead_2.modelProb = 0.0;
  lead_2.radar = false;
  
  // model
  l_has_poly = false;
  r_has_poly = false;
  p_has_poly = false;
  l_has_points = false;
  r_has_points = false;
  p_has_points = false;
  has_meta_desirePrediction = false;
  
  for(int i = 0; i < 4; ++i){
    l_poly[i] = 0.0;
    r_poly[i] = 0.0;
    p_poly[i] = 0.0;
  }
  
  for(int i = 0; i < 50; ++i){
    l_points[i] = 0.0;
    r_points[i] = 0.0;
    p_points[i] = 0.0;
  }
  
  l_prob = 0.0;
  r_prob = 0.0;

  for(int i = 0; i < 32; ++i){
    meta_desirePrediction[i] = 0.0;
  }
  // liveParameters
  angle_offset = 0.0;
  stiffnessFactor = 0.0;
  steerRatio = 0.0;
  angleOffset = 0.0;
  paramsValid = false;
  sensorValid = false;
  posenetValid = false;
}

Handler::~Handler()
{  
}

void Handler::handle_car_state(cereal::CarState::Reader car_state, double current_time)
{
  vEgo = car_state.getVEgo();
  steeringAngle = car_state.getSteeringAngle();
  aEgo = car_state.getAEgo();
  leftBlinker = car_state.getLeftBlinker();
  rightBlinker = car_state.getRightBlinker();
  brakePressed = car_state.getBrakePressed();
  steeringPressed = car_state.getSteeringPressed();
  steeringTorque = car_state.getSteeringTorque();
  car_state_time = current_time;
}

void Handler::handle_controls_state(cereal::ControlsState::Reader controls_state, double current_time) {
  active = controls_state.getActive();
  long_control_state = controls_state.getLongControlState();
  v_cruise_kph = controls_state.getVCruise();
  force_slow_decel = controls_state.getForceDecel();
  controls_state_time = current_time;
}

void Handler::handle_radar_state(cereal::RadarState::Reader radar_state, double current_time)
{
  cereal::RadarState::LeadData::Reader leadOne = radar_state.getLeadOne();
  lead_1.dRel = leadOne.getDRel();
  lead_1.yRel = leadOne.getYRel();
  lead_1.vRel = leadOne.getVRel();
  lead_1.aRel = leadOne.getARel();
  lead_1.vLead = leadOne.getVLead();
  lead_1.aLeadDEPRECATED = leadOne.getALeadDEPRECATED();
  lead_1.dPath = leadOne.getDPath();
  lead_1.vLat = leadOne.getVLat();
  lead_1.vLeadK = leadOne.getVLeadK();
  lead_1.aLeadK = leadOne.getALeadK();
  lead_1.fcw = leadOne.getFcw();
  lead_1.status = leadOne.getStatus();
  lead_1.aLeadTau = leadOne.getALeadTau();
  lead_1.modelProb = leadOne.getModelProb();
  lead_1.radar = leadOne.getRadar();
  
  cereal::RadarState::LeadData::Reader leadTwo = radar_state.getLeadTwo();
  lead_2.dRel = leadTwo.getDRel();
  lead_2.yRel = leadTwo.getYRel();
  lead_2.vRel = leadTwo.getVRel();
  lead_2.aRel = leadTwo.getARel();
  lead_2.vLead = leadTwo.getVLead();
  lead_2.aLeadDEPRECATED = leadTwo.getALeadDEPRECATED();
  lead_2.dPath = leadTwo.getDPath();
  lead_2.vLat = leadTwo.getVLat();
  lead_2.vLeadK = leadTwo.getVLeadK();
  lead_2.aLeadK = leadTwo.getALeadK();
  lead_2.fcw = leadTwo.getFcw();
  lead_2.status = leadTwo.getStatus();
  lead_2.aLeadTau = leadTwo.getALeadTau();
  lead_2.modelProb = leadTwo.getModelProb();
  lead_2.radar = leadTwo.getRadar();
  radar_state_time = current_time;
}

void Handler::handle_model(cereal::ModelData::Reader model, double current_time)
{
  cereal::ModelData::PathData::Reader l_lane = model.getLeftLane();
  cereal::ModelData::PathData::Reader r_lane = model.getRightLane();
  cereal::ModelData::PathData::Reader path = model.getPath();

  l_has_poly = l_lane.hasPoly();
  r_has_poly = r_lane.hasPoly();
  p_has_poly = path.hasPoly();

  l_has_points = l_lane.hasPoints();
  r_has_points = r_lane.hasPoints();
  p_has_points = path.hasPoints();

  if(l_has_poly){
    int l_ply = 0;
    for (float _l_poly: l_lane.getPoly()) {
      l_poly[l_ply++] = _l_poly;
    }
  }

  if(r_has_poly){
    int r_ply = 0;
    for (float _r_poly: r_lane.getPoly()) {
      r_poly[r_ply++] = _r_poly;
    }
  }

  if(p_has_poly){
    int p_ply = 0;
    for (float _p_poly: path.getPoly()) {
      p_poly[p_ply++] = _p_poly;
    }
  }

    if(l_has_points){
    int l_pts = 0;
    for (float _l_points: l_lane.getPoints()) {
      l_points[l_pts++] = _l_points;
    }
  }

  if(r_has_points){
    int r_pts = 0;
    for (float _r_points: r_lane.getPoints()) {
      r_points[r_pts++] = _r_points;
    }
  }

  if(p_has_points){
    int p_pts = 0;
    for (float _p_points: path.getPoints()) {
      p_points[p_pts++] = _p_points;
    }
  }

  l_prob = l_lane.getProb();
  r_prob = r_lane.getProb();
  
  cereal::ModelData::MetaData::Reader meta = model.getMeta();
  has_meta_desirePrediction = meta.hasDesirePrediction();

  if(has_meta_desirePrediction){
    int meta_dp = 0;
    for (float desirePrediction: meta.getDesirePrediction()) {
      meta_desirePrediction[meta_dp++] = desirePrediction;
    }
  }
  model_time = current_time;
}

void Handler::handle_live_parameters(cereal::LiveParametersData::Reader live_parameters, double current_time)
{
  angle_offset = live_parameters.getAngleOffset();
  stiffnessFactor = live_parameters.getStiffnessFactor();
  steerRatio = live_parameters.getSteerRatio();
  angleOffset = live_parameters.getAngleOffsetAverage();
  paramsValid = live_parameters.getValid();
  sensorValid = live_parameters.getSensorValid();
  posenetValid = live_parameters.getPosenetValid();
  live_parameters_time = current_time;
}

void Handler::handle_log(cereal::Event::Reader event) {
  double current_time = event.getLogMonoTime() / 1.0e9;

  if (prev_update_time < 0) {
    prev_update_time = current_time;
  }

  auto type = event.which();
  switch(type) {
  case cereal::Event::CAR_STATE:
    handle_car_state(event.getCarState(), current_time);
    break;
  case cereal::Event::CONTROLS_STATE:
    handle_controls_state(event.getControlsState(), current_time);
    break;
  case cereal::Event::RADAR_STATE:
    handle_radar_state(event.getRadarState(), current_time);
    break;
  case cereal::Event::MODEL:
    handle_model(event.getModel(), current_time);
    break;
  case cereal::Event::LIVE_PARAMETERS:
    handle_live_parameters(event.getLiveParameters(), current_time);
    break;
  default:
    break;
  }
}

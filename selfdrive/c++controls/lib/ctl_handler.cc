#include <cmath>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <eigen3/Eigen/Dense>
#include "common/timing.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "ctl_handler.h"

CHandler::CHandler()
{
  prev_update_time = -1;

  // thermal
  thermalStatus = cereal::ThermalData::ThermalStatus::GREEN;
  freeSpace = 0.0;
  batteryPercent = 100;
  chargingError = false;
  memUsedPercent = 0;

  // health
  controlsAllowed = true;

  // liveCalibration
  calStatus = 0;
  calPerc = 0;
  has_rpy = true;

  rpyCalib[0] = 0.0;
  rpyCalib[1] = 0.0;
  rpyCalib[2] = 0.0;

  // plan
  aTarget = 0.0;
  vTarget = 0.0;
  hasLead = false;
  jerkFactor = 0.0;
  gpsPlannerActive = false;
  vCurvature = 0.0;
  longitudinalPlanSource = cereal::Plan::LongitudinalPlanSource::CRUISE;
  mapValid = false;
  radarValid = false;
  radarCanError = false;
  vTargetFuture = 0.0;
  planMonoTime = 0;

  // pathPlan
  angleSteers = 0.0;
  rProb = 0.0;
  lProb = 0.0;

  for(int i = 0; i < 4; ++i){
    lPoly[i] = 0.0;
    rPoly[i] = 0.0;
  }

  angleOffset = 0.0;
  sensorValid = false;
  posenetValid = false;
  mpcSolutionValid = false;
  paramsValid = false;
  laneChangeState = cereal::PathPlan::LaneChangeState::OFF;
  pathPlanMonoTime = 0;

  // gpsLocation
  latitude = 0.0;
  longitude = 0.0;
  
  // model
  has_meta_desirePrediction = false;
  
  for(int i = 0; i < 32; ++i){
    meta_desirePrediction[i] = 0.0;
  }
  
}

CHandler::~CHandler()
{  
}

void CHandler::chandle_thermal(cereal::ThermalData::Reader thermal, unsigned long current_time)
{
  thermalStatus = thermal.getThermalStatus();
  freeSpace = thermal.getFreeSpace();
  batteryPercent = thermal.getBatteryPercent();
  chargingError = thermal.getChargingError();
  memUsedPercent = thermal.getMemUsedPercent();
}

void CHandler::chandle_health(cereal::HealthData::Reader health, unsigned long current_time)
{
  controlsAllowed = health.getControlsAllowed();
}

void CHandler::chandle_live_calibration(cereal::LiveCalibrationData::Reader live_calibration, unsigned long current_time)
{
  calStatus = live_calibration.getCalStatus();
  calPerc = live_calibration.getCalPerc();
  has_rpy = live_calibration.hasRpyCalib();

  if(has_rpy){
    int rpy_i = 0;
    for (float rpy: live_calibration.getRpyCalib()) {
      rpyCalib[rpy_i++] = rpy;
    }
  }
}

void CHandler::chandle_driver_monitoring(cereal::DriverMonitoring::Reader driver_monitoring, unsigned long current_time)
{}

void CHandler::chandle_plan(cereal::Plan::Reader plan, unsigned long current_time, int frame)
{
  rcv_frame_plan = frame;
  aStart = plan.getAStart();
  vStart = plan.getVStart();
  aTarget = plan.getATarget();
  vTarget = plan.getVTarget();
  hasLead = plan.getHasLead();
  jerkFactor = plan.getJerkFactor();
  gpsPlannerActive = plan.getGpsPlannerActive();
  vCurvature = plan.getVCurvature();
  longitudinalPlanSource = plan.getLongitudinalPlanSource();
  mapValid = plan.getMapValid();
  radarValid = plan.getRadarValid();
  radarCanError = plan.getRadarCanError();
  vTargetFuture = plan.getVTargetFuture();
  planMonoTime = current_time;
}

void CHandler::chandle_path_plan(cereal::PathPlan::Reader path_plan, unsigned long current_time)
{
  rProb = path_plan.getRProb();
  lProb = path_plan.getLProb();
  angleSteers = path_plan.getAngleSteers();
  int l_ply = 0;
  for (float _l_poly: path_plan.getLPoly()) {
    lPoly[l_ply++] = _l_poly;
  }

  int r_ply = 0;
  for (float _r_poly: path_plan.getRPoly()) {
    rPoly[r_ply++] = _r_poly;
  }

  angleOffset = path_plan.getAngleOffset();
  sensorValid = path_plan.getSensorValid();
  posenetValid = path_plan.getPosenetValid();
  mpcSolutionValid = path_plan.getMpcSolutionValid();
  paramsValid = path_plan.getParamsValid();
  laneChangeDirection = path_plan.getLaneChangeDirection();
  laneChangeState = path_plan.getLaneChangeState();
  pathPlanMonoTime = current_time;
}

void CHandler::chandle_model(cereal::ModelData::Reader model, unsigned long current_time)
{
  cereal::ModelData::MetaData::Reader meta = model.getMeta();
  
  int meta_i = 0;
  has_meta_desirePrediction = meta.hasDesirePrediction();
  if(has_meta_desirePrediction){
    for (float desirePrediction: meta.getDesirePrediction()) {
      meta_desirePrediction[meta_i++] = desirePrediction;
    }
  }
}

void CHandler::chandle_gps_location(cereal::GpsLocationData::Reader gps_location, unsigned long current_time)
{
  latitude = gps_location.getLatitude();
  longitude = gps_location.getLongitude();
}

void CHandler::chandle_log(cereal::Event::Reader event, int frame) {
  unsigned long current_time = event.getLogMonoTime();
  unsigned long rcv_time = nanos_since_boot();

  if (prev_update_time < 0) {
    prev_update_time = current_time;
  }
  this->frame = frame;
  auto type = event.which();
  switch(type) {
  case cereal::Event::THERMAL:
    chandle_thermal(event.getThermal(), current_time);
    break;
  case cereal::Event::HEALTH:
    chandle_health(event.getHealth(), current_time);
    break;
  case cereal::Event::LIVE_CALIBRATION:
    chandle_live_calibration(event.getLiveCalibration(), current_time);
    break;
  // case cereal::Event::DRIVER_MONITORING:
  //   chandle_driver_monitoring(event.getDriverMonitoring(), current_time);
  //   break;
  case cereal::Event::PLAN:
    chandle_plan(event.getPlan(), current_time, frame);
    break;
  case cereal::Event::PATH_PLAN:
    chandle_path_plan(event.getPathPlan(), current_time);
    break;
  case cereal::Event::MODEL:
    chandle_model(event.getModel(), current_time);
    break;
  case cereal::Event::GPS_LOCATION:
    chandle_gps_location(event.getGpsLocation(), current_time);
    break;
  default:
    break;
  }
}

#ifndef CONTROLS_HANDLER_H_
#define CONTROLS_HANDLER_H_

#include "cereal/gen/cpp/log.capnp.h"
#include "utils.h"

#define DEGREES_TO_RADIANS 0.017453292519943295

class CHandler
{
private:
  void chandle_thermal(cereal::ThermalData::Reader thermal, unsigned long current_time);
  void chandle_health(cereal::HealthData::Reader health, unsigned long current_time);
  void chandle_live_calibration(cereal::LiveCalibrationData::Reader live_calibration, unsigned long current_time, unsigned long rcv_time);
  void chandle_driver_monitoring(cereal::DriverMonitoring::Reader driver_monitoring, unsigned long current_time);
  void chandle_plan(cereal::Plan::Reader plan, unsigned long current_time);
  void chandle_path_plan(cereal::PathPlan::Reader path_plan, unsigned long current_time);
  void chandle_model(cereal::ModelData::Reader model, unsigned long current_time);
  void chandle_gps_location(cereal::GpsLocationData::Reader gps_location, unsigned long current_time);

public:
  float prev_update_time;
  // unsigned long car_state_time;
  // unsigned long controls_state_time;
  // unsigned long radar_state_time;
  // unsigned long model_time;
  // unsigned long liveParameters_time;
  // unsigned long radar_rcv_time;

  // thermal
  cereal::ThermalData::ThermalStatus thermalStatus;
  float freeSpace;
  int batteryPercent;
  bool chargingError;
  int memUsedPercent;

  // health
  bool controlsAllowed;

  // liveCalibration
  int calStatus;
  int calPerc;
  bool has_rpy;
  float rpyCalib[3];

  // plan
  float aTarget;
  float vTarget;
  bool hasLead;
  float jerkFactor;
  bool gpsPlannerActive;
  float vCurvature;
  cereal::Plan::LongitudinalPlanSource longitudinalPlanSource;
  bool mapValid;
  bool radarValid;
  bool radarCanError;
  float vTargetFuture;
  unsigned long planMonoTime;

  // pathPlan
  float rProb;
  float lProb;
  float lPoly[4];
  float rPoly[4];
  float angleOffset;
  bool sensorValid;
  bool posenetValid;
  bool mpcSolutionValid;
  bool sensorValid;
  bool paramsValid;
  cereal::PathPlan::LaneChangeState laneChangeState;
  unsigned long pathPlanMonoTime;

  // model
  bool has_meta_desirePrediction;
  float meta_desirePrediction[32];

  // gpsLocation
  double latitude;
  double longitude;

  CHandler();
  virtual ~CHandler();
  void chandle_log(cereal::Event::Reader event);

};

#endif // CONTROLS_HANDLER_H_
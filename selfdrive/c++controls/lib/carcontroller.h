#ifndef CARCONTROLLER_H_
#define CARCONTROLLER_H_
#include "vehicle_model.h"
#define MAX_STEER_DELTA 1
#define TOGGLE_DEBUG false

class CarController
{
public:
  CarController();
  ~CarController();
  void update();

  VehicleModel vehicle_model;
  bool enable_camera;
  bool enabled_last;
  bool main_on_last;
  int generic_toggle_last;
  bool steer_alert_last;
  int lkas_action;
};
#endif
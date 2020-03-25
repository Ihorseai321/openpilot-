#ifndef INTERFACE_H_
#define INTERFACE_H_
#include <vector>
#include <string>
#include "carparams.h"
#include "carstate.h"
#include "vehicle_model.h"
// #include "carcontroller.h"
#include "parser.h"
#include "utils.h"

class CarInterface
{
public:
  CarInterface(bool carcontroller);
  ~CarInterface();
  CARSTATE update(std::string can_strings);
  float calc_accel_override(float a_ego, float a_target, float v_ego, float v_target);

  int frame;
  bool gas_pressed_prev;
  bool brake_pressed_prev;
  bool cruise_enabled_prev;
private:
  Parser psr;
  CarParams CP;
  VehicleModel VM;
  CarState CS;
  // CarController *CC;
};
#endif // INTERFACE_H_
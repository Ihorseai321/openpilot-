#include "carcontroller.h"

CarController::CarController(bool enable_camera)
{
  this->enable_camera = enable_camera;
  enabled_last = false;
  main_on_last = false;
  generic_toggle_last = 0;
  steer_alert_last = false;
  lkas_action = 0;
}

CarController::~CarController()
{}

//sendcan TODO
CarController::update(enabled, CS, frame, actuators, visual_alert, pcm_cancel)
{

}
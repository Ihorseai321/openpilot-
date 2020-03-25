#include "interface.h"

SIGNAL signals[14] = {{"WhlRr_W_Meas", "WheelSpeed_CG1", 0.0},
                        {"WhlRr_W_Meas", "WheelSpeed_CG1", 0.0},
                        {"WhlRr_W_Meas", "WheelSpeed_CG1", 0.0},
                        {"WhlRr_W_Meas", "WheelSpeed_CG1", 0.0},
                        {"SteWhlRelInit_An_Sns", "Steering_Wheel_Data_CG1", 0.0},
                        {"Cruise_State", "Cruise_Status", 0.0},
                        {"Set_Speed", "Cruise_Status", 0.0},
                        {"LaActAvail_D_Actl", "Lane_Keep_Assist_Status", 0},
                        {"LaHandsOff_B_Actl", "Lane_Keep_Assist_Status", 0},
                        {"LaActDeny_B_Actl", "Lane_Keep_Assist_Status", 0},
                        {"ApedPosScal_Pc_Actl", "EngineData_14", 0.0},
                        {"Dist_Incr", "Steering_Buttons", 0.0},
                        {"Brake_Drv_Appl", "Cruise_Status", 0.0},
                        {"Brake_Lights", "BCM_to_HS_Body", 0.0}};


CarInterface::CarInterface(bool carcontroller):psr("ford_fusion_2018_pt.dbc", signals, 0)
{
  frame = 0;
  gas_pressed_prev = false;
  brake_pressed_prev = false;
  cruise_enabled_prev = false;

  if(carcontroller){
    // CC = new CarController(cp.dbc_name, CP.enableCamera, VM);
  }
}

CarInterface::~CarInterface()
{
  // if(CC){
  //   delete CC;
  // }
}

CARSTATE CarInterface::update(std::string can_strings)
{
  // ******************* do can recv *******************
  psr.update_strings(can_strings, false);

  CS.update(psr);

  // create message
  CARSTATE ret;

  ret.canValid = psr.can_valid;

  // speeds
  ret.vEgo = CS.v_ego;
  ret.vEgoRaw = CS.v_ego_raw;
  ret.standstill = CS.standstill;
  ret.wheelSpeeds.fl = CS.v_wheel_fl;
  ret.wheelSpeeds.fr = CS.v_wheel_fr;
  ret.wheelSpeeds.rl = CS.v_wheel_rl;
  ret.wheelSpeeds.rr = CS.v_wheel_rr;

  // steering wheel
  ret.steeringAngle = CS.angle_steers;
  ret.steeringPressed = CS.steer_override;

  // gas pedal
  ret.gas = CS.user_gas / 100.;
  ret.gasPressed = CS.user_gas > 0.0001;
  ret.brakePressed = CS.brake_pressed;
  ret.brakeLights = CS.brake_lights;

  ret.cruiseState.enabled = !(CS.pcm_acc_status == 0 || CS.pcm_acc_status == 3);
  ret.cruiseState.speed = CS.v_cruise_pcm;
  ret.cruiseState.available = CS.pcm_acc_status != 0;

  ret.genericToggle = CS.generic_toggle;

  // events
  // std::vector<CarEvent> events;

  if(CS.steer_error){
    ret.events.push_back({steerUnavailable, 0, 1, 0, 0, 0, 1, 0, 1});
  }
   // enable request in prius is simple, as we activate when Toyota is active (rising edge)
  if(ret.cruiseState.enabled && !cruise_enabled_prev){
    ret.events.push_back({pcmEnable, 1, 0, 0, 0, 0, 0, 0, 0});
  }
  else if(!ret.cruiseState.enabled){
    ret.events.push_back({pcmDisable, 0, 0, 0, 1, 0, 0, 0, 0});
  }

  // disable on pedals rising edge or when brake is pressed and speed isn't zero
  if ((ret.gasPressed && !gas_pressed_prev) || (ret.brakePressed && (!brake_pressed_prev || ret.vEgo > 0.001))){
    ret.events.push_back({pedalPressed, 0, 1, 0, 1, 0, 0, 0, 0});
  }

  if(ret.gasPressed){
    ret.events.push_back({pedalPressed, 0, 0, 0, 0, 0, 0, 1, 0});
  }

  if((CS.lkas_state != 2 || CS.lkas_state != 3) && ret.vEgo > 13.0 * MPH_TO_MS && ret.cruiseState.enabled){
    ret.events.push_back({steerTempUnavailableMute, 0, 0, 1, 0, 0, 0, 0, 0});
  }

  // ret.events = events;

  gas_pressed_prev = ret.gasPressed;
  brake_pressed_prev = ret.brakePressed;
  cruise_enabled_prev = ret.cruiseState.enabled;

  return ret;
}

float CarInterface::calc_accel_override(float a_ego, float a_target, float v_ego, float v_target)
{
  return 1.0;
}
#include <iostream>
#include "t_pub.h"
extern bool isActive(cereal::ControlsState::OpenpilotState state);
extern bool isEnabled(cereal::ControlsState::OpenpilotState state);

void car_state_publish2(PubSocket *car_state_sock, PubSocket *car_control_sock, PubSocket *controls_state_sock, 
               CHandler chandler, CARSTATE CS, CARCONTROL &CC ,CarInterface CI, CarParams CP, VehicleModel VM, 
               cereal::ControlsState::OpenpilotState state, ACTUATORS actuators, float v_cruise_kph, 
               RateKeeper rk, LatControlPID LaC, LongControl LoC, bool read_only, double start_time, 
               float v_acc, float a_acc, LATPIDState lac_log, int last_blinker_frame, bool is_ldw_enabled, int can_error_counter)
{
  cout << "---------before 16.0000--------" << endl;
  capnp::MallocMessageBuilder msg;
    cout << "---------before 16.0001--------" << endl;

  auto event = msg.initRoot<cereal::Event>();
    cout << "---------before 16.0002--------" << endl;

  event.setLogMonoTime(nanos_since_boot());
    cout << "---------before 16.0003--------" << endl;

  auto cc_send = event.initCarControl();
            cout << "---------before 16.0--------" << endl;

  cc_send.setEnabled(isEnabled(state));
  
  cereal::CarControl::Actuators::Builder acts = cc_send.initActuators();
  acts.setGas(actuators.gas);
  acts.setBrake(actuators.brake);
  acts.setSteer(actuators.steer);
  acts.setSteerAngle(actuators.steerAngle);
            cout << "---------before 16.1--------" << endl;

  cereal::CarControl::CruiseControl::Builder cruisectl = cc_send.initCruiseControl();
  cruisectl.setOverride(true);
  cruisectl.setCancel(!CP.enableCruise || (!isEnabled(state) && CS.cruiseState.enabled));
            cout << "---------before 16.2--------" << endl;

  // Some override values for Honda
  float brake_discount = (1.0 - clip(actuators.brake * 3., 0.0, 1.0));  // brake discount removes a sharp nonlinearity
  float speedOverride = CP.enableCruise ? ((0.0 < ((LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount)) ? ((LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount) : 0.0) : 0.0;
  cruisectl.setSpeedOverride(speedOverride);
  float accelOverride = CI.calc_accel_override(CS.aEgo, chandler.aTarget, CS.vEgo, chandler.vTarget);
  cruisectl.setAccelOverride(accelOverride);
              cout << "---------before 16.3--------" << endl;

  cereal::CarControl::HUDControl::Builder hud = cc_send.initHudControl();
  hud.setSetSpeed(v_cruise_kph * KPH_TO_MS);
  hud.setSpeedVisible(isEnabled(state));
  hud.setLanesVisible(isEnabled(state));
  hud.setLeadVisible(chandler.hasLead);
  hud.setRightLaneVisible(chandler.rProb > 0.5);
  hud.setLeftLaneVisible(chandler.lProb > 0.5);
            cout << "---------before 16.4--------" << endl;

  bool recent_blinker = (chandler.frame - last_blinker_frame) * DT_CTRL < 5.0;  // 5s blinker cooldown
  bool ldw_allowed = CS.vEgo > 31 * MPH_TO_MS && !recent_blinker && is_ldw_enabled && !isActive(state);
  
  float l_lane_change_prob = 0.0;
  float r_lane_change_prob = 0.0;
  bool l_lane_close = false;
  bool r_lane_close = false;
  bool leftLaneDepart = false;
  bool rightLaneDepart = false;
  bool left_lane_visible = chandler.lProb > 0.5;
  bool right_lane_visible = chandler.rProb > 0.5;
  if(chandler.has_meta_desirePrediction){
    l_lane_change_prob = chandler.meta_desirePrediction[2];
    r_lane_change_prob = chandler.meta_desirePrediction[3];

    l_lane_close = left_lane_visible && (chandler.lPoly[3] < (1.08 - CAMERA_OFFSET));
    r_lane_close = right_lane_visible && (chandler.rPoly[3] > -(1.08 + CAMERA_OFFSET));

    if(ldw_allowed){
      leftLaneDepart = l_lane_change_prob > LANE_DEPARTURE_THRESHOLD && l_lane_close;
      rightLaneDepart = r_lane_change_prob > LANE_DEPARTURE_THRESHOLD && r_lane_close; 
      hud.setLeftLaneDepart(leftLaneDepart);
      hud.setRightLaneDepart(rightLaneDepart);
    }
  }
              cout << "---------before 16.5--------" << endl;

  if(rightLaneDepart || leftLaneDepart){
    // AM.add(sm.frame, 'ldwPermanent', False);
    CS.events.push_back({ldw, 0, 0, 0, 0, 0, 0, 0, 1});
  }

  CC.enabled = isEnabled(state);
  CC.actuators = actuators;
  CC.cruiseControl.override = true;
  CC.cruiseControl.cancel = !CP.enableCruise || (!isEnabled(state) && CS.cruiseState.enabled);
  CC.cruiseControl.speedOverride = speedOverride;
  CC.cruiseControl.accelOverride = accelOverride;
  CC.hudControl.setSpeed = v_cruise_kph * KPH_TO_MS;
  CC.hudControl.speedVisible = isEnabled(state);
  CC.hudControl.lanesVisible = isEnabled(state);
  CC.hudControl.leadVisible = chandler.hasLead;
  CC.hudControl.rightLaneVisible = chandler.rProb > 0.5;
  CC.hudControl.leftLaneVisible = chandler.lProb > 0.5;


  // CC.actuators.brake = ;
  // CC.actuators.steer = ;
  // CC.steeringAngle = ;


  auto cc_words = capnp::messageToFlatArray(msg);
  auto cc_bytes = cc_words.asBytes();
  car_control_sock->send((char*)cc_bytes.begin(), cc_bytes.size());
}
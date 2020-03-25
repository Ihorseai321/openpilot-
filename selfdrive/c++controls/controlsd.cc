#include <cassert>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "common/timing.h"
#include "lib/ctl_handler.h"
#include "lib/utils.h"
#include <cmath>
#include "lib/interface.h"
#include "lib/ratekeeper.h"
#include "lib/longcontrol.h"
#include "lib/latcontrol_pid.h"
#include <iostream>
using namespace std;
// kph
#define V_CRUISE_MAX 144
#define V_CRUISE_MIN 8
#define V_CRUISE_DELTA 8
#define V_CRUISE_ENABLE_MIN 40


bool is_metric = false;
bool is_ldw_enabled = true;
bool passive = false;
bool openpilot_enabled_toggle = true;
bool community_feature_toggle = false;

float update_v_cruise(int v_cruise_kph, std::vector<BUTTONEVENT> buttonEvents, bool enabled)
{
    for(int i = 0; i < buttonEvents.size(); ++i){
        if(enabled && !buttonEvents[i].pressed){
            if(buttonEvents[i].type == accelCruise){
                v_cruise_kph += V_CRUISE_DELTA - ((int)v_cruise_kph % V_CRUISE_DELTA);
            }
            else if(buttonEvents[i].type == decelCruise){
                v_cruise_kph -= V_CRUISE_DELTA - ((int)(V_CRUISE_DELTA - v_cruise_kph) % V_CRUISE_DELTA);
            }

            v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX);
        }
    }

    return v_cruise_kph;
}

float initialize_v_cruise(float v_ego, std::vector<BUTTONEVENT> buttonEvents, int v_cruise_last)
{
    for(int i = 0; i < buttonEvents.size(); ++i){
        // 250kph or above probably means we never had a set speed
        if(buttonEvents[i].type == accelCruise && v_cruise_last < 250){
            return v_cruise_last;
        }
    }

    return int(round(clip(v_ego * MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)));
}

bool get_events(std::vector<CarEvent> events, int type)
{
    for(int i = 0; i < events.size(); ++i){
        switch(type){
            case 1: if(events[i].enable == true){ return true;}
                    break;
            case 2: if(events[i].noEntry == true){ return true;}
                    break;
            case 3: if(events[i].warning == true){ return true;}
                    break;
            case 4: if(events[i].userDisable == true){ return true;}
                    break;
            case 5: if(events[i].softDisable == true){ return true;}
                    break;
            case 6: if(events[i].immediateDisable == true){ return true;}
                    break;
            case 7: if(events[i].preEnable == true){ return true;}
                    break;
            case 8: if(events[i].permanent == true){ return true;}
                    break;
        }
    }

    return false;
}

void add_lane_change_event(std::vector<CarEvent> &events, CHandler chandler)
{
    if(chandler.laneChangeState == cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE){
        if(chandler.laneChangeDirection == cereal::PathPlan::LaneChangeDirection::LEFT){
            events.push_back({preLaneChangeLeft, 0, 0, 1, 0, 0, 0, 0, 0});
        }
        else{
            events.push_back({preLaneChangeRight, 0, 0, 1, 0, 0, 0, 0, 0});
        }
    }
    else if(chandler.laneChangeState == cereal::PathPlan::LaneChangeState::LANE_CHANGE_STARTING || chandler.laneChangeState == cereal::PathPlan::LaneChangeState::LANE_CHANGE_FINISHING){
        events.push_back({laneChange, 0, 0, 1, 0, 0, 0, 0, 0});
    }
}

bool isActive(cereal::ControlsState::OpenpilotState state)
{
    return (state == cereal::ControlsState::OpenpilotState::ENABLED || state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING);
}

bool isEnabled(cereal::ControlsState::OpenpilotState state)
{
    return (isActive(state) || state == cereal::ControlsState::OpenpilotState::PRE_ENABLED);
}

std::string drain_sock_raw(SubSocket *sock)
{
  std::string ret;
  Message *msg;
  while(true){
    if(ret.empty()){
        msg = sock->receive();
        if(msg->getSize() > 0){
            for(int i = 0; i < msg->getSize(); ++i){
                ret += msg->getData()[i];
            }
        }
    }
    else{
      break;
    }
  }

  return ret;
}

CARSTATE data_sample(CarInterface CI, CHandler chandler, SubSocket *can_sock, cereal::ControlsState::OpenpilotState state, int &mismatch_counter, int &can_error_counter, int &cal_perc)
{
    // Update carstate from CAN and create events
    std::string can_strs = drain_sock_raw(can_sock);
    CARSTATE CS = CI.update(can_strs);

    // events = list(CS.events);
    add_lane_change_event(CS.events, chandler);
    bool enabled = isEnabled(state);

    // Check for CAN timeout
    if(can_strs.empty()){
        can_error_counter += 1;
        CS.events.push_back({canError, 0, 1, 0, 0, 0, 1, 0, 0});
    }

    bool overtemp = chandler.thermalStatus >= cereal::ThermalData::ThermalStatus::RED;
    bool free_space = chandler.freeSpace < 0.07;  // under 7% of space free no enable allowed
    bool low_battery = chandler.batteryPercent < 1 && chandler.chargingError;  // at zero percent battery, while discharging, OP should not allowed
    bool mem_low = chandler.memUsedPercent > 90;

    // Create events for battery, temperature and disk space
    if(low_battery){
        CS.events.push_back({lowBattery, 0, 1, 0, 0, 1, 0, 0, 0});
    }

    if(overtemp){
        CS.events.push_back({overheat, 0, 1, 0, 0, 1, 0, 0, 0});
    }

    if(free_space){
        CS.events.push_back({outOfSpace, 0, 1, 0, 0, 0, 0, 0, 0});
    }

    if(mem_low){
        CS.events.push_back({lowMemory, 0, 1, 0, 0, 1, 0, 0, 1});
    }

    if(CS.stockAeb){
        CS.events.push_back({stockAeb, 0, 0, 0, 0, 0, 0, 0, 0});
    }

    // Handle calibration
    int cal_status = chandler.calStatus;
    cal_perc = chandler.calPerc;

    int cal_rpy[3] = {0, 0, 0};
    if(cal_status != 1){
        if(cal_status == 0){
            CS.events.push_back({calibrationIncomplete, 0, 1, 0, 0, 1, 0, 0, 1});
        }
        else{
            CS.events.push_back({calibrationInvalid, 0, 1, 0, 0, 1, 0, 0, 0});
        }
    }
    else{
        if(sizeof(chandler.rpyCalib)/sizeof(chandler.rpyCalib[0]) == 3){
            cal_rpy[0] = chandler.rpyCalib[0];
            cal_rpy[1] = chandler.rpyCalib[1];
            cal_rpy[2] = chandler.rpyCalib[2];
        }
    }

    // When the panda and controlsd do not agree on controls_allowed
    // we want to disengage openpilot. However the status from the panda goes through
    // another socket other than the CAN messages and one can arrive earlier than the other.
    // Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if(enabled){
        mismatch_counter = 0;
    }

    bool controls_allowed = chandler.controlsAllowed;
    if(!controls_allowed && enabled){
        mismatch_counter += 1;
    }
    if(mismatch_counter >= 200){
        CS.events.push_back({controlsMismatch, 0, 0, 0, 0, 0, 1, 0, 0});
    }

    return CS;
}

cereal::ControlsState::OpenpilotState state_transition(CARSTATE CS, CarParams CP, cereal::ControlsState::OpenpilotState state, int &soft_disable_timer, float &v_cruise_kph, float &v_cruise_kph_last)
{
  // Compute conditional state transitions and execute actions on state transitions// 
  bool enabled = isEnabled(state);

  v_cruise_kph_last = v_cruise_kph;

  // if(stock cruise is completely disabled, then we can use our own set speed logic
  if(CP.enableCruise){
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled);
  }
  else if(CP.enableCruise && CS.cruiseState.enabled){
    v_cruise_kph = CS.cruiseState.speed * MS_TO_KPH;
  }

  // decrease the soft disable timer at every step, as it's reset on
  // entrance in SOFT_DISABLING state
  soft_disable_timer = 0 < soft_disable_timer - 1 ? soft_disable_timer - 1 : 0;

  // DISABLED
  if(state == cereal::ControlsState::OpenpilotState::DISABLED){
    if(get_events(CS.events, ENABLE)){
      if(get_events(CS.events, NO_ENTRY)){
      }
      else{
        if(get_events(CS.events, PRE_ENABLE)){
          state = cereal::ControlsState::OpenpilotState::PRE_ENABLED;
        }
        else{
          state = cereal::ControlsState::OpenpilotState::ENABLED;
        }
        v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, v_cruise_kph_last);
      }
    }
  }
  // ENABLED
  else if(state == cereal::ControlsState::OpenpilotState::ENABLED){
    if(get_events(CS.events, USER_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }

    else if(get_events(CS.events, IMMEDIATE_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
    else if(get_events(CS.events, SOFT_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::SOFT_DISABLING;
      soft_disable_timer = 300;   // 3s
    }
  }
  // SOFT DISABLING
  else if(state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING){
    if(get_events(CS.events, USER_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
    else if(get_events(CS.events, IMMEDIATE_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
    else if(not get_events(CS.events, SOFT_DISABLE)){
      // no more soft disabling condition, so go back to ENABLED
      state = cereal::ControlsState::OpenpilotState::ENABLED;
    }
    else if(soft_disable_timer <= 0){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
  }
  // PRE ENABLING
  else if(state == cereal::ControlsState::OpenpilotState::PRE_ENABLED){
    if(get_events(CS.events, USER_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
    else if(get_events(CS.events, IMMEDIATE_DISABLE) || get_events(CS.events, SOFT_DISABLE)){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
    else if(!get_events(CS.events, PRE_ENABLE)){
      state = cereal::ControlsState::OpenpilotState::ENABLED;
    }
  }

  return state;
}

LATPIDState state_control(CHandler chandler, ACTUATORS &actuators, CARSTATE CS, CarParams CP, cereal::ControlsState::OpenpilotState state, float &v_cruise_kph, float v_cruise_kph_last,
                   RateKeeper rk, LatControlPID LaC, LongControl LoC, bool read_only, bool is_metric, int cal_perc, int frame, int &last_blinker_frame, float &v_acc_sol, float &a_acc_sol)
{
  // Given the state, this function returns an actuators packet// 
  bool enabled = isEnabled(state);
  bool active = isActive(state);

  // check if user has interacted with the car
  bool driver_engaged = CS.buttonEvents.size() > 0 || v_cruise_kph != v_cruise_kph_last || CS.steeringPressed;

  if(CS.leftBlinker || CS.rightBlinker){
    last_blinker_frame = frame;
  }

  if(state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::DISABLED){
    LaC.reset();
    LoC.reset(CS.vEgo);
  }

  float plan_age = DT_CTRL * (frame - chandler.rcv_frame_plan);
  float dt = plan_age < (LON_MPC_STEP + DT_CTRL) ? (plan_age + DT_CTRL) : (LON_MPC_STEP + DT_CTRL + DT_CTRL); // no greater than dt mpc + dt, to prevent too high extraps

  a_acc_sol = chandler.aStart + (dt / LON_MPC_STEP) * (chandler.aTarget - chandler.aStart);
  v_acc_sol = chandler.vStart + dt * (a_acc_sol + chandler.aStart) / 2.0;

  // Gas/Brake PID loop
  LCtrlRet loc_ret = LoC.update(active, CS.vEgo, CS.brakePressed, CS.standstill, CS.cruiseState.standstill,
                                              v_cruise_kph, v_acc_sol, chandler.vTargetFuture, a_acc_sol);
  actuators.gas = loc_ret.final_gas;
  actuators.brake = loc_ret.final_brake; 
  // Steering PID loop and lateral MPC
  LatPIDRet ret= LaC.update(active, CS.vEgo, CS.steeringAngle, CS.steeringRate, CS.steeringTorqueEps, 
                            CS.steeringPressed, CS.steeringRateLimited, chandler);
  
  actuators.steer = ret.output_steer;
  actuators.steerAngle = ret.angle_steers_des;

  return ret.pid_log;
}

void data_send(PubSocket *car_state_sock, PubSocket *car_control_sock, PubSocket *controls_state_sock, 
               CHandler chandler, CARSTATE CS, CARCONTROL &CC ,CarInterface CI, CarParams CP, VehicleModel VM, 
               cereal::ControlsState::OpenpilotState state, ACTUATORS actuators, float v_cruise_kph, 
               RateKeeper rk, LatControlPID LaC, LongControl LoC, bool read_only, double start_time, 
               float v_acc, float a_acc, LATPIDState lac_log, int last_blinker_frame, bool is_ldw_enabled, int can_error_counter)
{
  //"""Send actuators and hud commands to the car, send controlsstate and MPC logging"""
  // carControl
  capnp::MallocMessageBuilder cc_msg;
  cereal::Event::Builder cc_event = cc_msg.initRoot<cereal::Event>();
  cc_event.setLogMonoTime(nanos_since_boot());
  auto cc_send = cc_event.initCarControl();

  cc_send.setEnabled(isEnabled(state));
  
  cereal::CarControl::Actuators::Builder acts = cc_send.initActuators();
  acts.setGas(actuators.gas);
  acts.setBrake(actuators.brake);
  acts.setSteer(actuators.steer);
  acts.setSteerAngle(actuators.steerAngle);

  cereal::CarControl::CruiseControl::Builder cruisectl = cc_send.initCruiseControl();
  cruisectl.setOverride(true);
  cruisectl.setCancel(!CP.enableCruise || (!isEnabled(state) && CS.cruiseState.enabled));

  // Some override values for Honda
  float brake_discount = (1.0 - clip(actuators.brake * 3., 0.0, 1.0));  // brake discount removes a sharp nonlinearity
  float speedOverride = CP.enableCruise ? ((0.0 < ((LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount)) ? ((LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount) : 0.0) : 0.0;
  cruisectl.setSpeedOverride(speedOverride);
  float accelOverride = CI.calc_accel_override(CS.aEgo, chandler.aTarget, CS.vEgo, chandler.vTarget);
  cruisectl.setAccelOverride(accelOverride);
  
  cereal::CarControl::HUDControl::Builder hud = cc_send.initHudControl();
  hud.setSetSpeed(v_cruise_kph * KPH_TO_MS);
  hud.setSpeedVisible(isEnabled(state));
  hud.setLanesVisible(isEnabled(state));
  hud.setLeadVisible(chandler.hasLead);
  hud.setRightLaneVisible(chandler.rProb > 0.5);
  hud.setLeftLaneVisible(chandler.lProb > 0.5);

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


  auto cc_words = capnp::messageToFlatArray(cc_msg);
  auto cc_bytes = cc_words.asBytes();
  car_control_sock->send((char*)cc_bytes.begin(), cc_bytes.size());

  // CARCONTROL CC;
  // CC.enabled = isEnabled(state);
  // CC.actuators = actuators;

  // CC.cruiseControl.override = true;
  // CC.cruiseControl.cancel = !CP.enableCruise || (!isEnabled(state) && CS.cruiseState.enabled);
  

  // controlsState
  bool force_decel = false;
  
  capnp::MallocMessageBuilder ctls_msg;
  cereal::Event::Builder ctls_event = ctls_msg.initRoot<cereal::Event>();
  ctls_event.setLogMonoTime(nanos_since_boot());
  ctls_event.setValid(CS.canValid);
  auto ctls_send = ctls_event.initControlsState();

  ctls_send.setPlanMonoTime(chandler.planMonoTime);
  ctls_send.setPathPlanMonoTime(chandler.pathPlanMonoTime);
  ctls_send.setEnabled(isEnabled(state));
  ctls_send.setActive(isActive(state));
  ctls_send.setVEgo(CS.vEgo);
  ctls_send.setVEgoRaw(CS.vEgoRaw);
  ctls_send.setAngleSteers(CS.steeringAngle);
  ctls_send.setCurvature(VM.calc_curvature((CS.steeringAngle - chandler.angleOffset) * DEG_TO_RAD, CS.vEgo));
  ctls_send.setSteerOverride(CS.steeringPressed);
  ctls_send.setState(state);
  ctls_send.setEngageable(!get_events(CS.events, NO_ENTRY));
  ctls_send.setLongControlState(LoC.long_control_state);
  ctls_send.setVPid(LoC.v_pid);
  ctls_send.setVCruise(v_cruise_kph);
  ctls_send.setUpAccelCmd(LoC.pid.p);
  ctls_send.setUiAccelCmd(LoC.pid.i);
  ctls_send.setUfAccelCmd(LoC.pid.f);
  ctls_send.setAngleSteersDes(LaC.angle_steers_des);
  ctls_send.setVTargetLead(v_acc);
  ctls_send.setATarget(a_acc);
  ctls_send.setJerkFactor(chandler.jerkFactor);
  ctls_send.setGpsPlannerActive(chandler.gpsPlannerActive);
  ctls_send.setVCurvature(chandler.vCurvature);
  ctls_send.setDecelForModel(chandler.longitudinalPlanSource == cereal::Plan::LongitudinalPlanSource::MODEL);
  ctls_send.setStartMonoTime((unsigned long)start_time * 1e9);
  ctls_send.setMapValid(chandler.mapValid);
  ctls_send.setForceDecel(force_decel);
  ctls_send.setCanErrorCounter(can_error_counter);
  ctls_send.setCumLagMs(-rk.getremaining() * 1000.);

  cereal::ControlsState::LateralControlState::Builder lat_ctls = ctls_send.initLateralControlState();
  cereal::ControlsState::LateralPIDState::Builder pid_state = lat_ctls.initPidState();
  pid_state.setActive(lac_log.active);
  pid_state.setSteerAngle(lac_log.steerAngle);
  pid_state.setSteerRate(lac_log.steerRate);
  pid_state.setAngleError(lac_log.angleError);
  pid_state.setP(lac_log.p);
  pid_state.setI(lac_log.i);
  pid_state.setF(lac_log.f);
  pid_state.setOutput(lac_log.output);
  pid_state.setSaturated(lac_log.saturated);
    
  auto ctls_words = capnp::messageToFlatArray(ctls_msg);
  auto ctls_bytes = ctls_words.asBytes();
  controls_state_sock->send((char*)ctls_bytes.begin(), ctls_bytes.size());

  // carState
  capnp::MallocMessageBuilder cs_msg;
  cereal::Event::Builder cs_event = cs_msg.initRoot<cereal::Event>();
  cs_event.setLogMonoTime(nanos_since_boot());
  cs_event.setValid(CS.canValid);
  auto cs_send = cs_event.initCarState();

  cereal::CarState::WheelSpeeds::Builder wheelspeeds = cs_send.initWheelSpeeds();
  wheelspeeds.setFl(CS.wheelSpeeds.fl);
  wheelspeeds.setFr(CS.wheelSpeeds.fr);
  wheelspeeds.setRl(CS.wheelSpeeds.rl);
  wheelspeeds.setRr(CS.wheelSpeeds.rr);

  cereal::CarState::CruiseState::Builder cruise_state = cs_send.initCruiseState();
  cruise_state.setEnabled(CS.cruiseState.enabled);
  cruise_state.setSpeed(CS.cruiseState.speed);
  cruise_state.setAvailable(CS.cruiseState.available);
  cruise_state.setSpeedOffset(CS.cruiseState.speedOffset);
  cruise_state.setStandstill(CS.cruiseState.standstill);
  
  capnp::List<::cereal::CarState::ButtonEvent>::Builder btn_events = cs_send.initButtonEvents(CS.buttonEvents.size());
  for (int btn_i = 0; btn_i < CS.buttonEvents.size(); ++btn_i) {
    btn_events[btn_i].setPressed(CS.buttonEvents[btn_i].pressed);
    btn_events[btn_i].setType((cereal::CarState::ButtonEvent::Type)CS.buttonEvents[btn_i].type);
  }
  
  kj::ArrayPtr<const uint64_t> canmonotimes(&CS.canMonoTimes[0], CS.canMonoTimes.size());
  cs_send.setCanMonoTimes(canmonotimes);

  capnp::List< ::cereal::CarEvent>::Builder events = cs_send.initEvents(CS.events.size());
  for (int evt_i = 0; evt_i < CS.events.size(); ++evt_i) {
    events[evt_i].setName((cereal::CarEvent::EventName)CS.events[evt_i].name);
    events[evt_i].setEnable(CS.events[evt_i].enable);
    events[evt_i].setNoEntry(CS.events[evt_i].noEntry);
    events[evt_i].setWarning(CS.events[evt_i].warning);
    events[evt_i].setUserDisable(CS.events[evt_i].userDisable);
    events[evt_i].setSoftDisable(CS.events[evt_i].softDisable);
    events[evt_i].setImmediateDisable(CS.events[evt_i].immediateDisable);
    events[evt_i].setPreEnable(CS.events[evt_i].preEnable);
    events[evt_i].setPermanent(CS.events[evt_i].permanent);
  }

  cs_send.setVEgo(CS.vEgo);
  cs_send.setAEgo(CS.aEgo);
  cs_send.setGas(CS.gas);
  cs_send.setGasPressed(CS.gasPressed);
  cs_send.setBrake(CS.brake);
  cs_send.setBrakePressed(CS.brakePressed);
  cs_send.setSteeringAngle(CS.steeringAngle);
  cs_send.setSteeringTorque(CS.steeringTorque);
  cs_send.setSteeringPressed(CS.steeringPressed);
  cs_send.setSteeringRate(CS.steeringRate);
  cs_send.setVEgoRaw(CS.vEgoRaw);
  cs_send.setStandstill(CS.standstill);
  cs_send.setBrakeLights(CS.brakeLights);
  cs_send.setLeftBlinker(CS.leftBlinker);
  cs_send.setRightBlinker(CS.rightBlinker);
  cs_send.setYawRate(CS.yawRate);
  cs_send.setGenericToggle(CS.genericToggle);
  cs_send.setDoorOpen(CS.doorOpen);
  cs_send.setSeatbeltUnlatched(CS.seatbeltUnlatched);
  cs_send.setCanValid(CS.canValid);
  cs_send.setSteeringTorqueEps(CS.steeringTorqueEps);
  cs_send.setClutchPressed(CS.clutchPressed);
  cs_send.setSteeringRateLimited(CS.steeringRateLimited);
  cs_send.setStockAeb(CS.stockAeb);
  cs_send.setStockFcw(CS.stockFcw);
  cs_send.setGearShifter((cereal::CarState::GearShifter)CS.gearShifter);

  auto cs_words = capnp::messageToFlatArray(cs_msg);
  auto cs_bytes = cs_words.asBytes();
  car_state_sock->send((char*)cs_bytes.begin(), cs_bytes.size());
  // TODO
  // cs_send.carState = CS;
  // cs_send.carState.events = events;
  // return CC;
}

int main(int argc, char const *argv[])
{
  cout << "---------main--------" << endl;
    CARCONTROL CC;
    CARSTATE CS;
    ACTUATORS actuators;
    bool has_relay = false;

    int frame = -1;
        cout << "---------before 1--------" << endl;

    LongControl LoC;
        cout << "---------before 1.2--------" << endl;

    LatControlPID LaC;
        cout << "---------before 1.4--------" << endl;

    CarParams CP;
    VehicleModel VM;
        cout << "---------before 1.6--------" << endl;

    
        cout << "---------before 1.8--------" << endl;

    CHandler chandler;
        cout << "---------before 1.9--------" << endl;

    RateKeeper rk(100);
    cout << "---------before 2--------" << endl;

    LATPIDState lac_log;
    cout << "---------before 3--------" << endl;

    cereal::ControlsState::OpenpilotState state = cereal::ControlsState::OpenpilotState::DISABLED;
    int soft_disable_timer = 0;
    float v_cruise_kph = 255;
    float v_cruise_kph_last = 0;
    int mismatch_counter = 0;
    int can_error_counter = 0;
    int last_blinker_frame = 0;
    int cal_perc = 0;
    cout << "---------before 4--------" << endl;

    bool internet_needed = false;
    
    chandler.calStatus = 2;
    chandler.sensorValid = true;
    chandler.posenetValid = true;
    chandler.freeSpace = 1.0;
    cout << "---------before 5--------" << endl;

    float v_acc = 0.; 
    float a_acc = 0.;

    bool read_only = true;
    bool community_feature_disallowed = false;

    float v_acc_sol = 0.0;
    float a_acc_sol = 0.0;
    cout << "---------before 6--------" << endl;

    Context * c = Context::create();
    SubSocket *thermal_sock = SubSocket::create(c, "thermal");
    SubSocket *health_sock = SubSocket::create(c, "health");
    SubSocket *live_calibration_sock = SubSocket::create(c, "liveCalibration");
    SubSocket *driver_monitoring_sock = SubSocket::create(c, "driverMonitoring");
    SubSocket *plan_sock = SubSocket::create(c, "plan");
    SubSocket *path_plan_sock = SubSocket::create(c, "pathPlan");
    SubSocket *model_sock = SubSocket::create(c, "model");
    SubSocket *gps_location_sock = SubSocket::create(c, "gpsLocation");
    SubSocket *can_sock = SubSocket::create(c, "can");

    cout << "---------before 7--------" << endl;

    // PubSocket *sendcan_sock = PubSocket::create(c, "sendcan");
    PubSocket *controls_state_sock = PubSocket::create(c, "controlsState");
    PubSocket *car_state_sock = PubSocket::create(c, "carState");
    PubSocket *car_control_sock = PubSocket::create(c, "carControl");
    // PubSocket *car_events_sock = PubSocket::create(c, "carEvents");
    assert(thermal_sock != NULL);
    assert(health_sock != NULL);
    assert(live_calibration_sock != NULL);
    assert(driver_monitoring_sock != NULL);
    assert(plan_sock != NULL);
    assert(path_plan_sock != NULL);
    assert(model_sock != NULL);
    assert(gps_location_sock != NULL);
    assert(can_sock != NULL);

    // assert(sendcan_sock != NULL);
    assert(controls_state_sock != NULL);
    assert(car_state_sock != NULL);
    assert(car_control_sock != NULL);
    // assert(car_events_sock != NULL);
    cout << "---------before 8--------" << endl;

    CarInterface CI(false);
    Poller * poller = Poller::create({thermal_sock, health_sock, live_calibration_sock, 
                                      driver_monitoring_sock, plan_sock, path_plan_sock, 
                                      model_sock, gps_location_sock});
    while (true){
        frame += 1;
        for (auto s : poller->poll(100)){
            Message * msg = s->receive();
            auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
            memcpy(amsg.begin(), msg->getData(), msg->getSize());
            capnp::FlatArrayMessageReader capnp_msg(amsg);
            cereal::Event::Reader event = capnp_msg.getRoot<cereal::Event>();

            chandler.chandle_log(event, frame);

            delete msg;
        }
        double start_time = sec_since_boot();

        CS = data_sample(CI, chandler, can_sock, state, mismatch_counter, can_error_counter, cal_perc);
            
        if(!chandler.mpcSolutionValid){
            CS.events.push_back({plannerError, 0, 1, 0, 0, 0, 1, 0, 0});
        }
        if(!chandler.sensorValid){
            CS.events.push_back({sensorDataInvalid, 0, 1, 0, 0, 0, 0, 0, 1});
        }
        if(!chandler.paramsValid){
            CS.events.push_back({vehicleModelInvalid, 0, 0, 1, 0, 0, 0, 0, 0});
        }
        if(!chandler.posenetValid){
            CS.events.push_back({posenetInvalid, 0, 1, 1, 0, 0, 0, 0, 0});
        }
        if(!chandler.radarValid){
            CS.events.push_back({radarFault, 0, 1, 0, 0, 1, 0, 0, 0});
        }
        if(chandler.radarCanError){
           CS.events.push_back({radarCanError, 0, 1, 0, 0, 1, 0, 0, 0});
        }
        if(!CS.canValid){
            CS.events.push_back({canError, 0, 1, 0, 0, 0, 1, 0, 0});
        }
        // if(!sounds_available){
        //     CS.events.push_back({soundsUnavailable, 0, 1, 0, 0, 0, 0, 0, 1});
        // }
        if(internet_needed){
            CS.events.push_back({internetConnectivityNeeded, 0, 1, 0, 0, 0, 0, 0, 1});
        }
        if(community_feature_disallowed){
            CS.events.push_back({communityFeatureDisallowed, 0, 0, 0, 0, 0, 0, 0, 1});
        }
        if(read_only && !passive){
            CS.events.push_back({carUnrecognized, 0, 0, 0, 0, 0, 0, 0, 1});
        }

        // Only allow engagement with brake pressed when stopped behind another stopped car
        if(CS.brakePressed && chandler.vTargetFuture >= 0.5 && false && CS.vEgo < 0.3){
            CS.events.push_back({noTarget, 0, 1, 0, 0, 0, 1, 0, 0});
        }
        if(!read_only){
            // update control state
            state = state_transition(CS, CP, state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last);
            // Compute actuators (runs PID loops and lateral MPC)
        }

        lac_log = state_control(chandler, actuators, CS, CP, state, v_cruise_kph, v_cruise_kph_last, 
                                rk, LaC, LoC, read_only, is_metric, cal_perc, frame, last_blinker_frame, 
                                v_acc_sol, a_acc_sol);
      
        data_send(car_state_sock, car_control_sock, controls_state_sock, chandler, CS, CC,
                  CI, CP, VM, state, actuators, v_cruise_kph, rk, LaC, LoC, read_only, 
                  start_time, v_acc, a_acc, lac_log, last_blinker_frame, is_ldw_enabled, 
                  can_error_counter);
        rk.monitor_time();
    }

    delete thermal_sock;
    delete health_sock;
    delete live_calibration_sock;
    delete driver_monitoring_sock;
    delete plan_sock;
    delete path_plan_sock;
    delete model_sock;
    delete gps_location_sock;
    delete can_sock;

    // assert(sendcan_sock != NULL);
    delete controls_state_sock;
    delete car_state_sock;
    delete car_control_sock;
    delete poller;
    delete c;
    return 0;
}
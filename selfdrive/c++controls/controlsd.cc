#include <cassert>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "controlsd_handler.h"
#include "lib/utils.h"
#define ENABLE 1
#define PRE_ENABLE 2
#define NO_ENTRY 3
#define WARNING 4
#define USER_DISABLE 5
#define SOFT_DISABLE 6
#define IMMEDIATE_DISABLE 7
#define PERMANENT 8

typedef struct{
    // range from 0.0 - 1.0
    float gas;
    float brake;
    // range from -1.0 - 1.0
    float steer;
    float steerAngle;
}Actuators; 

typedef struct{
  bool cancel;
  bool override;
  float speedOverride;
  float accelOverride;
}CruiseControl;

typedef struct{
  bool speedVisible;
  setSpeed @1: Float32;
  bool lanesVisible;
  bool leadVisible;
  VisualAlert visualAlert;
  AudibleAlert audibleAlert;
  bool rightLaneVisible;
  bool leftLaneVisible;
  bool rightLaneDepart;
  bool leftLaneDepart;

  enum VisualAlert {
    // these are the choices from the Honda
    // map as good as you can for your car
    none @0;
    fcw @1;
    steerRequired @2;
    brakePressed @3;
    wrongGear @4;
    seatbeltUnbuckled @5;
    speedTooHigh @6;
    ldw @7;
  }

  enum AudibleAlert {
    // these are the choices from the Honda
    // map as good as you can for your car
    none @0;
    chimeEngage @1;
    chimeDisengage @2;
    chimeError @3;
    chimeWarning1 @4;
    chimeWarning2 @5;
    chimeWarningRepeat @6;
    chimePrompt @7;
  }

}HUDControl;

typedef struct{
  bool enabled;
  bool active;

  float gasDEPRECATED;
  float brakeDEPRECATED;
  float steeringTorqueDEPRECATED;

  Actuators actuators;

  CruiseControl cruiseControl;
  HUDControl hudControl;
}CARCONTROL;

bool is_metric = false;
bool is_ldw_enabled = true;
bool passive = false;
bool openpilot_enabled_toggle = true;
bool community_feature_toggle = false;
std::map <std::string, std::vector<int>> events;

void add_lane_change_event(CHandler chandler)
{
    std::vector<int> type;
    if(chandler.laneChangeState == cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE){
        if(chandler.laneChangeDirection == cereal::PathPlan::LaneChangeDirection::LEFT){
            type.push_back(WARNING);
            events.insert(std::map<std::string, std::vector<int>>::value_type("preLaneChangeLeft", type));
        }
        else{
            type.push_back(WARNING);
            events.insert(std::map<std::string, std::vector<int>>::value_type("preLaneChangeRight", type));
        }
    }
    else if(chandler.laneChangeState == cereal::PathPlan::LaneChangeState::LANE_CHANGE_STARTING, cereal::PathPlan::LaneChangeState::LANE_CHANGE_FINISHING){
        type.push_back(WARNING);
        events.insert(std::map<std::string, std::vector<int>>::value_type("laneChange", type));
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

void data_sample(CarInterface CI, CARCONTROL CC, CHandler chandler, SubSocket *can_sock, cereal::ControlsState::OpenpilotState state, int &mismatch_counter, int &can_error_counter)
{
    std::vector<int> type;
    // Update carstate from CAN and create events
    // can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=true);
    // CS = CI.update(can_strs);

    // events = list(CS.events);
    add_lane_change_event(chandler);
    bool enabled = isEnabled(state);

    // Check for CAN timeout
    if(!can_strs){
        can_error_counter += 1;
        type.push_back(NO_ENTRY);
        type.push_back(IMMEDIATE_DISABLE);
        events.insert(std::map<std::string, std::vector<int>>::value_type("canError", type));
        type.clear();
    }

    bool overtemp = chandler.thermalStatus >= cereal::ThermalData::ThermalStatus::RED;
    bool free_space = chandler.freeSpace < 0.07;  // under 7% of space free no enable allowed
    bool low_battery = chandler.batteryPercent < 1 && chandler.chargingError;  // at zero percent battery, while discharging, OP should not allowed
    bool mem_low = chandler.memUsedPercent > 90;

    // Create events for battery, temperature and disk space
    if(low_battery){
        type.push_back(NO_ENTRY);
        type.push_back(SOFT_DISABLE);
        events.insert(std::map<std::string, std::vector<int>>::value_type("lowBattery", type));
        type.clear();
    }

    if(overtemp){
        type.push_back(NO_ENTRY);
        type.push_back(SOFT_DISABLE);
        events.insert(std::map<std::string, std::vector<int>>::value_type("overheat", type));
        type.clear();
    }

    if(free_space){
        type.push_back(NO_ENTRY);
        events.insert(std::map<std::string, std::vector<int>>::value_type("outOfSpace", type));
        type.clear();
    }

    if(mem_low){
        type.push_back(NO_ENTRY);
        type.push_back(SOFT_DISABLE);
        type.push_back(PERMANENT);
        events.insert(std::map<std::string, std::vector<int>>::value_type("lowMemory", type));
        type.clear();
    }

    if(CS.stockAeb){
        events.insert(std::map<std::string, std::vector<int>>::value_type("stockAeb", type));
    }

    // Handle calibration
    cal_status = chandler.calStatus;
    cal_perc = chandler.calPerc;

    int cal_rpy[3] = {0, 0, 0};
    if(cal_status != 1){
        if(cal_status == 0){
            type.push_back(NO_ENTRY);
            type.push_back(SOFT_DISABLE);
            type.push_back(PERMANENT);
            events.insert(std::map<std::string, std::vector<int>>::value_type("calibrationIncomplete", type));
            type.clear();
        }
        else{
            type.push_back(NO_ENTRY);
            type.push_back(SOFT_DISABLE);
            events.insert(std::map<std::string, std::vector<int>>::value_type("calibrationInvalid", type));
            type.clear();
        }
    }
    else{
        if(len(rpy) == 3){
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
        type.push_back(IMMEDIATE_DISABLE);
        events.insert(std::map<std::string, std::vector<int>>::value_type("controlsMismatch", type));
        type.clear();
    }

    return CS;
}

void state_transition(frame, CS, cereal::ControlsState::OpenpilotState state, soft_disable_timer, v_cruise_kph){
{
  // Compute conditional state transitions and execute actions on state transitions// 
  bool enabled = isEnabled(state);

  float v_cruise_kph_last = v_cruise_kph;

  // if(stock cruise is completely disabled, then we can use our own set speed logic
  if(CP.enableCruise){
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled);
  }
  else if(CP.enableCruise && CS.cruiseState.enabled){
    v_cruise_kph = CS.cruiseState.speed * MS_TO_KPH;
  }

  // decrease the soft disable timer at every step, as it's reset on
  // entrance in SOFT_DISABLING state
  soft_disable_timer = max(0, soft_disable_timer - 1);

  // DISABLED
  if(state == cereal::ControlsState::OpenpilotState::DISABLED){
    if(get_events(events, [ET.ENABLE])){
      if(get_events(events, [ET.NO_ENTRY])){
        for e in get_events(events, [ET.NO_ENTRY]){
          AM.add(frame, str(e) + "NoEntry", enabled);
        }
      }
      else{
        if(get_events(events, [ET.PRE_ENABLE])){
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
    if(get_events(events, [ET.USER_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }

    else if(get_events(events, [ET.IMMEDIATE_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]){
        AM.add(frame, e, enabled);
      }
    }
    else if(get_events(events, [ET.SOFT_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::SOFT_DISABLING;
      soft_disable_timer = 300;   // 3s
      for e in get_events(events, [ET.SOFT_DISABLE]){
        AM.add(frame, e, enabled);
      }
    }
  }
  // SOFT DISABLING
  else if(state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING){
    if(get_events(events, [ET.USER_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      AM.add(frame, "disable", enabled);
    }
    else if(get_events(events, [ET.IMMEDIATE_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]){
        AM.add(frame, e, enabled);
      }
    }
    else if(not get_events(events, [ET.SOFT_DISABLE])){
      // no more soft disabling condition, so go back to ENABLED
      state = cereal::ControlsState::OpenpilotState::ENABLED;
    }
    else if(get_events(events, [ET.SOFT_DISABLE]) and soft_disable_timer > 0){
      for e in get_events(events, [ET.SOFT_DISABLE]){
        AM.add(frame, e, enabled);
      }
    }

    else if(soft_disable_timer <= 0){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
    }
  }
  // PRE ENABLING
  else if(state == cereal::ControlsState::OpenpilotState::PRE_ENABLED){
    if(get_events(events, [ET.USER_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      AM.add(frame, "disable", enabled);
    }
    else if(get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE])){
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for e in get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]){
        AM.add(frame, e, enabled);
      }
    }
    else if(not get_events(events, [ET.PRE_ENABLE])){
      state = cereal::ControlsState::OpenpilotState::ENABLED;
    }
  }

  return state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last;
}

void state_control(frame, rcv_frame, plan, path_plan, CS, CP, state, events, v_cruise_kph, v_cruise_kph_last,
                  AM, rk, driver_status, LaC, LoC, read_only, is_metric, cal_perc, last_blinker_frame)
{
  // Given the state, this function returns an actuators packet// 

  actuators = car.CarControl.Actuators.new_message();

  bool enabled = isEnabled(state);
  bool active = isActive(state);

  // check if user has interacted with the car
  bool driver_engaged = len(CS.buttonEvents) > 0 || v_cruise_kph != v_cruise_kph_last || CS.steeringPressed;

  if(CS.leftBlinker || CS.rightBlinker){
    last_blinker_frame = frame;
  }

  // add eventual driver distracted events
  events = driver_status.update(events, driver_engaged, isActive(state), CS.standstill);

  if(plan.fcw){
    // send FCW alert if triggered by planner
    AM.add(frame, "fcw", enabled);
  }
  else if(CS.stockFcw){
    // send a silent alert when stock fcw triggers, since the car is already beeping
    AM.add(frame, "fcwStock", enabled);
  // State specific actions
  }

  if(state in [State.preEnabled, State.disabled]){
    LaC.reset();
    LoC.reset(v_pid=CS.vEgo);
  }
  else if(state in [State.enabled, State.softDisabling]){
    // parse warnings from car specific interface
    for(e in get_events(events, [ET.WARNING])){
      extra_text = "";
      if(e == "belowSteerSpeed"){
        if(is_metric){
          extra_text = str(int(round(CP.minSteerSpeed * MS_TO_KPH))) + " kph";
        }
        else{
          extra_text = str(int(round(CP.minSteerSpeed * MS_TO_MPH))) + " mph";
        }
      }
      AM.add(frame, e, enabled, extra_text_2=extra_text);
    }
  }

  plan_age = DT_CTRL * (frame - rcv_frame['plan']);
  dt = min(plan_age, LON_MPC_STEP + DT_CTRL) + DT_CTRL; // no greater than dt mpc + dt, to prevent too high extraps

  a_acc_sol = plan.aStart + (dt / LON_MPC_STEP) * (plan.aTarget - plan.aStart);
  v_acc_sol = plan.vStart + dt * (a_acc_sol + plan.aStart) / 2.0;

  // Gas/Brake PID loop
  actuators.gas, actuators.brake = LoC.update(active, CS.vEgo, CS.brakePressed, CS.standstill, CS.cruiseState.standstill,
                                              v_cruise_kph, v_acc_sol, plan.vTargetFuture, a_acc_sol, CP);
  // Steering PID loop and lateral MPC
  actuators.steer, actuators.steerAngle, lac_log = LaC.update(active, CS.vEgo, CS.steeringAngle, CS.steeringRate, CS.steeringTorqueEps, CS.steeringPressed, CS.steeringRateLimited, CP, path_plan);

  // Send a "steering required alert" if saturation count has reached the limit
  if(lac_log.saturated && !CS.steeringPressed){
    // Check if we deviated from the path
    bool left_deviation = actuators.steer > 0 && path_plan.dPoly[3] > 0.1;
    bool right_deviation = actuators.steer < 0 && path_plan.dPoly[3] < -0.1;

    if(left_deviation && right_deviation){
      AM.add(frame, "steerSaturated", enabled);
    }
  }
  // Parse permanent warnings to display constantly
  for e in get_events(events, [ET.PERMANENT]){
    extra_text_1, extra_text_2 = "", "";
    if(e == "calibrationIncomplete"){
      extra_text_1 = str(cal_perc) + "%";
      if(is_metric){
        extra_text_2 = str(int(round(Filter.MIN_SPEED * MS_TO_KPH))) + " kph";
      }
      else{
        extra_text_2 = str(int(round(Filter.MIN_SPEED * MS_TO_MPH))) + " mph";
      }
    }
    AM.add(frame, str(e) + "Permanent", enabled, extra_text_1=extra_text_1, extra_text_2=extra_text_2);
  }

  return actuators, v_cruise_kph, driver_status, v_acc_sol, a_acc_sol, lac_log, last_blinker_frame;
}

void data_send(PubSocket *car_state_sock, PubSocket *car_control_sock, PubSocket *controls_state_sock, CHandler chandler, CS, CI, CP, VM, cereal::ControlsState::OpenpilotState state, actuators, v_cruise_kph, rk, AM,
              driver_status, LaC, LoC, read_only, start_time, v_acc, a_acc, lac_log, events_prev,
              last_blinker_frame, is_ldw_enabled, can_error_counter)
{
  //"""Send actuators and hud commands to the car, send controlsstate and MPC logging"""
  // carControl
  capnp::MallocMessageBuilder cc_msg;
  cereal::Event::Builder cc_event = cc_msg.initRoot<cereal::Event>();
  cc_event.setLogMonoTime(nanos_since_boot());
  auto cc_send = cc_event.initCarControl();

  cc_send.setEnabled(isEnabled(state));
  cc_send.setValid(CS.canValid);
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
  float speedOverride = CP.enableCruise ? max(0.0, (LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount) : 0.0;
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

  bool recent_blinker = (sm.frame - last_blinker_frame) * DT_CTRL < 5.0;  // 5s blinker cooldown
  bool ldw_allowed = CS.vEgo > 31 * MPH_TO_MS && !recent_blinker && is_ldw_enabled && !isActive(state);
  
  float l_lane_change_prob = 0.0;
  float r_lane_change_prob = 0.0;
  bool l_lane_close = false;
  bool r_lane_close = false;
  bool leftLaneDepart = false;
  bool rightLaneDepart = false;
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
  
  std::vector<int> type;
  if(rightLaneDepart || leftLaneDepart){
    // AM.add(sm.frame, 'ldwPermanent', False);
    type.push_back(PERMANENT);
    events.insert(std::map<std::string, std::vector<int>>::value_type("ldw", type));
    type.clear();
  }

  auto words = capnp::messageToFlatArray(cc_msg);
  auto bytes = words.asBytes();
  car_control_sock->send((char*)bytes.begin(), bytes.size());
  // AM.process_alerts(sm.frame);
  // CC.hudControl.visualAlert = AM.visual_alert;

  // controlsState
  bool force_decel = false;
  
  capnp::MallocMessageBuilder ctls_msg;
  cereal::Event::Builder ctls_event = ctls_msg.initRoot<cereal::Event>();
  ctls_event.setLogMonoTime(nanos_since_boot());
  auto ctls_send = ctls_event.initControlsState();

  ctls_send.setValid(CS.canValid);
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
  ctls_send.setEngageable(!get_events(events, [ET.NO_ENTRY]));
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
  ctls_send.setCumLagMs(-rk.remaining * 1000.);

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
    
  auto words = capnp::messageToFlatArray(ctls_msg);
  auto bytes = words.asBytes();
  controls_state_sock->send((char*)bytes.begin(), bytes.size());

  // carState
  capnp::MallocMessageBuilder cs_msg;
  cereal::Event::Builder cs_event = cs_msg.initRoot<cereal::Event>();
  cs_event.setLogMonoTime(nanos_since_boot());
  auto cs_send = cs_event.initCarState();

  cs_send.setValid(CS.canValid);

  auto words = capnp::messageToFlatArray(cs_msg);
  auto bytes = words.asBytes();
  car_state_sock->send((char*)bytes.begin(), bytes.size());
  // TODO
  // cs_send.carState = CS;
  // cs_send.carState.events = events;
  return CC;
}

int main(int argc, char const *argv[])
{
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


    // PubSocket *sendcan_sock = PubSocket::create(c, "sendcan");
    PubSocket *controls_state_sock = PubSocket::create(c, "controlsState");
    PubSocket *car_state_sock = SubSocket::create(c, "carState");
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

    Poller * poller = Poller::create({thermal_sock, health_sock, live_calibration_sock, 
                                      driver_monitoring_sock, plan_sock, path_plan_sock, 
                                      model_sock, gps_location_sock});

    bool has_relay = false;
    
    LongControl LoC;
    LatControlPID LaC;
    CarParams CP;
    VehicleModel VM;
    CHandler chandler;

    cereal::ControlsState::OpenpilotState state = cereal::ControlsState::OpenpilotState::DISABLED;
    int soft_disable_timer = 0;
    float v_cruise_kph = 255;
    float v_cruise_kph_last = 0;
    int mismatch_counter = 0;
    int can_error_counter = 0;
    int last_blinker_frame = 0;
    
    chandler.calStatus = 2;
    chandler.sensorValid = true;
    chandler.posenetValid = true;
    chandler.freeSpace = 1.0;

    while (true){
        double start_time = sec_since_boot();
        for (auto s : poller->poll(100)){
            Message * msg = s->receive();
            auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
            memcpy(amsg.begin(), msg->getData(), msg->getSize());
            capnp::FlatArrayMessageReader capnp_msg(amsg);
            cereal::Event::Reader event = capnp_msg.getRoot<cereal::Event>();

            handler.handle_log(event);

      
            delete msg;
        }

    return 0;
}
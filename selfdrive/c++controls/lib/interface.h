#ifndef INTERFACE_H_
#define INTERFACE_H_

std::vector<struct SINGNAL> signals;

enum EventName{
  // TODO: copy from error list
  canError;
  steerUnavailable;
  brakeUnavailable;
  gasUnavailable;
  wrongGear;
  doorOpen;
  seatbeltNotLatched;
  espDisabled;
  wrongCarMode;
  steerTempUnavailable;
  reverseGear;
  buttonCancel;
  buttonEnable;
  pedalPressed;
  cruiseDisabled;
  radarCanError;
  dataNeeded;
  speedTooLow;
  outOfSpace;
  overheat;
  calibrationIncomplete;
  calibrationInvalid;
  controlsMismatch;
  pcmEnable;
  pcmDisable;
  noTarget;
  radarFault;
  modelCommIssueDEPRECATED;
  brakeHold;
  parkBrake;
  manualRestart;
  lowSpeedLockout;
  plannerError;
  ipasOverride;
  debugAlert;
  steerTempUnavailableMute;
  resumeRequired;
  preDriverDistracted;
  promptDriverDistracted;
  driverDistracted;
  geofence;
  driverMonitorOn;
  driverMonitorOff;
  preDriverUnresponsive;
  promptDriverUnresponsive;
  driverUnresponsive;
  belowSteerSpeed;
  calibrationProgress;
  lowBattery;
  invalidGiraffeHonda;
  vehicleModelInvalid;
  controlsFailed;
  sensorDataInvalid;
  commIssue;
  tooDistracted;
  posenetInvalid;
  soundsUnavailable;
  preLaneChangeLeft;
  preLaneChangeRight;
  laneChange;
  invalidGiraffeToyota;
  internetConnectivityNeeded;
  communityFeatureDisallowed;
  lowMemory;
  stockAeb;
  ldw;
  carUnrecognized;
  radarCommIssue;
}

typedef struct{
  EventName name;
  bool enable;
  bool noEntry;
  bool warning;
  bool userDisable;
  bool softDisable;
  bool immediateDisable;
  bool preEnable;
  bool permanent;
}CarEvent;

typedef struct{
  // optional wheel speeds
  float fl;
  float fr;
  float rl;
  float rr;
}WheelSpeeds;

typedef struct{
  bool enabled;
  float speed;
  bool available;
  float speedOffset;
  bool standstill;
}CruiseState;

enum GearShifter {
  unknown;
  park;
  drive;
  neutral;
  reverse;
  sport;
  low;
  brake;
  eco;
  manumatic;
}

enum Type {
  unknown;
  leftBlinker;
  rightBlinker;
  accelCruise;
  decelCruise;
  cancel;
  altButton1;
  altButton2;
  altButton3;
  setCruise;
  resumeCruise;
  gapAdjustCruise;
}
  // send on change
typedef struct{
  bool pressed;
  Type type;
}ButtonEvent;

typedef struct{
  std::vector<EventName> errorsDEPRECATED;
  std::vector<CarEvent> events;

  // car speed
  float vEgo;         // best estimate of speed
  float aEgo;        // best estimate of acceleration
  float vEgoRaw;     // unfiltered speed from CAN sensors
  float yawRate;     // best estimate of yaw rate
  bool standstill;
  WheelSpeeds wheelSpeeds;

  // gas pedal, 0.0-1.0
  float gas;        // this is user + computer
  bool gasPressed;    // this is user pedal only

  // brake pedal, 0.0-1.0
  float brake;      // this is user pedal only
  bool brakePressed;  // this is user pedal only
  bool brakeLights;

  // steering wheel
  float steeringAngle;   // deg
  float steeringRate;   // deg/s
  float steeringTorque;  // TODO: standardize units
  float steeringTorqueEps;  // TODO: standardize units
  bool steeringPressed;    // if the user is using the steering wheel
  bool steeringRateLimited;    // if the torque is limited by the rate limiter
  bool stockAeb;
  bool stockFcw;

  // cruise state
  CruiseState cruiseState;

  // gear
  GearShifter gearShifter;

  // button presses
  std::vector<ButtonEvent> buttonEvents;
  bool leftBlinker;
  bool rightBlinker;
  bool genericToggle;

  // lock info
  bool doorOpen;
  bool seatbeltUnlatched;
  bool canValid;

  // clutch (manual transmission only)
  bool clutchPressed;

  // which packets this state came from
  std::vector<UInt64> canMonoTimes;
}CARSTATE;

class CarInterface
{
public:
  CarInterface();
  ~CarInterface();
  CARSTATE update(std::string can_strings);

  int frame;
  bool gas_pressed_prev;
  bool brake_pressed_prev;
  bool cruise_enabled_prev;
private:
  Parser cp;
  CarParams CP;
  VehicleModel VM;
  CarState CS;
  CarController *CC;
};
#endif // INTERFACE_H_
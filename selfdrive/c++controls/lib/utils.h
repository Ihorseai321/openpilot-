#ifndef UTILS_H_
#define UTILS_H_
#include <vector>
#include <string>

#define DT_MDL 0.05
#define DT_CTRL 0.01
#define MPH_TO_MS 0.44704
#define MS_TO_KPH 3.6
#define MS_TO_MPH 2.23693629
#define KPH_TO_MS 0.27777778
#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.29577957
#define CAMERA_OFFSET 0.06
#define LON_MPC_STEP 0.2
#define LANE_DEPARTURE_THRESHOLD 0.1
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))


#define ENABLE 1
#define PRE_ENABLE 2
#define NO_ENTRY 3
// #define WARNING 4
#define USER_DISABLE 5
#define SOFT_DISABLE 6
#define IMMEDIATE_DISABLE 7
#define PERMANENT 8

typedef struct{
  bool active;
  float steerAngle;
  float steerRate;
  float angleError;
  float p;
  float i;
  float f;
  float output;
  bool saturated;
}LATPIDState;

typedef struct
{
  float output_steer;
  float angle_steers_des;
  LATPIDState pid_log;
}LatPIDRet;

enum EventName{
  // TODO: copy from error list
  canError,
  steerUnavailable,
  brakeUnavailable,
  gasUnavailable,
  wrongGear,
  doorOpen,
  seatbeltNotLatched,
  espDisabled,
  wrongCarMode,
  steerTempUnavailable,
  reverseGear,
  buttonCancel,
  buttonEnable,
  pedalPressed,
  cruiseDisabled,
  radarCanError,
  dataNeeded,
  speedTooLow,
  outOfSpace,
  overheat,
  calibrationIncomplete,
  calibrationInvalid,
  controlsMismatch,
  pcmEnable,
  pcmDisable,
  noTarget,
  radarFault,
  modelCommIssueDEPRECATED,
  brakeHold,
  parkBrake,
  manualRestart,
  lowSpeedLockout,
  plannerError,
  ipasOverride,
  debugAlert,
  steerTempUnavailableMute,
  resumeRequired,
  preDriverDistracted,
  promptDriverDistracted,
  driverDistracted,
  geofence,
  driverMonitorOn,
  driverMonitorOff,
  preDriverUnresponsive,
  promptDriverUnresponsive,
  driverUnresponsive,
  belowSteerSpeed,
  calibrationProgress,
  lowBattery,
  invalidGiraffeHonda,
  vehicleModelInvalid,
  controlsFailed,
  sensorDataInvalid,
  commIssue,
  tooDistracted,
  posenetInvalid,
  soundsUnavailable,
  preLaneChangeLeft,
  preLaneChangeRight,
  laneChange,
  invalidGiraffeToyota,
  internetConnectivityNeeded,
  communityFeatureDisallowed,
  lowMemory,
  stockAeb,
  ldw,
  carUnrecognized,
  radarCommIssue
};

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
  unknown,
  park,
  drive,
  neutral,
  reverse,
  sport,
  low,
  brake,
  eco,
  manumatic
};

enum Type {
  unknownType,
  leftBlinker,
  rightBlinker,
  accelCruise,
  decelCruise,
  cancel,
  altButton1,
  altButton2,
  altButton3,
  setCruise,
  resumeCruise,
  gapAdjustCruise
};
  // send on change
typedef struct{
  bool pressed;
  int type;
}BUTTONEVENT;

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
  std::vector<BUTTONEVENT> buttonEvents;
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
  std::vector<unsigned long> canMonoTimes;
}CARSTATE;

enum VisualAlert {
  // these are the choices from the Honda
  // map as good as you can for your car
  none,
  fcw,
  steerRequired,
  brakePressed,
  wrongGear_visual,
  seatbeltUnbuckled,
  speedTooHigh,
  ldw_visual
};

enum AudibleAlert {
  // these are the choices from the Honda
  // map as good as you can for your car
  noneaudi,
  chimeEngage,
  chimeDisengage,
  chimeError,
  chimeWarning1,
  chimeWarning2,
  chimeWarningRepeat,
  chimePrompt
};

typedef struct{
    // range from 0.0 - 1.0
    float gas;
    float brake;
    // range from -1.0 - 1.0
    float steer;
    float steerAngle;
}ACTUATORS; 

typedef struct{
  bool cancel;
  bool override;
  float speedOverride;
  float accelOverride;
}CruiseControl;

typedef struct{
  bool speedVisible;
  float setSpeed;
  bool lanesVisible;
  bool leadVisible;
  VisualAlert visualAlert;
  AudibleAlert audibleAlert;
  bool rightLaneVisible;
  bool leftLaneVisible;
  bool rightLaneDepart;
  bool leftLaneDepart;
}HUDControl;

typedef struct{
  bool enabled;
  bool active;

  float gasDEPRECATED;
  float brakeDEPRECATED;
  float steeringTorqueDEPRECATED;

  ACTUATORS actuators;

  CruiseControl cruiseControl;
  HUDControl hudControl;
}CARCONTROL;

struct LeadData {
    float dRel;
    float yRel;
    float vRel;
    float aRel;
    float vLead;
    float aLeadDEPRECATED;
    float dPath;
    float vLat;
    float vLeadK;
    float aLeadK;
    bool fcw;
    bool status;
    float aLeadTau;
    float modelProb;
    bool radar;
};

typedef struct
{
    float final_gas;
    float final_brake;
}LCtrlRet;

typedef struct{
  std::string sig_name;
  std::string sig_address;
  double sig_default;
}SIGNAL;

double readclock(int clock_id);
double monotonic_time();
double sec_since_boot();
float interp(float x, float xp[], float fp[], int n);
float min_array(float array[], int len);
float clip(float x, float lo, float hi);
float sign(float x);
float _fabs(float f);
void matrix_mul(int row, int col, int m,float *mat1, float *mat2, float *result);
void matrix_sub(int row, int col, float *A, float *B, float *ret);
#endif // UTILS_H_
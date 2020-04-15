#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
// #include "cereal/gen/cpp/car.capnp.h"
#include "messaging.hpp"
#include "common/timing.h"
#include "lib/ctl_handler.h"
#include "lib/utils.h"
#include <cmath>
#include "lib/interface.h"
#include "lib/ratekeeper.h"
#include "lib/longcontrol.h"
#include "lib/latcontrol_pid.h"
using namespace std;
void car_state_publish2(PubSocket *car_state_sock, PubSocket *car_control_sock, PubSocket *controls_state_sock, 
               CHandler chandler, CARSTATE CS, CARCONTROL &CC ,CarInterface CI, CarParams CP, VehicleModel VM, 
               cereal::ControlsState::OpenpilotState state, ACTUATORS actuators, float v_cruise_kph, 
               RateKeeper rk, LatControlPID LaC, LongControl LoC, bool read_only, double start_time, 
               float v_acc, float a_acc, LATPIDState lac_log, int last_blinker_frame, bool is_ldw_enabled, int can_error_counter);
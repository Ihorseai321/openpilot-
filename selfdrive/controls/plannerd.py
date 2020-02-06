#!/usr/bin/env python3
import gc

from cereal import car
from common.params import Params
from common.realtime import set_realtime_priority
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.planner import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner import PathPlanner
import cereal.messaging as messaging

from selfdrive.config import Conversions as CV
STD_CARGO_KG = 136.
MAX_ANGLE = 87.
class CivicParams:
  MASS = 1326. + STD_CARGO_KG
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500
  
def scale_rot_inertia(mass, wheelbase):
  return CivicParams.ROTATIONAL_INERTIA * mass * wheelbase ** 2 / (CivicParams.MASS * CivicParams.WHEELBASE ** 2)

def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear

CP = car.CarParams.new_message()
CP.carName = "ford"
CP.carFingerprint = "FORD FUSION 2018"
CP.carVin = "0" * 17
CP.isPandaBlack = True
CP.safetyModel = car.CarParams.SafetyModel.ford
CP.dashcamOnly = True

# pedal
CP.enableCruise = True

CP.wheelbase = 2.85
CP.steerRatio = 14.8
CP.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG
CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kpBP = [[0.], [0.]]
CP.lateralTuning.pid.kpV, CP.lateralTuning.pid.kiV = [[0.01], [0.005]] # TODO: tune this
CP.lateralTuning.pid.kf = 1. / MAX_ANGLE   # MAX Steer angle to normalize FF
CP.steerActuatorDelay = 0.1  # Default delay, not measured yet
CP.steerLimitTimer = 0.8
CP.steerRateCost = 1.0
CP.centerToFront = CP.wheelbase * 0.44
tire_stiffness_factor = 0.5328

# min speed to enable ACC. if car can do stop and go, then set enabling speed
# to a negative value, so it won't matter.
CP.minEnableSpeed = -1.

# TODO: get actual value, for now starting with reasonable value for
# civic and scaling by mass and wheelbase
CP.rotationalInertia = scale_rot_inertia(CP.mass, CP.wheelbase)

# TODO: start from empirically derived lateral slip stiffness for the civic and scale by
# mass and CG position, so all cars will have approximately similar dyn behaviors
CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(CP.mass, CP.wheelbase, CP.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

# no rear steering, at least on the listed cars above
CP.steerRatioRear = 0.
CP.steerControlType = car.CarParams.SteerControlType.angle

# steer, gas, brake limitations VS speed
CP.steerMaxBP = [0.]  # breakpoints at 1 and 40 kph
CP.steerMaxV = [1.0]  # 2/3rd torque allowed above 45 kph
CP.gasMaxBP = [0.]
CP.gasMaxV = [0.5]
CP.brakeMaxBP = [5., 20.]
CP.brakeMaxV = [1., 0.8]

CP.enableCamera = False
CP.openpilotLongitudinalControl = False

CP.stoppingControl = False
CP.startAccel = 0.0

CP.longitudinalTuning.deadzoneBP = [0., 9.]
CP.longitudinalTuning.deadzoneV = [0., .15]
CP.longitudinalTuning.kpBP = [0., 5., 35.]
CP.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
CP.longitudinalTuning.kiBP = [0., 35.]
CP.longitudinalTuning.kiV = [0.54, 0.36]

def plannerd_thread(sm=None, pm=None):


  gc.disable()

  # start the loop
  set_realtime_priority(2)

  params = Params()
  params.put("CarParams", CP.to_bytes())

  cloudlog.info("plannerd is waiting for CarParams")
  #CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)
  PL = Planner(CP)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'model', 'liveParameters'])

  if pm is None:
    pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])

  sm['liveParameters'].valid = True
  sm['liveParameters'].sensorValid = True
  sm['liveParameters'].steerRatio = CP.steerRatio
  sm['liveParameters'].stiffnessFactor = 1.0
  
  intcount = 0
  while True:
    sm.update()
    print(sm['model'])
    if sm.updated['model']:
      print("----------------plannerd-model-updated----------------")
      PP.update(sm, pm, CP, VM)
    if sm.updated['radarState']:
      print("----------------plannerd-radarState-updated----------------")
      PL.update(sm, pm, CP, VM, PP)


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()

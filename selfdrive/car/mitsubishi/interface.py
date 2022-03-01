#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.mitsubishi.values import MIN_ACC_SPEED, CarControllerParams
from common.dp_common import common_interface_atl, common_interface_get_params_lqr

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.oldCruiseState = 0.

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value Here 2!!!
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "mitsubishi"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.mitsubishi)]
    #ret.safetyConfigs[0].safetyParam = 1 #EPS_SCALE[candidate] 0x399

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4
    ret.stoppingControl = False  # Toyota starts braking more when it thinks you want to stop

    stop_and_go = False

    ret.mass = 1800 + STD_CARGO_KG
    ret.wheelbase = 2.68986
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 14.3
    tire_stiffness_factor = 0.7933
    #set_lat_tune(ret.lateralTuning, LatTunes.PID_D)

    ret.steerActuatorDelay = 0.1
    ret.lateralTuning.pid.kf = 0.000039
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 10., 20.], [0., 10., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01, 0.05, 0.2], [0.003, 0.018, 0.025]]

    ret.steerRateCost = 1.

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # if the smartDSU is detected, openpilot can send ACC_CMD (and the smartDSU will block it from the DSU) or not (the DSU is "connected")
    ret.openpilotLongitudinalControl = True #smartDsu or ret.enableDsu or candidate in TSS2_CAR

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else MIN_ACC_SPEED

    #set_long_tune(ret.longitudinalTuning, LongTunes.PEDAL)
    return ret

  # returns a car.CarState
  # returns a car.CarState
  def update(self, c, can_strings, dragonconf):

    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    #self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    # dp
    self.dragonconf = dragonconf

    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise)
    #ret.cruiseState.enabled = True #common_interface_atl(ret, dragonconf.dpAtl)


    if ((ret.cruiseState.enabled == 1) and (self.oldCruiseState != ret.cruiseState.enabled)):
      events.add(EventName.buttonEnable)
      #print("Try send enable event %d and %d" % (ret.cruiseState.enabled, self.oldCruiseState))

    if ((ret.cruiseState.enabled == 0) and (self.oldCruiseState != ret.cruiseState.enabled)):
      events.add(EventName.buttonCancel)

    self.oldCruiseState = ret.cruiseState.enabled

    # low speed re-write
    if ret.cruiseState.enabled and dragonconf.dpToyotaCruiseOverride and ret.cruiseState.speed < dragonconf.dpToyotaCruiseOverrideAt * CV.KPH_TO_MS:
      if dragonconf.dpToyotaCruiseOverrideVego:
        if self.dp_cruise_speed == 0.:
          ret.cruiseState.speed = self.dp_cruise_speed = max(dragonconf.dpToyotaCruiseOverrideSpeed * CV.KPH_TO_MS,
                                                             ret.vEgo)
        else:
          ret.cruiseState.speed = self.dp_cruise_speed
      else:
        ret.cruiseState.speed = dragonconf.dpToyotaCruiseOverrideSpeed * CV.KPH_TO_MS
    else:
      self.dp_cruise_speed = 0.

    ret.canValid = self.cp.can_valid #and self.cp_cam.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # gear except P, R
    # extra_gears = [GearShifter.neutral, GearShifter.eco, GearShifter.manumatic, GearShifter.drive, GearShifter.sport,
    #                GearShifter.low, GearShifter.brake, GearShifter.unknown]

    # events
    # events = self.create_common_events(ret, extra_gears)
    #
    # if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
    #   events.add(EventName.lowSpeedLockout)
    # if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
    #   events.add(EventName.belowEngageSpeed)
    #   if c.actuators.accel > 0.3:
    #     # some margin on the actuator to not false trigger cancellation while stopping
    #     events.add(EventName.speedTooLow)
    #   if ret.vEgo < 0.001:
    #     # while in standstill, send a user alert
    #     events.add(EventName.manualRestart)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    hud_control = c.hudControl

    # can_sends = self.CC.update(c.enabled, self.CS, self.frame,
    #                            c.actuators, c.cruiseControl.cancel,
    #                            c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
    #                            c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
    #                            c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart, self.dragonconf)
    #ruiseState.available
    print("interface.py 126 steer %d, isEna %s, isAct %s" % (c.actuators.steer, c.enabled, c.active))

    ret = self.CC.update(c.enabled, self.CS, self.frame,
                         c.actuators, c.cruiseControl.cancel,
                         hud_control.visualAlert, hud_control.leftLaneVisible,
                         hud_control.rightLaneVisible, hud_control.leadVisible,
                         hud_control.leftLaneDepart, hud_control.rightLaneDepart, self.dragonconf)

    self.frame += 1
    return ret

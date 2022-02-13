from cereal import car
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, TSS2_CAR, EPS_SCALE


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = False
    self.angle_offset = 0

    self.low_speed_lockout = False
    self.acc_type = 1

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["DOORS_STATUS"]["DOOR_OPEN_FL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_FR"],
                        cp.vl["DOORS_STATUS"]["DOOR_OPEN_RL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["DOORS_STATUS"]["SEATBELT_DRIVER_UNLATCHED"] != 0

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0

    ret.gas = cp.vl["GAS_PEDAL"]["GAS_PEDAL"]
    ret.gasPressed = ret.gas > 2

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS_1"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS_1"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS_1"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS_2"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 10

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    #ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # continuous blinker signals for assisted lane change
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["WARNING_SIGNALS"]["TURN_LEFT_SIGNAL"], cp.vl["WARNING_SIGNALS"]["TURN_RIGHT_SIGNAL"])

    ret.steeringTorque = cp.vl["STEER_MOMENT_SENSOR"]["STEER_MOMENT"]
    ret.steeringTorqueEps = cp.vl["STEER_MOMENT_SENSOR"]["STEER_TORQUE_EPS"]
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > 1
    ret.steerWarning = 0

    ret.cruiseState.available = cp.vl["ACC_STATUS"]["CRUISE_ON"] != 0
    ret.cruiseState.speed = cp.vl["ACC_STATUS"]["SET_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.enabled = bool(cp.vl["ACC_STATUS"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = False#cp.vl["ACC_STATUS"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)

    ret.cruiseState.standstill = False

    #ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)
    #ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0
    ret.leftBlindspot = cp.vl["BSW_STATUS"]["LEFT_WARNING"]
    ret.rightBlindspot = cp.vl["BSW_STATUS"]["RIGHT_WARNING"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("DOOR_OPEN_FL", "DOORS_STATUS"),
      ("DOOR_OPEN_FR", "DOORS_STATUS"),
      ("DOOR_OPEN_RL", "DOORS_STATUS"),
      ("DOOR_OPEN_RR", "DOORS_STATUS"),
      ("SEATBELT_DRIVER_UNLATCHED", "DOORS_STATUS"),
      ("BRAKE_PRESSED", "BRAKE_MODULE"),
      ("GAS_PEDAL", "GAS_PEDAL"),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS_1"),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS_1"),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS_1"),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS_2"),
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("GEAR", "GEAR_PACKET"),
      ("TURN_LEFT_SIGNAL", "WARNING_SIGNALS"),
      ("TURN_RIGHT_SIGNAL", "WARNING_SIGNALS"),
      ("STEER_MOMENT", "STEER_MOMENT_SENSOR"),
      ("STEER_TORQUE_EPS", "STEER_MOMENT_SENSOR"),
      ("CRUISE_ON", "ACC_STATUS"),
      ("CRUISE_ACTIVE", "ACC_STATUS"),
      ("SET_SPEED", "ACC_STATUS"),
      ("LEFT_WARNING", "BSW_STATUS"),
      ("RIGHT_WARNING", "BSW_STATUS"),
    ]

    checks = [
      ("DOORS_STATUS", 1),
      ("BRAKE_MODULE", 1),
      ("GAS_PEDAL", 1),
      ("WHEEL_SPEEDS_1", 1),
      ("WHEEL_SPEEDS_2", 1),
      ("STEER_ANGLE_SENSOR", 1),
      ("GEAR_PACKET", 1),
      ("WARNING_SIGNALS", 1),
      ("STEER_MOMENT_SENSOR", 1),
      ("ACC_STATUS", 1),
      ("BSW_STATUS", 1),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)
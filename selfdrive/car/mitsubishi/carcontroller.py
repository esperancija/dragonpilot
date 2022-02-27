from selfdrive.car import apply_std_steer_torque_limits
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl
from selfdrive.car.mitsubishi.values import CAR, CarControllerParams

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.last_steer = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_limited = False
    self.use_interceptor = False

    self.packer = CANPacker(dbc_name)


  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
               left_line, right_line, lead, left_lane_depart, right_lane_depart, dragonconf):

    can_sends = []

    can_sends.append((0x18DAB0F1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 1))

    # *** steering ***


    return can_sends

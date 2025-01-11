from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR, LEGACY_SAFETY_MODE_CAR
from openpilot.selfdrive.car import make_can_msg
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican

from openpilot.selfdrive.frogpilot.controls.frogpilot_planner import FrogPilotPlanner

class CustomStockLongitudinal:
  def __init__(self):
    self.frogpilot_planner = FrogPilotPlanner()
    self.timer = 0
    self.button_type = 0
    self.button_count = 0
    self.t_interval = 7
    self.cruise_button = None
    self.last_button_frame = 0
    self.frame = 0

  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled or CS.cruise_buttons[-1] != Buttons.NONE:
      self.timer = 40
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_button_type(self, CS, target_speed):
    current_speed = round(CS.out.cruiseState.speed *
                         (CV.MS_TO_MPH if not CS.params_list.is_metric else CV.MS_TO_KPH))

    if self.button_type == 0:
      self.button_count = 0
      if target_speed > current_speed:
        self.button_type = 1
      elif target_speed < current_speed:
        self.button_type = 2
      return None

    elif self.button_type == 1:
      self.button_count += 1
      if target_speed <= current_speed or self.button_count > 5:
        self.button_count = 0
        self.button_type = 3
      return Buttons.RES_ACCEL

    elif self.button_type == 2:
      self.button_count += 1
      if target_speed >= current_speed or self.button_count > 5:
        self.button_count = 0
        self.button_type = 3
      return Buttons.SET_DECEL

    elif self.button_type == 3:
      self.button_count += 1
      if self.button_count > self.t_interval:
        self.button_type = 0
      return None

    return None

  def update_custom_stock_long(self, car_fingerprint, frame, CS, packer, CP=None):
    self.frame = frame

    if not CS.out.cruiseState.enabled or not self.get_cruise_buttons_status(CS):
      return []

    v_cruise_target = self.frogpilot_planner.v_cruise * CV.MS_TO_KPH
    self.cruise_button = self.get_button_type(CS, v_cruise_target)

    if self.cruise_button is None:
      return []

    # Handle CAN-FD vehicles
    if car_fingerprint in CANFD_CAR:
      if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        return []  # Skip for alt buttons for now

      if self.frame % 2 == 0:
        return [hyundaicanfd.create_buttons(packer, CP, CP.CAN,
                ((self.frame // 2) + 1) % 0x10, self.cruise_button)]

    # Handle regular CAN vehicles
    elif car_fingerprint in LEGACY_SAFETY_MODE_CAR:
      send_freq = 5 if not self.frogpilot_planner.road_curvature_detected else 1

      if (frame - self.last_button_frame) * DT_CTRL > 0.1 * send_freq:
        messages = [hyundaican.create_clu11(packer, frame, CS.clu11, self.cruise_button)] * 25

        if (frame - self.last_button_frame) * DT_CTRL >= 0.15 * send_freq:
          self.last_button_frame = frame

        return messages

    return []

"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.car.gm import gmcan
from opendbc.car.gm.values import CanBus, CruiseButtons
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.sunnypilot.car.gm.values_ext import GMFlagsSP

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: CruiseButtons.RES_ACCEL,
  SendButtonState.decrease: CruiseButtons.DECEL_SET,
}

# Match the aggressive cadence used by the legacy GM cruise spam tune
BUTTON_SPAM_INTERVAL = 0.2


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def _next_button_counter(self, CS) -> int:
    # Predict the next rolling counter slot used by ASCMSteeringButton
    return (CS.buttons_counter + 1) % 4

  def _compute_redneck_button(self, CC, CS) -> CruiseButtons | None:
    accel = CC.actuators.accel
    v_ego = CS.out.vEgo

    # TODO: detect metric clusters; default to MPH like the original tune
    ms_to_speed = CV.MS_TO_MPH

    speed_set = int(round(CS.out.cruiseState.speed * ms_to_speed))
    desired_set = int(round((v_ego * 1.01 + 3 * accel) * ms_to_speed))

    if self.CP.minEnableSpeed - (desired_set / ms_to_speed) > 3.25:
      return CruiseButtons.CANCEL

    if desired_set < speed_set and speed_set > self.CP.minEnableSpeed * ms_to_speed + 1:
      return CruiseButtons.DECEL_SET

    if desired_set > speed_set:
      return CruiseButtons.RES_ACCEL

    return None

  def update(self, CC, CC_SP, CS, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.CP_SP.flags & GMFlagsSP.NON_ACC:
      redneck_button = self._compute_redneck_button(CC, CS)
      if redneck_button:
        interval = 1.0 if abs(CC.actuators.accel) <= 0.15 else 0.2
        if (self.frame - self.last_button_frame) * DT_CTRL > interval:
          idx = self._next_button_counter(CS)
          can_sends.append(gmcan.create_buttons(packer, CanBus.POWERTRAIN, idx, redneck_button))
          self.last_button_frame = self.frame
      return can_sends

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if (self.frame - self.last_button_frame) * DT_CTRL > BUTTON_SPAM_INTERVAL:
        idx = self._next_button_counter(CS)
        can_sends.append(gmcan.create_buttons(packer, CanBus.POWERTRAIN, idx, send_button))
        self.last_button_frame = self.frame

    return can_sends

"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.mg import mgcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    # Free-running 2-bit alive/rolling counter (CCSwStsAlvRC). We inject our own
    # BO_481 frames, so we advance our own counter each send. On-car we may need
    # to instead sync to the live counter read from the stock BO_481 on bus 0.
    self.cc_sw_counter = 0

  def update(self, CC_SP, CS, packer, frame, last_button_frame) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      accel = self.ICBM.sendButton == SendButtonState.increase  # RES+ / speed increase
      decel = self.ICBM.sendButton == SendButtonState.decrease  # SET- / speed decrease

      # Send one button frame at ~10 Hz, advancing the 2-bit alive counter each time.
      # (Repeat count / rate may need tuning on-car for the ACC ECU to accept it.)
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
        self.cc_sw_counter = (self.cc_sw_counter + 1) % 4
        can_sends.append(mgcan.create_cruise_buttons(packer, self.cc_sw_counter, accel, decel))
        self.last_button_frame = self.frame

    return can_sends

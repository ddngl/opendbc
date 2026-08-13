from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import create_lka_steering, create_lka_hud
from opendbc.car.mg.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators

    can_sends = []

    # steering command
    if self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive:
        # calculate steer and also set limits due to driver torque
        new_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, CarControllerParams)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_steering(self.packer, (self.frame // CarControllerParams.STEER_STEP) % 16, apply_torque, CC.latActive))

      # HUD/nag suppression: spoof the FVCM camera status (0x167) onto bus 0
      # so the cluster stops the "hands on wheel" warning. Same 50Hz cadence.
      # ONLY while actually steering (latActive); disengaged/standstill sends caused the nag.
      # HUD/nag suppression: spoof FVCM 0x167 while steering. Kills the AUDIBLE nag.
      # (visual "hands on wheel" icon may remain — tuning HUD_* in mgcan.py.)
      if CC.latActive:
        can_sends.append(create_lka_hud(self.packer, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends

from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.values import CAR, MgSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]

    if candidate == CAR.MG_ZS_EV:
      ret.safetyConfigs[0].safetyParam |= MgSafetyFlags.ALT_BRAKE.value

    ret.steerActuatorDelay = 0.3
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True
    ret.enableBsm = True  # RDA_HSC1_P02 blind-spot

    ret.alphaLongitudinalAvailable = False
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= MgSafetyFlags.LONG_CONTROL.value

    ret.longitudinalActuatorDelay = 0.35
    ret.stopAccel = 0

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    # ICBM: with STOCK ACC (no openpilot longitudinal), spoof the cruise +/- buttons
    # (BO_481 / 0x1E1) to adjust set-speed. Only takes effect when not openpilotLongitudinalControl.
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret

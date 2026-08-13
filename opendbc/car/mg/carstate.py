from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mg.values import CAR, DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.tsr_spd = 0.0    # camera speed limit (km/h) to relay to cluster
    self.tsr_sts = 0
    self.tsr_dist = -100.0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Speed limit from the forward camera's traffic-sign recognition (ISA), for SLC.
    # TrgtSpdReqCamr = recognized limit (km/h) on the camera bus; status>0 when a sign is active.
    tsr_spd = cp_cam.vl["FVCM_HSC2_FrP02"]["TrgtSpdReqCamrHSC2"]
    tsr_sts = cp_cam.vl["FVCM_HSC2_FrP02"]["SpdAstReqStsCamrHSC2"]
    ret_sp.speedLimit = float(tsr_spd) * CV.KPH_TO_MS if (tsr_sts > 0 and 0 < tsr_spd < 200) else 0.0
    # stash raw camera TSR values so the carcontroller can relay them to the cluster (dash)
    self.tsr_spd = float(tsr_spd)
    self.tsr_sts = int(tsr_sts)
    self.tsr_dist = float(cp_cam.vl["FVCM_HSC2_FrP02"]["DistSinceTrgtCamrHSC2"])

    # Vehicle speed
    ret.vEgoRaw = cp.vl["SCS_HSC2_FrP19"]["VehSpdAvgHSC2"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["SCS_HSC2_FrP24"]["VehSdslStsHSC2"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["GW_HSC2_HCU_FrP00"]["EPTAccelActuPosHSC2"] > 0

    # Brake pedal
    if self.CP.carFingerprint == CAR.MG_ZS_EV:
      ret.brakePressed = cp.vl["GW_HSC2_HCU_FrP00"]["EPTBrkPdlDscrtInptStsHSC2"] == 1
    else:
      ret.brakePressed = cp.vl["EHBS_HSC2_FrP00"]["BrkPdlAppdHSC2"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngHSC2"]
    ret.steeringRateDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngGrdHSC2"]
    ret.steeringTorque = cp.vl["EPS_HSC2_FrP03"]["DrvrStrgDlvrdToqHSC2"]
    ret.steeringTorqueEps = cp.vl["EPS_HSC2_FrP03"]["ChLKARespToqHSC2"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp_cam.vl["FVCM_HSC2_FrP02"]["LDWSysFltStsHSC2"] != 0  # TODO: validate

    # Cruise state
    ret.cruiseState.enabled = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"] in (2, 3)  # Active, Override
    ret.cruiseState.available = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"] in (1, 2, 3)  # AOL: ACC main on (1=standby/2=active/3=override); 0=off at stalk
    ret.cruiseState.standstill = False  # TODO
    ret.cruiseState.speed = cp.vl["RADAR_HSC2_FrP02"]["ACCDrvrSelTrgtSpd_RadarHSC2"] * CV.KPH_TO_MS

    ret.accFaulted = cp_cam.vl["FVCM_HSC2_FrP02"]["TJAICASysFltStsHSC2"] != 0  # TODO: validate

    # Forward collision + pedestrian warning (stock, from front radar)
    ret.stockFcw = bool(cp.vl["RADAR_HSC2_FrP04"]["FCWAHSC2"] or cp.vl["RADAR_HSC2_FrP04"]["PedtrnColWrnngAHSC2"])

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["GW_HSC2_ECM_FrP04"]["TrEstdGearHSC2"]), GearShifter.unknown)

    # Doors
    ret.doorOpen = False  # TODO

    # Blinkers - the stalk switch (DircnIndLampSwStsHSC2: 0 off/1 left/2 right) is what
    # the cluster shows and is reliably populated; a brief "comfort" tap only reads for a
    # moment, so hold it ~1s via update_blinker_from_stalk so lane change still triggers.
    left_stalk = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 1
    right_stalk = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 2
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(100, left_stalk, right_stalk)

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["GW_HSC2_SDM_FrP00"]["DrvrSbltAtcHSC2"] != 1

    # Blindspot (RDA rear-corner radar; confirmed against a left-only capture)
    ret.leftBlindspot = cp.vl["RDA_HSC1_P02"]["LBSDAndLCAWrnng_HS"] > 0
    ret.rightBlindspot = cp.vl["RDA_HSC1_P02"]["RBSDAndLCAWrnng_HS"] > 0

    # AEB
    ret.stockAeb = False

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.radar: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }

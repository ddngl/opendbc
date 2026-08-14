def calc_checksum(values):
  lka_req_toq = values['LKAReqToqHSC2'] + 1024
  lka_req_toq_sts = values['LKAReqToqStsHSC2']
  lka_req_toq_v = values['LKAReqToqVHSC2']
  lka_alv_rc = values['LKAAlvRCHSC2']

  combined = ((lka_req_toq << 1) | (lka_req_toq_sts << 12) | lka_req_toq_v) & 0x3FFF
  with_counter = (combined + lka_alv_rc) & 0x3FFF
  checksum = ((~with_counter) + 1) & 0x3FFF

  return checksum


def create_lka_steering(packer, counter, apply_torque, active):

  values = {
    "LKAReqToqHSC2": apply_torque,
    "LKAReqToqVHSC2": 0,
    "LKAAlvRCHSC2": counter,
    "LDWLKAVbnLvlReqHSC2": 0,  # TODO: vibration level?
    "LKASysStsHSC2": 0,
    "LKAReqToqStsHSC2": active,
    "LKASysFltStsHSC2": 0,
    "LKADrvrTkovReqHSC2": 0,
    "LKAReqToqPVHSC2": 0
  }

  values["LKAReqToqPVHSC2"] = calc_checksum(values)
  return packer.make_can_msg("FVCM_HSC2_FrP03", 0, values)


# ---------------------------------------------------------------------------
# Nag suppression: FVCM_HSC2_FrP02 (0x167 / 359) = camera HUD status.
# On the MITM harness this frame lives ONLY on bus 2 (camera) and never
# reaches bus 0 (cluster) -> cluster shows the "hands on wheel" nag.
# openpilot spoofs it onto bus 0 with a benign, no-warning state so the
# cluster believes the FVCM is healthy and not requesting driver takeover.
#
# 0x167 has NO checksum/counter, so we just pack the fields.
# Values below are the tunables verified against stock camera captures:
#   - hands-off detection inactive  -> cluster does not raise the nag
#   - no haptic/visual warning
#   - TJA reported active while openpilot steers (matches LKA torque)
# Tweak here if a test-drive still nags.
# ---------------------------------------------------------------------------
HUD_HANDS_OFF_STA   = 0  # HandOffStrgWhlDetnStaHSC2  (44|2) 0 = no hands-off detected
HUD_HANDS_OFF_VALID = 0  # HandOffStrgWhlDetnStaVHSC2 (45|1) 0 = detection invalid -> ignored
HUD_HAPTIC_WARN     = 0  # LDWLKAHapticWrnngDspCmdHSC2(63|2) 0 = no haptic warning
HUD_LDW_DSP         = 5  # LDWLKADspCmdHSC2           (50|3) mirror camera idle
HUD_TJA_DSP_ON      = 2  # TJAICADspCmdHSC2           (55|2) 2 = display active
HUD_TJA_STS_ACTIVE  = 2  # TJAICASysStsHSC2           (58|3) 2 = TJA active (matches steering)
HUD_TJA_STS_READY   = 1  # TJAICASysStsHSC2                  1 = standby when not steering
HUD_TJA_FLT         = 0  # TJAICASysFltStsHSC2        (61|3) 0 = no fault


# ---------------------------------------------------------------------------
# Intelligent Cruise Button Management (ICBM): spoof the cruise +/- buttons.
# BO_ 481 (0x1E1) GW_HSC2_FrP04 "CCSwSts..." is the cruise-control switch-status
# frame (7 bytes, sent by the GW). openpilot injects it on bus 0 to nudge the
# STOCK ACC set-speed up/down without any longitudinal control.
#   - CCSwStsSpdIncSwA_h2HSC2 (bit 3)  = RES+ / speed-increase press
#   - CCSwStsSpdDecSwA_h2HSC2 (bit 2)  = SET- / speed-decrease press
#   - CCSwStsPV_h2HSC2       (15|8)    = 8-bit checksum  (data byte 1)
#   - CCSwStsAlvRC_h2HSC2    (17|2)    = 2-bit alive/rolling counter
# ---------------------------------------------------------------------------
CC_BUTTONS_MSG = "GW_HSC2_FrP04"  # BO_ 481 / 0x1E1


def calc_cc_checksum(dat: bytes) -> int:
  # VERIFIED against real BO_481 captures (route c7189f9c62 seg 12, 2000 bus-0
  # frames): the PV byte (data byte 1) makes the 8-bit sum of ALL 7 data bytes
  # equal 0, i.e. PV = two's complement of the sum of every other data byte.
  # Real idle samples that confirm it: 40 c0 00.. / 40 bf 01.. / 40 be 02.. / 40 bd 03..
  return (-sum(b for i, b in enumerate(dat) if i != 1)) & 0xFF


def create_cruise_buttons(packer, counter, accel=False, decel=False):
  # Build a single BO_481 frame with exactly one of RES+/SET- asserted.
  # CCSwStsOnSwA is the latched "cruise system on" state and is set constantly in
  # the stock stream (real idle byte0 = 0x40 = OnSwA=1). It MUST stay set or the
  # ACC ECU reads the frame as "system off" and ignores the button. ICBM only ever
  # sends while stock ACC is engaged, so OnSwA=1 always. A correct RES+ frame is
  # therefore 48 b8 00.. (0x40 OnSwA | 0x08 SpdInc, PV closes the sum to 0).
  values = {
    "CCSwStsSwDataIntgty_h2HSC2": 0,               # 0 = Data Valid
    "CCSwStsSpdIncSwA_h2HSC2": 1 if accel else 0,  # RES+ / speed increase
    "CCSwStsSpdDecSwA_h2HSC2": 1 if decel else 0,  # SET- / speed decrease
    "CCSwStsSetSwA_h2HSC2": 0,
    "CCSwStsRsmSwA_h2HSC2": 0,
    "CCSwStsOnSwA_h2HSC2": 1,                       # keep cruise-on base (0x40)
    "CCSwStsCanclSwA_h2HSC2": 0,
    "CCSwStsDistIncSwA_h2HSC2": 0,
    "CCSwStsDistDecSwA_h2HSC2": 0,
    "CCSwStsAlvRC_h2HSC2": counter & 0x3,
    "CCSwStsPV_h2HSC2": 0,
  }
  # Pack once with PV=0 to get the payload bytes, compute the checksum, then repack.
  _, dat, _ = packer.make_can_msg(CC_BUTTONS_MSG, 0, values)
  values["CCSwStsPV_h2HSC2"] = calc_cc_checksum(dat)
  return packer.make_can_msg(CC_BUTTONS_MSG, 0, values)


def create_lka_hud(packer, active, tsr_spd=0.0, tsr_sts=0, tsr_dist=-100.0):
  # Mimic the FVCM camera's own 0x167 (measured constant on this car) so the cluster
  # sees a "normal" HUD, but MUTE the audible/haptic warning (LDWhaptic=0).
  # Camera baseline: HandOff=1, Valid=0, LDWdsp=5, TJAdsp=2, TJAsts=0, TJAflt=0, LDWhaptic=2.
  values = {
    "HandOffStrgWhlDetnStaHSC2":   1,
    "HandOffStrgWhlDetnStaVHSC2":  0,
    "LDWLKAHapticWrnngDspCmdHSC2": 0,   # 0 = mute the "hands on wheel" SOUND
    "LDWLKADspCmdHSC2":  5,
    "TJAICADspCmdHSC2":  2,
    "TJAICASysStsHSC2":  0,
    "TJAICASysFltStsHSC2": 0,
    # Relay the camera's traffic-sign speed limit to the cluster so the dash shows the sign again
    "TrgtSpdReqCamrHSC2": tsr_spd,
    "SpdAstReqStsCamrHSC2": tsr_sts,
    "DistSinceTrgtCamrHSC2": tsr_dist,
  }
  return packer.make_can_msg("FVCM_HSC2_FrP02", 0, values)

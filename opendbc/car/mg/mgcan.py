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

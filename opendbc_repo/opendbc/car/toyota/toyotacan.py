from opendbc.car.structs import CarParams

SteerControlType = CarParams.SteerControlType


def create_steer_command(packer, steer, steer_req):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer_control_type, steer_angle, steer_req, frame, torque_wind_down):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": frame + 128,
    "SETME_X1": 1,  # suspected LTA feature availability
    # 1 for TSS 2.5 cars, 3 for TSS 2.0. Send based on whether we're using LTA for lateral control
    "SETME_X3": 1 if steer_control_type == SteerControlType.angle else 3,
    "PERCENTAGE": 100,
    "TORQUE_WIND_DOWN": torque_wind_down,
    "ANGLE": 0,
    "STEER_ANGLE_CMD": steer_angle,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "CLEAR_HOLD_STEERING_ALERT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)


def create_lta_steer_command_2(packer, frame):
  values = {
    "COUNTER": frame + 128,
  }
  return packer.make_can_msg("STEERING_LTA_2", 0, values)


def create_accel_command(packer, accel, accel_raw, permit_braking, aego, enabled, pcm_cancel, standstill_req, lead, acc_type, fcw_alert, distance):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel if enabled and not pcm_cancel else 0.,  # compensated accel command
    "ACC_TYPE": acc_type,
    "DISTANCE": distance,
    "MINI_CAR": lead,
    "PERMIT_BRAKING": permit_braking,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
    "ALLOW_LONG_PRESS": 1,
    "ACC_CUT_IN": fcw_alert,  # only shown when ACC enabled
    "ACCEL_CMD_ALT":  accel_raw if enabled else aego,  # raw accel command, pcm uses this to calculate a compensatory force
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)


def create_pcs_commands(packer, accel, active, mass):
  values1 = {
    "COUNTER": 0,
    "FORCE": round(min(accel, 0) * mass * 2),
    "STATE": 3 if active else 0,
    "BRAKE_STATUS": 0,
    "PRECOLLISION_ACTIVE": 1 if active else 0,
  }
  msg1 = packer.make_can_msg("PRE_COLLISION", 0, values1)

  values2 = {
    "DSS1GDRV": min(accel, 0),     # accel
    "PCSALM": 1 if active else 0,  # goes high same time as PRECOLLISION_ACTIVE
    "IBTRGR": 1 if active else 0,  # unknown
    "PBATRGR": 1 if active else 0, # noisy actuation bit?
    "PREFILL": 1 if active else 0, # goes on and off before DSS1GDRV
    "AVSTRGR": 1 if active else 0,
  }
  msg2 = packer.make_can_msg("PRE_COLLISION_2", 0, values2)

  return [msg1, msg2]


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "ACC_BRAKING": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "PCS_INDICATOR": 1,  # PCS turned off
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "PCS_OFF": 1,
    "PCS_SENSITIVITY": 0,
  }
  return packer.make_can_msg("PCS_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, enabled, stock_lkas_hud,
                      lda_left_lane, lda_right_lane, sws_beeps, lda_sa_toggle, alert_prompt,
                      alert_prompt_repeat, alert_immediate):
  values = {
    "TWO_BEEPS": chime or sws_beeps or alert_prompt,
    "LDA_ALERT":  3 if alert_immediate else 2 if alert_prompt_repeat else 1 if alert_prompt or steer else 0,
    "RIGHT_LINE": 3 if lda_right_lane else 1 if right_line else 2,
    "LEFT_LINE": 3 if lda_left_lane else 1 if left_line else 2,
    "BARRIERS": 1 if enabled else 0,
    "LDA_SA_TOGGLE": lda_sa_toggle,
    "REPEATED_BEEPS": 1 if alert_prompt_repeat or lda_right_lane or lda_left_lane else 0,
  }

  # lane sway functionality
  # not all cars have LKAS_HUD — update with camera values if available
  if len(stock_lkas_hud):
    values.update({s: stock_lkas_hud[s] for s in [
      # keep stock SWS
      "LANE_SWAY_FLD",
      "LANE_SWAY_BUZZER",
      "LANE_SWAY_WARNING",
      "LANE_SWAY_SENSITIVITY",
      "LANE_SWAY_TOGGLE",
      # keep stock LDA
      "LDW_EXIST",
      "ADJUSTING_CAMERA",
      "LDA_MALFUNCTION",
      "LDA_SENSITIVITY",
      "LDA_MESSAGES",
      "LDA_ON_MESSAGE",
      "TAKE_CONTROL",
      "LDA_FRONT_CAMERA_BLOCKED",
      "LKAS_STATUS",
      # keep these as well just in case
      "SET_ME_X01",
      "SET_ME_X02",
    ]})

  return packer.make_can_msg("LKAS_HUD", 0, values)

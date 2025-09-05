# =============================================================================
# Module      : BrownPanda CAN Utilities
# File        : brownpandacan.py
# Brief       : CAN bus layout and message pack helpers for BrownPanda
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================
from cereal import car
from opendbc.car import CanBusBase

# NOTE: Follow Honda-style helpers: keep CAN pack functions here
# so carcontroller calls into this module to build CAN messages.
# This mirrors openpilot's hondacan pattern.


# =============================================================================
# [Section] Class
# [Brief ] BrownPanda CAN bus mapping and accessors
# ---------------------------------------------------------------------------
# [Class ] CanBus
# [Brief ] Single-bus mapping (pt/radar/lkas -> same bus)
# [Bases ] CanBusBase
# =============================================================================
class CanBus(CanBusBase):
  # =============================================================================
  # [Section] Bus Layout
  # [Brief ] Single-bus mapping for all virtual buses (pt/radar/lkas)
  # ---------------------------------------------------------------------------
  # [Function] __init__
  # [Brief ] Initialize single-bus mapping for BrownPanda
  # [Params] CP: CarParams | None, fingerprint: dict | None
  # [Returns] None
  # =============================================================================
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP if fingerprint is None else None, fingerprint)

    # Single-bus architecture: all virtual buses map to the same physical bus
    # This simplifies wiring and reduces complexity while maintaining compatibility
    self._pt = self.offset      # Powertrain bus (primary)
    self._radar = self.offset   # Radar bus (not used - camera-based ADAS)
    self._lkas = self.offset    # LKAS bus (unified with powertrain)

  # Bus accessors
  @property
  def pt(self) -> int:
    return self._pt

  @property
  def radar(self) -> int:
    return self._radar

  @property
  def lkas(self) -> int:
    return self._lkas


# =============================================================================
# [Section] CAN Pack Helpers (Honda-style)
# [Brief ] High-level builders that construct values and pack messages
# =============================================================================
VisualAlert = car.CarControl.HUDControl.VisualAlert


# =============================================================================
# [Section] CAN: latCommand
# [Brief ] Build and pack steering command
# ---------------------------------------------------------------------------
# [Function] create_lat_command
# [Brief ] Build and pack latCommand (steering)
# [Params] packer: CANPacker, CAN: CanBus, CC: CarControl, steering_angle: float, steering_torque: float
# [Returns] tuple: packed CAN message
# =============================================================================
def create_lat_command(packer, CAN: CanBusBase, CC, steering_angle: float, steering_torque: float):
  # Build and pack latCommand from high-level inputs and CC state
  hud_alert = getattr(CC.hudControl, 'visualAlert', None)
  if hud_alert == VisualAlert.steerRequired:
    steer_alert_level, shake_level = 2, 2
  elif hud_alert == VisualAlert.fcw:
    steer_alert_level, shake_level = 3, 3
  else:
    steer_alert_level, shake_level = 0, 0

  # Lane departure attention
  if CC.latActive and getattr(CC.hudControl, 'leftLaneDepart', False):
    steer_alert_level = max(steer_alert_level, 1)
    shake_level = max(shake_level, 1)
  if CC.latActive and getattr(CC.hudControl, 'rightLaneDepart', False):
    steer_alert_level = max(steer_alert_level, 1)
    shake_level = max(shake_level, 1)

  values = {
    "cmdSteerAngle": max(min(steering_angle, 3276.7), -3276.8),
    "cmdSteerTorque": max(min(steering_torque, 327.67), -327.68),
    "cmdSteerMode": 1 if CC.latActive else 0,
    "cmdSteerEnable": 1 if CC.latActive else 0,
    "cmdSteerAlert": steer_alert_level,
    "cmdSteerShake": shake_level,
    "cmdLaneActive": 1 if CC.latActive else 0,
    "cmdLaneEnable": 1 if CC.latActive else 0,
    "cmdLaneAlert": 1 if steer_alert_level > 0 else 0,
  }
  return packer.make_can_msg("_0x06C_latCommand", CAN.pt, values)


# =============================================================================
# [Section] CAN: longCommand
# [Brief ] Build and pack primary longitudinal command
# ---------------------------------------------------------------------------
# [Function] create_long_command
# [Brief ] Build and pack longCommand (primary longitudinal)
# [Params] packer: CANPacker, CAN: CanBus, CC: CarControl, CS: CarState, accel: float, torque_command: float
# [Returns] tuple: packed CAN message
# =============================================================================
def create_long_command(packer, CAN: CanBusBase, CC, CS, accel: float, torque_command: float):
  # Build and pack longCommand (primary longitudinal)
  speed_target = getattr(getattr(CC, 'cruiseControl', None), 'speedTarget', None)
  if speed_target is None:
    speed_target = getattr(CS.out, 'vEgo', 0.0) * 3.6

  brake_pressure = abs(min(accel, 0.0)) * 20.0
  brake_pedal = abs(min(accel, 0.0)) * 25.0  # 100% at ~4 m/s² decel

  values = {
    "cmdSpeedTarget": max(min(speed_target, 655.35), 0.0),
    "cmdTorque": max(min(torque_command, 15475.0), -5000.0),
    "cmdBrakePressure": max(min(brake_pressure, 63.984375), 0.0),
    "cmdPedalBrk": max(min(brake_pedal, 102.0), 0.0),
  }
  return packer.make_can_msg("_0x06A_longCommand", CAN.pt, values)


# =============================================================================
# [Section] CAN: longCommand2
# [Brief ] Build and pack advanced longitudinal command
# ---------------------------------------------------------------------------
# [Function] create_long_command2
# [Brief ] Build and pack longCommand2 (advanced longitudinal)
# [Params] packer: CANPacker, CAN: CanBus, CC: CarControl, CS: CarState, accel: float, stopping_counter: int, params: CarControllerParams
# [Returns] tuple: packed CAN message
# =============================================================================
def create_long_command2(packer, CAN: CanBusBase, CC, CS, accel: float, stopping_counter: int, params):
  # Build and pack longCommand2 (advanced longitudinal)
  speed_accel = max(accel, 0.0) if CC.longActive else 0.0
  speed_decel = abs(min(accel, 0.0)) if CC.longActive else 0.0

  values = {
    "cmdSpeedAccel": max(min(speed_accel, 2.75), -10.0),
    "cmdSpeedDecel": max(min(speed_decel, 2.75), -10.0),
    "cmdAccelTune": 7,
    "cmdDecelTune": 7,

    "cmdCruiseEnabled": 1 if (CC.enabled and CS.out.cruiseState.enabled) else 0,
    "cmdCruiseActive": 1 if CC.longActive else 0,
    "cmdCruiseResume": 1 if getattr(getattr(CC, 'cruiseControl', None), 'resume', False) else 0,

    "cmdCruiseSsEn": 1 if hasattr(CC, 'standstill') else 0,
    "cmdCruiseSsActive": 1 if stopping_counter > 200 else 0,
    "cmdStandstillResume": 1 if getattr(CS.out, 'vEgo', 0.0) < 0.1 and CC.longActive else 0,

    "cmdRegenEn": 1 if hasattr(params, 'REGEN_STRENGTH') else 0,
    "cmdRegenKer": 1 if hasattr(params, 'REGEN_BRAKE_MAX') else 0,

    "cmdAccControllable": 1 if CC.longActive else 0,
    "cmdAccOverride": 0,
    "cmdReqActiveLow": 0,

    "cmdIdleStopEn": 1 if stopping_counter > 200 else 0,
    "cmdBrakeComp": max(min(abs(min(accel, 0.0)) * getattr(params, 'REGEN_BRAKE_MAX', 0.3), 1.0), 0.0),
    "cmdFcwAlert": 1 if getattr(getattr(CC, 'hudControl', None), 'visualAlert', None) == VisualAlert.fcw else 0,
    "cmdAebReq": 0,
    "cmdBrakePump": 1 if (accel < getattr(params, 'ACCEL_MIN', -3.5) * 0.5) else 0,
  }
  return packer.make_can_msg("_0x06B_longCommand2", CAN.pt, values)


# =============================================================================
# [Section] CAN: visionInfo
# [Brief ] Build and pack road/vision info
# ---------------------------------------------------------------------------
# [Function] create_vision_info
# [Brief ] Build and pack visionInfo (environment/road analysis)
# [Params] packer: CANPacker, CAN: CanBus, CC: CarControl, CS: CarState
# [Returns] tuple: packed CAN message
# =============================================================================
def create_vision_info(packer, CAN: CanBusBase, CC, CS):
  # Build and pack visionInfo (environment/road analysis)
  # Defaults; can be sourced from model outputs later
  values = {
    "roadType": 2,
    "tlState": 0,
    "obstacle": 0,
    "roadValid": 1,
    "curv": 0.0,
    "laneW": 3.7,
    "laneOffs": 0.0,
  }
  return packer.make_can_msg("_0x06F_visionInfo", CAN.pt, values)


# =============================================================================
# [Section] CAN: dispCommand
# [Brief ] Build and pack display/HUD info
# ---------------------------------------------------------------------------
# [Function] create_disp_command
# [Brief ] Build and pack dispCommand (HUD/UI)
# [Params] packer: CANPacker, CAN: CanBus, CC: CarControl, CS: CarState
# [Returns] tuple: packed CAN message
# =============================================================================
def create_disp_command(packer, CAN: CanBusBase, CC, CS):
  # Build and pack dispCommand (HUD/UI)
  hud = CC.hudControl

  hud_speed = min(getattr(CS.out, 'vEgo', 0.0) * 3.6, 140.0)
  left_lane = 1 if getattr(hud, 'leftLaneVisible', False) else 0
  right_lane = 1 if getattr(hud, 'rightLaneVisible', False) else 0
  center_lanes = 2 if getattr(hud, 'lanesVisible', False) else 0

  lead_center = 1 if getattr(hud, 'leadVisible', False) else 0
  lead_left = 1 if getattr(CS.out, 'leftBlindspot', False) else 0
  lead_right = 1 if getattr(CS.out, 'rightBlindspot', False) else 0

  depart_left = 1 if getattr(hud, 'leftLaneDepart', False) else 0
  depart_right = 1 if getattr(hud, 'rightLaneDepart', False) else 0

  sound_alert = 1 if (getattr(hud, 'visualAlert', None) in (VisualAlert.fcw, VisualAlert.steerRequired)) else 0
  follow_distance = getattr(hud, 'leadDistanceBars', 2)

  values = {
    "cmdDispCcEn": 1 if CC.enabled else 0,
    "cmdDispCcActive": 1 if CC.longActive else 0,
    "cmdDispLnEn": 1 if CC.latActive else 0,
    "cmdDispLnActive": 1 if CC.latActive else 0,

    "cmdDispSoundAlert": sound_alert,

    "cmdDispLeadC": lead_center,
    "cmdDispLeadL": lead_left,
    "cmdDispLeadR": lead_right,

    "cmdDispDepartLeft": depart_left,
    "cmdDispDepartRight": depart_right,

    "cmdDispLane1": left_lane,
    "cmdDispLane2": center_lanes,
    "cmdDispLane3": center_lanes,
    "cmdDispLane4": right_lane,

    "cmdDispRoadEdgeLeft": 1 if (CC.latActive and left_lane) else 0,
    "cmdDispRoadEdgeRight": 1 if (CC.latActive and right_lane) else 0,

    "cmdDispFollowDistance": max(min(follow_distance, 3), 0),
    "cmdDispSpeed": max(min(hud_speed, 655.35), 0.0),
    "cmdDispPassThrough": 0,
  }

  return packer.make_can_msg("_0x06E_dispCommand", CAN.pt, values)

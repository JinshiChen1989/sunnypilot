# =============================================================================
# Module      : BrownPanda CarState Parser
# File        : carstate.py
# Brief       : Parse BrownPanda CAN into cereal CarState
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================

# Standard library imports
from collections import defaultdict

# OpenPilot framework imports
from cereal import car
from opendbc.can import CANParser
from opendbc.car import create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.brownpanda.values import DBC, STEER_THRESHOLD, TEMP_STEER_FAULTS, PERM_STEER_FAULTS
from opendbc.car.brownpanda.brownpandacan import CanBus
from opendbc.car.interfaces import CarStateBase

TransmissionType = car.CarParams.TransmissionType
ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter

# Button and control mapping constants
# Maps physical buttons to cereal ButtonEvent types for standardized processing
# Button mapping: DBC signal names -> cereal ButtonEvent types
BUTTONS_DICT = {
  # Core cruise control buttons
  "btnSpdUp": ButtonType.accelCruise,       # Speed up / Resume
  "btnSpdDn": ButtonType.decelCruise,       # Speed down / Set
  "btnCcCancel": ButtonType.cancel,         # Cancel cruise control
  "btnCcResume": ButtonType.resumeCruise,   # Resume cruise control
  "btnCcEn": ButtonType.mainCruise,         # Enable cruise control
  "btnCcSet": ButtonType.setCruise,         # Set cruise speed

  # Following distance control
  "btnDistFar": ButtonType.gapAdjustCruise, # Increase following distance
  "btnDistNr": ButtonType.gapAdjustCruise,  # Decrease following distance

  # Lane keeping and driver assistance
  "btnLaneEn": ButtonType.lkas,       # Lane keeping enable
  "btnMads": ButtonType.altButton2,         # MADS (sunnypilot feature)

  # Turn signals
  "btnBlinkL": ButtonType.leftBlinker,      # Left turn signal
  "btnBlinkR": ButtonType.rightBlinker,     # Right turn signal
}

# Transmission gear mapping: DBC values -> cereal GearShifter enum
# Simple direct mapping for Chinese EV transmissions
GEAR_DICT = {
  0: GearShifter.park,     # P - Park
  1: GearShifter.reverse,  # R - Reverse
  2: GearShifter.neutral,  # N - Neutral
  3: GearShifter.drive,    # D - Drive
}


# =============================================================================
# [Section] Class
# [Brief ] Parse BrownPanda CAN into cereal CarState
# ---------------------------------------------------------------------------
# [Class ] CarState
# [Brief ] Provides update loop and CAN parser configuration
# [Bases ] CarStateBase
# =============================================================================
class CarState(CarStateBase):
  # =============================================================================
  # [Section] Initialization
  # [Brief ] Initialize state, default mappings, and system capabilities
  # ---------------------------------------------------------------------------
  # [Function] __init__
  # [Brief ] Initialize parser state and feature tracking
  # [Params] CP: CarParams
  # [Returns] None
  # =============================================================================
  def __init__(self, CP):
    super().__init__(CP)

    # Gear position tracking (simple direct mapping for EVs)
    self.shifter_values = {0: 0, 1: 1, 2: 2, 3: 3}

    # Button state tracking for edge detection
    self.button_states = defaultdict(int)

    # Driver assistance system states
    self.main_on_last = False
    self.lkas_enabled = False           # Lane keeping active status
    self.lane_enabled_status = True     # Lane keeping enabled in cluster

    # Following distance control (1=closest, 4=farthest)
    self.gap_index = 2                  # Default medium following distance
    self.gap_debounce_frames = 0        # Prevent button bounce

    # System capabilities
    self.has_camera = True              # Vision-based ADAS

  # =============================================================================
  # [Section] Update Loop
  # [Brief ] Parse CAN samples into standardized cereal CarState
  # ---------------------------------------------------------------------------
  # [Function] update
  # [Brief ] Parse CAN into cereal CarState
  # [Params] cp: CANParser
  # [Returns] car.CarState
  # =============================================================================
  def update(self, cp) -> car.CarState:
    ret = car.CarState.new_message()

    # Update button debounce
    if self.gap_debounce_frames > 0:
      self.gap_debounce_frames -= 1

    # Vehicle dynamics
    ret.vEgo = cp.vl["0x063_carState"]["vehSpd"] * CV.KPH_TO_MS  # Convert km/h to m/s
    ret.vEgoRaw = ret.vEgo
    ret.standstill = ret.vEgo < 0.1  # Vehicle stopped threshold

    # Wheel speeds
    ret.wheelSpeeds.fl = cp.vl["0x065_wheelSensor"]["wheelFL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["0x065_wheelSensor"]["wheelFR"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["0x065_wheelSensor"]["wheelRL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["0x065_wheelSensor"]["wheelRR"] * CV.KPH_TO_MS

    # Steering
    ret.steeringAngleDeg = cp.vl["0x061_userCommand"]["steerAngle"]  # Steering wheel angle
    ret.steeringTorque = cp.vl["0x063_carState"]["steerTorq"]        # Driver input torque
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD  # Driver override detection

    # Powertrain faults (steer/ACC)
    powertrain_fault = cp.vl["0x068_powertrainState"]["PWR_FAULT"]
    ret.steerFaultTemporary = powertrain_fault in TEMP_STEER_FAULTS   # Recoverable faults
    ret.steerFaultPermanent = powertrain_fault in PERM_STEER_FAULTS   # Non-recoverable faults
    ret.accFaulted = powertrain_fault in (5, 6)  # Sensor/communication faults affect ACC

    # Pedals
    ret.gas = cp.vl["0x061_userCommand"]["GAS_PEDAL"] / 100.0         # Accelerator pedal (0.0-1.0)
    ret.gasPressed = ret.gas > 0.05                           # 5% threshold for gas pressed
    ret.brake = cp.vl["0x061_userCommand"]["BRAKE_PEDAL"] / 100.0       # Brake pedal (0.0-1.0)
    ret.brakePressed = bool(cp.vl["0x061_userCommand"]["BRAKE_PEDAL_PRESSED"])  # Physical brake switch

    # Additional inputs (used for enhanced dynamics)
    _yaw_rate_user = cp.vl["0x061_userCommand"]["YAW_RATE"]  # Driver yaw rate input for enhanced dynamics

    # Gear and braking
    ret.gearShifter = GEAR_DICT.get(cp.vl["0x063_carState"]["GEAR_POS"], GearShifter.unknown)
    ret.regenBraking = cp.vl["0x063_carState"]["BRK_PRESSURE"] > 0  # Regenerative braking active

    # RPM and energy level
    ret.engineRPM = cp.vl["0x068_powertrainState"]["ENG_RPM"]  # Motor RPM for EVs, engine RPM for hybrids

    # Battery or fuel level
    eng_type = cp.vl["0x068_powertrainState"]["ENG_TYPE"]
    if eng_type == 2:  # BEV (Battery Electric Vehicle)
        ret.fuelGauge = cp.vl["0x068_powertrainState"]["BATT_LVL"] * 0.01  # Battery level (0.0-1.0)
    else:  # ICE, HEV, FCEV - use fuel level
        ret.fuelGauge = cp.vl["0x068_powertrainState"]["FUEL_LVL"] * 0.01  # Fuel level (0.0-1.0)

    # Charging status
    charge_state = cp.vl["0x064_carState2"]["CHRG_STATE"]
    ret.charging = bool(charge_state > 0)  # Active charging detection

    # Non-critical fault heuristic
    ret.carFaultedNonCritical = powertrain_fault in [1, 2, 5, 6]  # Overheat, voltage, sensor faults

    # IMU sensor
    ret.aEgo = cp.vl["0x069_imuSensor"]["IMU_AXIS_LONG"]  # Longitudinal acceleration (m/s²)
    ret.yawRate = cp.vl["0x061_userCommand"]["YAW_RATE"]   # Vehicle yaw rate (rad/s)

    # Cruise state
    ret.cruiseState.enabled = bool(cp.vl["0x063_carState"]["STAT_CC_ACTIVE"])   # Cruise control active
    ret.cruiseState.available = bool(cp.vl["0x063_carState"]["STAT_CC_EN"])     # Cruise control available

    # Set speed (if available)
    try:
      set_speed_kph = cp.vl["0x064_carState2"]["SET_SPEED_KPH"]
      if set_speed_kph > 0:
        ret.cruiseState.speedCluster = set_speed_kph * CV.KPH_TO_MS  # Convert to m/s
        ret.cruiseState.speed = ret.cruiseState.speedCluster
    except KeyError:
      ret.cruiseState.speed = ret.vEgo  # Fallback to current speed

    ret.cruiseState.standstill = ret.standstill

    # Lane keeping state
    self.lkas_enabled = bool(cp.vl["0x063_carState"]["STAT_LN_ACTIVE"])        # Lane keeping active
    self.lane_enabled_status = bool(cp.vl["0x063_carState"]["STAT_LN_EN"])     # Lane keeping enabled in cluster

    # Doors/seatbelt/parking brake
    ret.doorOpen = any([
      cp.vl["0x064_carState2"]["SWITCH_DOOR_FL"],  # Front left door
      cp.vl["0x064_carState2"]["SWITCH_DOOR_FR"],  # Front right door
      cp.vl["0x064_carState2"]["SWITCH_DOOR_RL"],  # Rear left door
      cp.vl["0x064_carState2"]["SWITCH_DOOR_RR"],  # Rear right door
    ])
    ret.seatbeltUnlatched = not cp.vl["0x064_carState2"]["SWITCH_BELT_DRIVER"]  # Driver seatbelt
    ret.parkingBrake = bool(cp.vl["0x063_carState"]["PARKING_BRAKE"])           # Parking brake engaged
    ret.genericToggle = bool(cp.vl["0x063_carState"]["RDY_DRIVE"])              # Vehicle ready to drive

    # Blind spot
    ret.blindSpotLeft = bool(cp.vl["0x063_carState"]["BLIND_SPOT_FL"]) or bool(cp.vl["0x063_carState"]["BLIND_SPOT_RL"])
    ret.blindSpotRight = bool(cp.vl["0x063_carState"]["BLIND_SPOT_FR"]) or bool(cp.vl["0x063_carState"]["BLIND_SPOT_RR"])

    # Lead detection (camera-based)
    ret.radarState.leadOne.status = bool(cp.vl["0x066_adasState"]["LEAD_CONF"] > 50)  # High confidence threshold
    ret.radarState.leadOne.dRel = cp.vl["0x066_adasState"]["LEAD_DIST_X"]              # Distance ahead (m)
    ret.radarState.leadOne.yRel = cp.vl["0x066_adasState"]["LEAD_DIST_Y"]              # Lateral offset (m)
    ret.radarState.leadOne.vRel = 0.0                                          # No relative velocity from camera
    ret.radarState.radarUnavailable = True

    # Vision processing (placeholders)
    _vision_road_type = cp.vl["0x06F_visionInfo"]["ROAD_TYPE"]      # Highway vs city detection
    _vision_traffic_light = cp.vl["0x06F_visionInfo"]["TL_STATE"]   # Traffic light state
    _path_curvature = cp.vl["0x06F_visionInfo"]["CURV"]            # Path curvature (1/m)
    _lane_width = cp.vl["0x06F_visionInfo"]["LANE_W"]               # Lane width (m)
    # Note: These could be used for enhanced lateral control and adaptive behavior

    # Additional blindspot mapping for cereal compatibility
    ret.leftBlindspot = bool(cp.vl["0x063_carState"]["BLIND_SPOT_FL"] or cp.vl["0x063_carState"]["BLIND_SPOT_RL"])
    ret.rightBlindspot = bool(cp.vl["0x063_carState"]["BLIND_SPOT_FR"] or cp.vl["0x063_carState"]["BLIND_SPOT_RR"])

    # Process button events for cruise control and driver assistance
    button_events = []
    for button_name, button_type in BUTTONS_DICT.items():
      current_pressed = bool(cp.vl["0x061_userCommand"][button_name])
      previous_pressed = self.button_states[button_name]

      # Detect button press/release events
      if current_pressed != previous_pressed:
        button_events.extend(create_button_events(
          int(current_pressed), int(previous_pressed), {1: button_type}
        ))
        self.button_states[button_name] = current_pressed

        # Handle following distance adjustment (with debounce)
        if current_pressed and self.gap_debounce_frames == 0:
          if button_name == "btnDistFar":
            self.gap_index = min(self.gap_index + 1, 4)  # Increase distance (max 4)
            self.gap_debounce_frames = 5                 # Prevent button bounce
          elif button_name == "btnDistNr":
            self.gap_index = max(self.gap_index - 1, 1)  # Decrease distance (min 1)
            self.gap_debounce_frames = 5                 # Prevent button bounce

    ret.buttonEvents = button_events

    # Following distance setting (prefer CAN signal if available, otherwise use button tracking)
    try:
      gi = int(cp.vl["0x064_carState2"]["GAP_INDEX"])
      if 1 <= gi <= 4:  # Valid gap index range
        self.gap_index = gi
    except KeyError:
      pass  # Use button-tracked gap_index
    ret.gapAdjustCruiseTr = int(self.gap_index)  # Current following distance setting

    # Turn signal processing (button input + actual lamp status)
    ret.leftBlinker = bool(cp.vl["0x061_userCommand"]["btnBlinkL"])    # Left turn button
    ret.rightBlinker = bool(cp.vl["0x061_userCommand"]["btnBlinkR"])   # Right turn button

    # Get actual turn signal lamp status (prefer lamp over button)
    lamp_l = ret.leftBlinker
    lamp_r = ret.rightBlinker
    try:
      lamp_l = bool(cp.vl["0x064_carState2"]["TURN_SIGNAL_L"]) or lamp_l  # Left lamp actual status
      lamp_r = bool(cp.vl["0x064_carState2"]["TURN_SIGNAL_R"]) or lamp_r  # Right lamp actual status
    except KeyError:
      pass  # Use button status as fallback

    # Hazard lights
    hazard = bool(cp.vl["0x061_userCommand"].get("HAZARD_BTN", 0))
    if hazard:
      lamp_l = True
      lamp_r = True

    ret.leftBlinkerOn = lamp_l   # Final left turn signal status
    ret.rightBlinkerOn = lamp_r  # Final right turn signal status

    # AEB/FCW (from vision)
    vision_obstacle = cp.vl["0x06F_visionInfo"]["OBSTACLE"]
    ret.stockAeb = bool(vision_obstacle > 2)  # Automatic emergency braking trigger
    ret.stockFcw = bool(vision_obstacle > 1)  # Forward collision warning

    # ACC fault (prefer explicit signal)
    try:
      ret.accFaulted = bool(cp.vl["0x064_carState2"]["ACC_FAULT"]) or ret.accFaulted
    except KeyError:
      pass  # Use powertrain fault mapping

    # ESP
    ret.espDisabled = not bool(cp.vl["0x067_chassisState"]["ESP_EN"])  # Electronic Stability Program

    # Chassis accel (backup to IMU)
    _chassis_accel_long = cp.vl["0x067_chassisState"]["LONG_ACCEL"]  # Longitudinal acceleration
    _chassis_accel_lat = cp.vl["0x067_chassisState"]["LAT_ACCEL"]    # Lateral acceleration
    # Note: These could be used as IMU backup for enhanced reliability

    # Finalize
    self.out = ret.as_reader()
    return ret

  @staticmethod
  # =============================================================================
  # [Section] CAN Parser (Powertrain bus)
  # [Brief ] Define message list and expected frequencies
  # ---------------------------------------------------------------------------
  # [Function] get_can_parser
  # [Brief ] Configure powertrain-bus CAN parser
  # [Params] CP: CarParams
  # [Returns] CANParser
  # =============================================================================
  def get_can_parser(CP):
    # CAN messages to parse with their frequencies (Hz)
    messages = [
      ("0x60_pandaMessage", 100), # Vehicle and panda device status
      ("0x061_userCommand", 100),  # Driver inputs (buttons, pedals, steering)
      ("0x62_driverState", 50),   # Driver monitoring and attention tracking
      ("0x063_carState", 100),     # Primary vehicle state (speed, gear, cruise)
      ("0x064_carState2", 100),    # Extended vehicle state (doors, charging)
      ("0x065_wheelSensor", 50),   # Individual wheel speed sensors
      ("0x066_adasState", 20),     # Camera-based ADAS data (lead vehicle)
      ("0x067_chassisState", 50),  # Stability control and chassis systems
      ("0x068_powertrainState", 20), # Motor/engine and energy management
      ("0x069_imuSensor", 100),    # Inertial measurement unit data
      ("0x06F_visionInfo", 50),    # Advanced vision processing results
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).pt)

  @staticmethod
  # =============================================================================
  # [Section] Camera CAN Parser
  # [Brief ] Single-bus design; return empty parser for camera bus
  # ---------------------------------------------------------------------------
  # [Function] get_cam_can_parser
  # [Brief ] Configure camera-bus CAN parser (single-bus: empty)
  # [Params] CP: CarParams
  # [Returns] CANParser
  # =============================================================================
  def get_cam_can_parser(CP):
    messages = []  # No separate camera bus messages
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).pt)


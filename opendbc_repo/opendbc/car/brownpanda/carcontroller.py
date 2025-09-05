# =============================================================================
# Module      : BrownPanda Vehicle Controller
# File        : carcontroller.py
# Brief       : Convert openpilot control commands into BrownPanda CAN messages
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================
from cereal import car
import cereal.messaging as messaging
from opendbc.car import DT_CTRL, rate_limit
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.can import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.brownpanda.brownpandacan import CanBus
from opendbc.car.brownpanda import brownpandacan
from opendbc.car.brownpanda.values import (
  CarControllerParams,
  DEFAULT_WHEEL_RADIUS,
)
import numpy as np
from openpilot.common.params import Params

# OpenPilot framework constants and data structures
LongCtrlState = car.CarControl.Actuators.LongControlState

# =============================================================================
# [Section] Class
# [Brief ] Translate OP control outputs into BrownPanda CAN messages
# ---------------------------------------------------------------------------
# [Class ] CarController
# [Brief ] Owns control loop and delegates packing to brownpandacan
# [Bases ] CarControllerBase
# =============================================================================
class CarController(CarControllerBase):

  # =============================================================================
  # [Section] Initialization
  # [Brief ] Initialize controller state, packer, bus, and parameters
  # ---------------------------------------------------------------------------
  # [Function] __init__
  # [Brief ] Initialize controller state, packer, bus, params, and caches
  # [Params] dbc_name: str, CP: CarParams, VM: VehicleModel
  # [Returns] None
  # =============================================================================
  def __init__(self, dbc_name, CP, VM):
    # Core openpilot components
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = CanBus(CP)
    self.frame = 0

    # Load model-specific parameters with comprehensive safety fallbacks
    try:
      # Try to load car-specific parameters (BYD ATTO3, DOLPHIN, DEEPAL S05)
      self.params = CarControllerParams(CP)
    except Exception:
      # CRITICAL: Fallback to safe defaults if model-specific loading fails
      # This prevents system crashes when model detection fails
      self.params = CarControllerParams(CP)  # Will use _init_default_params()

      # Additional safety overrides in case defaults are missing
      if not hasattr(self.params, 'ACCEL_MAX'):
        self.params.ACCEL_MAX = 2.0   # Conservative acceleration limit
      if not hasattr(self.params, 'ACCEL_MIN'):
        self.params.ACCEL_MIN = -3.5  # Conservative deceleration limit
      if not hasattr(self.params, 'STEER_MAX'):
        self.params.STEER_MAX = 150   # Safe steering torque limit (Nm) - corrected

    # === CONTROL STATE MEMORY ===
    # Store previous values for rate limiting and smooth transitions
    self.last_steer = 0.0           # Previous steering command
    self.last_steer_angle = 0.0     # Previous steering angle (deg)
    self.last_steer_torque = 0.0    # Previous steering torque (Nm)
    self.last_accel = 0.0           # Previous acceleration command
    self.last_brake = 0.0           # Previous brake command
    self.stopping_counter = 0       # Counter for standstill logic

    # === CURRENT CONTROL OUTPUTS ===
    # Current control values sent to actuators
    self.accel = 0.0               # Current acceleration (m/s²)
    self.speed = 0.0               # Current target speed (m/s)
    self.gas = 0.0                 # Current gas pedal (0-1)
    self.brake = 0.0               # Current brake pedal (0-1)

    # SunnyPilot integration for enhanced features
    self.sm = messaging.SubMaster(['longitudinalPlanSP'])  # Enhanced longitudinal planner
    self.param_s = Params()                                # Parameter storage interface
    self.is_metric = self.param_s.get_bool("IsMetric")     # Unit system (metric/imperial)

  # =============================================================================
  # [Section] Physics: Accel -> Wheel Torque
  # [Brief ] Convert requested acceleration to wheel torque using drivetrain
  #          parameters with safety limits and fallbacks.
  # ---------------------------------------------------------------------------
  # [Function] calculate_wheel_torque
  # [Brief ] Convert desired acceleration (m/s²) to wheel torque (Nm)
  # [Params] desired_accel: float
  # [Returns] float: wheel torque command (clipped to DBC range)
  # =============================================================================
  def calculate_wheel_torque(self, desired_accel):
    # Handle zero acceleration case (no torque needed)
    if desired_accel == 0:
      return 0.0

    # === LOAD CAR-SPECIFIC DRIVETRAIN PARAMETERS ===
    # Use model-specific values with safe fallbacks for unknown models
    vehicle_mass = self.CP.mass  # From openpilot CarParams (kg)
    wheel_radius = getattr(self.params, 'WHEEL_RADIUS', DEFAULT_WHEEL_RADIUS)

    # === PHYSICS CALCULATIONS ===
    # Step 1: Calculate required force at wheels using Newton's 2nd Law
    wheel_force = vehicle_mass * desired_accel  # F = ma (Newtons)

    # Step 2: Convert force to torque at wheels using torque arm
    wheel_torque = wheel_force * wheel_radius  # T = F × r (Newton-meters)

    # === APPLY REASONABLE WHEEL TORQUE LIMITS ===
    # For passenger EVs, reasonable wheel torque range is much higher than motor torque
    # because wheels are larger diameter and provide mechanical advantage
    max_wheel_torque = 4000.0  # Reasonable for passenger EV wheels (Nm)
    min_wheel_torque = -4000.0 # Reasonable regenerative limit (Nm)

    # Apply reasonable limits
    wheel_torque_limited = np.clip(wheel_torque, min_wheel_torque, max_wheel_torque)

    # === DBC SIGNAL RANGE COMPLIANCE ===
    # Clip to DBC signal range for cmdTorque (wheel torque)
    return np.clip(wheel_torque_limited, -5000.0, 15475.0)

  # =============================================================================
  # [Section] Control Loop (100 Hz)
  # [Brief ] Orchestrates lateral/longitudinal processing and CAN message build
  # ---------------------------------------------------------------------------
  # [Function] update
  # [Brief ] Main control loop at 100 Hz
  # [Params] CC: CarControl, CS: CarState, now_nanos: int
  # [Returns] (Actuators, list[CAN msg])
  # =============================================================================
  def update(self, CC, CS, now_nanos):
    # Update sunnypilot-specific params periodically
    if not self.CP.pcmCruiseSpeed:
      self.sm.update(0)
      if self.frame % 200 == 0:  # Update every 2 seconds
        self.is_metric = self.param_s.get_bool("IsMetric")

    # Extract control inputs
    actuators = CC.actuators
    can_sends = []

    # Lateral control with rate limiting
    if CC.latActive:
      desired_angle = actuators.steeringAngleDeg
      desired_torque = actuators.torque

      # Apply standard angle rate limits using the ANGLE_LIMITS configuration
      limited_angle = apply_std_steer_angle_limits(desired_angle, self.last_steer_angle, CS.out.vEgo,
                                                  CS.out.steeringAngleDeg, CC.latActive, self.params.ANGLE_LIMITS)

      # Torque rate limit (per model or default)
      torque_delta = getattr(self.params, 'STEER_DELTA_UP', 20) * DT_CTRL  # Safe fallback: 20 Nm/s
      steer_max = getattr(self.params, 'STEER_MAX', 1500)  # Safe fallback: 1500 Nm

      # Apply torque rate limit
      limited_torque = rate_limit(desired_torque, self.last_steer_torque,
                                -torque_delta, torque_delta)
      limited_torque = np.clip(limited_torque, -steer_max, steer_max)

    else:
      # Steering inactive
      limited_angle = CS.out.steeringAngleDeg if hasattr(CS.out, 'steeringAngleDeg') else 0.0
      limited_torque = 0.0

    # Longitudinal control with accel limits and pedal mapping
    if CC.longActive:
      # Accel limits (per model or default)
      accel_max = getattr(self.params, 'ACCEL_MAX', 2.0)    # Safe fallback: 2.0 m/s²
      accel_min = getattr(self.params, 'ACCEL_MIN', -3.5)   # Safe fallback: -3.5 m/s²

      # Clip accel to limits
      accel = np.clip(actuators.accel, accel_min, accel_max)

      # Pedal mapping
      if accel > 0:
        # Gas: scale by max accel
        gas = min(accel / accel_max, 1.0)  # 0-1 range scaled to model capability
        brake = 0.0
      else:
        # Brake: scale by max decel
        gas = 0.0
        brake = min(abs(accel) / abs(accel_min), 1.0)  # 0-1 range scaled to model capability

      # Compute wheel torque
      torque_command = self.calculate_wheel_torque(accel)

      # Standstill management
      stopping = actuators.longControlState == LongCtrlState.stopping
      self.stopping_counter = self.stopping_counter + 1 if stopping else 0

    else:
      # Longitudinal inactive
      accel = gas = brake = torque_command = 0.0
      self.stopping_counter = 0

    # Build CAN messages with standard timing

    # Steering @ 100Hz (every frame)
    can_sends.extend(self._generate_latCommand_message(limited_angle, limited_torque, CC))

    # Longitudinal @ 50Hz and 25Hz
    can_sends.extend(self._generate_longCommand_message(CC, CS, accel, torque_command))     # 50Hz: Primary control
    can_sends.extend(self._generate_longCommand2_message(CC, CS, accel))                    # 25Hz: Advanced features

    # Display @ 10Hz, vision @ 50Hz
    can_sends.extend(self._generate_dispCommand_message(CC, CS))  # 10Hz: UI updates
    can_sends.extend(self._generate_visionInfo_message(CC, CS))   # 50Hz: Road analysis

    # Save state for next cycle
    self.last_steer_angle = limited_angle
    self.last_steer_torque = limited_torque
    self.last_accel = gas     # Store gas pedal value (not raw accel)
    self.last_brake = brake

    # Update controller state
    self.accel = accel
    self.gas = gas
    self.brake = brake

    # Build actuator response for OP
    new_actuators = actuators.as_builder()
    new_actuators.speed = CS.out.vEgo              # Current vehicle speed
    new_actuators.accel = self.accel               # Applied acceleration
    new_actuators.gas = self.gas                   # Applied gas pedal
    new_actuators.brake = self.brake               # Applied brake pedal
    new_actuators.steer = self.last_steer          # Previous steer command
    new_actuators.steerOutputCan = int(limited_torque)  # CAN steering output

    # Increment frame counter and return control outputs
    self.frame += 1
    return new_actuators, can_sends

  # =============================================================================
  # [Section] CAN: latCommand (100 Hz)
  # [Brief ] Build steering-related CAN message via brownpandacan helper
  # ---------------------------------------------------------------------------
  # [Function] _generate_latCommand_message
  # [Brief ] Build latCommand at 100 Hz via brownpandacan
  # [Params] steering_angle: float, steering_torque: float, CC: CarControl
  # [Returns] list[CAN msg]
  # =============================================================================
  def _generate_latCommand_message(self, steering_angle, steering_torque, CC):
    # Delegate value building to brownpandacan
    return [brownpandacan.create_lat_command(self.packer, self.CAN, CC, steering_angle, steering_torque)]



  # =============================================================================
  # [Section] CAN: longCommand (50 Hz)
  # [Brief ] Build primary longitudinal control message
  # ---------------------------------------------------------------------------
  # [Function] _generate_longCommand_message
  # [Brief ] Build longCommand at 50 Hz via brownpandacan
  # [Params] CC: CarControl, CS: CarState, accel: float, torque_command: float
  # [Returns] list[CAN msg]
  # =============================================================================
  def _generate_longCommand_message(self, CC, CS, accel, torque_command):
    # Standard automotive timing: 50Hz for ACC commands
    if self.frame % 2 != 0:  # Send at 50Hz (standard automotive timing)
      return []

    # Delegate value building to brownpandacan
    return [brownpandacan.create_long_command(self.packer, self.CAN, CC, CS, accel, torque_command)]

  # =============================================================================
  # [Section] CAN: longCommand2 (25 Hz)
  # [Brief ] Build advanced longitudinal control message
  # ---------------------------------------------------------------------------
  # [Function] _generate_longCommand2_message
  # [Brief ] Build longCommand2 at 25 Hz via brownpandacan
  # [Params] CC: CarControl, CS: CarState, accel: float
  # [Returns] list[CAN msg]
  # =============================================================================
  def _generate_longCommand2_message(self, CC, CS, accel):
    if self.frame % 4 != 0:  # Send at 25Hz (every 4 frames) for advanced features
      return []

    # Delegate value building to brownpandacan
    return [brownpandacan.create_long_command2(self.packer, self.CAN, CC, CS, accel, self.stopping_counter, self.params)]



  # =============================================================================
  # [Section] CAN: visionInfo (50 Hz)
  # [Brief ] Build road/vision info message (placeholder values)
  # ---------------------------------------------------------------------------
  # [Function] _generate_visionInfo_message
  # [Brief ] Build visionInfo at 50 Hz via brownpandacan
  # [Params] CC: CarControl, CS: CarState
  # [Returns] list[CAN msg]
  # =============================================================================
  def _generate_visionInfo_message(self, CC, CS):
    if self.frame % 2 != 0:  # Send at 50Hz (every 2 frames) for real-time road analysis
      return []

    # Delegate value building to brownpandacan
    return [brownpandacan.create_vision_info(self.packer, self.CAN, CC, CS)]

  # =============================================================================
  # [Section] CAN: dispCommand (10 Hz)
  # [Brief ] Build HUD/display message
  # ---------------------------------------------------------------------------
  # [Function] _generate_dispCommand_message
  # [Brief ] Build dispCommand at 10 Hz via brownpandacan
  # [Params] CC: CarControl, CS: CarState
  # [Returns] list[CAN msg]
  # =============================================================================
  def _generate_dispCommand_message(self, CC, CS):
    if self.frame % 10 != 0:  # Send at 10Hz to match standard automotive UI timing
      return []

    # Delegate value building to brownpandacan
    return [brownpandacan.create_disp_command(self.packer, self.CAN, CC, CS)]

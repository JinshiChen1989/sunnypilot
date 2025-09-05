# =============================================================================
# Module      : BrownPanda Vehicle Configuration
# File        : values.py
# Brief       : Platform configs, flags, DBC mapping, and tuning parameters
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================

# Standard library and framework imports
from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from opendbc.car import CarSpecs, PlatformConfig, Platforms, DbcDict
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = car.CarParams.Ecu

# Vehicle operation constants and safety limits

# Speed thresholds
MIN_STEER_SPEED = 3. * CV.KPH_TO_MS     # Min speed for steering assist (3 km/h)
PEDAL_TRANSITION = 5. * CV.KPH_TO_MS    # Gas/brake transition speed (5 km/h)
STEER_THRESHOLD = 100                   # Driver torque threshold (Nm)

# Default drivetrain parameters
DEFAULT_VEHICLE_MASS = 1500.0          # kg
DEFAULT_WHEEL_RADIUS = 0.31            # m (approx 215/60R16)
DEFAULT_FINAL_DRIVE_RATIO = 3.5        # single-speed ratio
DEFAULT_MOTOR_EFFICIENCY = 0.9         # 0..1

# Wheel torque limits (wheel torque)
MAX_WHEEL_TORQUE = 4000.0              # Nm
MIN_WHEEL_TORQUE = -4000.0             # Nm (regen)

# Steering fault codes
TEMP_STEER_FAULTS = (0, 1, 2, 9, 11, 13, 21, 25)
PERM_STEER_FAULTS = (3, 4, 5, 17, 22, 31)

# Road type
ROAD_TYPE_UNKNOWN = 0
ROAD_TYPE_CITY = 1
ROAD_TYPE_HIGHWAY = 2
ROAD_TYPE_COUNTRY = 3

# Traffic light states
TRAFFIC_LIGHT_UNKNOWN = 0
TRAFFIC_LIGHT_RED = 1
TRAFFIC_LIGHT_YELLOW = 2
TRAFFIC_LIGHT_GREEN = 3

# =============================================================================
# [Section] Class
# [Brief ] Per-model control/tuning parameters with safe fallbacks
# ---------------------------------------------------------------------------
# [Class ] CarControllerParams
# [Brief ] Loads parameters based on CP.carFingerprint
# [Bases ] object
# =============================================================================
class CarControllerParams:

  # Common baseline parameters shared across all models
  STEER_STEP = 1                         # Steering command step size
  STEER_ERROR_MAX = 350                  # Maximum steering error tolerance (Nm)
  MOTOR_EFFICIENCY = 0.92                # Baseline motor efficiency
  TORQUE_WIND_DOWN = 0.96                # Torque reduction factor for smooth deactivation
  

  # =============================================================================
  # [Section] Parameter Selection
  # [Brief ] Select per-model parameters based on fingerprint
  # ---------------------------------------------------------------------------
  # [Function] __init__
  # [Brief ] Select per-model parameters based on fingerprint
  # [Params] CP: CarParams
  # [Returns] None
  # =============================================================================
  def __init__(self, CP):
    # Load model-specific parameters based on vehicle fingerprint
    if CP.carFingerprint == CAR.BYD_ATTO3:
      self._init_byd_atto3_params()         # Mid-size SUV parameters
    elif CP.carFingerprint == CAR.BYD_DOLPHIN:
      self._init_byd_dolphin_params()       # Compact city EV parameters
    elif CP.carFingerprint == CAR.DEEPAL_S05:
      self._init_deepal_s05_params()        # Performance sedan parameters
    else:
      self._init_default_params()           # Safe fallback parameters

  # =============================================================================
  # [Section] Model Init
  # [Brief ] Initialize BYD ATTO3 tuning parameters
  # ---------------------------------------------------------------------------
  # [Function] _init_byd_atto3_params
  # [Brief ] Initialize BYD ATTO3 tuning parameters
  # [Params] None
  # [Returns] None
  # =============================================================================
  def _init_byd_atto3_params(self):
    # Acceleration limits (0-100km/h in 7.3s)
    self.ACCEL_MAX = 2.0                   # Maximum acceleration (m/s²)
    self.ACCEL_MIN = -3.5                  # Maximum deceleration (m/s²)

    # Steering control parameters (aligned with KIA/Hyundai EV standards)
    self.STEER_MAX = 240                   # Maximum steering torque (Nm) - matches EV industry standards
    self.STEER_DELTA_UP = 15               # Torque ramp up rate
    self.STEER_DELTA_DOWN = 22             # Torque ramp down rate
    
    # Steering angle limits for BYD ATTO3
    self.ANGLE_LIMITS = AngleSteeringLimits(
      600,  # deg, maximum steering angle
      ([5, 25], [0.4, 0.2]),  # ANGLE_RATE_LIMIT_UP: (speed_bp, rate_v)
      ([5, 25], [0.5, 0.28])  # ANGLE_RATE_LIMIT_DOWN: (speed_bp, rate_v)
    )


    # Regenerative braking (60kW max regen power)
    self.REGEN_BRAKE_MAX = 0.38            # Maximum regen strength
    self.REGEN_STRENGTH = 0.75             # Regen feel calibration

    # Drivetrain specifications
    self.WHEEL_RADIUS = 0.334              # 215/55R18 tire radius (m)
    self.FINAL_DRIVE_RATIO = 3.36          # Single-speed transmission ratio
    self.MOTOR_EFFICIENCY = 0.91           # Front motor efficiency

  # =============================================================================
  # [Section] Model Init
  # [Brief ] Initialize BYD DOLPHIN tuning parameters
  # ---------------------------------------------------------------------------
  # [Function] _init_byd_dolphin_params
  # [Brief ] Initialize BYD DOLPHIN tuning parameters
  # [Params] None
  # [Returns] None
  # =============================================================================
  def _init_byd_dolphin_params(self):
    # Acceleration limits (0-100km/h in 7.5s)
    self.ACCEL_MAX = 2.3                   # Maximum acceleration (m/s²)
    self.ACCEL_MIN = -3.2                  # ADAS-tuned deceleration (m/s²)

    # Steering control parameters (aligned with compact EV standards)
    self.STEER_MAX = 210                   # Maximum steering torque (Nm) - matches compact EV standards
    self.STEER_DELTA_UP = 18               # ADAS-tuned torque ramp up
    self.STEER_DELTA_DOWN = 25             # ADAS-tuned torque ramp down
    
    # Steering angle limits for BYD DOLPHIN (more responsive for city driving)
    self.ANGLE_LIMITS = AngleSteeringLimits(
      520,  # deg, maximum steering angle (slightly lower for compact car)
      ([5, 25], [0.5, 0.25]),  # ANGLE_RATE_LIMIT_UP: faster response for city driving
      ([5, 25], [0.6, 0.35])   # ANGLE_RATE_LIMIT_DOWN: quicker recovery
    )


    # Regenerative braking (40kW max regen power)
    self.REGEN_BRAKE_MAX = 0.42            # Maximum regen strength
    self.REGEN_STRENGTH = 0.85             # Strong city regen feel

    # Drivetrain specifications
    self.WHEEL_RADIUS = 0.318              # 205/60R16 tire radius (m)
    self.FINAL_DRIVE_RATIO = 3.89          # Higher ratio for city efficiency
    self.MOTOR_EFFICIENCY = 0.93           # Efficient city motor

  # =============================================================================
  # [Section] Model Init
  # [Brief ] Initialize DEEPAL S05 tuning parameters
  # ---------------------------------------------------------------------------
  # [Function] _init_deepal_s05_params
  # [Brief ] Initialize DEEPAL S05 tuning parameters
  # [Params] None
  # [Returns] None
  # =============================================================================
  def _init_deepal_s05_params(self):
    # Acceleration limits (similar to ATTO3 - same performance class)
    self.ACCEL_MAX = 2.0                   # Maximum acceleration (m/s²) - matches ATTO3
    self.ACCEL_MIN = -3.5                  # Maximum deceleration (m/s²) - matches ATTO3

    # Steering control parameters (same as ATTO3)
    self.STEER_MAX = 240                   # Maximum steering torque (Nm) - matches ATTO3
    self.STEER_DELTA_UP = 15               # Torque ramp up rate - matches ATTO3
    self.STEER_DELTA_DOWN = 22             # Torque ramp down rate - matches ATTO3
    
    # Steering angle limits for DEEPAL S05 (performance sedan tuning)
    self.ANGLE_LIMITS = AngleSteeringLimits(
      600,  # deg, maximum steering angle (same as ATTO3)
      ([5, 25], [0.4, 0.2]),  # ANGLE_RATE_LIMIT_UP: matches ATTO3
      ([5, 25], [0.5, 0.28])  # ANGLE_RATE_LIMIT_DOWN: matches ATTO3
    )


    # Regenerative braking (similar to ATTO3)
    self.REGEN_BRAKE_MAX = 0.38            # Maximum regen strength - matches ATTO3
    self.REGEN_STRENGTH = 0.75             # Regen feel calibration - matches ATTO3

    # Drivetrain specifications (only tire size differs)
    self.WHEEL_RADIUS = 0.341              # 235/50R19 performance tire radius (m)
    self.FINAL_DRIVE_RATIO = 3.36          # Single-speed transmission ratio - matches ATTO3
    self.MOTOR_EFFICIENCY = 0.91           # Motor efficiency - matches ATTO3

  # =============================================================================
  # [Section] Model Init
  # [Brief ] Initialize conservative fallback parameters
  # ---------------------------------------------------------------------------
  # [Function] _init_default_params
  # [Brief ] Initialize conservative fallback parameters
  # [Params] None
  # [Returns] None
  # =============================================================================
  def _init_default_params(self):

    # Acceleration limits - conservative safe values
    self.ACCEL_MAX = 2.0      # Safe max acceleration (m/s²)
    self.ACCEL_MIN = -3.5     # Safe max deceleration (m/s²)

    # Steering parameters - moderate values for safety (CORRECTED)
    self.STEER_MAX = 150      # Safe max steering torque (Nm) - reasonable for EPS
    self.STEER_DELTA_UP = 15  # Conservative torque ramp up rate
    self.STEER_DELTA_DOWN = 25  # Standard torque ramp down rate
    
    # Steering angle limits - conservative defaults for unknown vehicles
    self.ANGLE_LIMITS = AngleSteeringLimits(
      450,  # deg, conservative maximum steering angle
      ([5, 25], [0.3, 0.15]),  # ANGLE_RATE_LIMIT_UP: conservative rates
      ([5, 25], [0.4, 0.2])    # ANGLE_RATE_LIMIT_DOWN: conservative rates
    )


    # Regenerative braking - moderate values
    self.REGEN_BRAKE_MAX = 0.3  # Standard regen strength
    self.REGEN_STRENGTH = 0.7   # Balanced regen feel

    # Drivetrain parameters - generic EV values (CRITICAL: needed for physics calculations)
    self.WHEEL_RADIUS = DEFAULT_WHEEL_RADIUS          # Default tire radius (0.31m)
    self.FINAL_DRIVE_RATIO = DEFAULT_FINAL_DRIVE_RATIO  # Default gear ratio (3.5)
    self.MOTOR_EFFICIENCY = DEFAULT_MOTOR_EFFICIENCY    # Default efficiency (0.9)


# -----------------------------------------------------------------------------
# [Section] Feature Flags
# [Brief ] Capabilities and configuration options by platform
# -----------------------------------------------------------------------------
# =============================================================================
# [Section] Class
# [Brief ] Feature flags for BrownPanda platform capabilities
# ---------------------------------------------------------------------------
# [Class ] BrownPandaFlags
# [Brief ] EV/HYBRID, control method, and sensors
# [Bases ] IntFlag
# =============================================================================
class BrownPandaFlags(IntFlag):
  # Powertrain type identification
  EV = 1                        # Battery Electric Vehicle
  HYBRID = 2                    # Hybrid Electric Vehicle

  # Lateral control method selection
  ANGLE_CONTROL = 16            # Steering angle control (default)
  TORQUE_CONTROL = 32           # Direct steering torque control

  # Advanced driving features
  STOP_AND_GO = 64              # Automatic stop and resume capability

  # Sensor and monitoring capabilities
  FOUR_WHEEL_SENSORS = 2048     # Individual wheel speed sensors available
  # Lateral control enhancements
  DTSA = 4096                   # Enable Dynamic Torque Steering Adjustment (per-model)


# -----------------------------------------------------------------------------
# [Section] Cruise Button Mapping
# [Brief ] Standardized mapping of button signals to functions
# -----------------------------------------------------------------------------
# =============================================================================
# [Section] Class
# [Brief ] Cruise/assist button to DBC signal mapping
# ---------------------------------------------------------------------------
# [Class ] CruiseButtons
# [Brief ] Names used in CAN for standard functions
# [Bases ] object
# =============================================================================
class CruiseButtons:
  RESUME_ACCEL = "SPEED_UP_BTN"      # Resume cruise / Increase speed
  DECEL_SET = "SPEED_DOWN_BTN"         # Set cruise / Decrease speed
  CANCEL = "CRUISE_CANCEL"         # Cancel cruise control
  SET = "CRUISE_SET"               # Set current speed as cruise target
  RESUME = "CRUISE_RESUME"         # Resume previously set cruise speed
  GAP_NEAR = "DISTANCE_NEAR_BTN"         # Decrease following distance
  GAP_FAR = "DISTANCE_FAR_BTN"         # Increase following distance


# Platform configuration classes
# Define documentation and configuration for BrownPanda vehicles

# -----------------------------------------------------------------------------
# [Section] Documentation Config
# -----------------------------------------------------------------------------
@dataclass
# =============================================================================
# [Section] Class
# [Brief ] Documentation metadata for BrownPanda platforms
# ---------------------------------------------------------------------------
# [Class ] BrownPandaCarDocs
# [Brief ] Sets harness/parts details
# [Bases ] CarDocs
# =============================================================================
class BrownPandaCarDocs(CarDocs):
  package: str = "BrownPanda ADAS"

  # =============================================================================
  # [Section] Documentation
  # [Brief ] Populate documentation parts/harness details
  # ---------------------------------------------------------------------------
  # [Function] init_make
  # [Brief ] Populate documentation parts/harness details
  # [Params] CP: CarParams
  # [Returns] None
  # =============================================================================
  def init_make(self, CP: car.CarParams):
    self.car_parts = CarParts.common([CarHarness.bosch_b])  # Standard Bosch harness


# -----------------------------------------------------------------------------
# [Section] Platform Configuration
# -----------------------------------------------------------------------------
@dataclass
# =============================================================================
# [Section] Class
# [Brief ] Platform configuration wrapper (flags + DBC)
# ---------------------------------------------------------------------------
# [Class ] BrownPandaPlatformConfig
# [Brief ] Provides init hook to set common flags
# [Bases ] PlatformConfig
# =============================================================================
class BrownPandaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {'pt': 'brownpanda'})

  # =============================================================================
  # [Section] Platform Config
  # [Brief ] Initialize common flags for all BrownPanda vehicles
  # ---------------------------------------------------------------------------
  # [Function] init
  # [Brief ] Initialize common flags for all BrownPanda vehicles
  # [Params] None
  # [Returns] None
  # =============================================================================
  def init(self):
    # Set standard capabilities for all BrownPanda models
    self.flags |= (BrownPandaFlags.ANGLE_CONTROL |      # Steering angle control
                   BrownPandaFlags.STOP_AND_GO |        # Stop and resume capability
                   BrownPandaFlags.FOUR_WHEEL_SENSORS)   # Individual wheel sensors


# Vehicle platform definitions
# Each supported BrownPanda model with specifications and capabilities

# -----------------------------------------------------------------------------
# [Section] Platform Definitions
# -----------------------------------------------------------------------------
# =============================================================================
# [Section] Class
# [Brief ] Platform enumeration for BrownPanda vehicles
# ---------------------------------------------------------------------------
# [Class ] CAR
# [Brief ] Defines ATTO3, DOLPHIN, DEEPAL_S05
# [Bases ] Platforms
# =============================================================================
class CAR(Platforms):
  # BYD ATTO3 - Mid-size SUV EV (balanced performance)
  BYD_ATTO3 = BrownPandaPlatformConfig(
    [BrownPandaCarDocs("BYD ATTO3")],
    CarSpecs(mass=1680, wheelbase=2.72, steerRatio=16.0,
             centerToFrontRatio=0.4, tireStiffnessFactor=0.8,
             minSteerSpeed=3 * CV.KPH_TO_MS, minEnableSpeed=0),
    flags=(BrownPandaFlags.ANGLE_CONTROL |              # Steering angle control
           BrownPandaFlags.STOP_AND_GO |                # Stop and resume
           BrownPandaFlags.FOUR_WHEEL_SENSORS |         # Wheel speed sensors
           BrownPandaFlags.EV),                         # Electric vehicle
  )

  # DEEPAL S05 - Performance sedan EV (single-motor system)
  DEEPAL_S05 = BrownPandaPlatformConfig(
    [BrownPandaCarDocs("DEEPAL S05")],
    CarSpecs(mass=1750, wheelbase=2.72, steerRatio=15.5,
             centerToFrontRatio=0.42, tireStiffnessFactor=0.85,
             minSteerSpeed=3 * CV.KPH_TO_MS, minEnableSpeed=0),
    flags=(BrownPandaFlags.ANGLE_CONTROL |              # Steering angle control
           BrownPandaFlags.STOP_AND_GO |                # Stop and resume
           BrownPandaFlags.FOUR_WHEEL_SENSORS |         # Wheel speed sensors
           BrownPandaFlags.EV),                         # Electric vehicle
  )

  # BYD DOLPHIN - Compact city EV (comfort-focused)
  BYD_DOLPHIN = BrownPandaPlatformConfig(
    [BrownPandaCarDocs("BYD DOLPHIN")],
    CarSpecs(mass=1405, wheelbase=2.70, steerRatio=14.5,
             centerToFrontRatio=0.40, tireStiffnessFactor=0.70,
             minSteerSpeed=5 * CV.KPH_TO_MS, minEnableSpeed=5 * CV.KPH_TO_MS),
    flags=(BrownPandaFlags.ANGLE_CONTROL |              # Steering angle control
           BrownPandaFlags.STOP_AND_GO |                # Stop and resume
           BrownPandaFlags.FOUR_WHEEL_SENSORS |         # Wheel speed sensors
           BrownPandaFlags.EV),                         # Electric vehicle
  )



# Firmware and DBC configuration
# All BrownPanda models use unified CAN protocol and fingerprinting
# Firmware query configuration (uses CAN fingerprinting instead)
# -----------------------------------------------------------------------------
# [Section] Firmware and DBC Configuration
# -----------------------------------------------------------------------------
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],                  # No firmware queries needed
  non_essential_ecus={},        # No non-essential ECUs
  extra_ecus=[],                # No extra ECUs
)

# Firmware version mappings (unified across all models)
FW_VERSIONS: dict = {
  CAR.BYD_ATTO3: {},            # Uses CAN fingerprinting
  CAR.DEEPAL_S05: {},           # Uses CAN fingerprinting
  CAR.BYD_DOLPHIN: {},          # Uses CAN fingerprinting
}

# DBC file mappings (unified BrownPanda protocol)
DBC = {
  CAR.BYD_ATTO3: {'pt': 'brownpanda'},      # Single bus architecture
  CAR.DEEPAL_S05: {'pt': 'brownpanda'},     # Single bus architecture
  CAR.BYD_DOLPHIN: {'pt': 'brownpanda'},    # Single bus architecture
}

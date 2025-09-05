# =============================================================================
# Module      : BrownPanda Vehicle Interface
# File        : interface.py
# Brief       : Configure vehicle parameters and ADAS feature flags for
#               BrownPanda platforms (BYD ATTO3, BYD DOLPHIN, DEEPAL S05).
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================

# OpenPilot framework imports
from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.brownpanda.carcontroller import CarController
from opendbc.car.brownpanda.carstate import CarState
from opendbc.car.brownpanda.values import CAR, BrownPandaFlags, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase

# Framework type aliases
SteerControlType = car.CarParams.SteerControlType


# =============================================================================
# [Section] Class
# [Brief ] Configure BrownPanda CarParams and expose control hooks
# ---------------------------------------------------------------------------
# [Class ] CarInterface
# [Brief ] Sets safety, tuning, and capability flags
# [Bases ] CarInterfaceBase
# =============================================================================
class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  # =============================================================================
  # [Section] Accel Limits
  # [Brief ] Provide PID accel bounds for longitudinal controller
  # ---------------------------------------------------------------------------
  # [Function] get_pid_accel_limits
  # [Brief ] Provide PID accel bounds for longitudinal controller
  # [Params] CP: CarParams, current_speed: float, cruise_speed: float
  # [Returns] (min_accel: float, max_accel: float)
  # =============================================================================
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  # =============================================================================
  # [Section] Params
  # [Brief ] Populate BrownPanda CarParams with platform-specific settings
  # ---------------------------------------------------------------------------
  # [Function] _get_params
  # [Brief ] Populate BrownPanda CarParams with platform-specific settings
  # [Params] ret: CarParams, candidate: CAR, fingerprint: dict, car_fw: list, experimental_long: bool, docs: bool
  # [Returns] CarParams
  # =============================================================================
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    # Basic vehicle identification
    ret.carName = "BrownPanda"

    # Safety configuration (hardware-level, following Honda Bosch pattern)
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.brownpanda)]

    # Extract platform configuration and flags
    platform_config = candidate
    ret.flags = platform_config.flags if hasattr(platform_config, 'flags') else 0

    # Vehicle physical specs
    if hasattr(platform_config, 'specs'):
      # Use model-specific specs from platform config
      specs = platform_config.specs
      ret.mass = specs.mass                                                    # Vehicle curb weight (kg)
      ret.wheelbase = specs.wheelbase                                          # Distance between axles (m)
      ret.steerRatio = specs.steerRatio                                        # Steering wheel to wheel angle ratio
      ret.centerToFrontRatio = getattr(specs, 'centerToFrontRatio', 0.4)      # CG position (0.5 = center)
      ret.tireStiffnessFactor = getattr(specs, 'tireStiffnessFactor', 0.7)     # Tire grip factor
      ret.rotationalInertia = specs.mass * (specs.wheelbase * 0.5) ** 2       # Vehicle rotational inertia
      ret.minSteerSpeed = getattr(specs, 'minSteerSpeed', 5.0)                # Min speed for steering (km/h)
      ret.minEnableSpeed = getattr(specs, 'minEnableSpeed', 5.0)              # Min speed for ADAS (km/h)
    else:
      # Conservative fallbacks for unknown vehicles
      ret.mass = 1700.                                  # Average EV mass
      ret.wheelbase = 2.72                              # Standard mid-size wheelbase
      ret.steerRatio = 16.0                             # Conservative steering ratio
      ret.centerToFrontRatio = 0.42                     # Slightly front-biased CG
      ret.tireStiffnessFactor = 0.7                     # Conservative tire grip
      ret.rotationalInertia = 1600 * (2.7 * 0.5) ** 2  # Calculated inertia
      ret.minSteerSpeed = 5.0                           # Safe minimum speeds
      ret.minEnableSpeed = 5.0

    ret.centerToFront = ret.wheelbase * ret.centerToFrontRatio  # Distance from CG to front axle
    ret.dashcamOnly = False                                     # Full ADAS capability

    # Lateral control type
    if ret.flags & BrownPandaFlags.TORQUE_CONTROL:
      ret.steerControlType = SteerControlType.torque  # Direct torque control (more precise)
    else:
      ret.steerControlType = SteerControlType.angle   # Angle control (default for current models)

    # Steering actuator delay by powertrain
    if ret.flags & BrownPandaFlags.EV:
      ret.steerActuatorDelay = 0.08  # Fast EV steering response (electric power steering)
    elif ret.flags & BrownPandaFlags.HYBRID:
      ret.steerActuatorDelay = 0.12  # Moderate hybrid response
    else:
      ret.steerActuatorDelay = 0.15  # Slower ICE response

    ret.steerLimitTimer = 0.4        # Standard automotive steering limit timer

    # Longitudinal control (camera-based)
    ret.openpilotLongitudinalControl = True                                 # Use openpilot for speed control
    ret.stoppingControl = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)     # Can stop/start automatically
    ret.startingState = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)       # Automatic starting capability
    ret.radarUnavailable = True                                             # Camera-only system

    # Low-speed behavior by powertrain
    if ret.flags & BrownPandaFlags.EV:
      ret.vEgoStopping = 0.3     # EVs can control precisely at low speeds
      ret.vEgoStarting = 0.3     # Smooth EV acceleration from stop
      ret.stopAccel = -2.5       # Strong regenerative braking capability
    else:
      ret.vEgoStopping = 0.5     # ICE less precise at very low speeds
      ret.vEgoStarting = 0.5     # Standard ICE starting threshold
      ret.stopAccel = -2.0       # Standard friction braking

    ret.stoppingDecelRate = 0.8    # Deceleration rate when approaching stop
    ret.maxSteeringAngleDeg = 1080 # Maximum steering wheel angle (3 turns)

    # Longitudinal PID tuning (per model)
    if candidate == CAR.BYD_ATTO3:
      # BYD ATTO3: Balanced SUV tuning for comfort and efficiency
      ret.longitudinalTuning.kf = 1.0                                     # Feedforward gain
      ret.longitudinalTuning.kpBP = [0., 5., 15., 30.]                    # Speed breakpoints (m/s)
      ret.longitudinalTuning.kpV = [1.6, 1.1, 0.7, 0.4]                  # Proportional gains
      ret.longitudinalTuning.kiBP = [0., 5., 15., 30.]                    # Integral breakpoints
      ret.longitudinalTuning.kiV = [0.20, 0.15, 0.10, 0.06]              # Integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.15                      # Min actuator delay
      ret.longitudinalActuatorDelayUpperBound = 0.25                      # Max actuator delay
    elif candidate == CAR.DEEPAL_S05:
      # DEEPAL S05: Performance sedan tuning for responsive acceleration
      ret.longitudinalTuning.kf = 1.2                                     # Higher feedforward for performance
      ret.longitudinalTuning.kpBP = [0., 8., 20., 35.]                    # Performance-oriented breakpoints
      ret.longitudinalTuning.kpV = [2.0, 1.4, 0.9, 0.6]                  # More aggressive gains
      ret.longitudinalTuning.kiBP = [0., 8., 20., 35.]                    # Matching integral points
      ret.longitudinalTuning.kiV = [0.30, 0.20, 0.14, 0.10]              # Higher integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.10                      # Fast performance response
      ret.longitudinalActuatorDelayUpperBound = 0.18
    elif candidate == CAR.BYD_DOLPHIN:
      # BYD DOLPHIN: City-focused compact EV with smooth, comfortable tuning
      ret.longitudinalTuning.kf = 1.1                                     # Balanced feedforward
      ret.longitudinalTuning.kpBP = [0., 6., 15., 30.]                    # City-optimized breakpoints
      ret.longitudinalTuning.kpV = [1.8, 1.2, 0.8, 0.5]                  # Smooth responsive gains
      ret.longitudinalTuning.kiBP = [0., 6., 15., 30.]                    # Matching integral points
      ret.longitudinalTuning.kiV = [0.24, 0.18, 0.12, 0.08]              # Moderate integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.12                      # Comfortable response
      ret.longitudinalActuatorDelayUpperBound = 0.22
    else:
      # Conservative fallback tuning for unknown vehicles
      ret.longitudinalTuning.kf = 1.0                                     # Safe feedforward
      ret.longitudinalTuning.kpBP = [0., 5., 35.]                         # Simple breakpoints
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]                       # Conservative gains
      ret.longitudinalTuning.kiBP = [0., 35.]                             # Basic integral points
      ret.longitudinalTuning.kiV = [0.18, 0.12]                          # Safe integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.20                      # Conservative delays
      ret.longitudinalActuatorDelayUpperBound = 0.30

    # Model-specific lateral control tuning for optimal steering feel
    if ret.steerControlType == SteerControlType.torque:
      ret.lateralTuning.init('torque')
      ret.lateralTuning.torque.useSteeringAngle = True
      ret.lateralTuning.torque.kf = 1.0
      ret.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0

      # Torque-controller tuning (per model)
      if candidate == CAR.BYD_ATTO3:
        ret.lateralTuning.torque.kp = 2.2
        ret.lateralTuning.torque.ki = 0.18
        ret.lateralTuning.torque.friction = 0.02
        ret.lateralParams.torqueBP = [0., 5., 15., 30., 50.]
        ret.lateralParams.torqueV = [0., 80., 160., 320., 500.]
        ret.lateralParams.dtsaEnable = True
        ret.lateralParams.dtsaTau = 0.5
        ret.lateralParams.dtsaMaxTrim = 0.4
        ret.lateralParams.dtsaScale = 0.02
        ret.lateralParams.dtsaDeadband = 0.05
        ret.lateralParams.dtsaMinSpeed = 5.0
      elif candidate == CAR.BYD_DOLPHIN:
        ret.lateralTuning.torque.kp = 2.4
        ret.lateralTuning.torque.ki = 0.20
        ret.lateralTuning.torque.friction = 0.015
        ret.lateralParams.torqueBP = [0., 5., 12., 22., 35.]
        ret.lateralParams.torqueV = [0., 70., 140., 280., 430.]
        ret.lateralParams.dtsaEnable = True
        ret.lateralParams.dtsaTau = 0.5
        ret.lateralParams.dtsaMaxTrim = 0.4
        ret.lateralParams.dtsaScale = 0.02
        ret.lateralParams.dtsaDeadband = 0.05
        ret.lateralParams.dtsaMinSpeed = 5.0
      elif candidate == CAR.DEEPAL_S05:
        ret.lateralTuning.torque.kp = 2.2  # Matches BYD ATTO3 (same performance class)
        ret.lateralTuning.torque.ki = 0.18  # Matches BYD ATTO3
        ret.lateralTuning.torque.friction = 0.02  # Matches BYD ATTO3
        ret.lateralParams.torqueBP = [0., 5., 15., 30., 50.]  # Matches BYD ATTO3 breakpoints
        ret.lateralParams.torqueV = [0., 80., 160., 320., 500.]  # Matches BYD ATTO3 torque curve
        ret.lateralParams.dtsaEnable = False
        ret.lateralParams.dtsaTau = 0.5
        ret.lateralParams.dtsaMaxTrim = 0.4
        ret.lateralParams.dtsaScale = 0.02
        ret.lateralParams.dtsaDeadband = 0.05
        ret.lateralParams.dtsaMinSpeed = 5.0
      else:
        # Generic EV/ICE defaults
        if ret.flags & BrownPandaFlags.EV:
          ret.lateralTuning.torque.kp = 1.2
          ret.lateralTuning.torque.ki = 0.12
          ret.lateralTuning.torque.friction = 0.08
          ret.lateralParams.torqueBP = [0., 8., 15., 25., 40.]
          ret.lateralParams.torqueV = [0., 60., 120., 250., 400.]
          # Provide DTSA defaults (COMMA2-like) but keep disabled by default
          ret.lateralParams.dtsaEnable = False
          ret.lateralParams.dtsaTau = 0.5
          ret.lateralParams.dtsaMaxTrim = 0.4
          ret.lateralParams.dtsaScale = 0.02
          ret.lateralParams.dtsaDeadband = 0.05
          ret.lateralParams.dtsaMinSpeed = 5.0
        else:
          ret.lateralTuning.torque.kp = 1.0
          ret.lateralTuning.torque.ki = 0.1
          ret.lateralTuning.torque.friction = 0.1
          ret.lateralParams.torqueBP = [0., 5., 10., 20., 30.]
          ret.lateralParams.torqueV = [0., 50., 100., 200., 300.]
          # Provide DTSA defaults (COMMA2-like) but keep disabled by default
          ret.lateralParams.dtsaEnable = False
          ret.lateralParams.dtsaTau = 0.5
          ret.lateralParams.dtsaMaxTrim = 0.4
          ret.lateralParams.dtsaScale = 0.02
          ret.lateralParams.dtsaDeadband = 0.05
          ret.lateralParams.dtsaMinSpeed = 5.0
    else:
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kf = 0.00008
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.]

      # PID (angle) tuning (per model)
      if candidate == CAR.BYD_ATTO3:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.18, 0.30, 0.45]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.009, 0.017, 0.027]
      elif candidate == CAR.BYD_DOLPHIN:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.22, 0.38, 0.55]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.012, 0.022, 0.032]
      elif candidate == CAR.DEEPAL_S05:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.16, 0.28, 0.42]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.008, 0.015, 0.024]
      else:
        if ret.flags & BrownPandaFlags.EV:
          ret.lateralTuning.pid.kpBP = [0., 9., 20.]
          ret.lateralTuning.pid.kpV = [0.2, 0.35, 0.5]
          ret.lateralTuning.pid.kiBP = [0., 9., 20.]
          ret.lateralTuning.pid.kiV = [0.01, 0.02, 0.03]
        else:
          ret.lateralTuning.pid.kpBP = [0., 9., 20.]
          ret.lateralTuning.pid.kpV = [0.15, 0.25, 0.4]
          ret.lateralTuning.pid.kiBP = [0., 9., 20.]
          ret.lateralTuning.pid.kiV = [0.008, 0.015, 0.025]

    # -----------------------------------------------------------------------------
    # [Section] ADAS Feature Configuration
    # [Brief ] Set explicit feature flags and capabilities. Keep minimal; tuning
    #          logic is centralized in values/CarControllerParams.
    # -----------------------------------------------------------------------------
    ret.autoResumeSng = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)
    ret.enableBsm = bool(ret.flags & BrownPandaFlags.FOUR_WHEEL_SENSORS)
    ret.enableApgs = False
    ret.enableDsu = False
    ret.enableGasInterceptor = False
    ret.experimentalLongitudinalAvailable = ret.openpilotLongitudinalControl
    ret.pcmCruise = False

    # -----------------------------------------------------------------------------
    # [Section] Camera-based ADAS
    # [Brief ] Inform openpilot that the platform uses a stock camera system.
    # -----------------------------------------------------------------------------
    ret.hasStockCamera = True

    # -----------------------------------------------------------------------------
    # [Section] Safety Parameter
    # [Brief ] Base safety parameter (0..65535). BrownPanda uses base value.
    # -----------------------------------------------------------------------------
    ret.safetyParam = 0
    assert 0 <= ret.safetyParam <= 65535, f"Invalid safetyParam: {ret.safetyParam}"

    return ret

  # Radarless compatibility helper removed; rely on standard CarParams fields.

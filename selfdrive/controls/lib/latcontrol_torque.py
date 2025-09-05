import math
import numpy as np

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI):
    super().__init__(CP, CP_SP, CI)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.CP = CP
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

    # DTSA (Dynamic Torque Steering Adjustment): optional timing trim for torque mode.
    # Junior note (real world): EPS delay changes with temperature/friction.
    # We measure a small torque mismatch each cycle and nudge the controller's
    # lookahead to keep steering stable. Enable per model via lateralParams.dtsaEnable.
    # Field tuning knobs: dtsaTau / dtsaMaxTrim / dtsaScale / dtsaDeadband / dtsaMinSpeed
    # Enable only if torque mode and the per‑model flag (lateralParams.dtsaEnable) is set
    self.enable_dtsa = (
      hasattr(CP, 'lateralTuning') and getattr(CP.lateralTuning, 'which')() == 'torque' and
      hasattr(CP, 'lateralParams') and getattr(CP.lateralParams, 'dtsaEnable', False)
    )
    # Small, smooth timing trim (seconds); per‑model knobs with safe defaults.
    # - dtsaTau:   bigger -> slower/smoother adaptation
    # - dtsaMaxTrim: cap so we never shift timing too far
    # - dtsaScale:  seconds of timing per 1.0 torque mismatch
    # - dtsaDeadband: ignore tiny mismatches (sensor noise)
    # - dtsaMinSpeed: only learn above this speed (low speed is noisy)
    self._dtsa_tau_s = getattr(getattr(CP, 'lateralParams', object()), 'dtsaTau', 0.5)
    self._dtsa_dt_s = 0.01
    from openpilot.common.filter_simple import FirstOrderFilter
    self._dtsa_filter = FirstOrderFilter(0.0, self._dtsa_tau_s, self._dtsa_dt_s)
    self._dtsa = 0.0
    # Default max trim follows COMMA2 convention (~0.4s). Ports can override per model.
    self._dtsa_max = getattr(getattr(CP, 'lateralParams', object()), 'dtsaMaxTrim', 0.4)
    self._last_req_torque = 0.0
    self._last_applied_torque = None

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # Apply DTSA timing trim from previous cycle
      # Why: controller lookahead is computed using these times. By applying the
      # previously learned trim first, this cycle benefits from the most recent
      # estimate of EPS delay (reduces overshoot/ping‑pong).
      if self.enable_dtsa:
        # Note: In this architecture, timing adjustments would need to be applied
        # to the extension system rather than direct timing variables
        pass

      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(setpoint - measurement)
      ff = gravity_adjusted_lateral_accel
      ff += get_friction(desired_lateral_accel - actual_lateral_accel, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
                                                     desired_curvature, actual_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

      # Update DTSA estimator after computing current torque command
      # Flow (each cycle):
      #   1) apply last _dtsa to timing (above)
      #   2) compute torque (requested)
      #   3) measure mismatch vs EPS (applied)
      #   4) low‑pass + clamp -> _dtsa to use next cycle
      if self.enable_dtsa:
        applied = getattr(CS, 'steeringTorque', None)
        applied_valid = applied is not None
        # requested uses controller sign convention (before final sign flip)
        requested = float(-output_torque)
        self._last_req_torque = requested
        if applied_valid:
          self._last_applied_torque = float(applied)
          # Only learn when inputs are trustworthy: speed high enough, no driver steer,
          # not limited by torque saturation.
          vmin = getattr(getattr(self, 'CP', self.CP), 'lateralParams', None)
          vmin = getattr(vmin, 'dtsaMinSpeed', 5.0)
          gated = (CS.vEgo > vmin and not CS.steeringPressed and not steer_limited_by_safety)
          if gated:
            # If EPS perfectly tracks the controller, requested + applied ≈ 0 (they cancel).
            # Bigger mismatch means more delay/friction right now.
            mismatch = abs(requested + self._last_applied_torque)
            deadband = getattr(getattr(self, 'CP', self.CP).lateralParams, 'dtsaDeadband', 0.05) if hasattr(getattr(self, 'CP', self.CP), 'lateralParams') else 0.05
            if mismatch < deadband:
              mismatch = 0.0
            # Convert mismatch to seconds of timing; per‑model scale with a safe default.
            scale = getattr(getattr(self, 'CP', self.CP).lateralParams, 'dtsaScale', 0.02) if hasattr(getattr(self, 'CP', self.CP), 'lateralParams') else 0.02
            dtsa_raw = mismatch * scale
            # Smooth (LPF) and cap the final timing adjustment.
            self._dtsa = max(0.0, min(self._dtsa_filter.update(dtsa_raw), self._dtsa_max))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log

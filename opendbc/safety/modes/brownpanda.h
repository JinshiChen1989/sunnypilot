#pragma once

#include "opendbc/safety/safety_declarations.h"

static bool brownpanda_longitudinal = false;

static void brownpanda_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 0) {
    // current steering angle from userCommand (0x6E1)
    if (addr == 0x6E1) {
      int angle_meas_new = (GET_BYTES(to_push, 2, 2) & 0xFFFFU);
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // gas and brakes from userCommand (0x6E1)  
    if (addr == 0x6E1) {
      gas_pressed = (GET_BYTE(to_push, 0) > 0U);      // BRAKE_PEDAL
      brake_pressed = (GET_BYTE(to_push, 1) > 0U);    // GAS_PEDAL
    }

    // vehicle speed from carState (0x6E3)
    if (addr == 0x6E3) {
      uint16_t speed_raw = (GET_BYTE(to_push, 3) << 5) | ((GET_BYTE(to_push, 4) & 0xF8U) >> 3);
      vehicle_moving = speed_raw != 0U;
      UPDATE_VEHICLE_SPEED(speed_raw * 0.05625 * KPH_TO_MS);  // Convert to m/s
    }

    // engage logic with buttons from driverState (0x6E2)
    if (addr == 0x6E2) {
      bool set_pressed = ((GET_BYTE(to_push, 6) >> 1U) & 1U) == 1U;   // btnCcSet
      bool res_pressed = ((GET_BYTE(to_push, 6) >> 3U) & 1U) == 1U;   // btnCcResume
      bool cancel = ((GET_BYTE(to_push, 6) >> 2U) & 1U) == 1U;        // btnCcCancel

      if (set_pressed || res_pressed) {
        controls_allowed = true;
      }

      if (cancel) {
        controls_allowed = false;
      }
    }
  }

}

static bool brownpanda_tx_hook(const CANPacket_t *to_send) {
  const AngleSteeringLimits BROWNPANDA_STEERING_LIMITS = {
    .max_angle = 2200,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {6., 4., 3.}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {8., 6., 4.}
    },
  };

  const LongitudinalLimits BROWNPANDA_LONG_LIMITS = {
    .max_accel = 130,       // 2.83 m/s^2
    .min_accel = 50,        // -3.2 m/s^2
    .inactive_accel = 100,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;
  int addr = GET_ADDR(to_send);

  // steer violation checks for latCommand (0x6EC)
  if (addr == 0x6EC) {
    int desired_angle = (GET_BYTES(to_send, 0, 2) & 0xFFFFU);
    bool lka_active = GET_BYTE(to_send, 7) & 1U;  // cmdLaneEnable

    desired_angle = to_signed(desired_angle, 16);

    if (steer_angle_cmd_checks(desired_angle, lka_active, BROWNPANDA_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // acc violation checks for longCommand (0x6EA)
  if ((addr == 0x6EA) && brownpanda_longitudinal) {
    int desired_accel = GET_BYTE(to_send, 2);  // cmdTorque field used for accel
    violation |= longitudinal_accel_checks(desired_accel, BROWNPANDA_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static safety_config brownpanda_init(uint16_t param) {
  UNUSED(param);
#ifdef ALLOW_DEBUG
  const int BROWNPANDA_FLAG_LONGITUDINAL_CONTROL = 1;
  brownpanda_longitudinal = GET_FLAG(param, BROWNPANDA_FLAG_LONGITUDINAL_CONTROL);
#endif

  static const CanMsg BROWNPANDA_TX_MSGS[] = {
    {0x6EC, 0, 8, .check_relay = true}, // latCommand
    {0x6E9, 0, 8, .check_relay = true}, // dispCommand
  };

  static const CanMsg BROWNPANDA_TX_LONG_MSGS[] = {
    {0x6EC, 0, 8, .check_relay = true}, // latCommand
    {0x6E9, 0, 8, .check_relay = true}, // dispCommand  
    {0x6EA, 0, 8, .check_relay = true}, // longCommand
    {0x6EB, 0, 8, .check_relay = true}  // longCommand2
  };

  static RxCheck brownpanda_rx_checks[] = {
    {.msg = {{0x6E1, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 10U}, { 0 }, { 0 }}}, // userCommand
    {.msg = {{0x6E2, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, // driverState  
    {.msg = {{0x6E3, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 10U}, { 0 }, { 0 }}}, // carState
    {.msg = {{0x6E5, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, // wheelSensor
  };

  safety_config ret;
  if (brownpanda_longitudinal) {
    ret = BUILD_SAFETY_CFG(brownpanda_rx_checks, BROWNPANDA_TX_LONG_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(brownpanda_rx_checks, BROWNPANDA_TX_MSGS);
  }

  return ret;
}

const safety_hooks brownpanda_hooks = {
  .init = brownpanda_init,
  .rx = brownpanda_rx_hook,
  .tx = brownpanda_tx_hook,
};
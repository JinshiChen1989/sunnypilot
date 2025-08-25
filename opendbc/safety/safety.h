#pragma once

// Main safety header - coordinates all brand safety implementations
// Based on BYD pattern for hardware-level safety enforcement

#include "safety_declarations.h"

// Include all brand-specific safety implementations
#include "brownpanda_safety.h"

// Safety model enumeration (matches cereal/car.capnp)
typedef enum {
  SAFETY_SILENT = 0,
  SAFETY_HONDA_NIDEC = 1,
  SAFETY_TOYOTA = 2,
  SAFETY_ELM327 = 3,
  SAFETY_GM = 4,
  SAFETY_HONDA_BOSCH_GIRAFFE = 5,
  SAFETY_FORD = 6,
  SAFETY_CADILLAC = 7,
  SAFETY_HYUNDAI = 8,
  SAFETY_CHRYSLER = 9,
  SAFETY_TESLA = 10,
  SAFETY_SUBARU = 11,
  SAFETY_GM_PASSIVE = 12,
  SAFETY_MAZDA = 13,
  SAFETY_NISSAN = 14,
  SAFETY_VOLKSWAGEN = 15,
  SAFETY_TOYOTA_IPAS = 16,
  SAFETY_ALLOUTPUT = 17,
  SAFETY_GM_ASCM = 18,
  SAFETY_NOOUTPUT = 19,
  SAFETY_HONDA_BOSCH = 20,
  SAFETY_VOLKSWAGEN_PQ = 21,
  SAFETY_SUBARU_PREGLOBAL = 22,
  SAFETY_HYUNDAI_LEGACY = 23,
  SAFETY_HYUNDAI_COMMUNITY = 24,
  SAFETY_VOLKSWAGEN_MLB = 25,
  SAFETY_HONGQI = 26,
  SAFETY_BODY = 27,
  SAFETY_HYUNDAI_CANFD = 28,
  SAFETY_VOLKSWAGEN_MQB_EVO = 29,
  SAFETY_CHRYSLER_CUSW = 30,
  SAFETY_PSA = 31,
  SAFETY_BROWNPANDA = 32,
} safety_mode_t;

// Global safety state
extern int controls_allowed;
extern int heartbeat_counter;
extern const safety_hooks *current_hooks;

// Safety mode configuration
int safety_set_mode(safety_mode_t mode, uint16_t param);
void safety_init(void);
int safety_rx_hook(CANPacket_t *to_push);
int safety_tx_hook(CANPacket_t *to_send);
int safety_tx_lin_hook(int lin_num, uint8_t *data, int len);
int safety_fwd_hook(int bus_num, CANPacket_t *to_fwd);

// Safety mode lookup table
static const safety_hooks *safety_mode_hooks[] = {
  [SAFETY_BROWNPANDA] = &brownpanda_hooks,
  // Add other brand hooks here when implemented
};

// Set safety mode
int safety_set_mode(safety_mode_t mode, uint16_t param) {
  if (mode >= sizeof(safety_mode_hooks) / sizeof(safety_mode_hooks[0])) {
    return 0; // Invalid mode
  }
  
  current_hooks = safety_mode_hooks[mode];
  if (current_hooks != NULL && current_hooks->init != NULL) {
    current_hooks->init(param);
    return 1; // Success
  }
  return 0; // Failed
}

// Global safety functions that delegate to current hooks
int safety_rx_hook(CANPacket_t *to_push) {
  if (current_hooks != NULL && current_hooks->rx != NULL) {
    return current_hooks->rx(to_push);
  }
  return 0;
}

int safety_tx_hook(CANPacket_t *to_send) {
  if (current_hooks != NULL && current_hooks->tx != NULL) {
    return current_hooks->tx(to_send);
  }
  return 0;
}

int safety_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  if (current_hooks != NULL && current_hooks->tx_lin != NULL) {
    return current_hooks->tx_lin(lin_num, data, len);
  }
  return 0;
}

int safety_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  if (current_hooks != NULL && current_hooks->fwd != NULL) {
    return current_hooks->fwd(bus_num, to_fwd);
  }
  return -1;
}
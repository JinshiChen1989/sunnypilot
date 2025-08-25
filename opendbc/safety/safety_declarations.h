#pragma once

// Safety function declarations for all car brands
// Based on BYD pattern implementation

#include <stdbool.h>
#include <stdint.h>

// Common safety types and structures
typedef struct {
  uint32_t CNT;
  uint32_t RDLR;
  uint32_t RDHR;
} CANPacket_t;

typedef struct {
  struct {
    int msg;
    int bus;
    int len;
    bool check_checksum;
    uint8_t max_counter;
    uint32_t expected_timestep;
  } msg[3];
} addr_checks;

typedef struct {
  void (*init)(uint16_t param);
  int (*rx)(CANPacket_t *to_push);
  int (*tx)(CANPacket_t *to_send);
  int (*tx_lin)(int lin_num, uint8_t *data, int len);
  int (*fwd)(int bus_num, CANPacket_t *to_fwd);
} safety_hooks;

// Common safety variables  
extern int controls_allowed;
extern uint32_t heartbeat_counter;

// Common safety functions
bool addr_safety_check(CANPacket_t *to_push, const addr_checks *rx_checks, 
                      int rx_checks_len, void *context1, void *context2, void *context3);
int steer_torque_cmd_checks(int desired_torque, int last_torque, uint32_t ts_last);
uint32_t microsecond_timer_get(void);
int to_signed(int value, int bits);

// Utility macros
#define GET_ADDR(msg) ((msg)->CNT >> 21)
#define GET_BUS(msg) (((msg)->CNT >> 4) & 0xF)
#define GET_LEN(arr) (sizeof(arr) / sizeof(arr[0]))
#define GET_BYTE(msg, idx) (((idx) < 4) ? (((msg)->RDLR >> (8 * (idx))) & 0xFF) : (((msg)->RDHR >> (8 * ((idx) - 4))) & 0xFF))
#define UNUSED(x) ((void)(x))

// Brand-specific safety declarations
extern const safety_hooks brownpanda_hooks;
extern const safety_hooks honda_nidec_hooks;
extern const safety_hooks honda_bosch_hooks;
extern const safety_hooks toyota_hooks;
extern const safety_hooks hyundai_hooks;
extern const safety_hooks ford_hooks;
extern const safety_hooks gm_hooks;
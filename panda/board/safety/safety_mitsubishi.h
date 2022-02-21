// global torque limit
const int MITSUBISHI_MAX_TORQUE = 1500;       // max torque cmd allowed ever

// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1000/sec
const int MITSUBISHI_MAX_RATE_UP = 10;        // ramp up slow
const int MITSUBISHI_MAX_RATE_DOWN = 25;      // ramp down fast
const int MITSUBISHI_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int MITSUBISHI_MAX_RT_DELTA = 375;      // max delta torque allowed for real time checks
const uint32_t MITSUBISHI_RT_INTERVAL = 250000;    // 250ms between real time checks

// longitudinal limits
const int MITSUBISHI_MAX_ACCEL = 2000;        // 2.0 m/s2
const int MITSUBISHI_MIN_ACCEL = -3500;       // -3.5 m/s2

const int MITSUBISHI_STANDSTILL_THRSLD = 100;  // 1kph

// Roughly calculated using the offsets in openpilot +5%:
// In openpilot: ((gas1_norm + gas2_norm)/2) > 15
// gas_norm1 = ((gain_dbc*gas1) + offset1_dbc)
// gas_norm2 = ((gain_dbc*gas2) + offset2_dbc)
// In this safety: ((gas1 + gas2)/2) > THRESHOLD
const int MITSUBISHI_GAS_INTERCEPTOR_THRSLD = 845;
#define MITSUBISHI_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks

const CanMsg MITSUBISHI_TX_MSGS[] = {{0x399, 0, 7}};  // interceptor

AddrCheckStruct mitsubishi_addr_checks[] = {
  {.msg = {{ 0xaa, 0, 8, .check_checksum = false, .expected_timestep = 12000U}, { 0 }, { 0 }}},
  {.msg = {{0x260, 0, 8, .check_checksum = true, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x1D2, 0, 8, .check_checksum = true, .expected_timestep = 30000U}, { 0 }, { 0 }}},
  {.msg = {{0x224, 0, 8, .check_checksum = false, .expected_timestep = 25000U},
           {0x226, 0, 8, .check_checksum = false, .expected_timestep = 25000U}, { 0 }}},
};
#define MITSUBISHI_ADDR_CHECKS_LEN (sizeof(mitsubishi_addr_checks) / sizeof(mitsubishi_addr_checks[0]))
addr_checks mitsubishi_rx_checks = {mitsubishi_addr_checks, MITSUBISHI_ADDR_CHECKS_LEN};

// global actuation limit states
int mitsubishi_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

static uint8_t mitsubishi_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static uint8_t mitsubishi_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static int mitsubishi_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {


  return TRUE;
}

static int mitsubishi_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, MITSUBISHI_TX_MSGS, sizeof(MITSUBISHI_TX_MSGS)/sizeof(MITSUBISHI_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // Check if msg is sent on BUS 0
  if (bus == 0) {
      // no torque if controls is not allowed
      if (!controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !controls_allowed) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }
    }
  }

  return tx;
}

static const addr_checks* mitsubishi_init(int16_t param) {
  controls_allowed = 0;
  relay_malfunction_reset();
  gas_interceptor_detected = 0;
  mitsubishi_dbc_eps_torque_factor = param;
  return &mitsubishi_rx_checks;
}

static int mitsubishi_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  if (!relay_malfunction) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      int addr = GET_ADDR(to_fwd);
      // block stock lkas messages and stock acc messages (if OP is doing ACC)
      // in TSS2, 0x191 is LTA which we need to block to avoid controls collision
      int is_lkas_msg = ((addr == 0x2E4) || (addr == 0x412) || (addr == 0x191));
      // in TSS2 the camera does ACC as well, so filter 0x343
      int is_acc_msg = (addr == 0x343);
      int block_msg = is_lkas_msg || is_acc_msg;
      if (!block_msg) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

const safety_hooks mitsubishi_hooks = {
  .init = mitsubishi_init,
  .rx = mitsubishi_rx_hook,
  .tx = mitsubishi_tx_hook,
  .fwd = mitsubishi_fwd_hook,
};

// WYVERN-E 4.0 -- SPI microSD flight logger, drained from the inter-core FIFO on core 1.
// ==========================================================================================
// Core 0 pushes one LogFrame per 500 Hz control tick into the RP2350 inter-core FIFO
// (pico/multicore.h, 32-bit words -- a LogFrame is split across N words and reassembled here).
// Core 1 drains the FIFO into a small RAM ring buffer, then flushes that ring to the SD card in
// bursts. This means an SD write that takes a few ms (common with SPI microSD) never blocks core 0
// -- it just makes the FIFO momentarily fuller, which core 1 catches up on, by design
// (01_FlightComputer_Spec.md section 1: "SD writes and Wi-Fi can stall here for milliseconds
// without ever jittering the control loop on core 0").
//
// SCHEMA v2 (expanded post-flight-readiness pass): every quantity the control loop already
// computes each tick is now logged -- not just the final actuator command. This is the difference
// between a post-flight CSV that can only say "here's what the gimbal did" and one that can say
// "here's WHY it did that" (setpoint, error, individual P/I/D contributions, IMU vote health, and
// loop-timing jitter), which is what root-causing an anomaly from data (rather than a repeat test
// flight) requires. Nothing here changes the control loop's own math -- these are read-only
// telemetry taps on state the loop already holds.
//
// File: WYV4_FLIGHT.csv -- header + one CSV row per control tick, columns documented below.
#pragma once
#include <SPI.h>
#include <SD.h>
#include "pico/multicore.h"

// One flight log record. Kept POD/flat so it can be pushed through multicore_fifo word-by-word.
// NOTE: multicore_fifo_push_blocking sends one 32-bit word at a time; we pack/unpack via a union.
// Size MUST stay a whole number of 32-bit words (static_assert below enforces this at compile time).
struct LogFrame {
  uint32_t t_ms;              // millis() at this tick
  float    t_flight_s;        // seconds since launch detect; NAN before launch (no t=0 yet)
  uint32_t loop_dt_us;        // actual elapsed micros() since the previous tick (500 Hz jitter diag)

  uint8_t state;              // FlightState enum value, see wyvern4_tvc.ino
  uint8_t imu_fault;          // TriImu::vote_fault()
  uint8_t rbf_pulled;         // 1 = remove-before-flight pin pulled (armed-eligible), 0 = inserted
  uint8_t batt_flags;         // bit0 = low_battery, bit1 = critical (BatteryMonitor thresholds)

  float qb_w, qb_x, qb_y, qb_z;      // voted body quaternion (post 2-of-3 vote)
  float qg_w, qg_x, qg_y, qg_z;      // gimbal quaternion (dedicated I2C1 bus)
  float vote_disagree_rad;           // TriImu::vote_disagreement_rad(); -1 = only one IMU reporting

  float body_pitch_rad, body_yaw_rad;    // small-angle Euler readout of qb (control loop's own use)
  float defl_pitch_rad, defl_yaw_rad;    // q_body^-1 (x) q_gimbal -- actual nozzle deflection

  float setp_pitch_rad;               // commanded pitch setpoint this tick (0 outside BOOST/maneuver)
  float err_pitch_rad, err_yaw_rad;   // PID input error each axis (setpoint - measurement)

  float pid_p_pitch, pid_i_pitch, pid_d_pitch;   // pitch PID term breakdown, kp*err / ki*integral / kd*deriv
  float pid_p_yaw,   pid_i_yaw,   pid_d_yaw;     // yaw PID term breakdown

  float cmd_pitch_rad, cmd_yaw_rad;   // PID output commanded to servos (post slew-limit/clamp)

  float baro_alt_m, baro_temp_c;
  float accel_mag_g;
  float batt_v;                       // 2S LiPo pack voltage (BatteryMonitor, cross-core snapshot -- see
                                       // g_batt_v in wyvern4_tvc.ino; FIXED from schema v1, which left
                                       // this NAN because core 0 never had a battery reading to log)
  uint32_t dropped_frames_cum;        // cumulative g_dropped_log_frames at this tick (FIFO-full drops)
};
static_assert(sizeof(LogFrame) % 4 == 0, "LogFrame must be a whole number of 32-bit words for the FIFO");

constexpr size_t LOGFRAME_WORDS = sizeof(LogFrame) / 4;

// ---- core 0 side: push one frame into the FIFO. Non-blocking-ish: if the FIFO is full (core 1
// fell badly behind, e.g. mid SD-card stall), this DROPS the frame rather than blocking core 0 --
// a dropped log sample is acceptable, a jittered control loop is not. ----
inline bool log_push(const LogFrame& f) {
  // pico/multicore.h: rp2040.fifo has 8 words of hardware buffering; check space before writing
  // the whole frame so we never block, and never leave a half-written frame in the FIFO.
  if (rp2040.fifo.available() < (int)LOGFRAME_WORDS) {
    return false;  // dropped -- caller may increment a dropped-frame counter for telemetry
  }
  const uint32_t* words = reinterpret_cast<const uint32_t*>(&f);
  for (size_t i = 0; i < LOGFRAME_WORDS; i++) rp2040.fifo.push(words[i]);
  return true;
}

// ---- core 1 side: drain whatever whole frames are available right now. ----
inline size_t log_drain(LogFrame* out, size_t max_frames) {
  size_t n = 0;
  while (n < max_frames && rp2040.fifo.available() >= (int)LOGFRAME_WORDS) {
    uint32_t* words = reinterpret_cast<uint32_t*>(&out[n]);
    for (size_t i = 0; i < LOGFRAME_WORDS; i++) rp2040.fifo.pop(&words[i]);
    n++;
  }
  return n;
}

class SdLogger {
public:
  static constexpr uint8_t PIN_SCK = 2, PIN_MOSI = 3, PIN_MISO = 4, PIN_CS = 5;
  static constexpr const char* FILENAME = "WYV4_FLIGHT.csv";
  static constexpr size_t BURST_FRAMES = 32;     // frames pulled off the FIFO per service() call

  bool begin() {
    SPI.setSCK(PIN_SCK); SPI.setTX(PIN_MOSI); SPI.setRX(PIN_MISO);
    if (!SD.begin(PIN_CS)) { ok_ = false; return false; }
    file_ = SD.open(FILENAME, FILE_WRITE);
    if (!file_) { ok_ = false; return false; }
    file_.println(
      "t_ms,t_flight_s,loop_dt_us,state,imu_fault,rbf_pulled,batt_low,batt_critical,"
      "qb_w,qb_x,qb_y,qb_z,qg_w,qg_x,qg_y,qg_z,vote_disagree_deg,"
      "body_pitch_deg,body_yaw_deg,defl_pitch_deg,defl_yaw_deg,"
      "setp_pitch_deg,err_pitch_deg,err_yaw_deg,"
      "pid_p_pitch_deg,pid_i_pitch_deg,pid_d_pitch_deg,pid_p_yaw_deg,pid_i_yaw_deg,pid_d_yaw_deg,"
      "cmd_pitch_deg,cmd_yaw_deg,baro_alt_m,baro_temp_c,accel_g,batt_v,dropped_frames_cum");
    file_.flush();
    ok_ = true;
    return true;
  }

  // Call from loop1() every iteration. Drains up to BURST_FRAMES from the FIFO and appends them.
  // Flushes periodically (not every row -- flushing every row would make every write a multi-ms
  // SD stall; flushing every ~FLUSH_EVERY rows trades a small worst-case data-loss window on power
  // loss for dramatically less SPI bus time per row).
  void service() {
    if (!ok_) return;
    LogFrame buf[BURST_FRAMES];
    size_t n = log_drain(buf, BURST_FRAMES);
    for (size_t i = 0; i < n; i++) {
      const LogFrame& f = buf[i];
      bool batt_low = f.batt_flags & 0x01;
      bool batt_crit = f.batt_flags & 0x02;
      // vote_disagree_rad is -1 as a "only one IMU reporting" sentinel; keep that sentinel in
      // degrees too (-57.3) rather than silently degrees()-converting it into a bogus small angle.
      float vote_disagree_deg = (f.vote_disagree_rad < 0.0f) ? f.vote_disagree_rad
                                                              : degrees(f.vote_disagree_rad);
      file_.printf(
        "%lu,%.3f,%lu,%u,%u,%u,%u,%u,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%lu\n",
        (unsigned long)f.t_ms, f.t_flight_s, (unsigned long)f.loop_dt_us,
        f.state, f.imu_fault, f.rbf_pulled, (unsigned)batt_low, (unsigned)batt_crit,
        f.qb_w, f.qb_x, f.qb_y, f.qb_z, f.qg_w, f.qg_x, f.qg_y, f.qg_z, vote_disagree_deg,
        degrees(f.body_pitch_rad), degrees(f.body_yaw_rad),
        degrees(f.defl_pitch_rad), degrees(f.defl_yaw_rad),
        degrees(f.setp_pitch_rad), degrees(f.err_pitch_rad), degrees(f.err_yaw_rad),
        degrees(f.pid_p_pitch), degrees(f.pid_i_pitch), degrees(f.pid_d_pitch),
        degrees(f.pid_p_yaw), degrees(f.pid_i_yaw), degrees(f.pid_d_yaw),
        degrees(f.cmd_pitch_rad), degrees(f.cmd_yaw_rad),
        f.baro_alt_m, f.baro_temp_c, f.accel_mag_g, f.batt_v,
        (unsigned long)f.dropped_frames_cum);
      rows_since_flush_++;
    }
    if (rows_since_flush_ >= FLUSH_EVERY) { file_.flush(); rows_since_flush_ = 0; }
  }

  // Force a flush + close, e.g. on LANDED state entry so the card is safe to remove.
  void finalize() { if (ok_) { file_.flush(); file_.close(); ok_ = false; } }
  bool ok() const { return ok_; }

private:
  static constexpr int FLUSH_EVERY = 50;   // ~0.1 s of data at 500 Hz before a forced flush
  File file_;
  bool ok_ = false;
  int rows_since_flush_ = 0;
};

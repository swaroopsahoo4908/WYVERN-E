// WYVERN-E 4.0 — tri-IMU (3x BNO085) Game Rotation Vector driver with 2-of-3 voting fault detection.
// =====================================================================================================
// Three identical Adafruit BNO085 (BNO080-family) modules, all run in SH2_GAME_ROTATION_VECTOR mode
// (gyro+accel sensor fusion, NO magnetometer -- a rocket's motor/avionics fields make raw magnetic
// heading useless, and GRV doesn't need it; see 01_FlightComputer_Spec.md section 2):
//   - gimbal   : I2C1 (Wire1), dedicated bus, address 0x4A -- "true nozzle attitude inside the gimbal"
//   - body     : I2C0 (Wire) via PCA9548A mux channel 0, address 0x4A -- primary vehicle attitude
//   - recovery : I2C0 (Wire) via PCA9548A mux channel 1, address 0x4A -- redundant vehicle attitude
//
// Why voting only covers body vs. recovery, not gimbal: the gimbal IMU measures something physically
// different (nozzle attitude, not body attitude) -- you cannot vote it against the body/recovery pair
// for "is the vehicle attitude reading correct", but body and recovery DO measure the same physical
// quantity (vehicle body attitude) from two independent sensors on two independent mux channels, so
// they're exactly the redundant pair the design calls for ("3rd BNO085 ... for 2-of-3 voting/fault
// detection", 01_FlightComputer_Spec.md section 1). With only 2 IMUs measuring the same quantity,
// "2-of-3" reduces to: agree -> trust both; disagree -> flag a fault and fall back to whichever one
// last looked sane, while keeping the gimbal IMU (the deflection's other operand) on its own
// dedicated bus so it's never affected by a body/recovery mux fault.
#pragma once
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "i2c_mux.h"

struct Quat {
  float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

inline Quat quat_mul(const Quat& a, const Quat& b) {
  return Quat{
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}
inline Quat quat_conj(const Quat& q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

// Angular separation between two unit quaternions, in radians. Used both for the deflection
// readout's plausibility check and for the body/recovery voting disagreement metric.
inline float quat_angle_between(const Quat& a, const Quat& b) {
  Quat d = quat_mul(quat_conj(a), b);
  float w = d.w; if (w > 1.0f) w = 1.0f; else if (w < -1.0f) w = -1.0f;
  return 2.0f * acosf(w);
}

class GrvImu {
public:
  // bus == nullptr means "this IMU lives behind the mux on I2C0"; pass the mux + channel.
  // bus != nullptr (Wire1) means "dedicated bus, no mux" (the gimbal IMU).
  GrvImu(TwoWire* bus, I2CMux* mux, uint8_t mux_channel, uint8_t addr, const char* name)
    : bus_(bus), mux_(mux), ch_(mux_channel), addr_(addr), name_(name) {}

  // with_accel: also enable the SH2_ACCELEROMETER report on this unit. Only the body IMU needs
  // this (launch-detect + landing-detect both key off |a|, see launch_status.h); the gimbal and
  // recovery IMUs are left at GRV-only to keep their I2C/report bandwidth minimal.
  bool begin(bool with_accel = false) {
    select_();
    ok_init_ = bno_.begin_I2C(addr_, bus_ ? bus_ : &Wire);
    if (ok_init_) {
      // 5000 us report interval ~= 200 Hz sensor-side; core-0 loop reads whatever's newest at
      // its own 500 Hz cadence (event-driven getSensorEvent(), not a blocking wait).
      ok_init_ = bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    }
    if (ok_init_ && with_accel) {
      accel_enabled_ = bno_.enableReport(SH2_ACCELEROMETER, 5000);
    }
    healthy_ = ok_init_;
    return ok_init_;
  }

  // Poll for new reports; updates internal quaternion and/or accel state, whichever event the BNO085
  // delivers this call (the Adafruit driver surfaces one sensor event per getSensorEvent() call, so
  // GRV and accel reports interleave across successive polls -- both are still read at the core-0
  // control rate, just not necessarily on the exact same tick). Returns true if a fresh GRV sample
  // was consumed this call (caller can treat false as "no new attitude data this tick, keep using
  // last_quat()"); accel freshness is tracked separately via accel_is_stale().
  bool poll(unsigned long now_ms) {
    select_();
    sh2_SensorValue_t v;
    if (!bno_.getSensorEvent(&v)) { consecutive_misses_++; return false; }
    if (v.sensorId == SH2_GAME_ROTATION_VECTOR) {
      auto& r = v.un.gameRotationVector;
      q_ = Quat{r.real, r.i, r.j, r.k};
      last_update_ms_ = now_ms;
      consecutive_misses_ = 0;
      return true;
    }
    if (v.sensorId == SH2_ACCELEROMETER) {
      auto& a = v.un.accelerometer;
      // BNO08x accelerometer report is in m/s^2; convert to g for the launch-detect threshold
      // (THRESHOLD_G = 3.0) and landing-quiescence check (~1g at rest) used in launch_status.h.
      accel_mag_g_ = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z) / 9.80665f;
      last_accel_ms_ = now_ms;
    }
    consecutive_misses_++;   // this event wasn't a GRV sample; caller's "fresh GRV" answer is false
    return false;
  }

  const Quat& last_quat() const { return q_; }
  unsigned long last_update_ms() const { return last_update_ms_; }
  float accel_mag_g() const { return accel_mag_g_; }
  bool accel_enabled() const { return accel_enabled_; }
  bool accel_is_stale(unsigned long now_ms, unsigned long stale_ms = 100) const {
    return !accel_enabled_ || last_accel_ms_ == 0 || (now_ms - last_accel_ms_) > stale_ms;
  }
  bool init_ok() const { return ok_init_; }
  // Stale = no fresh report for too long (sensor stopped responding, bus fault, etc).
  bool is_stale(unsigned long now_ms, unsigned long stale_ms = 50) const {
    return !ok_init_ || (now_ms - last_update_ms_) > stale_ms;
  }
  const char* name() const { return name_; }

private:
  void select_() { if (mux_) mux_->select(ch_); }

  TwoWire* bus_;
  I2CMux* mux_;
  uint8_t ch_;
  uint8_t addr_;
  const char* name_;
  Adafruit_BNO08x bno_;
  Quat q_;
  unsigned long last_update_ms_ = 0;
  unsigned long consecutive_misses_ = 0;
  bool ok_init_ = false;
  bool healthy_ = false;
  bool accel_enabled_ = false;
  float accel_mag_g_ = 1.0f;          // sane resting default (1g) until the first real sample arrives
  unsigned long last_accel_ms_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Tri-IMU manager: owns gimbal/body/recovery GrvImu instances, runs the 2-of-3-style vote between
// body and recovery, and exposes the single "voted body attitude" the control loop should use.
// ---------------------------------------------------------------------------------------------
class TriImu {
public:
  static constexpr float VOTE_DISAGREE_THRESHOLD_RAD = 0.0349f;  // ~2 deg -- beyond normal sensor
                                                                  // noise/lag between two GRV units
                                                                  // sampling the same rigid body.
  TriImu(TwoWire& wire0, TwoWire& wire1, I2CMux& mux, uint8_t bno_addr)
    : gimbal_(&wire1, nullptr, 0, bno_addr, "gimbal"),
      body_(&wire0, &mux, I2CMux::CH_BODY, bno_addr, "body"),
      recovery_(&wire0, &mux, I2CMux::CH_RECOVERY, bno_addr, "recovery") {}

  // Returns a bitmask of which of the 3 IMUs initialized OK: bit0=gimbal, bit1=body, bit2=recovery.
  // Flight-critical minimum is gimbal + at least one of {body, recovery} -- the control loop can run
  // on a single good body-attitude source, just without the voting cross-check.
  uint8_t begin() {
    uint8_t mask = 0;
    if (gimbal_.begin())          mask |= 0x01;
    if (body_.begin(true))        mask |= 0x02;   // with_accel=true: launch/landing detect source
    if (recovery_.begin())        mask |= 0x04;
    return mask;
  }

  // |a| in g from the body IMU's accelerometer report -- the launch-detect and landing-quiescence
  // source (see launch_status.h and wyvern4_tvc.ino's DESCENT-state landing check). Falls back to
  // a sane resting default (1g) if the accel channel never initialized or has gone stale, rather
  // than returning NAN into a threshold comparison.
  float body_accel_mag_g(unsigned long now_ms) const {
    if (body_.accel_is_stale(now_ms)) return 1.0f;
    return body_.accel_mag_g();
  }

  // Poll all three and run the body/recovery vote. Call once per 500 Hz control tick from core 0.
  void update(unsigned long now_ms) {
    gimbal_.poll(now_ms);
    body_.poll(now_ms);
    recovery_.poll(now_ms);

    bool body_ok = !body_.is_stale(now_ms);
    bool rec_ok = !recovery_.is_stale(now_ms);

    if (body_ok && rec_ok) {
      float disagreement = quat_angle_between(body_.last_quat(), recovery_.last_quat());
      voted_disagree_rad_ = disagreement;
      if (disagreement <= VOTE_DISAGREE_THRESHOLD_RAD) {
        // Both healthy and agree: use body (primary) as the voted attitude.
        voted_q_ = body_.last_quat();
        fault_ = false;
      } else {
        // Both report but disagree beyond threshold: cannot tell which is wrong from only 2
        // independent sources (true 2-of-3 needs a 3rd attitude vote, which we don't have for
        // body-frame attitude -- the gimbal IMU measures a different quantity). Flag fault, keep
        // using body's reading (it's the one wired as primary / used historically) but surface the
        // disagreement so ground software and the in-flight log can catch it.
        voted_q_ = body_.last_quat();
        fault_ = true;
      }
    } else if (body_ok) {
      voted_q_ = body_.last_quat(); fault_ = true; voted_disagree_rad_ = -1.0f; // recovery is down
    } else if (rec_ok) {
      voted_q_ = recovery_.last_quat(); fault_ = true; voted_disagree_rad_ = -1.0f; // body is down
    } else {
      fault_ = true; voted_disagree_rad_ = -1.0f; // neither responding -- voted_q_ holds last-known
    }
  }

  const Quat& voted_body_quat() const { return voted_q_; }
  const Quat& gimbal_quat() const { return gimbal_.last_quat(); }
  bool vote_fault() const { return fault_; }
  float vote_disagreement_rad() const { return voted_disagree_rad_; }
  bool gimbal_stale(unsigned long now_ms) const { return gimbal_.is_stale(now_ms); }

  GrvImu& gimbal() { return gimbal_; }
  GrvImu& body() { return body_; }
  GrvImu& recovery() { return recovery_; }

private:
  GrvImu gimbal_, body_, recovery_;
  Quat voted_q_;
  bool fault_ = false;
  float voted_disagree_rad_ = -1.0f;
};

// Nozzle deflection relative to body, per 02_tvc_control_loop.mermaid:
//   deflection = q_body^-1 (x) q_gimbal -> pitch = 2*y component, yaw = 2*z component (small-angle
//   quaternion-to-Euler readout, matches t2_imu_grv_deflection.ino's existing convention exactly).
struct Deflection { float pitch_rad, yaw_rad; };
inline Deflection compute_deflection(const Quat& q_body, const Quat& q_gimbal) {
  Quat d = quat_mul(quat_conj(q_body), q_gimbal);
  return Deflection{ 2.0f * d.y, 2.0f * d.z };
}

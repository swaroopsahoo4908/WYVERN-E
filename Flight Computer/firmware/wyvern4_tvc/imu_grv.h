// WYVERN-E — dual-IMU attitude driver (onboard BNO055 + external BNO085) with 2-of-2 voting.
// =====================================================================================================
// RECONCILED 2026-08-11 against the ACTUAL fabricated PCB1 netlist/BOM/schematic
// (PCB/Netlist_PCB1_2026-08-11.tel, PCB/BOM_Board1_PCB1_2026-08-11.xlsx,
// PCB/SCH_Schematic1_2026-08-11.pdf) -- traced pin-by-pin, not assumed from prior documentation.
// Two prior versions of this file were both wrong about the real hardware:
//   1. The original tri-IMU (gimbal/body/recovery) design assumed a PCA9548A mux and 3x BNO085 --
//      no mux chip exists on the real board at all.
//   2. The first "fix" (2026-08-11a) assumed 2x BNO085 (one onboard via mux ch0, one external via
//      a dedicated I2C1 bus) -- WRONG. The real onboard IMU (U2 in the BOM) is a Bosch BNO055, a
//      completely different chip with a different register map, different I2C address, and no
//      SH-2 report system, so the Adafruit_BNO08x driver cannot talk to it at all. There is also
//      no PCA9548A mux and no second I2C bus: the schematic shows U1 (RP2350B) GPIO0/GPIO1 as ONE
//      shared I2C bus carrying every onboard sensor (BNO055, BME680, INA226, LIS3MDL) AND the
//      external STEMMA-QT connector (CN2), differentiated purely by I2C address.
//
// Current, netlist-verified architecture:
//   - body     : onboard Bosch BNO055 (U2), shared bus (Wire, GP0 SDA / GP1 SCL). Address CONFIRMED
//                0x28: pin 17 (COM3, the address-select pin in I2C mode) traces to the board's GND
//                net in Netlist_PCB1_2026-08-11.tel, and Bosch's COM3-low convention is 0x28. Also
//                confirmed from the same trace: PS1 (pin5, floating -> internal pulldown = 0) and
//                PS0 (pin6, tied GND = 0) select I2C mode (PS1:PS0 = 0:0); pins 26/27 (XIN32/XOUT32)
//                are unpopulated, confirming no external 32kHz crystal, matching Bno055Body::begin()'s
//                setExtCrystalUse(false) call below. One item worth a bench look: pin9 (CAP, the
//                internal-regulator bypass pin per Bosch's reference design) shows no net membership
//                in the extracted netlist at all -- verify against the board whether a cap is actually
//                placed there; if Bosch's reference design requires one and none is populated, the
//                chip's internal regulator may not be clean.
//   - external : off-board Adafruit BNO085 breakout plugged into the STEMMA-QT port (CN2), SAME
//                shared bus, address 0x4A (Adafruit board default -- CN2 is a plain 4-pin STEMMA-QT
//                passthrough, GND/3V3/SDA/SCL, confirmed in netlist order). Mounted at the
//                TVC-bay/electronics boundary near the bulkhead joint (Recovery.md #1), NOT on the
//                gimbal.
//   Both run accel+gyro fusion with no magnetometer reference (rocket motor/avionics fields make
//   raw magnetic heading useless): BNO085 in SH2_GAME_ROTATION_VECTOR, BNO055 in OPERATION_MODE_IMUPLUS
//   (Bosch's equivalent -- 6-axis fusion, mag excluded from the estimate).
//
// CONSEQUENCE FOR DEFLECTION SENSING: unchanged from the prior pass -- there is no gimbal-mounted
// IMU on this vehicle, so nozzle deflection (q_body^-1 (x) q_gimbal) is not computable in flight.
// The control loop was already body-attitude-only (wyvern4_tvc.ino's BOOST case uses
// body_pitch_rad/body_yaw_rad, never a deflection value), so this does not touch control math.
// compute_deflection() remains a NaN stub for LogFrame/CSV schema compatibility.
//
// Address confidence, resolved by tracing Netlist_PCB1_2026-08-11.tel pin-by-pin against the
// labeled pinouts in SCH_Schematic1_1-P1_2026-08-11.svg (not assumed from datasheet defaults):
// BNO055 0x28 -- CONFIRMED (COM3 -> GND). BME680 0x76 -- CONFIRMED (baro.h; SDO -> GND, CSB -> 3V3
// selects I2C). LIS3MDL 0x1C -- CONFIRMED (SDO/SA1 -> GND), though the magnetometer itself is still
// unused by firmware. INA226 -- NOT resolved this way; see battery.h's file header for a real wiring
// concern found on that part's address-select pin, not just an unread datasheet default.
#pragma once
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

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
// readout's plausibility check and for the body/external voting disagreement metric.
inline float quat_angle_between(const Quat& a, const Quat& b) {
  Quat d = quat_mul(quat_conj(a), b);
  float w = d.w; if (w > 1.0f) w = 1.0f; else if (w < -1.0f) w = -1.0f;
  return 2.0f * acosf(w);
}

// ---------------------------------------------------------------------------------------------
// External unit: off-board Adafruit BNO085 breakout via STEMMA-QT, SH2 report protocol.
// ---------------------------------------------------------------------------------------------
class Bno085External {
public:
  Bno085External(TwoWire* bus, uint8_t addr, const char* name)
    : bus_(bus), addr_(addr), name_(name) {}

  bool begin() {
    ok_init_ = bno_.begin_I2C(addr_, bus_);
    if (ok_init_) {
      ok_init_ = bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    }
    return ok_init_;
  }

  bool poll(unsigned long now_ms) {
    sh2_SensorValue_t v;
    if (!bno_.getSensorEvent(&v)) return false;
    if (v.sensorId == SH2_GAME_ROTATION_VECTOR) {
      auto& r = v.un.gameRotationVector;
      q_ = Quat{r.real, r.i, r.j, r.k};
      last_update_ms_ = now_ms;
      return true;
    }
    return false;
  }

  const Quat& last_quat() const { return q_; }
  bool init_ok() const { return ok_init_; }
  bool is_stale(unsigned long now_ms, unsigned long stale_ms = 50) const {
    return !ok_init_ || (now_ms - last_update_ms_) > stale_ms;
  }
  const char* name() const { return name_; }

private:
  TwoWire* bus_;
  uint8_t addr_;
  const char* name_;
  Adafruit_BNO08x bno_;
  Quat q_;
  bool ok_init_ = false;
  unsigned long last_update_ms_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Onboard unit: Bosch BNO055, register-based fusion sensor (NOT SH2 protocol -- a completely
// different chip/driver than the BNO085 above). OPERATION_MODE_IMUPLUS = 6-axis accel+gyro
// fusion, magnetometer excluded from the orientation estimate, the closest BNO055 equivalent to
// the BNO085's Game Rotation Vector mode.
// ---------------------------------------------------------------------------------------------
class Bno055Body {
public:
  Bno055Body(TwoWire* bus, uint8_t addr, const char* name)
    : bus_(bus), addr_(addr), name_(name), bno_(-1, addr, bus) {}

  bool begin() {
    ok_init_ = bno_.begin(OPERATION_MODE_IMUPLUS);
    if (ok_init_) {
      bno_.setExtCrystalUse(false);   // no external 32.768kHz crystal wired to the BNO055 on this
                                       // board -- CONFIRMED via netlist (XIN32/XOUT32, U2 pins 26/27,
                                       // have no net membership at all) -- use its internal oscillator.
    }
    return ok_init_;
  }

  // Poll once per control tick. BNO055 has no event-driven read in this library; getQuat()/
  // getEvent() each issue a fresh I2C transaction and return the latest fusion output.
  bool poll(unsigned long now_ms) {
    if (!ok_init_) return false;
    imu::Quaternion q = bno_.getQuat();
    // A literal zero quaternion is what the library returns on an I2C read failure; treat it as
    // "no fresh sample" rather than a valid attitude.
    if (q.w() == 0 && q.x() == 0 && q.y() == 0 && q.z() == 0) return false;
    q_ = Quat{(float)q.w(), (float)q.x(), (float)q.y(), (float)q.z()};
    last_update_ms_ = now_ms;

    sensors_event_t accel_event;
    bno_.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accel_mag_g_ = sqrtf(accel_event.acceleration.x*accel_event.acceleration.x +
                          accel_event.acceleration.y*accel_event.acceleration.y +
                          accel_event.acceleration.z*accel_event.acceleration.z) / 9.80665f;
    last_accel_ms_ = now_ms;
    return true;
  }

  const Quat& last_quat() const { return q_; }
  float accel_mag_g() const { return accel_mag_g_; }
  bool accel_is_stale(unsigned long now_ms, unsigned long stale_ms = 100) const {
    return last_accel_ms_ == 0 || (now_ms - last_accel_ms_) > stale_ms;
  }
  bool init_ok() const { return ok_init_; }
  bool is_stale(unsigned long now_ms, unsigned long stale_ms = 50) const {
    return !ok_init_ || (now_ms - last_update_ms_) > stale_ms;
  }
  const char* name() const { return name_; }

private:
  TwoWire* bus_;
  uint8_t addr_;
  const char* name_;
  Adafruit_BNO055 bno_;
  Quat q_;
  unsigned long last_update_ms_ = 0;
  bool ok_init_ = false;
  float accel_mag_g_ = 1.0f;
  unsigned long last_accel_ms_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Dual-IMU manager: owns body (BNO055)/external (BNO085) instances, runs the 2-of-2-style vote
// between them, exposes the single "voted body attitude" the control loop should use. Class kept
// named TriImu (not renamed) so call sites in wyvern4_tvc.ino/sd_logger.h don't all need
// touching -- what changed is which physical sensors it owns, not its role.
// ---------------------------------------------------------------------------------------------
class TriImu {
public:
  static constexpr float VOTE_DISAGREE_THRESHOLD_RAD = 0.0349f;  // ~2 deg -- beyond normal sensor
                                                                  // noise/lag between two fused-
                                                                  // orientation units on one body.
  // Both units now share ONE I2C bus (the real board has no mux and no second bus) -- `wire` is
  // that shared bus, set up by the caller on GP0 SDA / GP1 SCL per the netlist trace.
  TriImu(TwoWire& wire, uint8_t bno085_addr, uint8_t bno055_addr)
    : external_(&wire, bno085_addr, "external"),
      body_(&wire, bno055_addr, "body") {}

  // Returns a bitmask of which IMUs initialized OK: bit0=external, bit1=body. Flight-critical
  // minimum is BOTH: with only 2 IMUs total, losing either one loses the 2-of-2 cross-check
  // entirely, and body is the primary attitude source the control loop reads, so a dead body IMU
  // is a no-go regardless of what else works.
  uint8_t begin() {
    uint8_t mask = 0;
    if (external_.begin()) mask |= 0x01;
    if (body_.begin())     mask |= 0x02;
    return mask;
  }

  // |a| in g from the body IMU's accelerometer -- the launch-detect and landing-quiescence
  // source (see launch_status.h and wyvern4_tvc.ino's DESCENT-state landing check). Falls back to
  // a sane resting default (1g) if the accel channel has gone stale, rather than returning a
  // garbage value into a threshold comparison.
  float body_accel_mag_g(unsigned long now_ms) const {
    if (body_.accel_is_stale(now_ms)) return 1.0f;
    return body_.accel_mag_g();
  }

  // Poll both and run the body/external vote. Call once per 500 Hz control tick from core 0.
  void update(unsigned long now_ms) {
    external_.poll(now_ms);
    body_.poll(now_ms);

    bool body_ok = !body_.is_stale(now_ms);
    bool ext_ok = !external_.is_stale(now_ms);

    if (body_ok && ext_ok) {
      float disagreement = quat_angle_between(body_.last_quat(), external_.last_quat());
      voted_disagree_rad_ = disagreement;
      if (disagreement <= VOTE_DISAGREE_THRESHOLD_RAD) {
        voted_q_ = body_.last_quat();
        fault_ = false;
      } else {
        voted_q_ = body_.last_quat();
        fault_ = true;
      }
    } else if (body_ok) {
      voted_q_ = body_.last_quat(); fault_ = true; voted_disagree_rad_ = -1.0f; // external is down
    } else if (ext_ok) {
      voted_q_ = external_.last_quat(); fault_ = true; voted_disagree_rad_ = -1.0f; // body is down
    } else {
      fault_ = true; voted_disagree_rad_ = -1.0f; // neither responding -- voted_q_ holds last-known
    }
  }

  const Quat& voted_body_quat() const { return voted_q_; }
  // Kept named gimbal_quat() for call-site/LogFrame-field compatibility (qg_* in sd_logger.h) --
  // it returns the EXTERNAL IMU's quaternion, not a gimbal-mounted sensor's. See file header.
  const Quat& gimbal_quat() const { return external_.last_quat(); }
  bool vote_fault() const { return fault_; }
  float vote_disagreement_rad() const { return voted_disagree_rad_; }
  bool gimbal_stale(unsigned long now_ms) const { return external_.is_stale(now_ms); }

  Bno085External& gimbal() { return external_; }   // name kept for compatibility
  Bno055Body& body() { return body_; }

private:
  Bno085External external_;
  Bno055Body body_;
  Quat voted_q_;
  bool fault_ = false;
  float voted_disagree_rad_ = -1.0f;
};

// RETIRED: this used to compute nozzle deflection as q_body^-1 (x) q_gimbal using a gimbal-mounted
// IMU that does not exist on the real PCB (see file header). The control loop never consumed this
// (wyvern4_tvc.ino's BOOST case is body-attitude-only), only the LogFrame telemetry did. Kept as a
// stub returning NaN, rather than deleted outright, so the LogFrame schema and we4_flight_reduce.py
// (which already handles all-NaN defl_pitch_deg gracefully) don't need a simultaneous schema
// change -- if a real gimbal-relative sensor is added later, this is where the math comes back.
struct Deflection { float pitch_rad, yaw_rad; };
inline Deflection compute_deflection(const Quat& q_body, const Quat& q_gimbal) {
  (void)q_body; (void)q_gimbal;
  return Deflection{ NAN, NAN };
}

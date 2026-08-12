// WYVERN-E — power-rail monitor via onboard INA226, shared I2C bus.
// ===============================================================================
// RECONCILED 2026-08-11 against the real PCB1 netlist/BOM, traced pin-by-pin (not assumed) against
// Netlist_PCB1_2026-08-11.tel and the labeled pinout in SCH_Schematic1_1-P1_2026-08-11.svg. This
// pass found two real problems with the original "INA226 = drop-in LiPo pack monitor" assumption --
// both are hardware findings, not firmware bugs, and both need a bench check before this class's
// output can be trusted for arm/no-arm decisions.
//
// FINDING 1 -- this INA226 does not measure pack voltage, it measures the regulated buck rail.
// U4's VBUS pin (pin 8) and VIN- (pin 9) both trace to the board's VBUCK net -- the OUTPUT of the
// TPS564201 buck (U15), not the raw 2S LiPo input. VIN+ (pin 10) traces to GND. CN1 (the XT30
// battery connector) feeds U15's input directly; the pack voltage itself is not present anywhere
// U4 can see it. Two consequences:
//   - getBusVoltage() reads the ~5V regulated rail (calculated from R5=56k/R6=10.2k against the
//     TPS564201 family's typical 0.768V feedback reference: Vout = 0.768*(1+56/10.2) = 4.98V --
//     bench-verify with a multimeter, this is a calculated estimate, not a datasheet-confirmed
//     figure), not the 6.0-8.4V 2S pack range. The 6.4V/6.0V (3.2/3.0 V-per-cell) cutoffs a true
//     pack monitor would use DO NOT APPLY to this reading and have been replaced below with
//     rail-sag thresholds -- see the caveat on those constants.
//   - VIN+/VIN- do not span a real shunt in the load's current path (R10, the 10 mOhm 2512 part,
//     sits in parallel with the power switch U13 between VBUCK and a floating-ish node, not in
//     series with pack current) -- so getCurrent()/getPower() are not physically meaningful here.
//     This class no longer calls setMaxCurrentShunt() or exposes a current reading for that reason.
//
// FINDING 2 -- the address-select pin (A1, U4 pin 1) is not strapped to a valid option. A0 (pin 2)
// traces cleanly to GND. A1 traces to the same node as R10/U13 (call it NET_X), and R10 alone
// (10 mOhm, always present regardless of switch position) ties NET_X to VBUCK, so A1 sees roughly
// VBUCK (~5V) rather than one of the INA226's four supported address-strap levels (GND / VS+ / SDA
// / SCL). VS+ on this same chip (pin 6) is tied to the 3V3 rail, not VBUCK -- so A1's ~5V is neither
// GND nor this chip's own VS+, meaning the resulting address is not reliably predictable from the
// datasheet's address table, and on a 3.3V-rail part, ~5V on an address-select input is also higher
// than the pin's supply rail, which is worth checking against the INA226's absolute max rating
// before assuming this is merely an addressing inconvenience rather than a stress condition.
// INA226_ADDR below (0x40, A0=A1=GND) is the closest "official" address to what's strapped, kept as
// the first bench-scan candidate, not a confirmed value -- run t1_i2c_scan.ino and update it to
// whatever the scan actually finds, and consider measuring NET_X's real voltage with a multimeter
// before repeated power-cycling.
//
// NEITHER finding is something firmware can correct -- they're both properties of how U4 is wired
// on PCB1. The real fix is a board revision: route U4's VIN+/VIN- across a shunt actually in series
// with pack current, tie A1 to GND or VS+ properly, and give VBUS its own trace back to the pack
// side of the power switch instead of the buck output. Until then, this class provides rail-voltage
// telemetry (useful on its own merits -- a sagging 5V rail is a real fault signal) but does NOT
// provide the LiPo-cell-accurate low-battery/critical protection the vehicle needs; treat the
// low_battery()/critical() flags here as a rail-health check, not a substitute for physically
// charging the pack before every flight and checking it with a separate cell-voltage checker.
#pragma once
#include <Arduino.h>
#include <INA226.h>

class BatteryMonitor {
public:
  static constexpr uint8_t INA226_ADDR = 0x40;   // bench-scan starting guess, NOT confirmed -- see
                                                  // FINDING 2 above; A1's strap is not a valid option
  // Rail-sag thresholds, NOT LiPo-cell thresholds. This INA226 reads the buck's ~5V OUTPUT (FINDING
  // 1), which stays roughly flat as the pack discharges until the pack voltage drops within the
  // buck's dropout margin of its target output -- at which point the rail sags along with it. These
  // numbers are a provisional estimate (assume ~300-400 mV typical dropout for this regulator class
  // at flight-computer-scale load current) of where that sag becomes visible, not a bench-measured
  // curve. They will trip LATER, closer to real pack exhaustion, than a true per-cell pack monitor
  // would -- do not treat a "PASS" here as confirmation the pack is safely charged.
  static constexpr float LOW_BATT_CUTOFF_V = 4.85f;   // rail sag warning (provisional)
  static constexpr float CRITICAL_CUTOFF_V = 4.60f;   // rail sag arm-inhibit (provisional)

  explicit BatteryMonitor(TwoWire& wire) : ina_(INA226_ADDR, &wire) {}

  bool begin() {
    ok_ = ina_.begin();
    // No setMaxCurrentShunt() call: VIN+/VIN- don't span a real shunt on this board (FINDING 1), so
    // current/power calibration would be calibrating a measurement that isn't physically meaningful.
    update();
    filt_v_ = ok_ ? ina_.getBusVoltage() : 0.0f;   // seed filter on first read instead of starting at 0
    return ok_;
  }

  // Call periodically from core 0 (shares core 0's I2C bus -- see wyvern4_tvc.ino's dual-core
  // ownership note). Applies a light exponential filter so a single noisy sample doesn't flicker
  // the LOW_BATT/CRITICAL flags.
  void update() {
    if (!ok_) return;
    float v = ina_.getBusVoltage();
    if (isfinite(v) && v > 0.0f) filt_v_ += (v - filt_v_) * 0.1f;   // ~10-sample time constant
  }

  bool sensor_ok() const { return ok_; }
  float voltage() const { return filt_v_; }   // rail voltage (~5V nominal), NOT pack voltage
  bool low_battery() const { return !ok_ || filt_v_ < LOW_BATT_CUTOFF_V; }
  bool critical() const { return !ok_ || filt_v_ < CRITICAL_CUTOFF_V; }

private:
  INA226 ina_;
  bool ok_ = false;
  float filt_v_ = 0.0f;
};

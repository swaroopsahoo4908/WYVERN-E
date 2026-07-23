// WYVERN-E 4.0 — flight-battery (2S LiPo) ADC voltage monitor.
// ===============================================================
// Power architecture (see ../../CONFLICTS.md section 4): a light 2S LiPo (7.4 V, ~450 mAh) feeds a
// single 5 V/6 V UBEC (set to 5 V), whose one rail powers Pico 2 W VSYS + camera + both servos
// (servos run at 5 V, ~1.8 kg·cm). Battery voltage is sensed on the pack (before the BEC) on GP26
// (ADC0) through:
//   R_top = 100k (VBAT -> node), R_bot = 62k (node -> GND)  =>  ratio = 62/162 = 0.38272
// The divider keeps 2S full-charge (8.4 V) at ~3.21 V, just under the 3.3 V ADC reference. There is
// no separate recovery rail (recovery is the F15-4 motor's own ejection charge), so nothing else is
// in scope.
#pragma once
#include <Arduino.h>

class BatteryMonitor {
public:
  static constexpr uint8_t PIN_VBAT_ADC = 26;     // GP26 / ADC0
  static constexpr float DIVIDER_RATIO = 62.0f / (100.0f + 62.0f);   // R_bot / (R_top + R_bot)
  static constexpr float ADC_VREF = 3.30f;
  static constexpr float ADC_COUNTS = 4095.0f;    // RP2350 12-bit ADC
  static constexpr float LOW_BATT_CUTOFF_V = 6.4f;   // 3.2 V/cell (2S LiPo) — warn / finish up
  static constexpr float CRITICAL_CUTOFF_V = 6.0f;   // 3.0 V/cell — do not arm/launch below this

  void begin() {
    pinMode(PIN_VBAT_ADC, INPUT);
    analogReadResolution(12);
    update();   // seed filt_v_ on first read instead of starting at 0
    filt_v_ = raw_voltage_();
  }

  // Call periodically from core 1 (battery state isn't control-loop-rate-critical). Applies a light
  // exponential filter so a single noisy ADC sample doesn't flicker the LOW_BATT/CRITICAL flags.
  void update() {
    float v = raw_voltage_();
    if (isfinite(v)) filt_v_ += (v - filt_v_) * 0.1f;   // ~10-sample time constant
  }

  float voltage() const { return filt_v_; }
  bool low_battery() const { return filt_v_ < LOW_BATT_CUTOFF_V; }
  bool critical() const { return filt_v_ < CRITICAL_CUTOFF_V; }

private:
  float raw_voltage_() const {
    int counts = analogRead(PIN_VBAT_ADC);
    float v_adc = (counts / ADC_COUNTS) * ADC_VREF;
    return v_adc / DIVIDER_RATIO;
  }
  float filt_v_ = 0.0f;
};

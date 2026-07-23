// WYVERN-E 4.0 — barometric altitude driver: BME688 (mux ch2, 0x76) + BMP388 (mux ch3, 0x77).
// =================================================================================================
// Two independent baro sources behind the PCA9548A mux on I2C0, matching t4_sensors_sdlog.ino's
// wiring exactly. Both are read every cycle on core 1 (baro is not control-loop-critical at 500 Hz;
// it's used for: ground datum at BOOT, launch-detect cross-check, apogee/landing detection during
// DESCENT, and the on-board altitude of record for post-flight reconstruction). With no RRC3+ in the
// vehicle anymore, these two baros ARE the altitude record — hence the two-source redundancy.
//
// The two sensors are simply averaged when both are healthy (cheap two-source redundancy, same
// spirit as the IMU vote but without a disagreement fault flag -- baro disagreement of a few hPa is
// normal sensor-to-sensor variation, not a fault worth latching).
#pragma once
#include <Wire.h>
#include <Adafruit_BMP3XX.h>     // BMP388 (Adafruit 3966) — replaces the BMP280
#include <Adafruit_BME680.h>
#include "i2c_mux.h"

class BaroPair {
public:
  BaroPair(TwoWire& wire, I2CMux& mux) : wire_(wire), mux_(mux), bmp_(), bme_() {}

  bool begin() {
    mux_.select(I2CMux::CH_BMP388);
    bmp_ok_ = bmp_.begin_I2C(0x77, &wire_);
    if (bmp_ok_) {                                   // BMP388 needs oversampling/filter/ODR set up
      bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
      bmp_.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp_.setOutputDataRate(BMP3_ODR_50_HZ);
    }
    mux_.select(I2CMux::CH_BME688);
    bme_ok_ = bme_.begin(0x76);
    if (bmp_ok_ || bme_ok_) {
      // Take the ground-level pressure datum now (called once at BOOT, vehicle stationary on the
      // pad) so altitude-above-ground can be computed without depending on a fixed sea-level
      // pressure constant that drifts with weather.
      read_();
      datum_hpa_ = pressure_hpa();
    }
    return bmp_ok_ || bme_ok_;
  }

  // Re-take the ground datum (e.g. if BOOT is re-entered after a long pad hold and weather drifted).
  void set_datum() { datum_hpa_ = pressure_hpa(); }

  void update() { read_(); }

  bool bmp_ok() const { return bmp_ok_; }
  bool bme_ok() const { return bme_ok_; }

  // Combined pressure (hPa), averaging whichever sensors are healthy.
  float pressure_hpa() const {
    if (bmp_ok_ && bme_ok_) return 0.5f * (bmp_p_ + bme_p_);
    if (bmp_ok_) return bmp_p_;
    if (bme_ok_) return bme_p_;
    return NAN;
  }
  float temperature_c() const {
    if (bmp_ok_ && bme_ok_) return 0.5f * (bmp_t_ + bme_t_);
    if (bmp_ok_) return bmp_t_;
    if (bme_ok_) return bme_t_;
    return NAN;
  }
  float gas_resistance_ohm() const { return bme_ok_ ? bme_gas_ : NAN; }  // BME688 only

  // Barometric altitude above the BOOT-time pad datum, meters. Standard hypsometric approximation
  // (ISA, valid for the few-hundred-meter altitudes this vehicle flies -- see Stability_FinSizing.md
  // apogee ~435 ft / 133 m).
  float altitude_agl_m() const {
    float p = pressure_hpa();
    if (!isfinite(p) || !isfinite(datum_hpa_) || datum_hpa_ <= 0.0f) return NAN;
    return 44330.0f * (1.0f - powf(p / datum_hpa_, 0.1903f));
  }

  // raw per-sensor accessors, for logging both channels independently (catches one sensor drifting)
  float bmp_pressure_hpa() const { return bmp_p_; }
  float bme_pressure_hpa() const { return bme_p_; }

private:
  void read_() {
    if (bmp_ok_) {
      mux_.select(I2CMux::CH_BMP388);
      if (bmp_.performReading()) {          // BMP3XX populates .pressure (Pa) and .temperature (C)
        bmp_p_ = bmp_.pressure / 100.0f;
        bmp_t_ = bmp_.temperature;
      }
    }
    if (bme_ok_) {
      mux_.select(I2CMux::CH_BME688);
      if (bme_.performReading()) {
        bme_p_ = bme_.pressure / 100.0f;
        bme_t_ = bme_.temperature;
        bme_gas_ = bme_.gas_resistance;
      }
    }
  }

  TwoWire& wire_;
  I2CMux& mux_;
  Adafruit_BMP3XX bmp_;
  Adafruit_BME680 bme_;
  bool bmp_ok_ = false, bme_ok_ = false;
  float bmp_p_ = NAN, bmp_t_ = NAN;
  float bme_p_ = NAN, bme_t_ = NAN, bme_gas_ = NAN;
  float datum_hpa_ = NAN;
};

// WYVERN-E 4.0 — REMOVED (2026-07).
// The VL53L4CD ToF-ring plane-fit tilt estimator has been removed from the design. Gimbal
// deflection on the solenoid balance is now taken from the 3-axis load-cell thrust vector
// (calibration.h) cross-checked against the BNO085 attitude — see the rig .ino. This header is
// intentionally empty; it is no longer #included by any sketch and can be deleted.
#pragma once

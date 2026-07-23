// WYVERN-E 4.0 — REMOVED (2026-07).
// This 2-state steady-state Kalman filter existed only to fuse the (now-removed) VL53L4CD ToF-ring
// angle with BNO085 gyro rate. With the ToF ring gone, there is no plane-fit angle to fuse; the
// solenoid rig reads gimbal deflection directly from the load balance + BNO085 attitude (see the
// rig .ino). This header is intentionally empty; it is no longer #included and can be deleted.
#pragma once

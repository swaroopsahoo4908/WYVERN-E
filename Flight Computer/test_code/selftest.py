#!/usr/bin/env python3
"""WYVERN-E 4.0 preflight self-test runner.

Connects to the Pico 2 W over USB serial, power-cycles or resets are NOT performed by this script
(reset the board yourself, or it will simply listen to whatever the board is already printing), and
tabulates the BOOT-state SELFTEST:* lines emitted by build/firmware/wyvern4_tvc.ino's setup()/
setup1(). This replaces the old static checklist stub -- the table below is now generated from a
live serial session via host_monitor.py, not printed unconditionally.

Checks performed by the firmware and reported here (see CONFLICTS.md for the frozen parameter
table these depend on):
  MUX            PCA9548A @0x70 acknowledges on I2C0
  IMU_GIMBAL     BNO085 on I2C1 (dedicated bus) inits + GRV report enabled
  IMU_BODY       BNO085 on I2C0 mux ch0 inits + GRV report enabled (+ accel report)
  IMU_RECOVERY   BNO085 on I2C0 mux ch1 inits + GRV report enabled
  IMU_MINIMUM    gimbal AND (body OR recovery) all initialized -- flight-critical minimum
  BARO_BMP       BMP388 on mux ch3 (0x77) inits
  BARO_BME       BME688 on mux ch2 (0x76) inits
  SERVO          pitch/yaw +-8 deg sweep completes without a hang (visually confirm travel on bench)
  CORE0_READY    core 0's own init sequence finished
  BATTERY        2S LiPo voltage above CRITICAL_CUTOFF_V (see battery.h)
  SD             microSD (SPI0) mounts and the flight-log file opens for write
  WIFI           bench WiFi association, only if WIFI_ENABLED is set in the .ino (SKIP otherwise)
  RBF            Remove-Before-Flight switch sensed pulled (PASS) or still inserted (WAIT)
  FIFO           core 0 -> core 1 inter-core log FIFO had zero drops during the boot window

Usage:
    python3 selftest.py [PORT] [--timeout SECONDS]
Exit code 0 only if every check above reported PASS (or SKIP for WIFI) and SELFTEST:DONE:PASS was
seen -- suitable for a CI-style "go/no-go" gate in a bench checklist script.
"""
import sys
import argparse

import host_monitor


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0",
                     help="serial port (e.g. /dev/ttyACM0, or /dev/tty.usbmodemXXXX on macOS)")
    ap.add_argument("--timeout", type=float, default=20.0, help="seconds to listen (default 20)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print("WYVERN-E 4.0 PREFLIGHT SELF-TEST")
    print("Reset the Pico now if you haven't already -- BOOT self-test runs once at power-up.\n")
    rc = host_monitor.run(args.port, args.timeout, args.baud)
    print("\n>>> PREFLIGHT GO <<<" if rc == 0 else "\n>>> PREFLIGHT NO-GO -- see FAIL/NOT SEEN rows above <<<")
    return rc


if __name__ == "__main__":
    sys.exit(main())

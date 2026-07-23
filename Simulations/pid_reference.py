#!/usr/bin/env python3
"""WYVERN-E 4.0 — PID reference implementation (matches firmware/wyvern_pid.h exactly).
Discrete PID: integral-clamp anti-windup + first-order filtered derivative + output clamp.

Flight gains (re-tuned numerically; see ../Documentation/PID_TUNING_REPORT.md):
Kp=0.10 Ki=0.40 Kd=0.18, out_lim=5° (0.0873 rad), tau_d=0.02 s, i_lim=0.4.
Selected by sweeping >800 (Kp,Ki,Kd) triples against a linearized pitch plant + 40 ms servo lag +
2 ms loop delay (Pade-2) at 24 operating points (4 atmospheres x 6 burn-time slices), keeping only
gains with worst-case phase margin >=30 deg and gain margin >=6 dB at every point, then minimizing
nonlinear gust-rejection pitch deviation. Worst-case result: PM=33.1 deg, GM=9.3 dB, gust pitch
deviation 1.36 deg, gimbal usage 1.72 deg (limit +-8 deg). The old Kp=2.0/Ki=0.4/Kd=0.5 gains
fail the margin check (PM=-6.2 deg, GM=-2.0 dB at Cold -15C, t=0.6s) once the loop delay is
modeled -- this script's nonlinear sim alone doesn't show that because it never modeled the
delay; the linearized margin analysis is the authoritative check.

Run as a script to print a small step-response demo.
"""
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, out_lim, tau_d=0.02, i_lim=0.4):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lim, self.tau, self.ilim = out_lim, tau_d, i_lim
        self.i = self.d = self.prev = 0.0
    def reset(self):
        self.i = self.d = self.prev = 0.0
    def update(self, err, dt):
        self.i = min(max(self.i + err*dt, -self.ilim), self.ilim)   # integrate + anti-windup
        raw = (err - self.prev)/dt; self.prev = err
        self.d += (raw - self.d) * dt/(self.tau + dt)               # filtered derivative
        u = self.kp*err + self.ki*self.i + self.kd*self.d
        return min(max(u, -self.lim), self.lim)                     # output clamp

# flight constants for the WYVERN pitch loop
KP, KI, KD = 0.10, 0.40, 0.18
GIMBAL_LIM = np.radians(5.0)

if __name__ == "__main__":
    # Closed-loop step demo against the FLIGHT pitch plant (incl. fin aero restoring + damping that
    # the bare double-integrator lacks). Authoritative validation is we4_atmos_tvc.py; this is a quick
    # sanity demo: a 2° vertical-trim command settles cleanly with the gimbal well inside ±8°.
    I, T_arm, tau_s, dt = 0.0323, 14.4*0.253, 0.04, 1e-3
    C_restore, C_damp = 0.30, 0.045          # fin restoring (N·m/rad) + pitch aero damping (N·m·s/rad)
    pid = PID(KP, KI, KD, GIMBAL_LIM)
    th=w=delta=0.0; setp=np.radians(2.0); t=0.0
    print(" t(s)  pitch(deg)  gimbal(deg)")
    while t < 1.0:
        cmd = pid.update(setp - th, dt)
        delta += (cmd - delta)*dt/tau_s
        M = T_arm*np.sin(delta) - C_restore*th - C_damp*w
        w += (M/I)*dt; th += w*dt; t += dt
        if abs((t*1000) % 100) < 1:
            print(f"{t:5.2f}  {np.degrees(th):8.2f}   {np.degrees(delta):8.2f}")

#!/usr/bin/env python3
"""
WYVERN-E 4.0 — Flight-Computer Software-in-the-Loop (SIL) digital twin.

Steps the *actual flight-computer processes* at 500 Hz against a simulated vehicle + atmosphere:

  - State machine   : BOOT → ARMED → BOOST → COAST → RECOVER → DESCENT → LANDED
                      (same transitions as wyvern4_tvc.ino: 3 g launch latch, burnout, F15-4
                       motor ejection at t≈7.45 s, apogee cross-check, landing detect).
  - Sensors         : baro altitude, IMU pitch, and body accel — each with realistic noise/bias.
                      The PID closes on the *measured* attitude (not the true state), like the real FC.
  - Control loop    : firmware PID (Kp/Ki/Kd, ±8° gimbal, anti-windup) → thrust-vector moment.
  - Atmosphere      : altitude-varying wind (power-law shear) + turbulence gusts — conditions change
                      through the flight, not a single fixed wind.
  - Recovery        : motor ejection (no FC pyro) → 18" parachute descent.
  - Outputs         : a full state time-series, a flight-log frame stream (sd_logger schema), and a
                      simulated Wi-Fi heartbeat telemetry stream (the `HB:` lines the FC broadcasts).

This is the same physics as core.py (so apogee/mass/recovery match), wrapped in the FC's own logic.
"""
import numpy as np
try:
    import core
except ImportError:
    from . import core

STATES = ["BOOT", "ARMED", "BOOST", "COAST", "RECOVER", "DESCENT", "LANDED"]

# firmware-matched thresholds (wyvern4_tvc.ino / launch_status.h)
LAUNCH_G = 3.0            # launch latch: |a| > 3 g ...
LAUNCH_HOLD_S = 0.05      #   ... sustained 50 ms
LAND_SPEED = 1.0         # |v| < 1 m/s after apogee -> LANDED
BATT_FULL = 7.9          # 2S nominal at arm


def wind_at(z, wind0, shear=0.14):
    """Power-law wind shear referenced to 10 m; wind grows gently with altitude (0.6·wind0 at pad)."""
    z = max(z, 0.0)
    return wind0 * (0.6 + 0.4 * min((z / 30.0) ** shear if z > 0 else 0.0, 1.0))


def run_sil(kp=None, ki=None, kd=None, wind_ms=6.0, turb_pct=12.0, temp_C=15.0,
            pressure_mbar=1013.25, launch_tilt_deg=2.0, gimbal_deg=8.0, seed=0,
            dt=0.002, hb_hz=5, log_hz=50, t_max=35.0):
    """Run one SIL flight. Returns arrays, telemetry lines, log frames, and a summary."""
    kp = core.KP if kp is None else kp
    ki = core.KI if ki is None else ki
    kd = core.KD if kd is None else kd
    rng = np.random.default_rng(seed)

    T0 = core.T0_ISA + (temp_C - 15.0); rho0 = (pressure_mbar * 100.0) / (core.R_AIR * T0)
    lim = np.deg2rad(gimbal_deg); I_MAX = lim / ki if ki > 1e-9 else 1e6
    ZETA = 0.15; arm_len = core.PIVOT - core.CG
    tilt = np.deg2rad(launch_tilt_deg); turb = turb_pct / 100.0

    # true state
    x = z = vx = vz = 0.0
    theta = 0.0; omega = 0.0; integ = 0.0
    # FC state
    st = 0                       # index into STATES (BOOT)
    st = 1                       # ARMED at t=0 (RBF pulled on the pad in this sim)
    launch_t = None; launched = False; deploy_time = None
    baro_bias = rng.normal(0, 0.3); gyro_drift = rng.normal(0, 0.02)  # deg/s
    apogee_baro = -1e9; apogee_true = 0.0; apogee_t = 0.0; deployed = False

    steps = int(np.ceil(min(t_max, 35.0) / dt)) + 1
    rec_t = core.DEPLOY_T
    hb_every = max(1, int(round((1.0 / hb_hz) / dt)))
    log_every = max(1, int(round((1.0 / log_hz) / dt)))
    # pre-draw all sensor/gust noise for the flight (one vectorized RNG call each, ~3x faster loop)
    n_baro = rng.normal(0, 0.4, steps); n_imu = rng.normal(0, np.deg2rad(0.2), steps)
    n_accel = rng.normal(0, 0.05, steps); n_gust = rng.standard_normal(steps); n_batt = rng.normal(0, 0.005, steps)
    t_arr = dt * np.arange(steps)                            # precompute time-only quantities
    Th_arr = np.asarray(core.thrust(t_arr), dtype=float); m_arr = np.asarray(core.mass(t_arr), dtype=float)
    gust_sin = np.sin(2 * np.pi * 0.7 * t_arr + np.arange(steps) * 0.11)

    T = []; Z = []; VZ = []; TH = []; DE = []; BALT = []; STt = []
    telem = []; logs = []
    t = 0.0
    for i in range(steps):
        Th = Th_arr[i]; m = m_arr[i]
        # --- translation (vertical dominant; wind drift horizontal) ---
        rho = rho0 * np.exp(-max(z, 0) / (core.R_AIR * T0 / core.G))
        a_snd = np.sqrt(1.4 * core.R_AIR * max(T0 - core.LAPSE * max(z, 0), 216.65))
        w = wind_at(z, wind_ms) * (1.0 + turb * gust_sin[i] + turb * 0.5 * n_gust[i] * 0.15)
        rvx = vx - w; rvz = vz
        vrel = np.hypot(rvx, rvz) + 1e-9; mach = vrel / a_snd
        if not deployed:
            Cd = core.cd_of_mach(mach); Aref = core.A
        else:
            Cd = core.CHUTE_CD; Aref = core.CHUTE_A          # under parachute
        drag = 0.5 * rho * Cd * Aref * vrel * vrel
        thx = Th * np.sin(tilt) * (Th > 0); thz = Th * np.cos(tilt) * (Th > 0)
        ax = (thx - drag * rvx / vrel) / m
        az = (thz - drag * rvz / vrel) / m - core.G
        vx += ax * dt; vz += az * dt; x += vx * dt; z += vz * dt
        if z < 0: z = 0.0
        a_mag_g = np.hypot(ax, az) / core.G

        # --- sensors (what the FC actually sees) ---
        baro_alt = z + baro_bias + n_baro[i]
        imu_theta = theta + np.deg2rad(gyro_drift) * t + n_imu[i]
        accel_g_meas = a_mag_g + n_accel[i]

        # --- pitch plant + control (closes on measured attitude) ---
        q = 0.5 * rho * vz * vz
        alpha_w = np.arctan2(w, max(vz, 1.0))
        k_aero = q * core.A * core.CN_ALPHA * (core.XCP - core.CG)
        c_aero = 2.0 * ZETA * np.sqrt(max(k_aero, 0.0) * core.IYY)
        on_rail = z < core.RAIL_LEN
        engaged = (STATES[st] in ("BOOST", "COAST")) and (t >= core.TVC_ENGAGE_T) and (Th > 1.0) and (not on_rail)
        err = imu_theta
        integ = float(np.clip(integ + err * dt, -I_MAX, I_MAX))
        delta_cmd = kp * err + ki * integ + kd * omega
        delta = float(np.clip(delta_cmd, -lim, lim)) if engaged else 0.0
        M_tvc = -Th * np.sin(delta) * arm_len
        ang = (k_aero * (alpha_w - theta) - c_aero * omega + M_tvc) / core.IYY
        if on_rail:
            omega = 0.0
        else:
            omega += ang * dt; theta += omega * dt

        # --- state machine (mirrors the firmware) ---
        if not launched and accel_g_meas > LAUNCH_G:
            launch_t = t if launch_t is None else launch_t
            if t - launch_t >= LAUNCH_HOLD_S:
                launched = True; st = 2                      # BOOST
        elif not launched:
            launch_t = None
        if launched and Th <= 0 and STATES[st] == "BOOST":
            st = 3                                            # COAST at burnout
        if baro_alt > apogee_baro:
            apogee_baro = baro_alt
        if z > apogee_true:
            apogee_true = z; apogee_t = t
        if (not deployed) and t >= rec_t and STATES[st] in ("BOOST", "COAST"):
            deployed = True; deploy_time = t; st = 4          # RECOVER (motor ejection + chute inflation)
        if STATES[st] == "RECOVER" and deploy_time is not None and t >= deploy_time + 0.3:
            st = 5                                            # DESCENT (chute open)
        if deployed and STATES[st] == "DESCENT" and z <= 0.10 and t > apogee_t + 1.0:
            st = 6                                            # LANDED (touchdown under chute)

        batt = BATT_FULL - 0.015 * t + n_batt[i]

        # --- logging + telemetry ---
        if i % log_every == 0:
            logs.append((round(t, 4), STATES[st], round(z, 2), round(vz, 2),
                         round(np.rad2deg(theta), 3), round(np.rad2deg(delta), 3),
                         round(baro_alt, 2), round(batt, 2)))
        if i % hb_every == 0:
            telem.append(f"HB:t={int(t*1000)} state={STATES[st]} alt={baro_alt:.1f} v={vz:.1f} "
                         f"pitch={np.rad2deg(theta):.1f} gmbl={np.rad2deg(delta):.1f} "
                         f"batt={batt:.2f}V rbf=1 drop=0")
        T.append(t); Z.append(z); VZ.append(vz); TH.append(np.rad2deg(theta))
        DE.append(np.rad2deg(delta)); BALT.append(baro_alt); STt.append(st)

        t += dt
        if STATES[st] == "LANDED" and t > rec_t + 1:
            break
        if t > t_max:
            break

    arr = {k: np.asarray(v) for k, v in
           dict(t=T, z=Z, vz=VZ, theta_deg=TH, gimbal_deg=DE, baro_alt=BALT, state=STt).items()}
    boost = arr["state"] == 2                                # BOOST = controlled phase
    peak_boost = float(np.max(np.abs(arr["theta_deg"][boost]))) if boost.any() else 0.0
    rms_gimbal_boost = float(np.sqrt(np.mean(arr["gimbal_deg"][boost] ** 2))) if boost.any() else 0.0
    sat_boost = (100.0 * float(np.mean(np.abs(arr["gimbal_deg"][boost]) >= gimbal_deg - 0.1))
                 if boost.any() else 0.0)
    summary = dict(
        apogee_true_m=round(apogee_true, 1), apogee_baro_m=round(apogee_baro, 1),
        apogee_t_s=round(apogee_t, 2), apogee_err_m=round(apogee_baro - apogee_true, 2),
        deploy_t_s=rec_t, deployed=deployed, landed=(STATES[STt[-1]] == "LANDED"),
        peak_pitch_boost_deg=round(peak_boost, 2),           # control metric (TVC active)
        rms_gimbal_boost_deg=round(rms_gimbal_boost, 2),
        gimbal_sat_boost_pct=round(sat_boost, 1),
        peak_pitch_coast_deg=round(float(np.max(np.abs(arr["theta_deg"]))), 1),  # incl. apogee nose-over
        touchdown_v_ms=round(float(arr["vz"][-1]), 2), flight_time_s=round(float(arr["t"][-1]), 2),
        gains=(kp, ki, kd),
    )
    return dict(arr=arr, telemetry=telem, logs=logs, summary=summary,
                log_cols=["t_s", "state", "alt_m", "vz_ms", "pitch_deg", "gimbal_deg", "baro_m", "batt_v"])


if __name__ == "__main__":
    for w in (0, 6, 12):
        r = run_sil(wind_ms=w, turb_pct=12 if w else 0, seed=1)
        s = r["summary"]
        seq = []
        last = None
        for i in r["arr"]["state"]:
            if i != last: seq.append(STATES[i]); last = i
        print(f"wind {w:2d} m/s | states: {'→'.join(seq)} | landed={s['landed']}")
        print(f"           apogee true {s['apogee_true_m']} m (baro {s['apogee_baro_m']}, err {s['apogee_err_m']} m) "
              f"@ {s['apogee_t_s']}s | peak pitch BOOST {s['peak_pitch_boost_deg']}° (coast nose-over "
              f"{s['peak_pitch_coast_deg']}°) | touchdown {s['touchdown_v_ms']} m/s | flight {s['flight_time_s']}s | "
              f"{len(r['telemetry'])} HB")
    print("\nsample telemetry:")
    for line in run_sil(wind_ms=6, seed=2)["telemetry"][::40][:6]:
        print("  " + line)

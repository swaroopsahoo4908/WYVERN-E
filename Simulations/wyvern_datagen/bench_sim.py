#!/usr/bin/env python3
"""
WYVERN-E 4.0 — ground-test bench simulators (backend-agnostic, no Tk).

Pure functions + matplotlib-Figure builders for three bench tools surfaced in the datagen GUI:
  1. Static motor tester   -> select a motor, get its thrust curve + the axial load-cell trace a
                              static stand would log (with sensor noise).
  2. Jetvane suitability    -> side force / axial-thrust loss / thermal-survival screen for a
                              graphite (or printed) jetvane in an Estes BP exhaust vs the servo TVC.
  3. Ground TVC test + PID  -> the 3-axis thrust-vector balance reading (Fz axial, Fx/Fy lateral)
                              while the firmware PID gimbals the nozzle through a bench maneuver,
                              incl. servo lag, and the resolved thrust vector (T, theta, phi).

Everything is deterministic given a seed so the GUI can redraw live. Physics reuse the canonical
constants in core.py where relevant (F15 curve, gains, gimbal limit, arm) so bench numbers line up
with the flight model.
"""
import numpy as np
from matplotlib.figure import Figure

try:
    from . import core
except ImportError:
    import core

_trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))

# ------------------------------------------------------------------ motors
# Estes F15 is the real digitized curve from core.py (49.6 N.s / 3.45 s). Others are represented by
# a plausible rise->sustain->tail shape scaled to the published total impulse / burn / peak so the
# tester shows the right impulse, average and peak. cls = NAR letter class.
MOTORS = {
    "Estes C6":  dict(It=8.8,  tb=1.86, peak=14.1, cls="C"),
    "Estes D12": dict(It=16.8, tb=1.65, peak=29.7, cls="D"),
    "Estes E12": dict(It=28.5, tb=2.62, peak=28.0, cls="E"),
    "Estes E16": dict(It=30.0, tb=1.90, peak=35.0, cls="E"),   # commissioning motor
    "Estes F15": dict(It=49.6, tb=3.45, peak=25.3, cls="F", real="F15"),
}
MOTOR_NAMES = list(MOTORS)


def _shape_curve(total, burn, peak, spike=0.14, n=80):
    """Generic BP thrust shape: linear rise to `peak` over the first spike*burn, then exponential
    decay toward a sustain, cut to zero at burnout; renormalized to the published total impulse."""
    t = np.linspace(0.0, burn, n)
    ts = spike * burn
    rise = peak * (t / ts)
    decay = peak * (0.55 + 0.45 * np.exp(-2.2 * (t - ts) / burn))
    f = np.where(t < ts, rise, decay)
    f[-1] = 0.0
    f = np.clip(f, 0.0, None)
    f *= total / _trapz(f, t)
    return t, f


def motor_curve(name):
    m = MOTORS[name]
    if m.get("real") == "F15":
        return core._TC.copy(), core._FC.copy()
    return _shape_curve(m["It"], m["tb"], m["peak"])


def motor_stats(name):
    t, f = motor_curve(name)
    It = float(_trapz(f, t)); tb = float(t[-1]); peak = float(f.max())
    avg = It / tb if tb > 0 else 0.0
    # NAR class check: total impulse doubles each letter; F = 40.01-80 N.s etc.
    return dict(name=name, It=It, tb=tb, avg=avg, peak=peak, cls=MOTORS[name]["cls"])


def static_stand_trace(name, cell_kg=5.0, sample_hz=80.0, noise_frac=0.004, seed=0):
    """Simulate what the single-axis static stand (5 kg axial load cell + HX711 at `sample_hz`)
    would log for `name`: the true thrust curve resampled at the DAQ rate plus gaussian cell noise
    (noise_frac of the cell's full-scale). Returns (t, thrust_true, cell_reading, stats, headroom)."""
    t, f = motor_curve(name)
    tb = t[-1]
    rng = np.random.default_rng(seed)
    ts = np.arange(0.0, tb + 0.5, 1.0 / sample_hz)
    true = np.interp(ts, t, f, left=0.0, right=0.0)
    fs_N = cell_kg * 9.80665
    reading = true + rng.normal(0.0, noise_frac * fs_N, ts.shape)
    st = motor_stats(name)
    headroom = fs_N / st["peak"] if st["peak"] > 0 else np.inf
    return ts, true, reading, st, dict(cell_fs_N=fs_N, headroom=headroom, sample_hz=sample_hz)


def make_motor_figure(name, cell_kg=5.0, seed=0):
    ts, true, reading, st, info = static_stand_trace(name, cell_kg=cell_kg, seed=seed)
    fig = Figure(figsize=(8, 5), dpi=100); ax = fig.add_subplot(111)
    ax.plot(ts, reading, color="#c0c0c0", lw=0.8, label=f"load cell ({info['sample_hz']:.0f} Hz, noisy)")
    ax.plot(ts, true, color="#2a6f97", lw=2.2, label="true thrust")
    ax.axhline(st["avg"], ls="--", color="#386641", lw=1, label=f"avg {st['avg']:.1f} N")
    ax.axhline(st["peak"], ls=":", color="#bc4749", lw=1, label=f"peak {st['peak']:.1f} N")
    ax.set_xlabel("t (s)"); ax.set_ylabel("thrust (N)")
    over = "  ⚠ cell under-ranged!" if info["headroom"] < 1.05 else ""
    ax.set_title(f"{name}: {st['It']:.1f} N·s ({st['cls']}-class) · burn {st['tb']:.2f} s · "
                 f"{cell_kg:.0f} kg cell = {info['cell_fs_N']:.0f} N FS ({info['headroom']:.1f}×){over}",
                 fontweight="bold")
    ax.grid(alpha=0.3); ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ jetvane suitability
# Jetvanes deflect the exhaust for TVC. Effectiveness ~ side force per degree; axial loss from vane
# drag + cosine; thermal is the killer for a 3.45 s black-powder burn on a printed vane.
EXHAUST = {
    "Estes BP (F15)": dict(Tflame_K=1150.0, kind="black powder"),
    "APCP composite":  dict(Tflame_K=2100.0, kind="ammonium-perchlorate composite"),
}
VANE_MAT = {
    "PC-FR (printed)":    dict(Tmax_K=383.0,  survives=False),  # HDT ~110 C -> ablates instantly
    "ASA-Aero (printed)": dict(Tmax_K=383.0,  survives=False),  # coupon: foamed ASA ablates
    "ABS (printed)":      dict(Tmax_K=371.0,  survives=False),  # coupon: HDT ~98 C -> ablates
    "Graphite":           dict(Tmax_K=3900.0, survives=True),   # sublimes ~3900 K; mild erosion in BP
    "Tungsten":           dict(Tmax_K=3695.0, survives=True),
}


def jetvane_analysis(motor="Estes F15", vane_mat="Graphite", exhaust="Estes BP (F15)",
                     max_defl_deg=15.0, vane_eff_per_deg=0.010, cd_vane=0.15):
    """Side force and axial-thrust loss vs vane deflection over a sweep, plus a thermal-survival
    verdict and a suitability call against the servo-gimbal TVC baseline (~8 deg -> 3.5 N side)."""
    st = motor_stats(motor)
    T = st["avg"]
    d = np.linspace(0.0, max_defl_deg, 60)
    # side force ~ effectiveness * thrust * deflection (small-angle linear region)
    side = vane_eff_per_deg * T * d
    # axial loss = cosine loss + vane profile drag growing with deflection
    axial_loss = T * (1 - np.cos(np.radians(d))) + cd_vane * T * (d / max_defl_deg) ** 2
    axial_loss_pct = 100.0 * axial_loss / T
    ex = EXHAUST[exhaust]; vm = VANE_MAT[vane_mat]
    survives = vm["survives"] and vm["Tmax_K"] > ex["Tflame_K"]
    # servo-TVC baseline side force at the vehicle's +-8 deg gimbal (peak thrust) for comparison
    servo_side = st["peak"] * np.sin(np.radians(8.0))
    side_at_max = side[-1]
    verdict = []
    if not survives:
        verdict.append(f"THERMAL FAIL: {vane_mat} (Tmax {vm['Tmax_K']:.0f} K) cannot survive the "
                       f"{ex['Tflame_K']:.0f} K {ex['kind']} exhaust for the full {st['tb']:.1f} s burn.")
    else:
        verdict.append(f"Thermal OK: {vane_mat} survives the {ex['Tflame_K']:.0f} K exhaust (expect "
                       f"mild leading-edge erosion over {st['tb']:.1f} s; profile the vane accordingly).")
    verdict.append(f"Side force at {max_defl_deg:.0f} deg vane ~= {side_at_max:.2f} N "
                   f"(servo gimbal at 8 deg gives ~{servo_side:.2f} N for comparison).")
    verdict.append(f"Axial thrust loss at {max_defl_deg:.0f} deg ~= {axial_loss_pct[-1]:.1f}% of average thrust.")
    ok = survives and side_at_max >= 0.6 * servo_side and axial_loss_pct[-1] <= 12.0
    verdict.append("SUITABLE (graphite/tungsten only) — jetvanes give control authority comparable to "
                   "the servo gimbal, at the cost of a few % axial loss." if ok else
                   "NOT the preferred TVC for WYVERN — the servo (or magnetic) nozzle gimbal beats a "
                   "jetvane on axial loss and, for any printed vane, on survivability.")
    return dict(d=d, side=side, axial_loss_pct=axial_loss_pct, survives=survives,
                servo_side=servo_side, verdict="\n".join("• " + v for v in verdict), suitable=ok)


def make_jetvane_figure(motor="Estes F15", vane_mat="Graphite", exhaust="Estes BP (F15)",
                        max_defl_deg=15.0, vane_eff_per_deg=0.010):
    r = jetvane_analysis(motor, vane_mat, exhaust, max_defl_deg, vane_eff_per_deg)
    fig = Figure(figsize=(8, 5), dpi=100); ax = fig.add_subplot(111)
    ax.plot(r["d"], r["side"], color="#2a6f97", lw=2.2, label="jetvane side force (N)")
    ax.axhline(r["servo_side"], ls="--", color="#386641", lw=1.2, label=f"servo gimbal @8° ({r['servo_side']:.1f} N)")
    ax.set_xlabel("vane deflection (deg)"); ax.set_ylabel("side force (N)", color="#2a6f97")
    ax2 = ax.twinx()
    ax2.plot(r["d"], r["axial_loss_pct"], color="#bc4749", lw=2.0, label="axial thrust loss (%)")
    ax2.set_ylabel("axial thrust loss (%)", color="#bc4749")
    tv = "SUITABLE" if r["suitable"] else ("THERMAL FAIL" if not r["survives"] else "not preferred")
    ax.set_title(f"Jetvane suitability — {motor} · {vane_mat} vane · {exhaust} → {tv}", fontweight="bold")
    ax.grid(alpha=0.3)
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [l.get_label() for l in lines], loc="upper left", fontsize=8)
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ ground TVC test + PID
def ground_tvc_test(motor="Estes F15", kp=None, ki=None, kd=None, gimbal_deg=8.0,
                    tau_servo=0.04, scenario="Step to 5°, then hold", link_ratio=1.0,
                    axial_cell_kg=5.0, lat_cell_kg=1.0, noise_frac=0.004, dt=0.002, seed=0):
    """Simulate the 3-axis thrust-vector balance during a bench run: the DAQ commands a gimbal
    profile (or the firmware PID rejects a bench disturbance), the servo tracks it through a
    first-order lag, and the balance load cells read the resolved thrust vector with noise.

    Returns time series (t, cmd_deg, meas_deg, Fz, Fx, T, theta_deg) + metrics.
    """
    kp = core.KP if kp is None else kp
    ki = core.KI if ki is None else ki
    kd = core.KD if kd is None else kd
    t, f = motor_curve(motor)
    tb = t[-1]
    rng = np.random.default_rng(seed)
    n = int(np.ceil((tb + 0.5) / dt))
    T_axis = np.arange(n) * dt
    thrust = np.interp(T_axis, t, f, left=0.0, right=0.0)
    lim = gimbal_deg

    # commanded gimbal profile
    step_mode = scenario.startswith("Step")
    sweep_mode = scenario.startswith("Sweep")
    pid_mode = scenario.startswith("PID")
    cmd = np.zeros(n)
    if step_mode:
        cmd[T_axis >= 0.4] = 5.0
    elif sweep_mode:
        cmd = lim * 0.8 * np.sin(2 * np.pi * 0.7 * T_axis)
    # (pid_mode handled in the loop below: PID rejects a 3 deg mount misalignment disturbance)

    meas = np.zeros(n)         # measured nozzle angle (servo lag + linkage)
    Fz = np.zeros(n); Fx = np.zeros(n)
    g = 0.0; integ = 0.0; theta = 3.0 if pid_mode else 0.0; prev = 0.0
    fs_ax = axial_cell_kg * 9.80665; fs_lat = lat_cell_kg * 9.80665
    sat = 0
    for i in range(n):
        Th = thrust[i]
        if pid_mode:
            # firmware PID drives the bench "attitude proxy" theta back to 0 (disturbance rejection)
            err = -theta
            integ = float(np.clip(integ + err * dt, -lim / max(ki, 1e-6), lim / max(ki, 1e-6)))
            deriv = (err - prev) / dt; prev = err
            c = kp * err + ki * integ + kd * deriv
        else:
            c = cmd[i]
        c = float(np.clip(c, -lim, lim))
        if abs(c) >= lim - 1e-6:
            sat += 1
        # servo first-order lag toward the commanded (link_ratio scales cmd->nozzle)
        g += (c * link_ratio - g) * dt / tau_servo
        meas[i] = g
        # balance reads the resolved thrust vector
        fz = Th * np.cos(np.radians(g)); fx = Th * np.sin(np.radians(g))
        Fz[i] = fz + rng.normal(0, noise_frac * fs_ax)
        Fx[i] = fx + rng.normal(0, noise_frac * fs_lat)
        if pid_mode and Th > 1.0:
            # nozzle side force produces a restoring rate on the bench attitude proxy (toy 2nd order)
            theta += (-Th * np.sin(np.radians(g)) * 0.02) * dt * 100 * dt
    # resolve
    T_mag = np.sqrt(Fz ** 2 + Fx ** 2)
    theta_meas = np.degrees(np.arctan2(np.abs(Fx), np.maximum(Fz, 1e-6)))
    st = motor_stats(motor)
    metrics = dict(
        peak_side_N=float(np.nanmax(np.abs(Fx))),
        peak_axial_N=float(np.nanmax(Fz)),
        max_gimbal_deg=float(np.nanmax(np.abs(meas))),
        sat_pct=100.0 * sat / n,
        lat_headroom=(fs_lat / max(float(np.nanmax(np.abs(Fx))), 1e-6)),
        ax_headroom=(fs_ax / max(float(np.nanmax(Fz)), 1e-6)),
        motor=st,
    )
    return dict(t=T_axis, cmd=cmd if not pid_mode else None, meas=meas,
                Fz=Fz, Fx=Fx, T=T_mag, theta_deg=theta_meas, metrics=metrics, pid_mode=pid_mode)


def make_ground_tvc_figure(motor="Estes F15", kp=None, ki=None, kd=None, gimbal_deg=8.0,
                           scenario="Step to 5°, then hold", tau_servo=0.04, seed=0):
    r = ground_tvc_test(motor, kp, ki, kd, gimbal_deg=gimbal_deg, scenario=scenario,
                        tau_servo=tau_servo, seed=seed)
    m = r["metrics"]
    fig = Figure(figsize=(8, 6), dpi=100)
    ax1 = fig.add_subplot(211); ax2 = fig.add_subplot(212)
    if r["cmd"] is not None:
        ax1.plot(r["t"], r["cmd"], "k:", lw=1.2, label="commanded δ")
    ax1.plot(r["t"], r["meas"], color="#bc4749", lw=1.8, label="measured nozzle δ (servo lag)")
    ax1.axhline(gimbal_deg, ls="--", color="#999", lw=0.7); ax1.axhline(-gimbal_deg, ls="--", color="#999", lw=0.7)
    ax1.set_ylabel("gimbal δ (deg)"); ax1.grid(alpha=0.3); ax1.legend(fontsize=8, loc="upper right")
    ax1.set_title(f"Ground 3-axis TVC balance — {motor} · "
                  f"{'PID reject' if r['pid_mode'] else scenario} · sat {m['sat_pct']:.0f}%",
                  fontweight="bold")
    ax2.plot(r["t"], r["Fz"], color="#2a6f97", lw=1.6, label="Fz axial (load cell)")
    ax2.plot(r["t"], r["Fx"], color="#bc4749", lw=1.6, label="Fx lateral (load cell)")
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("force (N)"); ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    return fig, m


if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg")
    for nm in MOTOR_NAMES:
        s = motor_stats(nm)
        print(f"{nm:10} It={s['It']:5.1f} N·s  avg={s['avg']:5.1f} N  peak={s['peak']:5.1f} N  burn={s['tb']:.2f}s  {s['cls']}")
    j = jetvane_analysis("Estes F15", "PC-FR (printed)")
    print("jetvane PC-FR survives:", j["survives"], "| graphite:",
          jetvane_analysis("Estes F15", "Graphite")["survives"])
    r = ground_tvc_test("Estes F15", scenario="PID reject 3° mount tilt")
    print("ground TVC PID:", {k: round(v, 3) for k, v in r["metrics"].items() if isinstance(v, float)})
    print("bench_sim self-test OK")

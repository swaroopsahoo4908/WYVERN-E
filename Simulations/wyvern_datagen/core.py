#!/usr/bin/env python3
"""
WYVERN-E 4.0 — vectorized Monte-Carlo flight-physics core.

Everything here operates on NumPy arrays of shape (N,) so that N independent flights
integrate in lock-step; this is what makes million-flight datasets tractable in pure
Python/NumPy. The nominal physics match `we4_flightsim.py` (the project's canonical
RK4+Barrowman engine) so datasets are consistent with the single-flight results.

Three products are exposed, matching the three requested dataset types:
  - simulate_outcomes()  -> one row per flight (apogee, maxQ, maxMach, drift, landing, ...)
  - simulate_trace()     -> full time-series (t, x, z, v, a, Mach, q, mass) for a set of flights
  - simulate_tvc()       -> per-flight closed-loop TVC control metrics under wind disturbance

Canonical vehicle (F15-4, ASA-Aero airframe, PC-FR bulkheads/tube/engine, 72 mm fins, i3 cam):
  m_lift = 0.705 kg, m_dry = 0.603 kg, D = 70 mm, burn 3.45 s, apogee ~132.6 m / 435 ft @ 6.81 s.
"""
import numpy as np

# NumPy 2.0 renamed trapz -> trapezoid (and removed trapz in 2.x). Support both.
_trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))

# ----------------------------------------------------------------------------- constants
G       = 9.80665
D       = 0.070                      # body diameter [m]
RB      = D / 2.0
A       = np.pi * RB**2              # reference area [m^2]
LTOT    = 0.74                       # overall length [m]
LNOSE   = 0.12
M_LIFT  = 0.705                      # liftoff mass [kg]  (canonical, we4_flightsim; incl. i3 36 g cam)
M_DRY   = 0.603                      # burnout/dry mass [kg]
PROP    = 0.060                      # propellant mass [kg]
TB      = 3.45                       # burn time [s]
CG      = 0.491                      # CG from nose [m] (i3 cam is fwd of CG -> margin up)
XCP     = 0.568                      # CP from nose [m] (Barrowman, 72 mm fins)
IYY     = 0.0210                     # pitch inertia [kg m^2]
PIVOT   = 0.62                       # gimbal pivot station from nose [m]
CN_ALPHA= 12.0                       # total normal-force slope [1/rad] (nose+fins, order-of-mag)

# F15-4 ejection delay: charge fires 4 s after burnout -> recovery deploy time
DEPLOY_T = TB + 4.0                  # = 7.45 s

# recovery
CHUTE_D  = 0.4572                    # 18 in [m]
CHUTE_A  = np.pi * (CHUTE_D/2)**2    # [m^2]
CHUTE_CD = 1.5

# TVC control law (matches firmware wyvern_pid.h, 500 Hz)
KP, KI, KD = 0.10, 0.40, 0.18
GIMBAL_LIM = np.deg2rad(8.0)         # +-8 deg mechanical limit (raised from 5 for wind/weathercock authority)
TVC_ENGAGE_T = 0.5                   # controller enabled at t = 0.5 s
RAIL_LEN = 1.0                       # launch-rod length [m]; vehicle held straight until rail exit

# ISA
T0_ISA, P0_ISA, R_AIR, LAPSE = 288.15, 101325.0, 287.05, 0.0065
RHO0_ISA = P0_ISA / (R_AIR * T0_ISA)

# --- F15-4 thrust curve (digitized, renormalized to 49.6 N.s over 3.45 s) ---
_TC = np.array([0, .05, .12, .2, .3, .5, 1, 1.5, 2, 2.5, 3, 3.3, 3.45])
_FC = np.array([0, 12, 25.3, 22, 16, 13, 12.5, 12.2, 12, 11.8, 11.5, 7, 0], dtype=float)
_FC *= 49.6 / _trapz(_FC, _TC)


def thrust(t):
    """Thrust [N] at time t (scalar or array). Zero outside the burn."""
    t = np.asarray(t, dtype=float)
    f = np.interp(t, _TC, _FC, left=0.0, right=0.0)
    return np.where((t >= 0) & (t <= TB), f, 0.0)


def mass(t):
    """Instantaneous mass [kg] (linear propellant burn)."""
    t = np.clip(np.asarray(t, dtype=float), 0, TB)
    return M_LIFT - (PROP / TB) * t


def cd_of_mach(mach):
    """Barrowman-style drag buildup (subsonic, mild transonic rise). ~0.54 nominal."""
    base = 0.539
    # small transonic bump; WYVERN tops out near Mach 0.45 so this is essentially flat
    return base * (1.0 + 0.6 * np.clip(mach - 0.8, 0, None)**2)


# ----------------------------------------------------------------------------- atmosphere
def _density_profile(z, rho0, T0):
    """Air density [kg/m^3] at altitude z given sampled sea-level rho0 and temperature T0."""
    scale_h = R_AIR * T0 / G                       # ~8400 m
    return rho0 * np.exp(-np.maximum(z, 0.0) / scale_h)


def _sound_speed(z, T0):
    T = np.maximum(T0 - LAPSE * np.maximum(z, 0.0), 216.65)
    return np.sqrt(1.4 * R_AIR * T)


# default sampling envelope (field conditions for a small F-class flight)
ENVELOPE = dict(
    wind_ms=(0.0, 15.0),         # mean wind speed
    wind_dir_deg=(0.0, 360.0),   # bearing wind blows toward
    turb_pct=(0.0, 30.0),        # turbulence intensity (% of mean)
    temp_C=(-15.0, 40.0),        # surface temperature
    pressure_mbar=(985.0, 1030.0),
    launch_tilt_deg=(0.0, 8.0),  # rod angle off vertical
    site_alt_m=(0.0, 300.0),     # field elevation
)


def sample_conditions(rng, n, envelope=None):
    """Draw N atmospheric/launch conditions uniformly over the envelope. Returns dict of (N,) arrays."""
    e = dict(ENVELOPE);
    if envelope: e.update(envelope)
    def u(lo_hi): lo, hi = lo_hi; return rng.uniform(lo, hi, n) if hi > lo else np.full(n, lo)
    temp_C   = u(e["temp_C"]);   press_mb = u(e["pressure_mbar"])
    T0 = T0_ISA + (temp_C - 15.0)                      # 15 C ISA surface reference
    P0 = press_mb * 100.0
    rho0 = P0 / (R_AIR * T0)
    wind = u(e["wind_ms"])
    return dict(
        temp_C=temp_C, pressure_mbar=press_mb, T0=T0, P0=P0, rho0=rho0,
        wind_ms=wind, wind_dir_deg=u(e["wind_dir_deg"]),
        turb_pct=u(e["turb_pct"]), launch_tilt_deg=u(e["launch_tilt_deg"]),
        site_alt_m=u(e["site_alt_m"]),
    )


# ----------------------------------------------------------------------------- trajectory
def _integrate_ascent(cond, dt=0.002, trace_stride=0):
    """
    Vectorized 2-D point-mass ascent+coast from launch to deploy (t=DEPLOY_T).
    x = downwind horizontal [m], z = altitude [m]. Wind blows toward +x at speed `wind_ms`.
    Returns an outcomes dict of (N,) arrays and, if trace_stride>0, a trace dict of (steps,N) arrays.
    """
    n = cond["rho0"].shape[0]
    rho0, T0, wind, tilt = cond["rho0"], cond["T0"], cond["wind_ms"], np.deg2rad(cond["launch_tilt_deg"])
    site = cond["site_alt_m"]

    x = np.zeros(n); z = np.zeros(n)
    vx = np.zeros(n); vz = np.zeros(n)
    # per-flight accumulators
    apo = np.zeros(n); apo_t = np.zeros(n)
    vmax = np.zeros(n); mmax = np.zeros(n); qmax = np.zeros(n)
    amax = np.zeros(n)
    bo_alt = np.zeros(n); bo_v = np.zeros(n); bo_done = np.zeros(n, bool)

    steps = int(np.ceil(DEPLOY_T / dt))
    tr = None
    if trace_stride:
        nsp = steps // trace_stride + 1
        tr = {k: np.empty((nsp, n), dtype=np.float32) for k in
              ("t", "x", "z", "vx", "vz", "accel_g", "mach", "q")}
        tr_i = 0

    t = 0.0
    for i in range(steps):
        m = mass(t); Th = thrust(t)
        rho = _density_profile(z + site, rho0, T0)
        a_snd = _sound_speed(z + site, T0)
        # air-relative velocity (wind adds +x air motion -> rocket sees -wind relative)
        rvx = vx - wind; rvz = vz
        vrel = np.sqrt(rvx*rvx + rvz*rvz) + 1e-9
        mach = vrel / a_snd
        q = 0.5 * rho * vrel * vrel
        Cd = cd_of_mach(mach)
        drag = q * Cd * A                              # magnitude
        dfx = -drag * rvx / vrel
        dfz = -drag * rvz / vrel
        # thrust direction: mostly vertical; small fixed launch tilt toward +x while on/near rod
        powered = Th > 0
        thx = Th * np.sin(tilt) * powered
        thz = Th * np.cos(tilt) * powered + Th * (~powered) * 0.0
        ax = (thx + dfx) / m
        az = (thz + dfz) / m - G
        # integrate (semi-implicit Euler)
        vx = vx + ax * dt; vz = vz + az * dt
        x = x + vx * dt;   z = z + vz * dt
        t += dt
        # accumulate
        acc_g = np.sqrt(ax*ax + az*az) / G
        amax = np.maximum(amax, acc_g)
        spd = np.sqrt(vx*vx + vz*vz)
        vmax = np.maximum(vmax, spd); mmax = np.maximum(mmax, mach); qmax = np.maximum(qmax, q)
        newapo = z > apo
        apo_t = np.where(newapo, t, apo_t); apo = np.maximum(apo, z)
        just_bo = (t >= TB) & (~bo_done)
        bo_alt = np.where(just_bo, z, bo_alt); bo_v = np.where(just_bo, spd, bo_v)
        bo_done = bo_done | (t >= TB)
        if trace_stride and (i % trace_stride == 0):
            tr["t"][tr_i] = t; tr["x"][tr_i] = x; tr["z"][tr_i] = z
            tr["vx"][tr_i] = vx; tr["vz"][tr_i] = vz; tr["accel_g"][tr_i] = acc_g
            tr["mach"][tr_i] = mach; tr["q"][tr_i] = q; tr_i += 1

    # state at deploy
    out = dict(
        apogee_m=apo, apogee_t=apo_t, apogee_ft=apo*3.28084,
        max_speed_ms=vmax, max_mach=mmax, max_q_pa=qmax, max_accel_g=amax,
        burnout_alt_m=bo_alt, burnout_speed_ms=bo_v,
        deploy_alt_m=z.copy(), deploy_vspeed_ms=vz.copy(), deploy_x_m=x.copy(),
    )
    if trace_stride:
        for k in tr: tr[k] = tr[k][:tr_i]
    return out, tr


def _descent(cond, out):
    """Analytic parachute descent from deploy altitude to ground; adds landing/drift/time."""
    rho0, T0, wind, site = cond["rho0"], cond["T0"], cond["wind_ms"], cond["site_alt_m"]
    z0 = out["deploy_alt_m"]
    rho_mid = _density_profile(0.5 * z0 + site, rho0, T0)
    v_term = np.sqrt(2.0 * M_DRY * G / (rho_mid * CHUTE_CD * CHUTE_A))   # terminal descent [m/s]
    # ballistic gap deploy->apogee already folded in; treat descent from deploy altitude
    t_desc = np.maximum(z0, 0.0) / v_term
    out["descent_rate_ms"] = v_term
    out["flight_time_s"] = DEPLOY_T + t_desc
    # chute drifts with the wind; ballistic downrange already in deploy_x
    out["landing_x_m"] = out["deploy_x_m"] + wind * t_desc
    out["drift_from_pad_m"] = np.abs(out["landing_x_m"])
    return out


def simulate_outcomes(n, seed=0, envelope=None, dt=0.002):
    """Monte-Carlo N flights -> per-flight outcomes dict (all (N,) arrays), incl. sampled conditions."""
    rng = np.random.default_rng(seed)
    cond = sample_conditions(rng, n, envelope)
    out, _ = _integrate_ascent(cond, dt=dt, trace_stride=0)
    out = _descent(cond, out)
    out.update({k: cond[k] for k in
                ("wind_ms", "wind_dir_deg", "turb_pct", "temp_C", "pressure_mbar",
                 "launch_tilt_deg", "site_alt_m")})
    return out


def simulate_trace(n, seed=0, envelope=None, dt=0.002, trace_stride=10):
    """N flights with full time-series. Returns (flat_records dict for tabular write, cond)."""
    rng = np.random.default_rng(seed)
    cond = sample_conditions(rng, n, envelope)
    out, tr = _integrate_ascent(cond, dt=dt, trace_stride=trace_stride)
    return out, tr, cond


# ----------------------------------------------------------------------------- TVC control
def simulate_tvc(n, seed=0, envelope=None, dt=0.002):
    """
    Reduced-order closed-loop pitch model at 500 Hz under a wind-gust disturbance, per flight.
    Returns per-flight control metrics (peak pitch error, RMS gimbal, saturation %, settle time).
    """
    rng = np.random.default_rng(seed)
    cond = sample_conditions(rng, n, envelope)
    rho0, T0, wind, turb = cond["rho0"], cond["T0"], cond["wind_ms"], cond["turb_pct"] / 100.0

    theta = np.zeros(n)      # pitch deviation from vertical [rad]
    omega = np.zeros(n)      # pitch rate [rad/s]
    integ = np.zeros(n)      # PID integral (anti-windup clamped)
    z = np.zeros(n); vz = np.full(n, 0.1)

    peak_err = np.zeros(n); sat_steps = np.zeros(n); nsteps = 0
    sum_d2 = np.zeros(n); settle_t = np.full(n, np.nan)

    arm = PIVOT - CG
    I_MAX = GIMBAL_LIM / KI          # anti-windup: cap integral's authority at the gimbal limit
    ZETA = 0.15                      # assumed aero pitch-damping ratio (light)
    t = 0.0
    steps = int(np.ceil((TB + 1.0) / dt))
    # Stable 2nd-order pitch plant:  I*theta'' = k_aero*(alpha_w - theta) - c*omega + M_tvc
    #   k_aero*(alpha_w - theta) is the aerodynamic moment proportional to angle of attack
    #   (restoring, since CP is aft of CG -> static margin > 0); alpha_w is the wind-induced AoA.
    #   TVC drives theta toward 0 (vertical). Equilibrium sits between weathercock and vertical.
    for i in range(steps):
        Th = thrust(t); m = mass(t)
        vz = vz + np.where(Th > 0, Th / m - G, np.where(z > 0, -G, 0.0)) * dt
        vz = np.maximum(vz, 0.1)
        z = z + vz * dt
        rho = _density_profile(z, rho0, T0)
        q = 0.5 * rho * vz * vz
        gust = wind * (1.0 + turb * np.sin(2 * np.pi * 0.7 * t + i * 0.13))   # turbulence modulation
        alpha_w = np.arctan2(gust, np.maximum(vz, 1.0))                       # wind-induced AoA
        k_aero = q * A * CN_ALPHA * (XCP - CG)                               # restoring stiffness (>0)
        c_aero = 2.0 * ZETA * np.sqrt(np.maximum(k_aero, 0.0) * IYY)          # damping
        # rail phase: while on the ~1 m launch rod the vehicle is held straight (theta pinned to 0).
        # Aerodynamic weathercocking + TVC only act after rail exit -- this is where the documented
        # low-speed weathercock (~tens of deg in strong wind on this low-T/W vehicle) begins.
        on_rail = z < RAIL_LEN
        # PID controller (engaged after 0.5 s and while thrust gives gimbal authority).
        # delta>0 for theta>0 so that M_tvc = -T*sin(delta)*arm is a *restoring* (negative-feedback)
        # moment that drives theta back toward vertical.
        err = theta
        integ = np.clip(integ + err * dt, -I_MAX, I_MAX)
        deriv = omega
        delta_cmd = KP * err + KI * integ + KD * deriv
        engaged = (t >= TVC_ENGAGE_T) & (Th > 1.0) & (~on_rail)
        delta = np.clip(delta_cmd, -GIMBAL_LIM, GIMBAL_LIM) * engaged
        sat_steps += ((np.abs(delta_cmd) >= GIMBAL_LIM) & engaged)
        M_tvc = -Th * np.sin(delta) * arm
        ang_acc = (k_aero * (alpha_w - theta) - c_aero * omega + M_tvc) / IYY
        omega = np.where(on_rail, 0.0, omega + ang_acc * dt)
        theta = np.where(on_rail, 0.0, theta + omega * dt)
        adeg = np.abs(np.rad2deg(theta))
        peak_err = np.maximum(peak_err, adeg)
        sum_d2 += np.rad2deg(delta) ** 2
        newly = np.isnan(settle_t) & (adeg < 1.0) & (t > TVC_ENGAGE_T)
        settle_t = np.where(newly, t, settle_t)
        t += dt; nsteps += 1

    return dict(
        peak_pitch_err_deg=peak_err,
        rms_gimbal_deg=np.sqrt(sum_d2 / nsteps),
        gimbal_saturation_pct=100.0 * sat_steps / nsteps,
        settle_time_s=np.nan_to_num(settle_t, nan=-1.0),
        wind_ms=cond["wind_ms"], turb_pct=cond["turb_pct"],
        temp_C=cond["temp_C"], pressure_mbar=cond["pressure_mbar"],
        launch_tilt_deg=cond["launch_tilt_deg"],
    )


def simulate_tvc_trace(kp=KP, ki=KI, kd=KD, gimbal_deg=8.0, wind=8.0, turb=15.0,
                       disturbance="Wind gust", temp_C=15.0, pressure_mbar=1013.25, dt=0.002):
    """
    Single-flight closed-loop pitch response for interactive PID tuning. Same plant as
    simulate_tvc() but scalar, with user gains + gimbal limit, returning full time-series and
    tuning metrics. Two disturbances:
      "Wind gust"      : aerodynamic AoA forcing from the (turbulent) crosswind.
      "Initial tip 10 deg": released 10 deg off vertical in calm air (classic step recovery).
    """
    T0 = T0_ISA + (temp_C - 15.0); rho0 = (pressure_mbar * 100.0) / (R_AIR * T0)
    lim = np.deg2rad(gimbal_deg)
    I_MAX = lim / ki if ki > 1e-9 else 1e6
    ZETA = 0.15; arm = PIVOT - CG
    step_mode = disturbance.startswith("Initial")
    theta = np.deg2rad(10.0) if step_mode else 0.0
    omega = 0.0; integ = 0.0; z = 0.0; vz = 0.1; t = 0.0
    steps = int(np.ceil((TB + 1.5) / dt))
    T = np.empty(steps); TH = np.empty(steps); DE = np.empty(steps)
    for i in range(steps):
        Th = float(thrust(t)); m = float(mass(t))
        vz += (Th / m - G) * dt if Th > 0 else (-G * dt if z > 0 else 0.0)
        vz = max(vz, 0.1); z += vz * dt
        rho = rho0 * np.exp(-z / (R_AIR * T0 / G)); q = 0.5 * rho * vz * vz
        if step_mode:
            alpha_w = 0.0
        else:
            gust = wind * (1.0 + (turb / 100.0) * np.sin(2 * np.pi * 0.7 * t + i * 0.13))
            alpha_w = np.arctan2(gust, max(vz, 1.0))
        k_aero = q * A * CN_ALPHA * (XCP - CG)
        c_aero = 2.0 * ZETA * np.sqrt(max(k_aero, 0.0) * IYY)
        on_rail = z < RAIL_LEN
        err = theta
        integ = float(np.clip(integ + err * dt, -I_MAX, I_MAX))
        delta_cmd = kp * err + ki * integ + kd * omega
        engaged = (t >= TVC_ENGAGE_T) and (Th > 1.0) and (not on_rail)
        delta = float(np.clip(delta_cmd, -lim, lim)) if engaged else 0.0
        M_tvc = -Th * np.sin(delta) * arm
        ang_acc = (k_aero * (alpha_w - theta) - c_aero * omega + M_tvc) / IYY
        if on_rail:
            omega = 0.0
        else:
            omega += ang_acc * dt; theta += omega * dt
        T[i] = t; TH[i] = np.rad2deg(theta); DE[i] = np.rad2deg(delta)
        t += dt
    # metrics (after controller engages)
    m_eng = T >= TVC_ENGAGE_T
    adeg = np.abs(TH)
    peak = float(adeg[m_eng].max()) if m_eng.any() else 0.0
    ss = float(np.mean(adeg[T >= T[-1] - 0.3]))
    # settle: last time |theta| exceeded 1 deg (after engage)
    over = np.where(m_eng & (adeg > 1.0))[0]
    settle = float(T[over[-1]]) if over.size else TVC_ENGAGE_T
    sat = 100.0 * float(np.mean(np.abs(DE[m_eng]) >= gimbal_deg - 1e-6)) if m_eng.any() else 0.0
    rms = float(np.sqrt(np.mean(DE[m_eng] ** 2))) if m_eng.any() else 0.0
    metrics = dict(peak_pitch_deg=peak, steady_err_deg=ss, settle_time_s=settle,
                   rms_gimbal_deg=rms, gimbal_saturation_pct=sat)
    return dict(t=T, theta_deg=TH, delta_deg=DE, metrics=metrics)


if __name__ == "__main__":
    # quick nominal sanity check (zero-wind, ISA) against we4_flightsim (~132.6 m / 435 ft)
    import numpy as _np
    nominal = dict(wind_ms=(0, 0), wind_dir_deg=(0, 0), turb_pct=(0, 0),
                   temp_C=(15, 15), pressure_mbar=(1013.25, 1013.25),
                   launch_tilt_deg=(0, 0), site_alt_m=(0, 0))
    o = simulate_outcomes(1, seed=1, envelope=nominal, dt=0.001)
    print(f"nominal apogee = {o['apogee_m'][0]:.1f} m / {o['apogee_ft'][0]:.0f} ft "
          f"@ t={o['apogee_t'][0]:.2f} s ; max Mach {o['max_mach'][0]:.2f} ; "
          f"descent {o['descent_rate_ms'][0]:.1f} m/s ; flight {o['flight_time_s'][0]:.1f} s")

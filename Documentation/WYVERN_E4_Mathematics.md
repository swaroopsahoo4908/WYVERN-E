# WYVERN-E — Mathematics & Recalculated Mass Budget

*All values from `../Simulations/we4_sim.py`. Single-stage finned F15-4 TVC sustainer (motor-ejection
recovery via two-body-tube separation at the bulkhead joint), 70 mm OD.*

**⚠ Airframe geometry changed since these numbers were last computed:** the vehicle is now **two
separate body tubes (Lower BT, Upper BT)** joined at one bulkhead, replacing the prior four-bay
single-tube layout. The table below is regrouped into Lower BT / Upper BT to match the new physical
split — but the per-item masses, CG, inertia, T/W, and trajectory figures throughout this file are
still the **old single-tube values** and have not been recalculated for the new joint/coupler
hardware the two-BT split adds. Per project convention (`core.py`/`we4_sim.py` cascade is the one
source of truth for derived numbers), these need a real sim re-run, not a hand edit — flagged
throughout this file rather than guessed at.

## 1. Mass budget (bays regrouped into the two-BT layout — **numbers below are stale, pending sim re-run**)

| Body tube | Section | Items | Mass |
|---|---|---|---|
| Upper BT | Nose (PLA) | ellipsoid nose cone | 21 g |
| Upper BT | Recovery wadding + FC bay (PLA) | wadding, custom PCB flight computer, BNO055/BME680/LIS3MDL, µSD, i3 4K Thumb Action Camera, 2S LiPo + 5 V UBEC | 122 g* |
| Lower BT | Chute + shock cord + wadding (PLA) | chute+cord, Nomex protector, wadding | 137 g* |
| Lower BT | TVC bay (PETG-CF) | bay tube, gimbal assy, 2 servos, external BNO085 (gimbal-mounted), motor mount | 268 g |
| Both | Fins + wiring + bulkhead joint | 4× PLA fins (72 mm), wiring/connectors, **bulkhead + separation-joint hardware (not yet in this total — new part)** | 50 g + TBD |
| **Dry total** | | | **690 g + TBD (stale)** |
| Motor | Estes F15-4 loaded (60 g propellant) | | 102 g |
| **Liftoff** | | | **792 g + TBD (stale)** |

\* These two rows carry over the old "Recovery bay" and "FC bay" line items directly — the old
Recovery-bay row also included the bypass-gas-tube and sealed-Bulkhead-B hardware that no longer
exists in this design (§6), so 137 g is very likely an overstatement now, and the FC-bay row still
says **Pico 2 W**, which reflects the pre-custom-PCB electronics stack this project's other docs
(CONFLICTS.md, COMPATIBILITY.md) describe — flagging that mismatch for you to resolve separately,
since it's an electronics-architecture question, not a bay-layout one.

*Material strategy: PLA is the main construction (nose, body, fins, FC/recovery); PETG-CF is reserved
for the TVC bay/motor mount/gimbal and the new bulkhead separation joint.*

PLA (foamed, ~0.5–0.7 g/cm³) for the body cuts ~100–110 g vs PETG-CF (1.25 g/cm³); the original
812 g spec used a PETG-CF body. The lighter vehicle *raises* T/W and apogee (see §3–4) — this
relationship holds regardless of the two-BT split, but the exact numbers need the sim re-run.

## 2. CG, inertia, control arm — **pending sim re-run for the two-BT geometry**

$$x_{cg}=\frac{\sum m_i x_i}{\sum m_i}=49.1\ \mathrm{cm\ (liftoff)},\ 47.4\ \mathrm{cm\ (burnout)};\quad
I_{yy}=\sum m_i (x_i-x_{cg})^2 + \tfrac14 m r^2 = 0.0257\ \mathrm{kg\,m^2}$$

Gimbal pivot at 62 cm from the nose → **control arm $L=x_{pivot}-x_{cg}=12.9$ cm (liftoff), 14.6 cm
(burnout)**. The vehicle is *finned*, so it also carries a real static margin: CP at 56.8 cm gives
**+1.20 cal** at liftoff, rising toward 1.3 cal at burnout as the CG moves forward. Stability is
therefore hybrid — passive fins through the ignition transient, active TVC from t = 0.5 s.

## 3. Thrust-to-weight

$$\mathrm{(T/W)_{avg}}=\frac{14.4}{0.705\cdot 9.81}=2.08,\qquad \mathrm{(T/W)_{peak}}=\frac{25.3}{0.705\cdot 9.81}=3.66$$

The F15 black-powder curve is front-loaded (25.3 N spike → ~14 N sustain), so the rocket gets a
**3.66** T/W kick off the rail, then holds ~2.0. Comfortable for a TVC launch (3.0's two-stage F-boost
was marginal at ~1.8; the lighter single stage is better).

## 4. Trajectory (RK4 + Barrowman engine) — **pending sim re-run for the two-BT geometry**
*Solved by `we4_flightsim.py` — 4th-order Runge-Kutta (dt = 2×10⁻⁴ s) with Barrowman drag buildup;
finned ⇒ static margin +1.20 cal, with active TVC taking over at t = 0.5 s.*
 (RK4 point mass, Cd = 0.539, A = π(0.035)² m²)

Burnout 3.45 s at **59.1 m, 28.9 m/s**; coast to apogee **98.9 m / ~324 ft at t = 6.27 s** (unified RK4 + Barrowman engine, `we4_flightsim.py`, Cd 0.539). Monte-Carlo
(±5 % mass, ±15 % Cd, N = 1000) → 5–95 % apogee **356–513 ft**. *Higher than the 291 ft spec because of the
PLA mass cut* — still low and no-waiver (< 125 g propellant, < G, < 1.5 kg).

## 5. TVC control (rigid-body pitch, servo lag τ=0.04 s, PID)

$$I_{yy}\dot q = T\sin\delta\,L - T\sin(1°)L_{\text{(misalign)}},\quad \dot\theta=q,\quad
\delta=\mathrm{clip}(K_p e+K_i\!\int\! e+K_d\dot e,\ \pm5°)$$

The loop stabilizes to vertical then tracks a 4° commanded maneuver with the **gimbal staying inside
±8°** and peak pitch deviation <4°. Control authority (restoring moment $T\sin8°\,L$ vs a 2°-AoA-
equivalent disturbance) is **positive throughout the burn and falls to zero only as thrust → 0 at
burnout** — which is exactly why recovery is forced right after burnout (no thrust ⇒ no control on a
finned body plus the gimbal). See `plots4/03_tvc_control.png`, `04_control_authority.png`.

## 6. Recovery

Deploy is by the **F15-4 motor ejection charge**, fired 4 s after burnout at **t ≈ 7.45 s** (1.18 s
past apogee), pressurizing the Lower BT and **separating the two body tubes at the bulkhead joint**
(see `WYVERN_E4_Recovery.md`); the finned uncontrolled body can tumble far before that. At t = 4.0 s
the vehicle is still ascending at **~33 m/s** (faster than the 812 g spec's ~20 m/s, because lighter)
— size the chute/cord for a hard opening, or push the timer to ~5 s for ~20 m/s. An **24″ chute**
gives terminal **~6 m/s**:

$$v_t=\sqrt{\frac{2 m g}{\rho\,C_d A_{chute}}}=\sqrt{\frac{2(0.56)(9.81)}{1.225(1.5)\pi(0.23)^2}}\approx 6.0\ \mathrm{m/s}$$

Recovery is a single passive event (the motor's own charge) — there is no electronic deploy path or
backup channel; robustness comes from the bay-pressurization margin against the bulkhead joint's
release-force target (see `WYVERN_E4_Recovery.md` §4, §6 — also flagged there as needing a re-check
against the new two-BT volume).

## 7. No-waiver / class

Single F15-4: 49.6 N·s, 60 g propellant, < 125 g cap, ≤ F class, liftoff 792 g < 1500 g → FAA
Class-1, no waiver, no L1 cert.

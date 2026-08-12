# WYVERN-E, Mathematics & Recalculated Mass Budget

*All values from `../Simulations/we4_sim.py`. Single-stage finned F15-4 TVC sustainer (motor-ejection
recovery via two-body-tube separation at the bulkhead joint), 70 mm OD.*

**⚠ Airframe geometry changed since these numbers were last computed:** the vehicle is now **two
separate body tubes (Lower BT, Upper BT)** joined at one bulkhead, replacing the prior four-bay
single-tube layout. The table below is regrouped into Lower BT / Upper BT to match the new physical
split, but the per-item masses, CG, inertia, T/W, and trajectory figures throughout this file are
still the **old single-tube values** and have not been recalculated for the new joint/coupler
hardware the two-BT split adds. Per project convention (`core.py`/`we4_sim.py` cascade is the one
source of truth for derived numbers), these need a real sim re-run, not a hand edit, flagged
throughout this file rather than guessed at.

## 1. Mass budget (bays regrouped into the two-BT layout, **numbers below are stale, pending sim re-run**)

The material zoning changed on 2026-08-10: ASA-Aero (foamed, 0.65 g/cm³) for the upper body that
houses avionics, PETG-CF (1.30 g/cm³) for the lower body and fins, PC-FR (1.20 g/cm³) for the TVC
assembly. Fin span also grew from 72 to 87 mm in the same pass, since the lighter forward section
plus heavier PETG-CF fins pulled CG aft enough to drop margin under the 1.0 cal floor at the old
span. The single-tube item-level breakdown lives in `we4_sim.py`; the table below is the two-BT
regroup and still needs a real sim re-run for the new bulkhead/joint hardware, exactly as before.

| Body tube | Section | Items | Mass |
|---|---|---|---|
| Upper BT | Nose (ASA-Aero) | ellipsoid nose cone | 16 g |
| Upper BT | Recovery wadding + FC bay (ASA-Aero) | wadding, custom PCB flight computer, BNO055/BME680/LIS3MDL, µSD, i3 4K Thumb Action Camera, 2S LiPo + 5 V UBEC | 79 g* |
| Lower BT | Chute + shock cord + wadding | chute+cord, Nomex protector, wadding | 137 g* |
| Lower BT | TVC bay (PETG-CF tube) + TVC assembly (PC-FR) | bay tube, gimbal assy, 2 servos, motor mount | 214 g |
| Both | Fins + wiring + bulkhead joint | 4x PETG-CF fins (87 mm), wiring/connectors, **bulkhead + separation-joint hardware (not yet in this total, new part)** | 93 g + TBD |
| **Dry total** | | | **627 g + TBD (stale)** |
| Motor | Estes F15-4 loaded (60 g propellant) | | 102 g |
| **Liftoff** | | | **729 g + TBD (stale)** |

\* These two rows carry over the old "Recovery bay" and "FC bay" line items, rescaled to ASA-Aero
density from the single-tube stack. The Recovery-bay row still includes hardware sized for the
sealed-bulkhead architecture that no longer applies (§6), so 137 g is very likely an overstatement
now, and the FC-bay row still says **Pico 2 W**, which reflects the pre-custom-PCB electronics
stack this project's other docs (CONFLICTS.md, COMPATIBILITY.md) describe. Flagging that mismatch
for you to resolve separately, since it's an electronics-architecture question, not a bay-layout or
materials-zoning one.

*Material strategy: ASA-Aero for the upper body (avionics housing, no motor heat or gas load);
PETG-CF for the lower body and fins; PC-FR for the TVC assembly proper (motor mount, gimbal).*

The all-ASA-Aero upper body cuts real mass versus a PETG-CF or PLA equivalent, offsetting most of
the fin-span increase; net liftoff mass still drops from 792 to 729 g even with 87 mm fins instead
of 72 mm. The lighter vehicle raises T/W and apogee (see §3-4); this relationship holds regardless
of the two-BT split, but the exact two-BT numbers need the sim re-run.

## 2. CG, inertia, control arm, **pending sim re-run for the two-BT geometry**

$$x_{cg}=\frac{\sum m_i x_i}{\sum m_i}=50.8\ \mathrm{cm\ (liftoff)},\ 49.1\ \mathrm{cm\ (burnout)};\quad
I_{yy}=\sum m_i (x_i-x_{cg})^2 + \tfrac14 m r^2 = 0.0257\ \mathrm{kg\,m^2}$$

Gimbal pivot at 62 cm from the nose → **control arm $L=x_{pivot}-x_{cg}=11.2$ cm (liftoff), 12.9 cm
(burnout)**. The vehicle is *finned*, so it also carries a real static margin: CP at 59.3 cm (87 mm
fins) gives **+1.20 cal** at liftoff, rising toward 1.3 cal at burnout as the CG moves forward.
Stability is therefore hybrid: passive fins through the ignition transient, active TVC from t = 0.5 s.

## 3. Thrust-to-weight

$$\mathrm{(T/W)_{avg}}=\frac{14.4}{0.7292\cdot 9.81}=2.01,\qquad \mathrm{(T/W)_{peak}}=\frac{25.3}{0.7292\cdot 9.81}=3.54$$

The F15 black-powder curve is front-loaded (25.3 N spike → ~14 N sustain), so the rocket gets a
**3.54** T/W kick off the rail, then holds ~2.01. Comfortable for a TVC launch (3.0's two-stage F-boost
was marginal at ~1.8; the lighter single stage is better).

## 4. Trajectory (RK4 + Barrowman engine), **pending sim re-run for the two-BT geometry**
*Solved by `we4_flightsim.py`, 4th-order Runge-Kutta (dt = 2x10⁻⁴ s) with Barrowman drag buildup;
finned ⇒ static margin +1.20 cal, with active TVC taking over at t = 0.5 s.*
 (RK4 point mass, Cd = 0.539, A = π(0.0435)² m², 87 mm fins)

Burnout 3.45 s at **68.7 m, 33.7 m/s**; coast to apogee **121.1 m / ~397 ft at t = 6.67 s** (unified RK4 + Barrowman engine, `we4_flightsim.py`, Cd 0.539). Monte-Carlo
dispersion for this configuration has not been re-run since the 87 mm fin/material change; treat the
apogee spread as pending alongside the two-BT recompute. Higher than the earlier 291 ft spec because
of the lighter zoned airframe; still low and no-waiver (< 125 g propellant, < G, < 1.5 kg).

## 5. TVC control (rigid-body pitch, servo lag τ=0.04 s, PID)

$$I_{yy}\dot q = T\sin\delta\,L - T\sin(1°)L_{\text{(misalign)}},\quad \dot\theta=q,\quad
\delta=\mathrm{clip}(K_p e+K_i\!\int\! e+K_d\dot e,\ \pm5°)$$

The loop stabilizes to vertical then tracks a 4° commanded maneuver with the **gimbal staying inside
±8°** and peak pitch deviation <4°. Control authority (restoring moment $T\sin8°\,L$ vs a 2°-AoA-
equivalent disturbance) is **positive throughout the burn and falls to zero only as thrust → 0 at
burnout**, which is exactly why recovery is forced right after burnout (no thrust ⇒ no control on a
finned body plus the gimbal). See `plots4/03_tvc_control.png`, `04_control_authority.png`.

## 6. Recovery

Deploy is by the **F15-4 motor ejection charge**, fired 4 s after burnout at **t ≈ 7.45 s** (0.78 s
past apogee), pressurizing the Lower BT and **separating the two body tubes at the bulkhead joint**
(see `WYVERN_E4_Recovery.md`); the finned uncontrolled body can tumble far before that. At burnout
the vehicle is still climbing at **~34 m/s**, faster than the 812 g spec's ~20 m/s because the zoned
airframe is lighter; size the chute/cord for a hard opening, or push the timer if a softer opening
is preferred. An **24″ chute** gives terminal **~4.8 m/s**:

$$v_t=\sqrt{\frac{2 m g}{\rho\,C_d A_{chute}}}=\sqrt{\frac{2(0.6272)(9.81)}{1.225(1.5)\pi(0.3048)^2}}\approx 4.8\ \mathrm{m/s}$$

Recovery is a single passive event (the motor's own charge), there is no electronic deploy path or
backup channel; robustness comes from the bay-pressurization margin against the bulkhead joint's
release-force target (see `WYVERN_E4_Recovery.md` §4, §6, also flagged there as needing a re-check
against the new two-BT volume).

## 7. No-waiver / class

Single F15-4: 49.6 N·s, 60 g propellant, < 125 g cap, ≤ F class, liftoff 729 g < 1500 g → FAA
Class-1, no waiver, no L1 cert.

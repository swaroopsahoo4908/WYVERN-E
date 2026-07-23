# WYVERN-E 4.0 — Mathematics & Recalculated Mass Budget

*All values from `../Simulations/we4_sim.py`. Single-stage finned F15-4 TVC sustainer (motor-ejection recovery), 70 mm OD.*

## 1. Mass budget (recalculated — PC-FR nose/engine, ASA-Aero body)

| Section | Items | Mass |
|---|---|---|
| Nose (ASA-Aero) | ellipsoid nose cone | 21 g |
| Recovery bay (ASA-Aero) | bay tube, chute+cord, Nomex protector, BNO085 (vote), **PC-FR** bypass gas tube, **PC-FR** sealed Bulkhead B, ejection plenum + nose retention | 137 g |
| FC bay (ASA-Aero) | bay tube, Pico 2 W, BNO085, baro (BMP/BME), µSD, i3 4K Thumb Action Camera, 2S LiPo + 5 V UBEC | 122 g |
| Engine/TVC bay (PC-FR) | bay tube, Bulkhead A, gimbal assy, 2 servos, BNO085, motor mount | 268 g |
| Fins + wiring | 4× ASA-Aero fins (72 mm), wiring/connectors | 50 g |
| **Dry total** | | **603 g** |
| Motor | Estes F15-4 loaded (60 g propellant) | 102 g |
| **Liftoff** | | **705 g** |

*Material strategy: ASA-Aero is the main construction (nose, body, fins, FC/recovery bays); PC-FR is reserved for the two bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay + motor mount + gimbal).*

ASA-Aero (foamed, ~0.5–0.7 g/cm³) for the body cuts ~100–110 g vs PC-FR (1.25 g/cm³); the original
812 g spec used a PC-FR body. The lighter vehicle *raises* T/W and apogee (see §3–4).

## 2. CG, inertia, control arm

$$x_{cg}=\frac{\sum m_i x_i}{\sum m_i}=45.0\ \mathrm{cm\ (liftoff)},\ 42.7\ \mathrm{cm\ (burnout)};\quad
I_{yy}=\sum m_i (x_i-x_{cg})^2 + \tfrac14 m r^2 = 0.0219\ \mathrm{kg\,m^2}$$

Gimbal pivot at 62 cm from the nose → **control arm $L=x_{pivot}-x_{cg}=17.0$ cm (liftoff), 19.3 cm
(burnout)**. For a finless vehicle there is no static margin; stability is *active* (TVC) — the
control arm and gimbal authority replace the fin/CP role.

## 3. Thrust-to-weight

$$\mathrm{(T/W)_{avg}}=\frac{14.4}{0.705\cdot 9.81}=2.08,\qquad \mathrm{(T/W)_{peak}}=\frac{25.3}{0.705\cdot 9.81}=3.66$$

The F15 black-powder curve is front-loaded (25.3 N spike → ~12 N sustain), so the rocket gets a
~3.9 T/W kick off the rail, then holds ~2.0. Comfortable for a TVC launch (3.0's two-stage F-boost
was marginal at ~1.8; the lighter single stage is better).

## 4. Trajectory (RK4 + Barrowman engine)
*Solved by `we4_flightsim.py` — 4th-order Runge-Kutta with Barrowman drag buildup; finless ⇒ static margin −5.6 cal ⇒ active TVC.*
 (RK4 point mass, Cd = 0.5, A = π(0.035)² m²)

Burnout 3.45 s at **75.3 m, 35.5 m/s**; coast to apogee **133 m / ~435 ft at t = 6.81 s** (unified RK4 + Barrowman engine, `we4_flightsim.py`, Cd 0.539). Monte-Carlo
(±5 % mass, ±15 % Cd) → 5–95 % apogee **423–595 ft**. *Higher than the 291 ft spec because of the
ASA-Aero mass cut* — still low and no-waiver (< 125 g propellant, < G, < 1.5 kg).

## 5. TVC control (rigid-body pitch, servo lag τ=0.04 s, PID)

$$I_{yy}\dot q = T\sin\delta\,L - T\sin(1°)L_{\text{(misalign)}},\quad \dot\theta=q,\quad
\delta=\mathrm{clip}(K_p e+K_i\!\int\! e+K_d\dot e,\ \pm5°)$$

The loop stabilizes to vertical then tracks a 4° commanded maneuver with the **gimbal staying inside
±8°** and peak pitch deviation <4°. Control authority (restoring moment $T\sin8°\,L$ vs a 2°-AoA-
equivalent disturbance) is **positive throughout the burn and falls to zero only as thrust → 0 at
burnout** — which is exactly why recovery is forced right after burnout (no thrust ⇒ no control on a
finless body). See `plots4/03_tvc_control.png`, `04_control_authority.png`.

## 6. Recovery

Deploy is by the **F15-4 motor ejection charge**, fired 4 s after burnout at **t ≈ 7.45 s** (0.66 s past apogee), routed through the bypass tube into the recovery bay; the finned
uncontrolled body can tumble far. At t = 4.0 s the vehicle is still ascending at **~33 m/s** (faster
than the 812 g spec's ~20 m/s, because lighter) — size the chute/cord for a hard opening, or push
the timer to ~5 s for ~20 m/s. An **18″ chute** gives terminal **~6 m/s**:

$$v_t=\sqrt{\frac{2 m g}{\rho\,C_d A_{chute}}}=\sqrt{\frac{2(0.56)(9.81)}{1.225(1.5)\pi(0.23)^2}}\approx 6.0\ \mathrm{m/s}$$

Recovery is a single passive event (the motor's own charge) — there is no electronic deploy path or backup channel; robustness comes from the 3.4× bay-pressurization margin (see `WYVERN_E4_Recovery.md`).

## 7. No-waiver / class

Single F15-4: 49.6 N·s, 60 g propellant, < 125 g cap, ≤ F class, liftoff 705 g < 1500 g → FAA
Class-1, no waiver, no L1 cert.

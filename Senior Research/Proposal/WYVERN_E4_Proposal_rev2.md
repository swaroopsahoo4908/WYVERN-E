# WYVERN-E: Engineering Design and Experimental Validation of Closed-Loop Thrust-Vector Control, Hybrid Passive–Active Stability, and Zoned Additive-Manufacturing Materials in a Subscale Single Stage Solid-Fuelled Prototype Rocket Demonstrator

**Allison Hong, Chris Liu, Swaroop K. Sahoo**
*A Skylight Rocketry Venture*

---

## Abstract

The WYVERN-E vehicle is a 70 mm-diameter, single-stage, solid-fuelled, additively-manufactured prototype rocket demonstrator developed to provide quantitative, ground-based and flight-validated data on closed-loop thrust-vector control (TVC) actuation, hybrid passive/active stability architectures, and zone-specific additive-manufacturing material selection. The vehicle is powered by a single Estes F15-4 solid motor and relies on four fixed fins sized to the minimum statically-stable margin (1.0 caliber) to survive launch rail departure and the motor's ignition transient, after which authority is transferred to a closed-loop proportional–integral–derivative (PID) thrust-vector-control system operating on the smooth portion of the thrust curve. Two candidate TVC actuation schemes — a tri-solenoid magnetic gimbal and a servo-actuated gimbal — are characterized and directly compared on a purpose-built three-axis thrust-vector load balance prior to flight, isolating actuator dynamics (bandwidth, slew rate, overshoot, steady-state error, and maximum achievable deflection angle) from flight-to-flight variability. All flight avionics, sensor fusion, and control-law execution are consolidated onto a single dual-core microcontroller (Raspberry Pi Pico 2 W, RP2350), with one core dedicated exclusively to deterministic 500 Hz control execution and the second core handling data logging and wireless telemetry — eliminating the blocking-I/O risk inherent to single-threaded flight computer architectures. Structural members are fabricated by fused deposition modeling (FDM) using a thermally-zoned material strategy: PLA (ρ ≈ 1.24 g/cm³, heat-deflection temperature ≈ 55 °C) is the primary construction material for the nose, body tubes and fins — everything that sees neither motor heat nor ejection gas — while carbon-filled PETG (PETG-CF, ρ ≈ 1.30 g/cm³, HDT ≈ 80 °C) is used for the entire ejection-gas path (both structural bulkheads and the bypass tube) and for the thrust-vector assemblies (engine/TVC bay, motor mount, and gimbal). The zoning is driven by heat-deflection temperature rather than mass: PLA is not survivable in contact with the motor's ejection gas, and PETG-CF is the minimum defensible material there. Fin aerodynamics are characterized analytically by the Barrowman (1967) normal-force/center-of-pressure method embedded in the program's RK4 trajectory suite and validated against recovered flight telemetry, from which an in-situ drag coefficient and static margin are reconstructed. All flights are conducted under FAA Class 1 (model rocket) provisions, requiring no airworthiness waiver. Pre-flight Monte Carlo and validation simulation predict an apogee near 324 ft (98.9 m) with a positive thrust-to-weight ratio throughout the burn (1.85 average, 3.26 peak) and a closed-loop pitch deviation under 2.5° across the modeled atmospheric envelope. All CAD, firmware, simulation code, and flight and ground-test datasets will be released publicly through a version-controlled, openly accessible Git repository and presented in summary form within this paper upon completion of the program.

---

## 1. Introduction

### 1.1 Background and Motivation

Active flight control in small-scale, hobbyist-accessible rocketry sits at the intersection of aerodynamics, embedded real-time systems, additive-manufacturing materials science, and classical control theory, and progress on any one axis is frequently constrained by the others. Thrust-vector control — gimbaling the rocket motor's nozzle or exhaust path to produce a control moment without dedicated aerodynamic control surfaces — is the actuation method used on essentially every orbital launch vehicle, yet open, reproducible, and quantitatively validated implementations at the sounding-rocket scale remain comparatively rare in the amateur and academic literature. Existing low-cost demonstrations of vector control (e.g., BPS.space's *Signal*/*Echo* flight series) establish feasibility but do not typically isolate and quantify actuator-class performance independent of flight-to-flight aerodynamic and atmospheric variability.

WYVERN-E addresses this gap with a deliberately simplified architecture relative to prior program iterations: a single fixed-fin airframe that is passively stable through the motor's ignition transient, a single bare-metal flight controller executing the entire sensing-and-control pipeline, and — critically — an actuator comparison that is moved off the flight vehicle and onto a repeatable ground-test apparatus. This restructuring is intended to produce a controlled, statistically tractable dataset on actuator performance (the central engineering question of the program) while retaining a flight-validation phase that demonstrates the complete closed-loop system performs as designed under real atmospheric and motor-burn conditions.

### 1.2 Program Lineage

The WYVERN-E line has progressed through four major design iterations. WYVERN-E 1.0 was a 70 mm, two-stage airframe with a custom flight computer, magnetic-solenoid thrust-vector-control actuation, and actively controlled fins on both stages; 2.0 introduced an 84 mm two-stage airframe with custom avionics; 3.0 implemented a Raspberry Pi 5–based flight computer and flew a magnetic-solenoid-versus-servo actuator A/B comparison in flight. Each iteration established components of the aerodynamic, structural, and avionics groundwork carried forward into the current design. The current iteration consolidates these lessons into the simplest vehicle configuration that still answers the program's central control-authority question, replacing the in-flight actuator A/B test with a ground-based load-balance comparison and replacing the distributed avionics stack with a single dual-core microcontroller.

### 1.3 Central Hypothesis

A small fixed-fin, single-stage, solid-fuelled prototype rocket demonstrator can rely on passive aerodynamic stability to survive launch-rail departure and the motor's ignition transient, then transfer control authority to a closed-loop thrust-vector-control system once the thrust curve smooths; and the relative performance of competing TVC actuator classes (magnetic-solenoid versus servo) can be characterized quantitatively, and more economically, on a ground-based three-axis load balance prior to committing either actuator to flight.

---

## 2. Research Questions

**Research Question 1 — Magnetic-Solenoid versus Servo Actuation for Thrust-Vector Control Authority.**
Substituting a tri-solenoid magnetic gimbal actuator for a servo-driven gimbal actuator as the thrust-vector-control mechanism, evaluated under identical commanded-deflection inputs on a three-axis thrust-vector load balance, will produce measurable differences in actuation bandwidth, slew rate, step-response overshoot, steady-state deflection error, and maximum achievable gimbal angle, with the objective of determining which actuator class provides superior control authority for a given motor thrust regime.

**Research Question 2 — PLA versus Carbon-Filled PETG for Thrust-Vector Structures.**
Holding all FDM process parameters constant across PLA and carbon-filled PETG (PETG-CF), this study characterizes and compares each material's flexural stiffness and modulus under three-point bend loading, and its heat-deflection margin against the measured engine-bay wall temperature from the static-fire campaign, with the objective of justifying the zoned allocation actually flown: PLA for the unheated primary structure, PETG-CF for the ejection-gas path and the thrust-vector assemblies. The question the comparison answers is whether PETG-CF's higher heat-deflection temperature and stiffness are necessary in those zones, or whether a single-material PLA airframe would have sufficed — a decision every low-cost printed TVC vehicle has to make and few justify with data.

**Research Question 3 — Predicted versus In-Situ Passive Stability of the Fixed-Fin Geometry.**
Sizing the four fixed fins to the minimum conventionally stable static margin by the Barrowman (1967) normal-force and center-of-pressure method, the predicted static margin, zero-lift drag coefficient, and weathercock response will be compared against the same quantities reconstructed from recovered flight telemetry — static margin from the observed pitch-rate response to the measured crosswind, and drag coefficient from the coast-phase deceleration between burnout and apogee — with the objective of establishing how accurately a Barrowman-class analytical model predicts the as-built vehicle's passive aerodynamics, and confirming that the selected fin geometry holds the minimum stable margin through the pre-TVC phase without an unnecessary drag or mass penalty.

**Research Question 4 — Closed-Loop Control Gain Sensitivity in a Single-Controller TVC Architecture.**
Operating the proportional-integral-derivative thrust-vector control loop on the single flight microcontroller under multiple candidate gain sets (including a step-response-tuned baseline and a simulation-refined gain set validated across modeled atmospheric and gust conditions), peak pitch deviation, gimbal-angle utilization, and settling behavior will be quantified and compared across repeated flights to determine which gain configuration produces the most accurate and best-damped trajectory tracking on a deterministic single-core-equivalent control architecture.

---

## 3. Project Goal

The goal of the WYVERN-E program is to design, fabricate, ground-test, and flight-validate a recoverable, FAA Class 1, 70 mm-diameter prototype rocket demonstrator that serves as an integrated experimental platform for: (i) quantitatively comparing thrust-vector-control actuator classes independent of flight-to-flight variability; (ii) validating a thermally-zoned additive-manufacturing material strategy against airframe mass and structural-margin objectives; (iii) establishing the predictive accuracy of a Barrowman-class analytical stability model against the as-built vehicle's in-situ passive aerodynamics; and (iv) quantifying control-loop gain sensitivity on a consolidated single-controller avionics architecture. Testing is conducted across three complementary tiers — computational simulation, ground-based instrumented testing, and in-situ powered flight — so that each research question is addressed through at least two independent and mutually cross-validating methods, as mapped in Table 0.

**Table 0. Research-question to method mapping (two independent methods per question).**

| RQ | Method A | Method B |
|---|---|---|
| 1 — TVC actuator class | 3-axis thrust-vector load balance, F15-0 firings (§5.4) | Closed-loop SIL + bench-model actuator dynamics, `wyvern_datagen/bench_sim.py` (§5.5) |
| 2 — PLA vs PETG-CF for TVC structures | Three-point bend coupons, both materials, identical print parameters (§5.1) | Lumped-capacitance thermal model checked against the engine-bay wall temperature measured during the static fires (§5.1, §5.4) |
| 3 — Passive stability | Barrowman CP/CN + RK4 trajectory and Monte Carlo dispersion (§4.2, §5.2) | Flight-telemetry reconstruction of static margin and coast-phase drag (§5.2, §5.6) |
| 4 — Control gain sensitivity | 24-point phase/gain-margin sweep + atmospheric Monte Carlo (§5.3) | Repeated flights under each candidate gain set, onboard log analysis (§5.6) |

---

## 4. Vehicle Architecture

### 4.1 Configuration and Mass Budget

The vehicle is a 70 mm outer-diameter, single-stage airframe approximately 0.74 m in length, comprising three pressure/equipment bays (engine/TVC, flight-computer, recovery) separated by two structural bulkheads, with four fixed fins providing passive stability. The recalculated mass budget, derived from the thermally-zoned material allocation described in §4.2 and Research Question 2, is summarized in Table 1.

**Table 1. WYVERN-E mass budget.**

| Section | Material | Key contents | Mass |
|---|---|---|---|
| Nose cone | PLA | Ellipsoid nose | 21 g |
| Engine/TVC bay | PETG-CF | Engine-bay tube, Bulkhead A, gimbal assembly, 2× servo actuators, attitude IMU, motor mount | 268 g |
| Flight-computer bay | PLA | FC-bay tube, Pico 2 W, attitude IMU, barometric sensors, microSD logger, 2S LiPo + 5 V UBEC, i3 4K Thumb Action Camera camera | 122 g |
| Recovery bay | PLA | Recovery-bay tube, parachute + 1/8″ Kevlar shock cord, Nomex canopy protector, redundant attitude IMU (vote), **PETG-CF** bypass ejection tube, **PETG-CF** sealed Bulkhead B, ejection plenum + nose retention | 137 g |
| Structure | PLA | 4× fixed fins (72 mm), internal wiring | 50 g |
| **Dry mass total** | | | **690 g** |
| Motor (loaded) | — | Estes F15-4, 60 g propellant | 102 g |
| **Liftoff mass total** | | | **792 g** |

Foamed PLA is used for the primary structure (nose, body tube, fins, and the flight-computer and recovery bays); the heat- and flame-rated PETG-CF is reserved for the two structural bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay, motor mount, and gimbal), where either sustained motor-plume heating or the ejection-charge pressure pulse demands it. This all-ASA-except-where-necessary allocation reduces dry mass by approximately 130–150 g relative to an all-PETG-CF baseline (specified at 812 g liftoff in an earlier design iteration), directly raising the thrust-to-weight ratio and predicted apogee. Because the lighter ASA nose moves the center of gravity aft, the fin span was increased to 72 mm to preserve the 1.0-caliber static margin without nose ballast (see §4.2).

### 4.2 Stability Architecture: Passive Margin with Active Handoff

Because the closed-loop TVC system cannot be engaged instantaneously at ignition (servo and control-loop settling, combined with the high-amplitude transient of the motor's ignition spike, would risk a destabilizing initial command), the vehicle must be passively stable through approximately the first 0.5 s of flight. Four fixed fins (root chord 70 mm, tip chord 35 mm, leading-edge sweep 25°, span 72 mm) are sized to the minimum conventionally stable static margin of 1.0 caliber at liftoff (1.20 cal; center of gravity at 48.4 cm from the nose, center of pressure at 56.8 cm, by the Barrowman (1967) method), increasing to approximately 1.3 caliber by burnout as propellant mass is consumed. The 72 mm span (larger than the 58 mm of the earlier all-material-mixed layout) compensates for the aft CG shift introduced by the lightweight ASA nose, holding the 1.0-cal margin with no ballast. A parametric apogee-versus-ballast sweep (Table 2) confirmed that adding nose ballast to increase margin is counter-productive: each gram of ballast lowers apogee, so no ballast is carried, and stability is achieved purely through fin sizing at the minimum stable margin.

**Table 2. Apogee sensitivity to nose ballast (RK4 + Barrowman simulation, 1.0 cal margin held constant by fin resizing).**

| Ballast | Fin span | Liftoff mass | Predicted apogee |
|---|---|---|---|
| 0 g (selected) | 72 mm | 792 g | ~324 ft |
| 60 g | 62 mm | 765 g | ~374 ft |
| 150 g | 54 mm | 855 g | ~291 ft |

At t = 0.5 s, with the motor on the smooth, sustained portion of its thrust curve, the TVC controller is enabled and assumes full attitude authority, commanding the gimbal to stabilize the vehicle to vertical and subsequently execute a small commanded pitch maneuver, while the fins continue to provide passive restoring moment as a stability backstop for the remainder of the powered phase.

---

## 5. Proposed Methodology

The program employs a three-tier experimental framework spanning computational simulation, ground-based instrumented testing, and in-situ powered flight. Each research question is addressed through at least two of these tiers to enable cross-validation. All fabrication uses hobbyist-accessible manufacturing methods and commercially available components, consistent with the program's open-source objectives.

### 5.1 Airframe and Materials (Research Question 2)

All primary structural components — body tube, nose cone, fin set, and bulkheads — are fabricated via fused deposition modeling (FDM) on a Bambu Lab X1C printer using the stock 0.4 mm brass nozzle. Print parameters (layer height, infill pattern and density, wall count, nozzle and bed temperature, and part cooling) are held constant within each material class to isolate material as the independent variable, following the parameter-control methodology of Popescu et al. (2018). Two structural candidate materials are evaluated: carbon-filled PETG (PETG-CF, ρ ≈ 1.30 g/cm³, heat-deflection temperature ≈ 80 °C), allocated to the entire ejection-gas path (both structural bulkheads and the bypass tube) and to the thrust-vector assemblies (engine/TVC bay, motor mount, and gimbal); and PLA (ρ ≈ 1.24 g/cm³, HDT ≈ 55 °C) for the primary structure — nose, both body tubes, and the fin set — where neither motor heat nor ejection gas is present. Flexural stiffness and modulus of both materials are characterized under standardized three-point bend loading following the mechanical-characterization framework of Dizon et al. (2018). The allocation is driven by heat-deflection temperature rather than by mass: PLA at ≈ 55 °C is not survivable in contact with the motor's ejection gas, which is the single consideration that sets the boundary between the two materials. Engine-bay thermal performance is verified by a first-order lumped-capacitance transient model of the PETG-CF motor-bay wall against the F15's 3.45 s burn, predicting a peak wall temperature near 40 °C against the 80 °C heat-deflection limit, and is cross-checked in this program against a thermocouple reading taken on the wall during the static-fire campaign. First-order structural margins on the airframe exceed a safety factor of 300× against the motor's peak axial and TVC-induced bending loads, confirming that wall thickness is set by printability and handling robustness rather than by flight loads — which is why the PLA wall could be reduced from 1.6 mm to 1.2 mm, recovering 45.6 g.

### 5.2 Fin Geometry and Passive Stability (Research Question 3)

The flight fin employs a symmetric aerofoil cross-section (root chord 70 mm, tip chord 35 mm, leading-edge sweep 25 mm, span 72 mm, 3 mm thickness) sized to deliver the minimum 1.0 caliber static margin identified in §4.2. Because the wind-tunnel campaign has been removed from the program (§5.5), passive aerodynamics are established analytically and validated in flight rather than in a ground-based flow facility.

**Analytical prediction (Method A).** Fin normal-force slope and center of pressure are computed by the Barrowman (1967) slender-body method, with the fin term

$$(C_N)_f = k_{fb}\,\frac{4N\left(\dfrac{s}{d}\right)^{2}}{1+\sqrt{1+\left(\dfrac{2\ell_f}{c_r+c_t}\right)^{2}}}, \qquad k_{fb} = 1 + \frac{r_b}{s + r_b}$$

where $N=4$ fins, $s$ is exposed semi-span, $\ell_f$ the mid-chord line length, and $k_{fb}$ the body-interference factor. Zero-lift drag is built up componentwise — skin friction over the wetted area at the flight Reynolds number, base drag, pressure/forebody drag, and fin profile drag — giving the nominal $C_D$ carried by the RK4 trajectory integrator. Reference low-angle-of-attack behaviour for symmetric sections of this class is benchmarked against the thin-airfoil and low-Reynolds-number frameworks of Lissaman (1983) and Mueller and DeLaurier (2003), and against tabulated NACA section data (Abbott & Von Doenhoff, 1959). Sensitivity of the predicted margin to build tolerance is quantified by a CG-tolerance sweep across ±20 mm of as-built CG error, and the resulting margin, apogee, and drift distributions by Monte Carlo dispersion over the atmospheric envelope. An independent implementation of the same geometry in OpenRocket 23.09 (`Simulations/WYVERN_E4_F15-4.ork`) serves as a third-party cross-check on the CP and trajectory prediction.

**Flight reconstruction (Method B).** Two quantities are recovered from the onboard log and compared against the prediction. Coast-phase drag coefficient is reconstructed from the deceleration between burnout and apogee, where thrust is zero and the only forces are drag and gravity:

$$C_D = \frac{2m\left(-\dot{v} - g\right)}{\rho(h)\,A\,v^{2}}$$

evaluated over the high-dynamic-pressure portion of the coast and averaged, with $\rho(h)$ from the barometric altitude and the measured surface conditions. Static margin is reconstructed from the weathercock response: the observed steady pitch offset into the measured crosswind, together with the pitch-rate transient at rail exit, yields the aerodynamic restoring stiffness $k_\alpha = qA C_{N\alpha}(X_{cp}-X_{cg})$, from which $X_{cp}$ follows given the measured CG and dynamic pressure. Agreement between predicted and reconstructed values is the reported result for this research question; disagreement is itself a quantified finding about the predictive limits of a Barrowman-class model on a short, low-Reynolds-number, additively-manufactured airframe.

### 5.3 Flight Computer, Sensing, and Control Architecture (Research Questions 1 and 4)

**Consolidated single-controller architecture.** All flight avionics functions — attitude estimation, control-law execution, actuator commanding, data logging, and telemetry — are consolidated onto a single Raspberry Pi Pico 2 W (RP2350: dual-core 150 MHz Arm Cortex-M33, 520 KB SRAM, on-board Wi-Fi/BLE radio), replacing the distributed multi-board avionics architecture used in earlier program iterations. The two processor cores are functionally partitioned to preserve hard real-time determinism: Core 0 executes the 500 Hz thrust-vector-control loop exclusively — reading the gimbal- and body-mounted inertial measurement units, computing nozzle deflection, evaluating the PID control law, and commanding the gimbal servos — and is permitted no blocking operations of any kind. Core 1 drains a logged-data ring buffer to a microSD card over SPI and services an optional Wi-Fi telemetry link for ground-station monitoring, isolating all non-deterministic I/O latency from the control path. This division directly addresses the principal failure mode of single-threaded flight-computer architectures, in which storage or radio I/O can transiently block control-loop execution.

**Power.** The entire avionics domain runs off a light 2S LiPo (7.4 V, ~450 mAh) feeding a single 5 V/6 V UBEC set to 5 V, whose one rail powers the Pico 2 W VSYS, the camera, and both TVC servos (the servos run at 5 V, ~1.8 kg·cm, comfortably above the ~0.56 kg·cm gimbal demand); a separate 6 V servo BEC is not required at this scale. Because the servos and the flight computer share the 5 V rail, the servo and VSYS feeds are star-wired from the UBEC output with bulk and hold-up capacitance (1000 µF at the servos, 100 µF plus an SS34 Schottky at VSYS) so that ~1 A servo-stall transients cannot brown-out the controller. Pack voltage is monitored on GP26/ADC0 (before the BEC) through a 100 kΩ/62 kΩ divider — keeping 2S full-charge (8.4 V) at ~3.21 V, just under the 3.3 V ADC reference — with firmware warning at 6.4 V (3.2 V/cell) and inhibiting arming below 6.0 V (3.0 V/cell). The power-plus-camera group (LiPo ~30 g, UBEC ~10 g, i3 4K Thumb Action Camera ~36 g) totals roughly 76 g, within the 122 g flight-computer-bay allocation. The i3 camera is ~26 g heavier than the thumb-cam originally budgeted; this is carried through the flight numbers (liftoff 792 g, apogee ~324 ft, T/W 1.85/3.26) and, because the camera sits forward of the CG, actually raises the static margin to ~1.20 cal.

**Attitude sensing.** Three nine-axis inertial measurement units (Bosch BNO085) are deployed — one rigidly referenced to the gimbal/nozzle, one to the vehicle body in the flight-computer bay, and a third redundant unit in the recovery bay for two-of-three fault voting — each configured in Game Rotation Vector mode (accelerometer–gyroscope fusion with the magnetometer disabled), because the magnetic field generated by the adjacent gimbal servos would otherwise corrupt a magnetically-referenced heading estimate. Effective nozzle deflection relative to the vehicle body is computed each control cycle as the quaternion difference q_defl = q_body⁻¹ ⊗ q_gimbal, which captures true mechanical nozzle attitude (including any linkage backlash or structural flex) rather than an assumed commanded angle.

**Control law (Research Question 4).** The per-axis control law is a discrete PID controller with integral anti-windup clamping and a first-order low-pass-filtered derivative term, executed at 500 Hz with output clipped to a ±8° gimbal deflection limit (raised from ±5° to give control-authority margin against crosswind weathercocking without adding passive fin stability, which would otherwise reduce the very disturbance the TVC system is built to demonstrate). The flight gain set — Kp = 0.10, Ki = 0.40, Kd = 0.18 — was selected by a phase/gain-margin analysis across 24 operating points (phase margin ≈ 33°, gain margin ≈ 12.6 dB) and independently confirmed by a time-domain robust multi-wind auto-tune; it holds worst-case gust pitch deviation to 1.96° with wide gimbal headroom against the ±8° limit (2.35° peak gimbal), while avoiding the resonance against finite servo lag (≈ 40 ms) observed in higher-proportional-gain configurations. The TVC loop is inhibited for the first 0.5 s of flight (§4.2), after which it engages to stabilize the vehicle to vertical and execute a small commanded maneuver; required gimbal torque is estimated at ≈ 0.56 kg·cm at ±8°, well within the selected servo class, and simulated control authority remains positive throughout the powered phase across the modeled gain sets.

**Actuator comparison (Research Question 1).** Two TVC actuator classes are evaluated using the identical control electronics, gimbal mechanism, and software control law, isolating actuator dynamics as the experimental variable: a tri-solenoid magnetic gimbal actuator, and a servo-driven gimbal actuator. The comparison is conducted entirely on the ground-based three-axis thrust-vector load balance described in §5.4, rather than in flight, to remove flight-to-flight aerodynamic and atmospheric variability from the actuator comparison; the flight vehicle itself carries the servo actuator, selected on the basis of the ground-comparison results.

### 5.4 Ground Test Program (Research Question 1)

The ground test program is structured as the program's primary experimental apparatus rather than a preliminary check, since both the actuator-comparison (Research Question 1) and the motor-characterization data that feed every downstream simulation (Research Question 4; §4) are generated here rather than in flight. Two purpose-built stands are constructed: a three-axis thrust-vector load balance for actuator comparison, and a single-axis static-thrust stand for motor-curve verification and materials erosion screening. Both stands are fully printable in PETG-CF (selected per Research Question 2 for its motor-plume thermal margin) and instrumented with strain-gauge load cells and HX711 24-bit bridge-amplifier breakouts, logged to onboard microSD by a dedicated data-acquisition microcontroller independent of the flight avionics.

**Three-axis thrust-vector load balance.** The actuator under test (magnetic-solenoid or servo gimbal) is mounted to a thrust block restrained from a fixed base by three strain-gauge load cells acting through flexures — one axial and two lateral — resolving the complete thrust vector in magnitude and direction:

$$T = \sqrt{F_x^2 + F_y^2 + F_z^2}, \qquad \theta = \arctan\!\left(\frac{\sqrt{F_x^2+F_y^2}}{F_z}\right), \qquad \phi = \operatorname{atan2}(F_y, F_x)$$

The cells are sized to the expected loading envelope of the test motor (Estes F15-0, 25.3 N peak axial thrust, side force at the ±8° gimbal limit ≈ 3.5 N), using a 5 kg axial cell and two 1 kg lateral cells digitized at 80 samples per second. An alternative single-piece design — a cruciform flexure instrumented with one Wheatstone bridge per arm, forming a unified three-axis force/torque sensor — is held as a fallback configuration should the discrete three-cell assembly prove difficult to align. The rig is actuator-agnostic, so the magnetic-versus-servo comparison runs entirely on this one fixture under nominally identical thrust conditions; commanded-versus-measured deflection angle (θ, φ) is logged across a series of step and ramp commands under the F15-0 thrust condition to extract bandwidth, slew rate, step-response overshoot, steady-state error, and maximum sustained deflection for each actuator. Because each F15-0 firing provides a 3.45 s control window, the independent step/ramp command set for each actuator system is built up across multiple firings on this stand.

**Static thrust stand.** A single-axis, load-cell-only stand fitted with a steel blast deflector validates the as-fired thrust curve of every motor class used in the program against its published specification, and carries a thermocouple on the engine-bay wall to measure the peak temperature that the Research Question 2 heat-deflection argument depends on. Jetvane erosion screening, present in earlier revisions of this program, has been removed from scope. The deflector and mounting hardware are sized to the F15's 3.45 s burn thermal case. Ground firings use the plugged, 0-delay Estes F15-0 (identical thrust curve to the flight F15-4, but no ejection charge) so that nothing fires into the stand fixtures after burnout.

**Motor plan and firing counts.** Table 3 summarizes the verified motor specifications and the planned firing allocation across flight, both ground stands, and stand commissioning.

**Table 3. Motor plan and verified specifications.**

| Motor | Total impulse | Avg / peak thrust | Burn time | Role |
|---|---|---|---|---|
| Estes F15-4 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Flight only (4 s delay + ejection = recovery system) |
| Estes F15-0 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Ground only — static thrust curves, and the magnetic and servo TVC runs on the load balance (0-delay/plugged; same curve, no ejection into fixtures) |
| Estes/AeroTech E16-4 | — | ~16 N avg | E-class | Stand commissioning and calibration only |

Planned firing counts are 4 Estes F15-4 motors for flight testing, and 10 plugged Estes F15-0 motors for ground testing (6 on the TVC balance at three firings per actuator system, 2 on the static stand for thrust-curve verification and the engine-bay wall temperature measurement, and 2 held in reserve), plus 6 Estes/AeroTech E16-4 motors for stand commissioning at two firings per stand. The F15-0 and F15-4 share an identical thrust curve (same F15 propellant); the 0-delay F15-0 is used on the ground so no ejection charge fires into the stand fixtures, while the flight motor carries the 4-second delay whose ejection charge is the recovery system.

**Calibration and commissioning sequence.** Each stand's load cells are first calibrated independently of any motor firing, using a series of known hanging dead weights spanning the expected force range, to establish a force-to-voltage transfer function for every channel. Each stand is then commissioned with a minimum of two low-cost E16-4 motor firings before any data-collection firing is conducted, to validate the as-built stand's measured thrust curve against the motor's independently published reference curve and confirm that structural compliance in the stand itself is not corrupting the force measurement. Only after a stand passes this commissioning check are F15-0 data-collection firings conducted on it. All raw and reduced ground-test data (thrust curves, vector-deflection logs, and materials-erosion observations) are archived under a dedicated data directory structure separating motor thrust-curve data from TVC vector/control data, in the same repository used for flight data release (§5.6).

### 5.5 Simulation Suite and Software-in-the-Loop Validation (All Research Questions)

Because the program no longer operates a physical flow facility, the computational tier carries the full analytical load for aerodynamics and control, and is correspondingly held to an explicit verification standard: every simulated result reported here is produced by a version-controlled script, paired with the dataset it wrote, and gated against hard pass/fail criteria rather than inspected qualitatively.

**Trajectory and dispersion.** A fourth-order Runge–Kutta integrator advances the two-dimensional point-mass state under the digitized F15 thrust curve, the componentwise Barrowman drag buildup of §5.2, an exponential-density atmosphere referenced to the measured surface temperature and pressure, and a power-law wind-shear profile. Monte Carlo dispersion samples the full field envelope — mean wind, turbulence intensity, surface temperature and pressure, launch-rod angle, and site elevation — together with vehicle-side dispersions in liftoff mass, CG station, drag coefficient, and total impulse, producing distributions of apogee, maximum dynamic pressure, deployment velocity, landing dispersion, and static margin at every point in the burn.

**Closed-loop control simulation.** The pitch-plane control model reproduces the flight law as executed: the discrete PID of §5.3 at the firmware's 500 Hz rate, a first-order servo lag with a measured-class time constant, an explicit control-loop transport delay, gimbal-rate and travel limits, and gust forcing generated from a Dryden-form turbulence spectrum rather than a single sinusoid. Frequency-domain phase and gain margins are evaluated at every point in a burn-time × atmosphere grid, and a candidate gain set is accepted only if it clears the margin target at every point in that grid — the procedure that superseded two earlier gain sets (§5.3).

**Software-in-the-loop flight computer.** The flight firmware's state machine, sensor models, and control law are exercised end-to-end in `wyvern_datagen/fc_sil.py` against simulated barometer, IMU, and battery signals carrying representative noise, bias, and quantization, producing synthetic flight logs in the same schema as the onboard recorder. This lets the ground-station analysis pipeline and every pass/fail gate be exercised and debugged before any motor is fired, and provides the Method-B leg for Research Question 1 by driving the bench actuator models under the identical control law used on the balance.

**Bench-model cross-validation of the ground stands.** The instrumented stands of §5.4 are modeled at the signal level in `wyvern_datagen/bench_sim.py` — load-cell full scale and bridge noise, HX711 sample rate and quantization, stand structural compliance and its resulting mount resonance, actuator lag and linkage ratio — so that the expected measurement, its uncertainty, and the required cell sizing are predicted before the stand is built, and any departure of the as-built stand from that prediction is itself detectable.

**Reproducibility.** All simulated aerodynamic, trajectory, and control-loop datasets generated during the program — the RK4-plus-Barrowman trajectory and dispersion simulations of §4.2 and §5.2, the gate-based flight-validation suite referenced in §5.6 and §8, the atmospheric-sweep and margin-analysis control-loop simulations of §5.3, and the bench and software-in-the-loop models described above — are version-controlled alongside the physical test data in the program's public repository, with each simulation script paired against the dataset it produced, so that every plotted or tabulated simulated result in this paper and its supporting materials can be regenerated and independently checked against the as-built hardware.

### 5.6 Flight Test Plan, Logging, and Data Sharing (Research Questions 3 and 4)

Each flight is conducted in compliance with the National Association of Rocketry (2023) safety code and motor classification standards, under the supervision of a NAR-certified or similarly qualified range safety officer. Multiple flights are conducted with the control law configured under each candidate gain set identified in §5.3, and post-flight telemetry recovered from the onboard microSD log is analyzed for peak pitch deviation during the powered phase, gimbal-angle utilization, and qualitative settling/overshoot behavior, to determine which gain configuration produces the most accurate and best-damped attitude tracking. All onboard sensor data (full-rate inertial, barometric, and control-loop telemetry) is logged at the full control-loop sample rate; where the bench/range Wi-Fi telemetry link is in range, a parallel live feed is monitored for real-time anomaly detection, though the onboard log remains the data of record. Upon conclusion of the program, all CAD files, firmware source code, simulation scripts, and the complete flight and ground-test datasets are released through a version-controlled, publicly accessible Git repository, and the principal reduced results from each research question are additionally presented in summary form within this paper.

---

## 6. Recovery System

The vehicle's passive fin stability is retained through the coast phase to apogee (predicted apogee ≈ 324 ft at t ≈ 6.27 s). Recovery is initiated not by an independent electronic altimeter but by the flight motor's own factory ejection charge: the flight configuration uses an Estes F15-4 (a four-second ejection delay) in place of the previously plugged F15-0, so that approximately four seconds after propellant burnout — t ≈ 7.45 s, roughly 0.64 s past the predicted apogee — the motor's integral ejection charge fires. Because the flight-computer bay is sealed gas-tight between the two structural bulkheads, the hot ejection gas is not permitted to vent through the avionics; it is instead routed through a dedicated solid-walled bypass tube (12 mm internal diameter, flame-retardant polycarbonate) running from an ejection plenum at the motor-side bulkhead (Bulkhead A), alongside the sealed flight-computer bay, to the recovery bay above Bulkhead B, where it pressurizes the bay and releases a friction-fit nose cone carrying the parachute.

A first-order feasibility analysis (`Simulations/we4_ejection_feasibility.py`) supports the approach on two independent grounds. First, the bypass tube imposes a negligible flow penalty: the pressure loss across the 12 mm bore at the ejection mass-flow is on the order of 0.06 kPa. Second, the recovery bay pressurizes to approximately 140 kPa against a friction-fit nose-release threshold of 14–41 kPa, a pressurization margin of roughly 3.4×. The F15-4's four-second delay is the closest available Estes delay to the coast-to-apogee optimum (≈ 3.5 s); the longer F15-6 and F15-8 delays are rejected because they fire approximately 2.5 s and 4.5 s past apogee, deploying at high descent speed and, in the F15-8 case, at dangerously low altitude. This motor-ejection architecture eliminates the independent RRC3+ recovery computer, its isolated 9 V battery, the e-match and black-powder charge well, and the associated recovery wiring of the earlier design — reducing parts count, cost, and an entire electronic failure domain — at the cost of a single passive deployment event with no electronic backup channel, a trade justified by the 3.4× pressurization margin and by the finned airframe's aerodynamic stability through apogee. Shock-cord and parachute sizing (1/8″ tubular Kevlar cord, 24″ ripstop nylon canopy, with a Nomex blanket shielding the canopy from the ejection gas) are verified against the worst-case deployment scenario — ejection ≈ 1.18 s past apogee while the vehicle still carries ≈ 4.6 m/s of vertical velocity — yielding a structural safety factor exceeding 800× on the recovery harness and a predicted terminal descent rate near 6 m/s. The recovery-bay bulkhead (Bulkhead B) and bypass tube are checked against the ≈ 140 kPa ejection pressure in the structural analysis (`WYVERN_E4_FEA_Structural.md` §4), returning safety factors of ≈ 8× and ≈ 107× respectively.

---

## 7. Safety and Regulatory Compliance

All flights use a single Estes F15-4 motor (49.6 N·s total impulse, 60 g propellant, F-class, four-second ejection delay), and the fully loaded liftoff mass of 792 g is well under the FAA's Class 1 (model rocket) threshold of 1,500 g loaded weight per motor, requiring no Federal Aviation Administration airworthiness waiver and no NAR/Tripoli high-power certification. Range procedures include remote ignition, a minimum 3 m personnel standoff from both ground-test stands during firing, a fail-safe neutral-gimbal default state on any control-system fault, and standard model-rocketry motor-handling discipline for the motor's integral ejection charge — the igniter is installed last, at the pad, and there are no independent pyrotechnic or electronic ejection circuits in the vehicle to arm or inhibit (recovery is effected solely by the motor's own delay/ejection charge).

---

## 8. Expected Outcomes

The program is expected to produce a quantitative, ground-validated comparison of magnetic-solenoid and servo thrust-vector-control actuators — including bandwidth, slew rate, overshoot, and steady-state error for each — directly informing actuator selection for future closed-loop rocketry programs without requiring a dedicated in-flight A/B comparison. A validated thermally-zoned additive-manufacturing material allocation is expected to demonstrate a measurable dry-mass reduction (approximately 100–150 g, or roughly 15–20% of dry mass) relative to a uniform heat-rated-material baseline, with no corresponding loss of structural margin, offering a transferable design pattern for hobbyist and academic rocketry programs using FDM fabrication. A quantified accuracy bound on Barrowman-class stability prediction for short, low-Reynolds-number, additively-manufactured airframes — expressed as the discrepancy between predicted and telemetry-reconstructed static margin and coast-phase drag coefficient — will be released as an open dataset, giving subsequent low-cost programs a defensible error bar to design against when no flow facility is available. Finally, flight-validated comparison of control-loop gain configurations on a consolidated dual-core single-controller architecture is expected to clarify the practical control-authority and determinism benefits of separating real-time control execution from logging and telemetry I/O on a single low-cost microcontroller, relative to either a single-threaded controller or a distributed multi-board avionics stack. All resulting design files, firmware, simulation code, and flight datasets will be released publicly through the National Association of Rocketry in support of the program's open-source objectives.

---

## References

Abbott, I. H., & Von Doenhoff, A. E. (1959). *Theory of wing sections: Including a summary of airfoil data.* Dover Publications.

Barrowman, J. S. (1967). *The practical calculation of the aerodynamic characteristics of slender finned vehicles* (NASA NTRS accession 20010047838). https://ntrs.nasa.gov/citations/20010047838

Bosch Sensortec. (n.d.). *BNO085 9-axis absolute orientation IMU — datasheet.*

BPS.space. (n.d.). *Thrust vector control.* Retrieved from https://bps.space/products/thrust-vector-control

Dizon, J. R. C., Espera, A. H., Chen, Q., & Advincula, R. C. (2018). Mechanical characterization of 3D-printed polymers. *Additive Manufacturing, 20,* 44–67. https://doi.org/10.1016/j.addma.2017.12.002

Lissaman, P. B. S. (1983). Low-Reynolds-number airfoils. *Annual Review of Fluid Mechanics, 15,* 223–239.

Mueller, T. J., & DeLaurier, J. D. (2003). Aerodynamics of small vehicles. *Annual Review of Fluid Mechanics, 35,* 89–111. https://doi.org/10.1146/annurev.fluid.35.101101.161102

National Advisory Committee for Aeronautics. (1951). *Aerodynamic characteristics of NACA 0012 airfoil section at angles of attack from 0° to 180°* (NACA TN 2502).

National Association of Rocketry. (2023). *NAR safety code and motor classification standards.* https://www.nar.org/safety-information/

NASA. (1968). *Thrust-vector control requirements for solid-propellant launch vehicles* (NASA TN D-4971).

OpenRocket Project. (2023). *OpenRocket technical documentation v23.09.* https://openrocket.info/documentation.html

Pérez Gordillo, A., Simplício, P., Iannelli, A., & Marcos, A. (2023). Thrust vector control and state estimation architecture for low-cost small-scale launchers. *arXiv.* https://arxiv.org/pdf/2303.16983

Popescu, D., Zapciu, A., Amza, C., Baciu, F., & Marinescu, R. (2018). FDM process parameters influence over the mechanical properties of polymer specimens. *Polymer Testing, 69,* 157–166. https://doi.org/10.1016/j.polymertesting.2018.05.020

Raspberry Pi Foundation. (2024). *RP2350 datasheet.*

Sahoo, S. (2026, April 11). WYVERN PTD Portal. Skylight Industries. https://wyvern-e.base44.app/

Ziegler, J. G., & Nichols, N. B. (1942). Optimum settings for automatic controllers. *Transactions of the ASME, 64,* 759–768.

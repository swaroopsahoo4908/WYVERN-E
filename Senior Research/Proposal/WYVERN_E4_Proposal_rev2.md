# WYVERN-E: Engineering Design and Experimental Validation of Closed-Loop Thrust-Vector Control, Hybrid Passive–Active Stability, and Zoned Additive-Manufacturing Materials in a Subscale Single Stage Solid-Fuelled Prototype Rocket Demonstrator

**Allison Hong, Chris Liu, Swaroop K. Sahoo**
*A Skylight Rocketry Venture*

---

## Abstract

The WYVERN-E vehicle is a 70 mm-diameter, single-stage, solid-fuelled, additively-manufactured prototype rocket demonstrator developed to provide quantitative, ground-based and flight-validated data on closed-loop thrust-vector control (TVC) actuation, hybrid passive/active stability architectures, and zone-specific additive-manufacturing material selection. The vehicle is powered by a single Estes F15-4 solid motor and relies on four fixed fins sized to the minimum statically-stable margin (1.0 caliber) to survive launch rail departure and the motor's ignition transient, after which authority is transferred to a closed-loop proportional–integral–derivative (PID) thrust-vector-control system operating on the smooth portion of the thrust curve. Two candidate TVC actuation schemes — a tri-solenoid magnetic gimbal and a servo-actuated gimbal — are characterized and directly compared on a purpose-built three-axis thrust-vector load balance prior to flight, isolating actuator dynamics (bandwidth, slew rate, overshoot, steady-state error, and maximum achievable deflection angle) from flight-to-flight variability. All flight avionics, sensor fusion, and control-law execution are consolidated onto a single dual-core microcontroller (Raspberry Pi Pico 2 W, RP2350), with one core dedicated exclusively to deterministic 500 Hz control execution and the second core handling data logging and wireless telemetry — eliminating the blocking-I/O risk inherent to single-threaded flight computer architectures. Structural members are fabricated by fused deposition modeling (FDM) using a thermally-zoned material strategy: low-density foamed acrylonitrile styrene acrylate (ASA-Aero) is the primary construction material (nose, body tube, fins, and the flight-computer and recovery bays), with flame-and-heat-rated polycarbonate (PC-FR) reserved for the two structural bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay, motor mount, and gimbal), reducing dry mass by approximately 130–150 g relative to a uniform-material baseline. Aerofoil sections are evaluated in a custom open-return low-speed wind tunnel constructed to the Hofferth (2025) modular design, providing ground-based aerodynamic characterization that is cross-validated against flight telemetry. All flights are conducted under FAA Class 1 (model rocket) provisions, requiring no airworthiness waiver. Pre-flight Monte Carlo and validation simulation predict an apogee near 435 ft (133 m) with a positive thrust-to-weight ratio throughout the burn (2.08 average, 3.66 peak) and a closed-loop pitch deviation under 1° across the modeled atmospheric envelope. All CAD, firmware, simulation code, and flight and ground-test datasets will be released publicly through a version-controlled, openly accessible Git repository and presented in summary form within this paper upon completion of the program.

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

**Research Question 2 — Structural Performance of Zone-Specific 3D-Printable Materials.**
Holding all FDM process parameters constant across flame-retardant polycarbonate (PC-FR) and foamed acrylonitrile styrene acrylate (ASA-Aero), this study characterizes and compares each material's flexural stiffness under three-point bend loading, heat-deflection margin under simulated motor-bay thermal loading, and mass-specific structural performance, with the objective of validating a thermally-zoned material allocation (heat-rated polymer confined to the motor/gimbal bay, lightweight foamed polymer elsewhere) as superior to a single-material airframe on a mass-to-margin basis. Separately, ASA-Aero and standard ABS are both exposed as jetvane material coupons in the motor exhaust to rank flame resistance and erosion (both printed polymers are expected to ablate, motivating a graphite/tungsten vane).

**Research Question 3 — Aerofoil Profile Efficiency for Passive Subsonic Stability.**
Testing the fixed fin's symmetric aerofoil cross-section across incrementally increasing angles of attack in subsonic wind-tunnel flow, lift coefficient, drag coefficient, lift-to-drag ratio, and stall onset angle will be measured and compared against thin-airfoil theory and published low-Reynolds-number reference data, with the objective of confirming that the selected fin geometry provides sufficient restoring moment to maintain the minimum stable static margin through the pre-TVC flight phase without imposing an unnecessary drag or mass penalty.

**Research Question 4 — Wind-Tunnel Calibration Against In-Situ Flight Performance.**
Incrementally adjusting fan speed across a range of test-section velocities in the custom open-return low-speed wind tunnel, the Reynolds-number similarity ratio, blockage-corrected drag coefficient, and cross-sectional flow uniformity will be evaluated at each condition to identify the operating point at which tunnel-measured aerodynamic loading most closely reproduces the loading predicted on the full-scale vehicle during powered ascent, and tunnel-derived drag coefficients will be cross-validated against telemetry-derived drag estimates from flight.

**Research Question 5 — Closed-Loop Control Gain Sensitivity in a Single-Controller TVC Architecture.**
Operating the proportional-integral-derivative thrust-vector control loop on the single flight microcontroller under multiple candidate gain sets (including a step-response-tuned baseline and a simulation-refined gain set validated across modeled atmospheric and gust conditions), peak pitch deviation, gimbal-angle utilization, and settling behavior will be quantified and compared across repeated flights to determine which gain configuration produces the most accurate and best-damped trajectory tracking on a deterministic single-core-equivalent control architecture.

---

## 3. Project Goal

The goal of the WYVERN-E program is to design, fabricate, ground-test, and flight-validate a recoverable, FAA Class 1, 70 mm-diameter prototype rocket demonstrator that serves as an integrated experimental platform for: (i) quantitatively comparing thrust-vector-control actuator classes independent of flight-to-flight variability; (ii) validating a thermally-zoned additive-manufacturing material strategy against airframe mass and structural-margin objectives; (iii) characterizing the aerodynamic performance of the vehicle's passive-stability fin geometry; (iv) validating a custom low-speed wind tunnel against in-situ flight telemetry; and (v) quantifying control-loop gain sensitivity on a consolidated single-controller avionics architecture. Testing is conducted across three complementary tiers — computational simulation, ground-based instrumented testing, and in-situ powered flight — so that each research question is addressed through at least two independent and mutually cross-validating methods.

---

## 4. Vehicle Architecture

### 4.1 Configuration and Mass Budget

The vehicle is a 70 mm outer-diameter, single-stage airframe approximately 0.74 m in length, comprising three pressure/equipment bays (engine/TVC, flight-computer, recovery) separated by two structural bulkheads, with four fixed fins providing passive stability. The recalculated mass budget, derived from the thermally-zoned material allocation described in §4.2 and Research Question 2, is summarized in Table 1.

**Table 1. WYVERN-E mass budget.**

| Section | Material | Key contents | Mass |
|---|---|---|---|
| Nose cone | ASA-Aero | Ellipsoid nose | 21 g |
| Engine/TVC bay | PC-FR | Engine-bay tube, Bulkhead A, gimbal assembly, 2× servo actuators, attitude IMU, motor mount | 268 g |
| Flight-computer bay | ASA-Aero | FC-bay tube, Pico 2 W, attitude IMU, barometric sensors, microSD logger, 2S LiPo + 5 V UBEC, i3 4K Thumb Action Camera camera | 122 g |
| Recovery bay | ASA-Aero | Recovery-bay tube, parachute + 1/8″ Kevlar shock cord, Nomex canopy protector, redundant attitude IMU (vote), **PC-FR** bypass ejection tube, **PC-FR** sealed Bulkhead B, ejection plenum + nose retention | 137 g |
| Structure | ASA-Aero | 4× fixed fins (72 mm), internal wiring | 50 g |
| **Dry mass total** | | | **603 g** |
| Motor (loaded) | — | Estes F15-4, 60 g propellant | 102 g |
| **Liftoff mass total** | | | **705 g** |

Foamed ASA-Aero is used for the primary structure (nose, body tube, fins, and the flight-computer and recovery bays); the heat- and flame-rated PC-FR is reserved for the two structural bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay, motor mount, and gimbal), where either sustained motor-plume heating or the ejection-charge pressure pulse demands it. This all-ASA-except-where-necessary allocation reduces dry mass by approximately 130–150 g relative to an all-PC-FR baseline (specified at 812 g liftoff in an earlier design iteration), directly raising the thrust-to-weight ratio and predicted apogee. Because the lighter ASA nose moves the center of gravity aft, the fin span was increased to 72 mm to preserve the 1.0-caliber static margin without nose ballast (see §4.2).

### 4.2 Stability Architecture: Passive Margin with Active Handoff

Because the closed-loop TVC system cannot be engaged instantaneously at ignition (servo and control-loop settling, combined with the high-amplitude transient of the motor's ignition spike, would risk a destabilizing initial command), the vehicle must be passively stable through approximately the first 0.5 s of flight. Four fixed fins (root chord 70 mm, tip chord 35 mm, leading-edge sweep 25°, span 72 mm) are sized to the minimum conventionally stable static margin of 1.0 caliber at liftoff (1.10 cal; center of gravity at 49.1 cm from the nose, center of pressure at 56.8 cm, by the Barrowman (1967) method), increasing to approximately 1.3 caliber by burnout as propellant mass is consumed. The 72 mm span (larger than the 58 mm of the earlier all-material-mixed layout) compensates for the aft CG shift introduced by the lightweight ASA nose, holding the 1.0-cal margin with no ballast. A parametric apogee-versus-ballast sweep (Table 2) confirmed that adding nose ballast to increase margin is counter-productive: each gram of ballast lowers apogee, so no ballast is carried, and stability is achieved purely through fin sizing at the minimum stable margin.

**Table 2. Apogee sensitivity to nose ballast (RK4 + Barrowman simulation, 1.0 cal margin held constant by fin resizing).**

| Ballast | Fin span | Liftoff mass | Predicted apogee |
|---|---|---|---|
| 0 g (selected) | 72 mm | 705 g | ~435 ft |
| 60 g | 62 mm | 765 g | ~374 ft |
| 150 g | 54 mm | 855 g | ~291 ft |

At t = 0.5 s, with the motor on the smooth, sustained portion of its thrust curve, the TVC controller is enabled and assumes full attitude authority, commanding the gimbal to stabilize the vehicle to vertical and subsequently execute a small commanded pitch maneuver, while the fins continue to provide passive restoring moment as a stability backstop for the remainder of the powered phase.

---

## 5. Proposed Methodology

The program employs a three-tier experimental framework spanning computational simulation, ground-based instrumented testing, and in-situ powered flight. Each research question is addressed through at least two of these tiers to enable cross-validation. All fabrication uses hobbyist-accessible manufacturing methods and commercially available components, consistent with the program's open-source objectives.

### 5.1 Airframe and Materials (Research Question 2)

All primary structural components — body tube, nose cone, fin set, and bulkheads — are fabricated via fused deposition modeling (FDM) on a Bambu Lab X1C printer using the stock 0.4 mm brass nozzle. Print parameters (layer height, infill pattern and density, wall count, nozzle and bed temperature, and part cooling) are held constant within each material class to isolate material as the independent variable, following the parameter-control methodology of Popescu et al. (2018). Two structural candidate materials are evaluated: flame-retardant polycarbonate (PC-FR, ρ ≈ 1.25 g/cm³, heat-deflection temperature ≈ 110 °C, UL94-V0 rated), allocated to the two structural bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay, motor mount, and gimbal), where sustained motor-plume heating or the ejection-gas pulse is expected; and a foamed acrylonitrile styrene acrylate (ASA-Aero, ρ ≈ 0.5–0.7 g/cm³), used for the primary structure — nose, body tube, fins, and the flight-computer and recovery bays — where no significant thermal load is present. Flexural stiffness of both materials is characterized under standardized three-point bend loading following the mechanical-characterization framework of Dizon et al. (2018). (Standard ABS, ρ ≈ 1.04 g/cm³, is not a structural airframe material here; it is printed only as a jetvane material coupon alongside ASA-Aero for the exhaust erosion screen, §5.4.) Engine-bay thermal performance is verified by a first-order lumped-capacitance transient model of the PC-FR motor-bay wall against the F15's 3.45 s burn (with a 0.5 mm phenolic liner as the inner thermal barrier), predicting a peak wall temperature near 47 °C — well below the PC-FR heat-deflection limit — and first-order structural margins on the airframe exceed a safety factor of 300× against the motor's peak axial and TVC-induced bending loads, confirming that the airframe wall thickness is set by printability and handling robustness rather than flight-load requirements.

### 5.2 Fins and Aerofoil Profile (Research Question 3)

The flight fin employs a symmetric aerofoil cross-section sized to deliver the minimum 1.0 caliber static margin identified in §4.2. Reference low-angle-of-attack lift and drag behavior for symmetric sections of this class is benchmarked against the classical thin-airfoil and low-Reynolds-number frameworks of Lissaman (1983) and Mueller and DeLaurier (2003), and against tabulated NACA section data (Abbott & Von Doenhoff, 1959). Wind-tunnel-measured lift coefficient, drag coefficient, lift-to-drag ratio, and stall onset angle for the fabricated fin article are compared directly against these references to confirm that the selected geometry produces adequate restoring moment without excess parasitic drag, and the resulting tunnel-measured aerodynamic coefficients feed the Barrowman center-of-pressure calculation used to confirm static margin in §4.2.

### 5.3 Flight Computer, Sensing, and Control Architecture (Research Questions 1 and 5)

**Consolidated single-controller architecture.** All flight avionics functions — attitude estimation, control-law execution, actuator commanding, data logging, and telemetry — are consolidated onto a single Raspberry Pi Pico 2 W (RP2350: dual-core 150 MHz Arm Cortex-M33, 520 KB SRAM, on-board Wi-Fi/BLE radio), replacing the distributed multi-board avionics architecture used in earlier program iterations. The two processor cores are functionally partitioned to preserve hard real-time determinism: Core 0 executes the 500 Hz thrust-vector-control loop exclusively — reading the gimbal- and body-mounted inertial measurement units, computing nozzle deflection, evaluating the PID control law, and commanding the gimbal servos — and is permitted no blocking operations of any kind. Core 1 drains a logged-data ring buffer to a microSD card over SPI and services an optional Wi-Fi telemetry link for ground-station monitoring, isolating all non-deterministic I/O latency from the control path. This division directly addresses the principal failure mode of single-threaded flight-computer architectures, in which storage or radio I/O can transiently block control-loop execution.

**Power.** The entire avionics domain runs off a light 2S LiPo (7.4 V, ~450 mAh) feeding a single 5 V/6 V UBEC set to 5 V, whose one rail powers the Pico 2 W VSYS, the camera, and both TVC servos (the servos run at 5 V, ~1.8 kg·cm, comfortably above the ~0.9 kg·cm gimbal demand); a separate 6 V servo BEC is not required at this scale. Because the servos and the flight computer share the 5 V rail, the servo and VSYS feeds are star-wired from the UBEC output with bulk and hold-up capacitance (1000 µF at the servos, 100 µF plus an SS34 Schottky at VSYS) so that ~1 A servo-stall transients cannot brown-out the controller. Pack voltage is monitored on GP26/ADC0 (before the BEC) through a 100 kΩ/62 kΩ divider — keeping 2S full-charge (8.4 V) at ~3.21 V, just under the 3.3 V ADC reference — with firmware warning at 6.4 V (3.2 V/cell) and inhibiting arming below 6.0 V (3.0 V/cell). The power-plus-camera group (LiPo ~30 g, UBEC ~10 g, i3 4K Thumb Action Camera ~36 g) totals roughly 76 g, within the 122 g flight-computer-bay allocation. The i3 camera is ~26 g heavier than the thumb-cam originally budgeted; this is carried through the flight numbers (liftoff 705 g, apogee ~435 ft, T/W 2.08/3.66) and, because the camera sits forward of the CG, actually raises the static margin to ~1.10 cal.

**Attitude sensing.** Three nine-axis inertial measurement units (Bosch BNO085) are deployed — one rigidly referenced to the gimbal/nozzle, one to the vehicle body in the flight-computer bay, and a third redundant unit in the recovery bay for two-of-three fault voting — each configured in Game Rotation Vector mode (accelerometer–gyroscope fusion with the magnetometer disabled), because the magnetic field generated by the adjacent gimbal servos would otherwise corrupt a magnetically-referenced heading estimate. Effective nozzle deflection relative to the vehicle body is computed each control cycle as the quaternion difference q_defl = q_body⁻¹ ⊗ q_gimbal, which captures true mechanical nozzle attitude (including any linkage backlash or structural flex) rather than an assumed commanded angle.

**Control law (Research Question 5).** The per-axis control law is a discrete PID controller with integral anti-windup clamping and a first-order low-pass-filtered derivative term, executed at 500 Hz with output clipped to a ±8° gimbal deflection limit (raised from ±5° to give control-authority margin against crosswind weathercocking without adding passive fin stability, which would otherwise reduce the very disturbance the TVC system is built to demonstrate). The flight gain set — Kp = 0.10, Ki = 0.40, Kd = 0.18 — was selected by a phase/gain-margin analysis across 24 operating points (phase margin ≈ 33°, gain margin ≈ 9.3 dB) and independently confirmed by a time-domain robust multi-wind auto-tune; it holds low-wind pitch deviation under roughly 2° with wide gimbal headroom against the ±8° limit, while avoiding the resonance against finite servo lag (≈ 40 ms) observed in higher-proportional-gain configurations. The TVC loop is inhibited for the first 0.5 s of flight (§4.2), after which it engages to stabilize the vehicle to vertical and execute a small commanded maneuver; required gimbal torque is estimated at ≈ 0.9 kg·cm at ±8°, well within the selected servo class, and simulated control authority remains positive throughout the powered phase across the modeled gain sets.

**Actuator comparison (Research Question 1).** Two TVC actuator classes are evaluated using the identical control electronics, gimbal mechanism, and software control law, isolating actuator dynamics as the experimental variable: a tri-solenoid magnetic gimbal actuator, and a servo-driven gimbal actuator. The comparison is conducted entirely on the ground-based three-axis thrust-vector load balance described in §5.4, rather than in flight, to remove flight-to-flight aerodynamic and atmospheric variability from the actuator comparison; the flight vehicle itself carries the servo actuator, selected on the basis of the ground-comparison results.

### 5.4 Ground Test Program (Research Question 1)

The ground test program is structured as the program's primary experimental apparatus rather than a preliminary check, since both the actuator-comparison (Research Question 1) and the motor-characterization data that feed every downstream simulation (Research Question 5; §4) are generated here rather than in flight. Two purpose-built stands are constructed: a three-axis thrust-vector load balance for actuator comparison, and a single-axis static-thrust stand for motor-curve verification and materials erosion screening. Both stands are fully printable in PC-FR (selected per Research Question 2 for its motor-plume thermal margin) and instrumented with strain-gauge load cells and HX711 24-bit bridge-amplifier breakouts, logged to onboard microSD by a dedicated data-acquisition microcontroller independent of the flight avionics.

**Three-axis thrust-vector load balance.** The actuator under test (magnetic-solenoid or servo gimbal) is mounted to a thrust block restrained from a fixed base by three strain-gauge load cells acting through flexures — one axial and two lateral — resolving the complete thrust vector in magnitude and direction:

$$T = \sqrt{F_x^2 + F_y^2 + F_z^2}, \qquad \theta = \arctan\!\left(\frac{\sqrt{F_x^2+F_y^2}}{F_z}\right), \qquad \phi = \operatorname{atan2}(F_y, F_x)$$

The cells are sized to the expected loading envelope of the test motor (Estes F15-0, 25.3 N peak axial thrust, side force at the ±8° gimbal limit ≈ 3.5 N), using a 5 kg axial cell and two 1 kg lateral cells digitized at 80 samples per second. An alternative single-piece design — a cruciform flexure instrumented with one Wheatstone bridge per arm, forming a unified three-axis force/torque sensor — is held as a fallback configuration should the discrete three-cell assembly prove difficult to align. The rig is actuator-agnostic, so the magnetic-versus-servo comparison runs entirely on this one fixture under nominally identical thrust conditions; commanded-versus-measured deflection angle (θ, φ) is logged across a series of step and ramp commands under the F15-0 thrust condition to extract bandwidth, slew rate, step-response overshoot, steady-state error, and maximum sustained deflection for each actuator. Because each F15-0 firing provides a 3.45 s control window, the independent step/ramp command set for each actuator system is built up across multiple firings on this stand.

**Static thrust and materials (jetvane) stand.** A single-axis, load-cell-only stand fitted with a steel blast deflector serves two simultaneous purposes: validating the as-fired thrust curve of every motor class used in the program against its published specification, and holding candidate 3D-printed material coupons directly in the motor exhaust plume to rank flame resistance and surface erosion in support of the Research Question 2 material-zoning conclusions. The deflector and mounting hardware are sized to the F15's 3.45 s burn thermal case. Ground firings use the plugged, 0-delay Estes F15-0 (identical thrust curve to the flight F15-4, but no ejection charge) so that nothing fires into the stand fixtures after burnout.

**Motor plan and firing counts.** Table 3 summarizes the verified motor specifications and the planned firing allocation across flight, both ground stands, and stand commissioning.

**Table 3. Motor plan and verified specifications.**

| Motor | Total impulse | Avg / peak thrust | Burn time | Role |
|---|---|---|---|---|
| Estes F15-4 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Flight only (4 s delay + ejection = recovery system) |
| Estes F15-0 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Ground only — static thrust stand, TVC load balance, jetvane erosion screen (0-delay/plugged; same curve, no ejection into fixtures) |
| Estes/AeroTech E16-4 | — | ~16 N avg | E-class | Stand commissioning and calibration only |

Planned firing counts are 4 Estes F15-4 motors for flight testing, and 13 plugged Estes F15-0 motors for ground testing (6 on the TVC balance at three firings per actuator system, 5 on the static/materials stand for curve verification and jetvane erosion ranking, and 2 held in reserve), plus a minimum of 4 (target 6) Estes/AeroTech E16-4 motors for stand commissioning at two to three firings per stand. The F15-0 and F15-4 share an identical thrust curve (same F15 propellant); the 0-delay F15-0 is used on the ground so no ejection charge fires into the stand fixtures, while the flight motor carries the 4-second delay whose ejection charge is the recovery system.

**Calibration and commissioning sequence.** Each stand's load cells are first calibrated independently of any motor firing, using a series of known hanging dead weights spanning the expected force range, to establish a force-to-voltage transfer function for every channel. Each stand is then commissioned with a minimum of two low-cost E16-4 motor firings before any data-collection firing is conducted, to validate the as-built stand's measured thrust curve against the motor's independently published reference curve and confirm that structural compliance in the stand itself is not corrupting the force measurement. Only after a stand passes this commissioning check are F15-0 data-collection firings conducted on it. All raw and reduced ground-test data (thrust curves, vector-deflection logs, and materials-erosion observations) are archived under a dedicated data directory structure separating motor thrust-curve data from TVC vector/control data, in the same repository used for flight data release (§5.6).

### 5.5 Wind Tunnel (Research Questions 3 and 4)

Ground-based aerodynamic characterization is treated as a primary data source for the program, not a secondary check on flight results, since it is the only test environment in which angle of attack, freestream velocity, and Reynolds number can each be controlled and varied independently — none of which is possible during a single powered flight. A custom open-return, low-speed wind tunnel is therefore constructed in parallel with the flight vehicle, following the modular configuration described by Hofferth (2025), comprising a bell-mouth inlet, a honeycomb-and-screen flow-conditioning section, a converging contraction section sized per the area-ratio and length criteria of Bell and Mehta (1988), an optically accessible test section, and a diffuser leading to a variable-speed fan.

**Fan selection and trade study.** Because Research Questions 3 and 4 require accurate aerodynamic force measurement rather than maximum achievable airspeed, fan selection is driven primarily by test-section velocity stability and acoustic/vibration noise floor rather than peak flow rate. Two fan classes are traded: a self-contained electronically-commutated 8-inch axial fan with an integrated ten-speed controller, paired with a diffuser-and-fan upgrade to the baseline Hofferth tunnel geometry, and a compact, high-static-pressure 120 mm centrifugal fan adapted to the tunnel inlet through a custom-printed collar, driven by a separate pulse-width-modulated 12 V supply. The first option is selected as the baseline configuration for force-measurement testing on the basis of its purpose-built diffuser pairing and lower acoustic noise at the tunnel's typical operating point; the second is retained as a compact backup path. The complete bill of materials, flow-conditioning screen specification, and full 3D-printed part manifest for the tunnel are maintained in the project's ground-support-equipment documentation and released through the program's public repository (§5.6).

**Instrumentation and calibration.** Prior to aerodynamic data collection, the tunnel is characterized across its full operating range using a traversed Pitot-static probe to map test-section freestream velocity and spatial uniformity (target: under 2% RMS variation in the core flow region), following the design and verification practices of Mehta and Bradshaw (1979) and Pope and Harper (1966). Blockage corrections are applied to all measured drag coefficients using the Maskell (1963) bluff-body correction method, and the fan operating point that produces a Reynolds-number similarity ratio matching predicted flight conditions is identified and used as the calibrated test condition for all fin-article testing.

**Fin-article force testing.** The fabricated fin article is mounted on a deflection-indexed single-fin test mount — either on a strut/sting base or a sidewall half-span mount — permitting controlled 0.5° angle-of-attack increments without remounting the article between runs. Lift and drag forces are recorded at equal angle-of-attack increments from 0° through stall, with a minimum of three repeated runs per angle to assess measurement repeatability; resulting coefficients are validated against the tabulated NACA reference data of Selig (2003) and Selig, Donovan, and Fraser (1989) prior to use in the stability calculation of §4.2.

**Computational cross-validation.** In parallel with the physical tunnel campaign, a two-dimensional constant-strength vortex-panel method (following Kuethe and Chow, as implemented for each candidate fin cross-section: NACA 0006, NACA 0012, a symmetric double-wedge, and a flat-plate baseline) generates an independent, purely computational lift-curve and surface-pressure-distribution prediction for every candidate profile, swept through the same 0.5° deflection increments at both the wind-tunnel Reynolds number (≈2×10⁵) and the predicted flight Reynolds number (≈3.4×10⁵). This panel-method baseline is validated internally against thin-airfoil theory (it recovers a lift-curve slope within approximately 10% of the inviscid 2π-per-radian ideal for the NACA 0012 section) before being used as a pre-test prediction against which the physical tunnel measurements are compared; because the panel method is inviscid, it is expected to reproduce lift coefficient and pressure distribution accurately but to systematically miss viscous drag, separation, and stall onset, so the physical tunnel campaign remains the authoritative source for drag coefficient and stall angle while the panel-method results serve as an independent check on the lift-curve slope and as a pre-test prediction for instrumentation sizing. All simulated aerodynamic, trajectory, and control-loop datasets generated during the program — the panel-method polars described here; the RK4-plus-Barrowman trajectory and dispersion simulations of §4.2; the gate-based flight-validation suite and Monte Carlo dispersion runs referenced in §5.6 and §8; and the atmospheric-sweep control-loop simulations of §5.3 — are version-controlled alongside the physical test data in the program's public repository, with each simulation script paired against the dataset it produced, so that every plotted or tabulated simulated result in this paper and its supporting materials can be regenerated and independently checked against the as-built hardware.

### 5.6 Flight Test Plan, Logging, and Data Sharing (Research Question 5)

Each flight is conducted in compliance with the National Association of Rocketry (2023) safety code and motor classification standards, under the supervision of a NAR-certified or similarly qualified range safety officer. Multiple flights are conducted with the control law configured under each candidate gain set identified in §5.3, and post-flight telemetry recovered from the onboard microSD log is analyzed for peak pitch deviation during the powered phase, gimbal-angle utilization, and qualitative settling/overshoot behavior, to determine which gain configuration produces the most accurate and best-damped attitude tracking. All onboard sensor data (full-rate inertial, barometric, and control-loop telemetry) is logged at the full control-loop sample rate; where the bench/range Wi-Fi telemetry link is in range, a parallel live feed is monitored for real-time anomaly detection, though the onboard log remains the data of record. Upon conclusion of the program, all CAD files, firmware source code, simulation scripts, and the complete flight and ground-test datasets are released through a version-controlled, publicly accessible Git repository, and the principal reduced results from each research question are additionally presented in summary form within this paper.

---

## 6. Recovery System

The vehicle's passive fin stability is retained through the coast phase to apogee (predicted apogee ≈ 435 ft at t ≈ 6.81 s). Recovery is initiated not by an independent electronic altimeter but by the flight motor's own factory ejection charge: the flight configuration uses an Estes F15-4 (a four-second ejection delay) in place of the previously plugged F15-0, so that approximately four seconds after propellant burnout — t ≈ 7.45 s, roughly 0.64 s past the predicted apogee — the motor's integral ejection charge fires. Because the flight-computer bay is sealed gas-tight between the two structural bulkheads, the hot ejection gas is not permitted to vent through the avionics; it is instead routed through a dedicated solid-walled bypass tube (12 mm internal diameter, flame-retardant polycarbonate) running from an ejection plenum at the motor-side bulkhead (Bulkhead A), alongside the sealed flight-computer bay, to the recovery bay above Bulkhead B, where it pressurizes the bay and releases a friction-fit nose cone carrying the parachute.

A first-order feasibility analysis (`Simulations/we4_ejection_feasibility.py`) supports the approach on two independent grounds. First, the bypass tube imposes a negligible flow penalty: the pressure loss across the 12 mm bore at the ejection mass-flow is on the order of 0.06 kPa. Second, the recovery bay pressurizes to approximately 140 kPa against a friction-fit nose-release threshold of 14–41 kPa, a pressurization margin of roughly 3.4×. The F15-4's four-second delay is the closest available Estes delay to the coast-to-apogee optimum (≈ 3.5 s); the longer F15-6 and F15-8 delays are rejected because they fire approximately 2.5 s and 4.5 s past apogee, deploying at high descent speed and, in the F15-8 case, at dangerously low altitude. This motor-ejection architecture eliminates the independent RRC3+ recovery computer, its isolated 9 V battery, the e-match and black-powder charge well, and the associated recovery wiring of the earlier design — reducing parts count, cost, and an entire electronic failure domain — at the cost of a single passive deployment event with no electronic backup channel, a trade justified by the 3.4× pressurization margin and by the finned airframe's aerodynamic stability through apogee. Shock-cord and parachute sizing (1/8″ tubular Kevlar cord, 18″ ripstop nylon canopy, with a Nomex blanket shielding the canopy from the ejection gas) are verified against the worst-case deployment scenario — ejection ≈ 0.64 s past apogee while the vehicle still carries ≈ 4.7 m/s of vertical velocity — yielding a structural safety factor exceeding 800× on the recovery harness and a predicted terminal descent rate near 6 m/s. The recovery-bay bulkhead (Bulkhead B) and bypass tube are checked against the ≈ 140 kPa ejection pressure in the structural analysis (`WYVERN_E4_FEA_Structural.md` §4), returning safety factors of ≈ 8× and ≈ 107× respectively.

---

## 7. Safety and Regulatory Compliance

All flights use a single Estes F15-4 motor (49.6 N·s total impulse, 60 g propellant, F-class, four-second ejection delay), and the fully loaded liftoff mass of 705 g is well under the FAA's Class 1 (model rocket) threshold of 1,500 g loaded weight per motor, requiring no Federal Aviation Administration airworthiness waiver and no NAR/Tripoli high-power certification. Range procedures include remote ignition, a minimum 3 m personnel standoff from both ground-test stands during firing, a fail-safe neutral-gimbal default state on any control-system fault, and standard model-rocketry motor-handling discipline for the motor's integral ejection charge — the igniter is installed last, at the pad, and there are no independent pyrotechnic or electronic ejection circuits in the vehicle to arm or inhibit (recovery is effected solely by the motor's own delay/ejection charge).

---

## 8. Expected Outcomes

The program is expected to produce a quantitative, ground-validated comparison of magnetic-solenoid and servo thrust-vector-control actuators — including bandwidth, slew rate, overshoot, and steady-state error for each — directly informing actuator selection for future closed-loop rocketry programs without requiring a dedicated in-flight A/B comparison. A validated thermally-zoned additive-manufacturing material allocation is expected to demonstrate a measurable dry-mass reduction (approximately 100–150 g, or roughly 15–20% of dry mass) relative to a uniform heat-rated-material baseline, with no corresponding loss of structural margin, offering a transferable design pattern for hobbyist and academic rocketry programs using FDM fabrication. Wind-tunnel-derived aerodynamic coefficients for the flight fin geometry, cross-validated against flight telemetry, will be released as an open dataset characterizing the program's custom low-speed tunnel as a viable, low-cost ground-test instrument for the broader rocketry and STEM-education community. Finally, flight-validated comparison of control-loop gain configurations on a consolidated dual-core single-controller architecture is expected to clarify the practical control-authority and determinism benefits of separating real-time control execution from logging and telemetry I/O on a single low-cost microcontroller, relative to either a single-threaded controller or a distributed multi-board avionics stack. All resulting design files, firmware, simulation code, and flight datasets will be released publicly through the National Association of Rocketry in support of the program's open-source objectives.

---

## References

Abbott, I. H., & Von Doenhoff, A. E. (1959). *Theory of wing sections: Including a summary of airfoil data.* Dover Publications.

Barrowman, J. S. (1967). *The practical calculation of the aerodynamic characteristics of slender finned vehicles* (NASA NTRS accession 20010047838). https://ntrs.nasa.gov/citations/20010047838

Bell, J. H., & Mehta, R. D. (1988). *Contraction design for small low-speed wind tunnels* (NASA CR-182747). https://ntrs.nasa.gov/api/citations/19880012661/downloads/19880012661.pdf

Bosch Sensortec. (n.d.). *BNO085 9-axis absolute orientation IMU — datasheet.*

BPS.space. (n.d.). *Thrust vector control.* Retrieved from https://bps.space/products/thrust-vector-control

Dizon, J. R. C., Espera, A. H., Chen, Q., & Advincula, R. C. (2018). Mechanical characterization of 3D-printed polymers. *Additive Manufacturing, 20,* 44–67. https://doi.org/10.1016/j.addma.2017.12.002

Hofferth, J. (2025). Modular wind tunnel for STEM education. *AIAA SciTech 2025 Forum.* https://doi.org/10.2514/6.2025-2630

Kuethe, A. M., & Chow, C.-Y. (1998). *Foundations of aerodynamics: Bases of aerodynamic design* (5th ed.). Wiley.

Lissaman, P. B. S. (1983). Low-Reynolds-number airfoils. *Annual Review of Fluid Mechanics, 15,* 223–239.

Maskell, E. C. (1963). *A theory of the blockage effects on bluff bodies and stalled wings in a closed wind tunnel* (ARC R&M 3400). Aeronautical Research Council.

Mehta, R. D., & Bradshaw, P. (1979). Design rules for small low speed wind tunnels. *Aeronautical Journal, 83*(827), 443–449. https://doi.org/10.1017/S0001924000031985

Mueller, T. J., & DeLaurier, J. D. (2003). Aerodynamics of small vehicles. *Annual Review of Fluid Mechanics, 35,* 89–111. https://doi.org/10.1146/annurev.fluid.35.101101.161102

National Advisory Committee for Aeronautics. (1951). *Aerodynamic characteristics of NACA 0012 airfoil section at angles of attack from 0° to 180°* (NACA TN 2502).

National Association of Rocketry. (2023). *NAR safety code and motor classification standards.* https://www.nar.org/safety-information/

NASA. (1968). *Thrust-vector control requirements for solid-propellant launch vehicles* (NASA TN D-4971).

OpenRocket Project. (2023). *OpenRocket technical documentation v23.09.* https://openrocket.info/documentation.html

Pérez Gordillo, A., Simplício, P., Iannelli, A., & Marcos, A. (2023). Thrust vector control and state estimation architecture for low-cost small-scale launchers. *arXiv.* https://arxiv.org/pdf/2303.16983

Popescu, D., Zapciu, A., Amza, C., Baciu, F., & Marinescu, R. (2018). FDM process parameters influence over the mechanical properties of polymer specimens. *Polymer Testing, 69,* 157–166. https://doi.org/10.1016/j.polymertesting.2018.05.020

Pope, A., & Harper, J. J. (1966). *Low-speed wind tunnel testing.* Wiley.

Raspberry Pi Foundation. (2024). *RP2350 datasheet.*

Sahoo, S. (2026, April 11). WYVERN PTD Portal. Skylight Industries. https://wyvern-e.base44.app/

Selig, M. S. (2003). *UIUC airfoil data site.* University of Illinois at Urbana-Champaign. https://m-selig.ae.illinois.edu/ads.html

Selig, M. S., Donovan, J. F., & Fraser, D. B. (1989). *Airfoils at low speeds* (Soartech 8). SoarTech Publications.

Ziegler, J. G., & Nichols, N. B. (1942). Optimum settings for automatic controllers. *Transactions of the ASME, 64,* 759–768.

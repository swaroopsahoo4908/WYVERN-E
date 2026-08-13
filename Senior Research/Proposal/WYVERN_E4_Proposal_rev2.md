# GTR70E WYVERN (Widespread Yield Variable Engagement Rapid-response Neutralizer): Engineering Design and Experimental Validation of Closed-Loop Thrust-Vector Control, Hybrid Passive–Active Stability, and Zoned Additive-Manufacturing Materials in a Subscale Single Stage Solid-Fuelled Prototype Rocket Demonstrator

## Student Information

**Student Names:** Swaroop Sahoo, Chris Liu, Allison Hong

**Project Title:** GTR70E WYVERN (Widespread Yield Variable Engagement Rapid-response Neutralizer): Engineering Design and Experimental Validation of Closed-Loop Thrust-Vector Control, Hybrid Passive–Active Stability, and Zoned Additive-Manufacturing Materials in a Subscale Single Stage Solid-Fuelled Prototype Rocket Demonstrator

**Research Pathway:** Engineering/Innovation Design

**Intended Majors:**

- *Swaroop Sahoo*: Electrical Engineering with Aerospace Engineering Minor
- *Chris Liu*: Aerospace Engineering with Business Minor
- *Allison Hong*: Mechanical Engineering with Aerospace Engineering Minor

The airframe uses a three-way material zoning: ASA-Aero for the upper body that houses avionics
(nose, upper body tube), PETG-CF for the lower body and fins, and PC-FR for the TVC assembly (motor
mount, gimbal). This zoning holds static margin above the program's 1.0-caliber pre-TVC passive
stability floor with 87 mm fins (CG 50.1 cm, CP 59.3 cm, margin 1.31 cal), independently derived from
the component mass-moment stack in `we4_sim.py` and cross-checked against `we4_flightsim.py`'s
RK4+Barrowman output. Liftoff mass is 698 g, dry mass 638 g, T/W 2.10 average / 3.70 peak, and
predicted apogee ≈439 ft (133.7 m) at 6.87 s. See `WYVERN_E4_Camera_Solution.md` for the camera
mass allocation. The jetvane test in RQ2 is a flat-coupon blast-shield melt-through screen; see RQ2
and §5.4 below.

---

## Abstract

The GTR70E WYVERN vehicle is a 70 mm-diameter, single-stage, solid-fuelled, additively-manufactured prototype rocket demonstrator developed to provide quantitative, ground-based and flight-validated data on closed-loop thrust-vector control (TVC) actuation, hybrid passive/active stability architectures, and zone-specific additive-manufacturing material selection. The vehicle is powered by a single Estes F15-4 solid motor and relies on four fixed fins sized to the minimum statically-stable margin (1.0 caliber) to survive launch rail departure and the motor's ignition transient, after which authority is transferred to a closed-loop proportional–integral–derivative (PID) thrust-vector-control system operating on the smooth portion of the thrust curve. Two candidate TVC actuation schemes, a tri-solenoid magnetic gimbal and a servo-actuated gimbal, are characterized and directly compared on two physically separate but instrumentally identical three-axis thrust-vector load balances prior to flight, isolating actuator dynamics (bandwidth, slew rate, overshoot, steady-state error, and maximum achievable deflection angle) from flight-to-flight variability. Fin aerofoil selection and stability-model calibration draw on a dedicated bench wind tunnel and companion 2D vortex-panel CFD solver, cross-checked against flight telemetry, and the zoned-materials study additionally screens jetvane filament performance under real motor-plume conditions on the static-fire stand. All flight avionics, sensor fusion, and control-law execution are consolidated onto a single custom flight-computer PCB (PCB1, a circular Ø62 mm 2-layer board built around a bare RP2350B QFN-80 die, dual-core), with one core dedicated exclusively to deterministic 500 Hz control execution and the second core handling data logging to onboard microSD, eliminating the blocking-I/O risk inherent to single-threaded flight computer architectures. PCB1 carries no onboard radio (no WiFi/BLE chip populated), so flight telemetry is logged rather than streamed; bench and ground-test telemetry runs over a separate Wi-Fi-capable Pico on the servo/solenoid rigs. Structural members are fabricated by fused deposition modeling (FDM) using a three-way thermally-zoned material strategy: ASA-Aero (foamed, ρ ≈ 0.65 g/cm³) forms the upper body tube that houses avionics (nose, flight-computer bay), where there is neither motor heat nor ejection gas; carbon-filled PETG (PETG-CF, ρ ≈ 1.30 g/cm³, HDT ≈ 80 °C) forms the lower body, the bulkhead joint, and the fins, which carry the ejection-gas path and the fin aerodynamic load; and fire-retardant polycarbonate (PC-FR, ρ ≈ 1.20 g/cm³) forms the TVC assembly proper (motor mount and gimbal), the zone closest to sustained motor-plume heating. The zoning is driven by thermal exposure and structural role rather than mass alone, though the foamed ASA-Aero upper body is also the largest single mass saving in the stack. Fin aerodynamics are characterized analytically by the Barrowman (1967) normal-force/center-of-pressure method embedded in the program's RK4 trajectory suite and validated against recovered flight telemetry, from which an in-situ drag coefficient and static margin are reconstructed. All flights are conducted under FAA Class 1 (model rocket) provisions, requiring no airworthiness waiver. Pre-flight Monte Carlo and validation simulation predict an apogee near 439 ft (133.7 m) with a positive thrust-to-weight ratio throughout the burn (2.10 average, 3.70 peak) and a closed-loop pitch deviation under 2.5° across the modeled atmospheric envelope. All CAD, firmware, simulation code, and flight and ground-test datasets will be released publicly through a version-controlled, openly accessible Git repository and presented in summary form within this paper upon completion of the program.

---

## 1. Introduction

### 1.1 Background and Motivation

Active flight control in small-scale, hobbyist-accessible rocketry sits at the intersection of aerodynamics, embedded real-time systems, additive-manufacturing materials science, and classical control theory, and progress on any one axis is frequently constrained by the others. Thrust-vector control, gimbaling the rocket motor's nozzle or exhaust path to produce a control moment without dedicated aerodynamic control surfaces, is the actuation method used on essentially every orbital launch vehicle, yet open, reproducible, and quantitatively validated implementations at the sounding-rocket scale remain comparatively rare in the amateur and academic literature. Existing low-cost demonstrations of vector control (e.g., BPS.space's *Signal*/*Echo* flight series) establish feasibility but do not typically isolate and quantify actuator-class performance independent of flight-to-flight aerodynamic and atmospheric variability.

GTR70E WYVERN addresses this gap with a deliberately simplified architecture relative to prior program iterations: a single fixed-fin airframe that is passively stable through the motor's ignition transient, a single bare-metal flight controller executing the entire sensing-and-control pipeline, and, critically, an actuator comparison that is moved off the flight vehicle and onto a repeatable ground-test apparatus. This restructuring is intended to produce a controlled, statistically tractable dataset on actuator performance (the central engineering question of the program) while retaining a flight-validation phase that demonstrates the complete closed-loop system performs as designed under real atmospheric and motor-burn conditions.

### 1.2 Program Lineage

The GTR70E WYVERN line has progressed through four major design iterations. GTR70E WYVERN 1.0 was a 70 mm, two-stage airframe with a custom flight computer, magnetic-solenoid thrust-vector-control actuation, and actively controlled fins on both stages; 2.0 introduced an 84 mm two-stage airframe with custom avionics; 3.0 implemented a Raspberry Pi 5–based flight computer and flew a magnetic-solenoid-versus-servo actuator A/B comparison in flight. Each iteration established components of the aerodynamic, structural, and avionics groundwork carried forward into the current design. The current iteration consolidates these lessons into the simplest vehicle configuration that still answers the program's central control-authority question, replacing the in-flight actuator A/B test with a ground-based load-balance comparison and replacing the distributed avionics stack with a single dual-core microcontroller.

### 1.3 Central Hypothesis

A small fixed-fin, single-stage, solid-fuelled prototype rocket demonstrator can rely on passive aerodynamic stability to survive launch-rail departure and the motor's ignition transient, then transfer control authority to a closed-loop thrust-vector-control system once the thrust curve smooths; and the relative performance of competing TVC actuator classes (magnetic-solenoid versus servo) can be characterized quantitatively, and more economically, on a ground-based three-axis load balance prior to committing either actuator to flight.

---

## 2. Research Questions

The program addresses five research questions spanning actuator selection, materials, aerofoil
performance, stability-model calibration, and control-loop tuning.

**Research Question 1, Magnetic-Solenoid versus Servo Actuation for Thrust-Vector Control Authority.**
Substituting a tri-solenoid magnetic gimbal actuator for a servo-driven gimbal actuator as the thrust-vector-control mechanism, evaluated under identical commanded-deflection inputs on two physically separate three-axis thrust-vector load balances (one per actuator class, sharing an identical flexure/DAQ chain so the two datasets are directly comparable), will produce measurable differences in actuation bandwidth, slew rate, step-response overshoot, steady-state deflection error, and maximum achievable gimbal angle, with the objective of determining which actuator class provides superior control authority for a given motor thrust regime.

**Research Question 2, Zoned Airframe Materials, and Jetvane Blast-Shield Screen.**
Holding all FDM process parameters constant across ASA-Aero, carbon-filled PETG (PETG-CF), and fire-retardant polycarbonate (PC-FR), this study characterizes and compares each material's flexural stiffness and modulus under three-point bend loading, and its heat-deflection margin against the measured engine-bay wall temperature from the static-fire campaign, with the objective of justifying the zoned allocation actually flown: ASA-Aero for the unheated avionics-housing upper body, PETG-CF for the lower body and fins, PC-FR for the TVC assembly. The question the comparison answers is whether PC-FR's fire-retardant grade and PETG-CF's stiffness are necessary in those zones, or whether a lighter single-material airframe would have sufficed, a decision every low-cost printed TVC vehicle has to make and few justify with data. A second, materials-focused thread under this same research question, run as a blast-shield screen rather than a mounted-vane test, fires the motor directly into a flat 5 mm, 100%-infill coupon plate of each of six candidate materials (PLA, PETG-CF, ABS, ASA-Aero, PC, and PC-FR), measuring melt-through, ablation depth, and slag buildup; materials that survive 5 mm are retested at 4, 3, and 2 mm until they fail, producing a failure-thickness ranking across all six. This is a materials data point, not a flight-hardware decision, since only the servo TVC system flies.

**Research Question 3, Fin Aerofoil Selection via Wind-Tunnel-Measured Performance.**
A bench wind tunnel directly measures lift, drag, and stall behavior for the candidate fin aerofoil sections across the flown fin's angle-of-attack range, complementing the inviscid 2D vortex-panel CFD solver (`Simulations/CFD/`) that predicts the same lift curves and surface-pressure distributions analytically. The objective is to select the fin aerofoil section on measured, not purely modeled, performance, and to characterize where the inviscid panel-method prediction diverges from measured behavior, the panel method captures circulation-driven lift but not viscous separation, stall onset, or pressure drag, which only the tunnel run can supply.

**Research Question 4, Wind-Tunnel-versus-Flight Calibration of Passive Stability.**
Sizing the four fixed fins to the minimum conventionally stable static margin by the Barrowman (1967) normal-force and center-of-pressure method, the predicted static margin, zero-lift drag coefficient, and weathercock response are compared against two independent measurements: the RQ3 tunnel-measured aerofoil coefficients, and the same quantities reconstructed from recovered flight telemetry, static margin from the observed pitch-rate response to the measured crosswind, and drag coefficient from the coast-phase deceleration between burnout and apogee. The objective is to establish how accurately a Barrowman-class analytical model, cross-checked against both a physical wind-tunnel measurement and flight telemetry, predicts the as-built vehicle's passive aerodynamics, and to confirm that the selected fin geometry holds the minimum stable margin through the pre-TVC phase without an unnecessary drag or mass penalty.

**Research Question 5, Closed-Loop Control Gain Sensitivity in a Single-Controller TVC Architecture.**
Operating the proportional-integral-derivative thrust-vector control loop on the single flight microcontroller under multiple candidate gain sets (including a step-response-tuned baseline and a simulation-refined gain set validated across modeled atmospheric and gust conditions), peak pitch deviation, gimbal-angle utilization, and settling behavior will be quantified and compared across repeated flights to determine which gain configuration produces the most accurate and best-damped trajectory tracking on a deterministic single-core-equivalent control architecture.

---

## 3. Project Goal

The goal of the GTR70E WYVERN program is to design, fabricate, ground-test, and flight-validate a recoverable, FAA Class 1, 70 mm-diameter prototype rocket demonstrator that serves as an integrated experimental platform for: (i) quantitatively comparing thrust-vector-control actuator classes independent of flight-to-flight variability; (ii) validating a thermally-zoned additive-manufacturing material strategy, including jetvane filament performance under real motor plume conditions, against airframe mass and structural-margin objectives; (iii) selecting a fin aerofoil section on wind-tunnel-measured performance; (iv) establishing the predictive accuracy of a Barrowman-class analytical stability model against both wind-tunnel measurement and the as-built vehicle's in-situ passive aerodynamics; and (v) quantifying control-loop gain sensitivity on a consolidated single-controller avionics architecture. Testing is conducted across three complementary tiers, computational simulation, ground-based instrumented testing, and in-situ powered flight, so that each research question is addressed through at least two independent and mutually cross-validating methods, as mapped in Table 0.

**Table 0. Research-question to method mapping (two independent methods per question).**

| RQ | Method A | Method B |
|---|---|---|
| 1, TVC actuator class | Servo TVC stand + magnetic TVC stand, F15-0 firings, identical flexure/DAQ chain (§5.4) | Closed-loop SIL + bench-model actuator dynamics, `wyvern_datagen/bench_sim.py` (§5.5) |
| 2, zoned airframe materials, jetvane blast-shield screen | Three-point bend coupons, all three flown materials, identical print parameters (§5.1) | Lumped-capacitance thermal model + jetvane blast-shield melt-through rig on the static-fire stand, checked against measured engine-bay wall temperature (§5.1, §5.4) |
| 3, Fin aerofoil selection | Wind-tunnel lift/drag/stall measurement (§5.2, §5.4) | 2D vortex-panel CFD, `Simulations/CFD/run_airfoil_cfd.py` (§5.2, §5.5) |
| 4, Tunnel-vs-flight stability calibration | Barrowman CP/CN + RK4 trajectory and Monte Carlo dispersion (§4.2, §5.2) | Wind-tunnel coefficients (§5.2, §5.4) and flight-telemetry reconstruction of static margin and coast-phase drag (§5.2, §5.6) |
| 5, Control gain sensitivity | 24-point phase/gain-margin sweep + atmospheric Monte Carlo (§5.3) | Repeated flights under each candidate gain set, onboard log analysis (§5.6) |

---

## 4. Vehicle Architecture

### 4.1 Configuration and Mass Budget

The vehicle is a 70 mm outer-diameter, single-stage airframe approximately 0.74 m in length, comprising two body tubes (Upper BT, Lower BT) joined at a single bulkhead, with four fixed fins providing passive stability. The recalculated mass budget, derived from the thermally-zoned material allocation described in §4.2 and Research Question 2, is summarized in Table 1.

**Table 1. GTR70E WYVERN mass budget** (structural rows are real CAD output, not scaled estimates).

| Section | Material | Key contents | Mass |
|---|---|---|---|
| Nose cone | ASA-Aero | Ellipsoid nose | 20.9 g |
| Upper BT | ASA-Aero tube, incl. Ø62 mm PCB1 standoffs | Nose-adjacent recovery wadding, camera, flight computer (custom RP2350B PCB1), body IMU, barometric sensor, microSD logger, 2S LiPo (buck-regulated on PCB1, no discrete UBEC) | 44.9 g structure + 89.5 g avionics |
| Lower BT | **ASA-Aero** tube, **PC-FR** gimbal + motor mount | Chute + 1/8″ Kevlar shock cord, Nomex canopy protector, wadding, TVC bay (gimbal assembly, 2× servo actuators, external attitude IMU at the bulkhead boundary, motor mount) | 94.2 g tube + 57.7 g mount + 105.6 g gimbal + 70 g recovery |
| Bulkhead joint | PETG-CF | Single separation joint, wiring pass-throughs for servo extensions + STEMMA-QT cable | 17.2 g |
| Structure | PETG-CF | 4× fixed fins (87 mm), rail buttons, internal wiring | 70.8 g fins + 1.2 g buttons + 8 g wiring |
| Actuation | — | 2× EMAX ES08MA II servo, 12 g each | 24 g |
| **Dry mass total** | | | **638 g** (596 g airframe + 42 g spent motor casing) |
| Motor (loaded) | — | Estes F15-4, 60 g propellant | 102 g |
| **Liftoff mass total** | | | **698 g** |

The Lower BT tube's ASA-Aero wall is hoop-stress-checked against the 140 kPa ejection pulse
(σ ≈ 2.93 MPa, SF ≈ 6–10×); the bulkhead (direct gas-exposure part) uses PETG-CF and the motor
mount/gimbal (thermal duty at the nozzle) uses PC-FR. The custom PCB1 assembly mass is a
component-level self-estimate, not a bench measurement.

Foamed ASA-Aero forms the upper body tube (nose, flight-computer bay, avionics), the section that houses avionics and sees no motor heat and no direct ejection-gas exposure; PETG-CF forms the lower body, the bulkhead joint, and the fins, which carry the ejection-gas path and the fin aerodynamic load; and fire-retardant PC-FR forms the TVC assembly proper (motor mount and gimbal), the zone closest to sustained motor-plume heating. This three-way allocation reduces dry mass relative to an all-PETG-CF baseline (specified at 812 g liftoff in an earlier design iteration) while still meeting each zone's thermal and structural requirement, directly raising the thrust-to-weight ratio and predicted apogee. Because the lighter ASA-Aero upper body moves the center of gravity aft while the heavier PETG-CF fins do the same, the fin span was increased from 72 to 87 mm to preserve the 1.0-caliber static margin without nose ballast (see §4.2).

### 4.2 Stability Architecture: Passive Margin with Active Handoff

Because the closed-loop TVC system cannot be engaged instantaneously at ignition (servo and control-loop settling, combined with the high-amplitude transient of the motor's ignition spike, would risk a destabilizing initial command), the vehicle must be passively stable through approximately the first 0.5 s of flight. Four fixed fins (root chord 70 mm, tip chord 35 mm, leading-edge sweep 25°, span 87 mm) are sized to the minimum conventionally stable static margin of 1.0 caliber at liftoff (1.31 cal; center of gravity at 50.1 cm from the nose, center of pressure at 59.3 cm, by the Barrowman (1967) method), increasing to approximately 1.5 caliber by burnout as propellant mass is consumed. The 87 mm span (larger than the 58 mm of the earlier all-material-mixed layout) compensates for the aft CG shift introduced by the lightweight ASA nose, holding the 1.0-cal margin with no ballast. A parametric apogee-versus-ballast sweep (Table 2) confirmed that adding nose ballast to increase margin is counter-productive: each gram of ballast lowers apogee, so no ballast is carried, and stability is achieved purely through fin sizing at the minimum stable margin.

**Table 2. Apogee sensitivity to nose ballast (RK4 + Barrowman simulation, 1.0 cal margin held constant by fin resizing).**

| Ballast | Fin span (1.0 cal) | Liftoff mass | Predicted apogee |
|---|---|---|---|
| 0 g (selected span basis) | 76.2 mm | 720 g | ~409 ft |
| 60 g | 59.4 mm | 767 g | ~352 ft |
| 150 g | 46.9 mm | 846 g | ~271 ft |

This sweep holds margin at exactly 1.0 cal; the flown vehicle carries a small additional buffer at
1.31 cal (87 mm fins, 698 g liftoff, ~439 ft) rather than the bare 1.0-cal minimum shown above.

At t = 0.5 s, with the motor on the smooth, sustained portion of its thrust curve, the TVC controller is enabled and assumes full attitude authority, commanding the gimbal to stabilize the vehicle to vertical and subsequently execute a small commanded pitch maneuver, while the fins continue to provide passive restoring moment as a stability backstop for the remainder of the powered phase.

---

## 5. Proposed Methodology

The program employs a three-tier experimental framework spanning computational simulation, ground-based instrumented testing, and in-situ powered flight. Each research question is addressed through at least two of these tiers to enable cross-validation. All fabrication uses hobbyist-accessible manufacturing methods and commercially available components, consistent with the program's open-source objectives.

### 5.1 Airframe and Materials (Research Question 2)

All primary structural components, body tube, nose cone, fin set, and bulkheads, are fabricated via fused deposition modeling (FDM) on a Bambu Lab X1C printer using the stock 0.4 mm brass nozzle. Print parameters (layer height, infill pattern and density, wall count, nozzle and bed temperature, and part cooling) are held constant within each material class to isolate material as the independent variable, following the parameter-control methodology of Popescu et al. (2018). Three structural candidate materials are evaluated: fire-retardant polycarbonate (PC-FR, ρ ≈ 1.20 g/cm³), allocated to the TVC assembly (motor mount and gimbal), the zone closest to sustained motor-plume heating; carbon-filled PETG (PETG-CF, ρ ≈ 1.30 g/cm³, heat-deflection temperature ≈ 80 °C), allocated to the lower body and fins, which carry the ejection-gas path (the bulkhead joint) and the fin aerodynamic load; and ASA-Aero (foamed, ρ ≈ 0.65 g/cm³) for the upper body tube, nose, and flight-computer bay, where neither motor heat nor ejection gas is present. Flexural stiffness and modulus of all three materials are characterized under standardized three-point bend loading following the mechanical-characterization framework of Dizon et al. (2018). The allocation is driven by thermal exposure and structural role: ASA-Aero is not survivable in contact with the motor's ejection gas or sustained plume heat, PETG-CF handles the gas path and fin loads, and PC-FR is reserved for the zone with the highest sustained thermal exposure. Engine-bay thermal performance is verified by a first-order lumped-capacitance transient model of the PETG-CF motor-bay wall against the F15's 3.45 s burn, predicting a peak wall temperature near 40 °C against the 80 °C heat-deflection limit, and is cross-checked in this program against a thermocouple reading taken on the wall during the static-fire campaign. First-order structural margins on the airframe exceed a safety factor of 300× against the motor's peak axial and TVC-induced bending loads, confirming that wall thickness is set by printability and handling robustness rather than by flight loads, which is why the ASA-Aero wall could be reduced from 1.6 mm to 1.2 mm, recovering 45.6 g.

### 5.2 Fin Geometry and Passive Stability (Research Questions 3 and 4)

The flight fin employs a symmetric aerofoil cross-section (root chord 70 mm, tip chord 35 mm, leading-edge sweep 25 mm, span 87 mm, 3 mm thickness) sized to deliver the minimum 1.0 caliber static margin identified in §4.2. Passive aerodynamics are established through three complementary channels: analytical prediction, direct wind-tunnel measurement (§5.4), and flight validation.

**Wind-tunnel measurement (Research Question 3).** Four candidate fin sections, NACA 0006, NACA 0012, a double-wedge profile, and a flat-plate reference, are evaluated on a bench wind tunnel across a 0.5° angle-of-attack/deflection sweep, at a representative tunnel Reynolds number (Re ≈ 2×10⁵) and cross-checked against the representative flight Reynolds number (Re ≈ 3.4×10⁵). This is the measured leg of the fin-selection decision; the section actually flown is the one the tunnel data favors, not the inviscid prediction alone.

**Computational cross-check.** A constant-strength 2D vortex-panel method (Kuethe & Chow methodology, `Simulations/CFD/run_airfoil_cfd.py`) solves the inviscid flow about each candidate section, returning lift coefficient and surface pressure distribution across the same 0.5° sweep used in the tunnel campaign, plus a flat-plate skin-friction viscous-drag estimate so lift-to-drag ratio is meaningful for section selection. The method is validated against thin-airfoil theory: the symmetric NACA 0012 section returns dCl/dα ≈ 0.120°⁻¹, ≈110% of the 2π/rad thin-airfoil ideal, the expected inviscid thickness over-prediction. Because the panel method is inviscid, it captures circulation-driven lift and pressure distribution but not viscous separation, stall onset, or pressure drag, those come from the tunnel campaign, which is exactly what this code is built to be checked against.

**Analytical prediction (Method A, Research Question 4).** Fin normal-force slope and center of pressure are computed by the Barrowman (1967) slender-body method, with the fin term

$$(C_N)_f = k_{fb}\,\frac{4N\left(\dfrac{s}{d}\right)^{2}}{1+\sqrt{1+\left(\dfrac{2\ell_f}{c_r+c_t}\right)^{2}}}, \qquad k_{fb} = 1 + \frac{r_b}{s + r_b}$$

where $N=4$ fins, $s$ is exposed semi-span, $\ell_f$ the mid-chord line length, and $k_{fb}$ the body-interference factor. Zero-lift drag is built up componentwise, skin friction over the wetted area at the flight Reynolds number, base drag, pressure/forebody drag, and fin profile drag, giving the nominal $C_D$ carried by the RK4 trajectory integrator. Reference low-angle-of-attack behaviour for symmetric sections of this class is benchmarked against the thin-airfoil and low-Reynolds-number frameworks of Lissaman (1983) and Mueller and DeLaurier (2003), and against tabulated NACA section data (Abbott & Von Doenhoff, 1959). Sensitivity of the predicted margin to build tolerance is quantified by a CG-tolerance sweep across ±20 mm of as-built CG error, and the resulting margin, apogee, and drift distributions by Monte Carlo dispersion over the atmospheric envelope. An independent implementation of the same geometry in OpenRocket 23.09 (`Simulations/WYVERN_E4_F15-4.ork`) serves as a third-party cross-check on the CP and trajectory prediction.

**Flight reconstruction (Method B, Research Question 4).** Two quantities are recovered from the onboard log and compared against both the analytical prediction and the RQ3 tunnel measurement. Coast-phase drag coefficient is reconstructed from the deceleration between burnout and apogee, where thrust is zero and the only forces are drag and gravity:

$$C_D = \frac{2m\left(-\dot{v} - g\right)}{\rho(h)\,A\,v^{2}}$$

evaluated over the high-dynamic-pressure portion of the coast and averaged, with $\rho(h)$ from the barometric altitude and the measured surface conditions. Static margin is reconstructed from the weathercock response: the observed steady pitch offset into the measured crosswind, together with the pitch-rate transient at rail exit, yields the aerodynamic restoring stiffness $k_\alpha = qA C_{N\alpha}(X_{cp}-X_{cg})$, from which $X_{cp}$ follows given the measured CG and dynamic pressure. Agreement between predicted and reconstructed values is the reported result for this research question; disagreement is itself a quantified finding about the predictive limits of a Barrowman-class model on a short, low-Reynolds-number, additively-manufactured airframe.

### 5.3 Flight Computer, Sensing, and Control Architecture (Research Questions 1 and 5)

**Consolidated single-controller architecture.** All flight avionics functions, attitude estimation, control-law execution, actuator commanding, and data logging, are consolidated onto a single custom flight-computer board (PCB1: a circular Ø62 mm 2-layer PCB built around a bare RP2350B QFN-80 die, dual-core 150 MHz Arm Cortex-M33, 520 KB SRAM, no onboard radio), replacing the distributed multi-board avionics architecture used in earlier WYVERN generations (§1.2). The two processor cores are functionally partitioned to preserve hard real-time determinism: Core 0 executes the 500 Hz thrust-vector-control loop exclusively, reading the gimbal- and body-mounted inertial measurement units, computing nozzle deflection, evaluating the PID control law, and commanding the gimbal servos, and is permitted no blocking operations of any kind. Core 1 drains a logged-data ring buffer to a microSD card over SPI, isolating all non-deterministic I/O latency from the control path; since PCB1 carries no WiFi/BLE radio chip, flight telemetry is logged rather than streamed, with the wireless-telemetry code path retained in firmware but disabled by default (`WIFI_ENABLED=0`) and used only on the ground-test rigs' separate Wi-Fi-capable Picos. This division directly addresses the principal failure mode of single-threaded flight-computer architectures, in which storage I/O can transiently block control-loop execution. Sensor inputs (onboard BNO055, external BNO085, BME680 barometer, LIS3MDL magnetometer, INA226 power monitor) share a single I2C bus, and an onboard TPS564201 buck regulator and AP2112K-3.3 LDO derive all board rails directly from the 2S pack, with no mux chip or second I2C bus.

**Power.** The entire avionics domain runs off a light 2S LiPo (7.4 V, ~450 mAh) feeding an onboard TPS564201 buck regulator (U15) directly on PCB1, stepping the pack down to an intermediate ~5 V rail — there is no discrete UBEC module; the buck on the flight computer board performs that function. An AP2112K-3.3 LDO (U7) derives the 3.3 V logic rail from that same buck output, and the four servo/expansion JST connectors draw off the ~5 V buck rail directly (the servos run at ~5 V, ~1.8 kg·cm, comfortably above the ~0.56 kg·cm gimbal demand). An INA226 power monitor (U4) sits on the shared I2C bus for pack-health telemetry; firmware currently reports rail-sag thresholds (~4.85 V warn / ~4.60 V critical) rather than true per-cell LiPo thresholds pending a board revision to route the monitor across a real current shunt (`CONFLICTS.md` §3), so the pack is also checked with a separate cell-voltage checker before every flight. The power-plus-camera group (2S LiPo ~27 g, PCB1's onboard buck/LDO folded into the ~14 g board estimate, i3 4K Thumb Action Camera ~36 g) totals roughly 63 g within the avionics allocation. This is carried through the flight numbers (liftoff 698 g, apogee ~439 ft, T/W 2.10/3.70) and, because the camera sits forward of the CG, contributes to the ~1.31 cal static margin.

**Attitude sensing.** Two nine-axis inertial measurement units (Bosch BNO085) are deployed: one onboard the flight computer in the upper body tube, and one external unit mounted at the bulkhead boundary between the two body tubes, connected via the flight computer's single STEMMA-QT port. Each runs in Game Rotation Vector mode (accelerometer–gyroscope fusion with the magnetometer disabled), because the magnetic field generated by the adjacent gimbal servos would otherwise corrupt a magnetically-referenced heading estimate. The two units vote against each other for attitude fault detection; the control law consumes body-referenced pitch/yaw directly rather than a gimbal-relative deflection angle, since neither IMU is gimbal-mounted on the flight vehicle (a gimbal-referenced IMU is used only on the ground-test TVC balances, §5.4).

**Control law (Research Question 5).** The per-axis control law is a discrete PID controller with integral anti-windup clamping and a first-order low-pass-filtered derivative term, executed at 500 Hz with output clipped to a ±8° gimbal deflection limit (raised from ±5° to give control-authority margin against crosswind weathercocking without adding passive fin stability, which would otherwise reduce the very disturbance the TVC system is built to demonstrate). The flight gain set, Kp = 0.10, Ki = 0.40, Kd = 0.18, was selected by a phase/gain-margin analysis across 24 operating points (phase margin ≈ 33°, gain margin ≈ 12.6 dB) and independently confirmed by a time-domain robust multi-wind auto-tune; it holds worst-case gust pitch deviation to 1.96° with wide gimbal headroom against the ±8° limit (2.35° peak gimbal), while avoiding the resonance against finite servo lag (≈ 40 ms) observed in higher-proportional-gain configurations. The TVC loop is inhibited for the first 0.5 s of flight (§4.2), after which it engages to stabilize the vehicle to vertical and execute a small commanded maneuver; required gimbal torque is estimated at ≈ 0.56 kg·cm at ±8°, well within the selected servo class, and simulated control authority remains positive throughout the powered phase across the modeled gain sets.

**Actuator comparison (Research Question 1).** Two TVC actuator classes are evaluated using the identical control electronics, gimbal mechanism, and software control law, isolating actuator dynamics as the experimental variable: a tri-solenoid magnetic gimbal actuator, and a servo-driven gimbal actuator. The comparison is conducted entirely on the ground-based three-axis thrust-vector load balance described in §5.4, rather than in flight, to remove flight-to-flight aerodynamic and atmospheric variability from the actuator comparison; the flight vehicle itself carries the servo actuator, selected on the basis of the ground-comparison results.

### 5.4 Ground Test Program (Research Questions 1, 2, 3)

The ground test program is structured as the program's primary experimental apparatus rather than a preliminary check, since the actuator comparison (Research Question 1), the jetvane/materials data (Research Question 2), the aerofoil measurement (Research Question 3), and the motor-characterization data that feed every downstream simulation (Research Question 5; §4) are all generated here rather than in flight. **Four purpose-built stands are constructed**: a servo TVC stand, a physically separate magnetic TVC stand, a static-fire stand (calibration, thrust curves, and the jetvane blast-shield materials screen), and a bench wind tunnel for aerofoil measurement. The three motor-fired stands are fully printable in PETG-CF (selected per Research Question 2 for its motor-plume thermal margin) and instrumented with strain-gauge load cells and HX711 24-bit bridge-amplifier breakouts, logged to onboard microSD by a dedicated data-acquisition microcontroller independent of the flight avionics; the wind tunnel is an unpowered-motor bench rig with its own instrumentation.

**Servo TVC stand and magnetic TVC stand.** Each actuator class is mounted to its own physically identical thrust block, restrained from a fixed base by three strain-gauge load cells acting through flexures, one axial and two lateral, resolving the complete thrust vector in magnitude and direction:

$$T = \sqrt{F_x^2 + F_y^2 + F_z^2}, \qquad \theta = \arctan\!\left(\frac{\sqrt{F_x^2+F_y^2}}{F_z}\right), \qquad \phi = \operatorname{atan2}(F_y, F_x)$$

The cells are sized to the expected loading envelope of the test motor (Estes F15-0, 25.3 N peak axial thrust, side force at the ±8° gimbal limit ≈ 3.5 N), using a 5 kg axial cell and two 1 kg lateral cells digitized at 80 samples per second, on both stands identically. An alternative single-piece design, a cruciform flexure instrumented with one Wheatstone bridge per arm, forming a unified three-axis force/torque sensor, is held as a fallback configuration should the discrete three-cell assembly prove difficult to align. Running the identical instrumentation chain on two separate physical stands (rather than swapping actuators on one shared fixture) is what makes the magnetic-versus-servo comparison valid under nominally identical thrust conditions; commanded-versus-measured deflection angle (θ, φ) is logged across a series of step and ramp commands under the F15-0 thrust condition on each stand to extract bandwidth, slew rate, step-response overshoot, steady-state error, and maximum sustained deflection for each actuator. Because each F15-0 firing provides a 3.45 s control window, the independent step/ramp command set for each actuator system is built up across multiple firings on its respective stand.

**Static-fire stand.** A single-axis, load-cell-only stand fitted with a steel blast deflector validates the as-fired thrust curve of every motor class used in the program against its published specification, and carries a thermocouple on the engine-bay wall to measure the peak temperature that the Research Question 2 heat-deflection argument depends on. **The jetvane test is defined as a blast-shield materials screen**: a flat coupon plate of each candidate material, 5 mm thick at 100% infill, is mounted directly in the exhaust path like a blast shield and fired on. Six materials go through the screen (PLA, PETG-CF, ABS, ASA-Aero, PC, PC-FR); response measured is melt-through, ablation depth, and slag buildup rather than thrust or deflection, and materials surviving 5 mm are retested at 4, then 3, then 2 mm until they fail, feeding Research Question 2's materials dataset as a failure-thickness ranking. The deflector and mounting hardware are sized to the F15's 3.45 s burn thermal case. Ground firings use the plugged, 0-delay Estes F15-0 (identical thrust curve to the flight F15-4, but no ejection charge) so that nothing fires into the stand fixtures after burnout.

**Wind tunnel.** A bench aerofoil rig provides direct lift/drag/stall measurement for the four candidate fin sections across a 0.5° angle-of-attack sweep, supplying the measured leg of Research Question 3 (fin selection) and Research Question 4 (tunnel-vs-flight calibration) described in §5.2.

**Motor plan and firing counts.** Table 3 summarizes the verified motor specifications and the planned firing allocation across flight, all three motor-fired ground stands, and stand commissioning.

**Table 3. Motor plan and verified specifications.**

| Motor | Total impulse | Avg / peak thrust | Burn time | Role |
|---|---|---|---|---|
| Estes F15-4 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Flight only (4 s delay + ejection = recovery system) |
| Estes F15-0 | 49.6 N·s | 14.4 N / 25.3 N | 3.45 s | Ground only, static-fire thrust curves and the jetvane blast-shield screen, plus the magnetic and servo TVC stand runs (0-delay/plugged; same curve, no ejection into fixtures) |
| Estes/AeroTech E16-4 | — | ~16 N avg | E-class | Stand commissioning and calibration only |

Planned firing counts are 4 Estes F15-4 motors for flight testing, and 13-24 plugged Estes F15-0 motors for ground testing (6 across the two TVC stands at three firings per actuator system, 2 on the static-fire stand for thrust-curve verification and the engine-bay wall temperature measurement, and the jetvane blast-shield screen, which is adaptive rather than fixed: six materials at up to four thickness steps each tops out at 24 firings if every material survives to 2 mm, though most candidates are expected to fail by 4 mm in practice, budget roughly 12-16 firings with 24 as the worst case), plus 6 Estes/AeroTech E16-4 motors for stand commissioning at two firings per motor-fired stand. The F15-0 and F15-4 share an identical thrust curve (same F15 propellant); the 0-delay F15-0 is used on the ground so no ejection charge fires into the stand fixtures, while the flight motor carries the 4-second delay whose ejection charge is the recovery system.

**Calibration and commissioning sequence.** Each stand's load cells are first calibrated independently of any motor firing, using a series of known hanging dead weights spanning the expected force range, to establish a force-to-voltage transfer function for every channel. Each stand is then commissioned with a minimum of two low-cost E16-4 motor firings before any data-collection firing is conducted, to validate the as-built stand's measured thrust curve against the motor's independently published reference curve and confirm that structural compliance in the stand itself is not corrupting the force measurement. Only after a stand passes this commissioning check are F15-0 data-collection firings conducted on it. All raw and reduced ground-test data (thrust curves, vector-deflection logs, and materials-erosion observations) are archived under a dedicated data directory structure separating motor thrust-curve data from TVC vector/control data, in the same repository used for flight data release (§5.6).

### 5.5 Simulation Suite and Software-in-the-Loop Validation (All Research Questions)

The computational tier carries the analytical and inviscid-CFD load for aerodynamics and control, and is held to an explicit verification standard: every simulated result reported here is produced by a version-controlled script, paired with the dataset it wrote, and gated against hard pass/fail criteria rather than inspected qualitatively. Where a physical measurement now exists for a given quantity, the wind-tunnel-measured aerofoil coefficients (§5.2, §5.4), the simulated result is reported alongside it as a cross-check, not as the only available answer.

**Trajectory and dispersion.** A fourth-order Runge–Kutta integrator advances the two-dimensional point-mass state under the digitized F15 thrust curve, the componentwise Barrowman drag buildup of §5.2, an exponential-density atmosphere referenced to the measured surface temperature and pressure, and a power-law wind-shear profile. Monte Carlo dispersion samples the full field envelope, mean wind, turbulence intensity, surface temperature and pressure, launch-rod angle, and site elevation, together with vehicle-side dispersions in liftoff mass, CG station, drag coefficient, and total impulse, producing distributions of apogee, maximum dynamic pressure, deployment velocity, landing dispersion, and static margin at every point in the burn.

**Closed-loop control simulation.** The pitch-plane control model reproduces the flight law as executed: the discrete PID of §5.3 at the firmware's 500 Hz rate, a first-order servo lag with a measured-class time constant, an explicit control-loop transport delay, gimbal-rate and travel limits, and gust forcing generated from a Dryden-form turbulence spectrum rather than a single sinusoid. Frequency-domain phase and gain margins are evaluated at every point in a burn-time × atmosphere grid, and a candidate gain set is accepted only if it clears the margin target at every point in that grid (§5.3).

**Software-in-the-loop flight computer.** The flight firmware's state machine, sensor models, and control law are exercised end-to-end in `wyvern_datagen/fc_sil.py` against simulated barometer, IMU, and battery signals carrying representative noise, bias, and quantization, producing synthetic flight logs in the same schema as the onboard recorder. This lets the ground-station analysis pipeline and every pass/fail gate be exercised and debugged before any motor is fired, and provides the Method-B leg for Research Question 1 by driving the bench actuator models under the identical control law used on the balance.

**Bench-model cross-validation of the ground stands.** The instrumented stands of §5.4 are modeled at the signal level in `wyvern_datagen/bench_sim.py`, load-cell full scale and bridge noise, HX711 sample rate and quantization, stand structural compliance and its resulting mount resonance, actuator lag and linkage ratio, so that the expected measurement, its uncertainty, and the required cell sizing are predicted before the stand is built, and any departure of the as-built stand from that prediction is itself detectable.

**Reproducibility.** All simulated aerodynamic, trajectory, and control-loop datasets generated during the program, the RK4-plus-Barrowman trajectory and dispersion simulations of §4.2 and §5.2, the gate-based flight-validation suite referenced in §5.6 and §8, the atmospheric-sweep and margin-analysis control-loop simulations of §5.3, and the bench and software-in-the-loop models described above, are version-controlled alongside the physical test data in the program's public repository, with each simulation script paired against the dataset it produced, so that every plotted or tabulated simulated result in this paper and its supporting materials can be regenerated and independently checked against the as-built hardware.

### 5.6 Flight Test Plan, Logging, and Data Sharing (Research Questions 4 and 5)

Each flight is conducted in compliance with the National Association of Rocketry (2023) safety code and motor classification standards, under the supervision of a NAR-certified or similarly qualified range safety officer. Multiple flights are conducted with the control law configured under each candidate gain set identified in §5.3, and post-flight telemetry recovered from the onboard microSD log is analyzed for peak pitch deviation during the powered phase, gimbal-angle utilization, and qualitative settling/overshoot behavior, to determine which gain configuration produces the most accurate and best-damped attitude tracking. All onboard sensor data (full-rate inertial, barometric, and control-loop telemetry) is logged at the full control-loop sample rate; where the bench/range Wi-Fi telemetry link is in range, a parallel live feed is monitored for real-time anomaly detection, though the onboard log remains the data of record. Upon conclusion of the program, all CAD files, firmware source code, simulation scripts, and the complete flight and ground-test datasets are released through a version-controlled, publicly accessible Git repository, and the principal reduced results from each research question are additionally presented in summary form within this paper.

---

## 6. Recovery System

The vehicle's passive fin stability is retained through the coast phase to apogee (predicted apogee ≈ 439 ft at t ≈ 6.87 s). Recovery is initiated not by an independent electronic altimeter but by the flight motor's own factory ejection charge: the flight configuration uses an Estes F15-4 (a four-second ejection delay), so that approximately four seconds after propellant burnout, t ≈ 7.45 s, roughly 0.58 s past the predicted apogee, the motor's integral ejection charge fires inside the lower body tube's TVC bay. Gas pressure builds directly against the bulkhead joint between the two body tubes, and the joint — friction-fit/shear-pinned to release at a target force — separates, deploying the parachute packed at the forward end of the lower body tube.

A first-order feasibility analysis (`Simulations/we4_ejection_feasibility.py`) supports the approach on two independent grounds. First, flow-path losses at the ejection mass-flow are on the order of 0.06 kPa, negligible against the driving pressure. Second, the recovery bay pressurizes to approximately 140 kPa against a friction-fit joint-release threshold of 14–41 kPa, a pressurization margin of roughly 3.4×. The F15-4's four-second delay is the closest available Estes delay to the coast-to-apogee optimum (≈ 3.5 s); the longer F15-6 and F15-8 delays are rejected because they fire approximately 2.5 s and 4.5 s past apogee, deploying at high descent speed and, in the F15-8 case, at dangerously low altitude. This motor-ejection architecture needs no independent recovery computer, no isolated recovery battery, no e-match or black-powder charge, and no dedicated recovery wiring, reducing parts count, cost, and an entire electronic failure domain, at the cost of a single passive deployment event with no electronic backup channel, a trade justified by the 3.4× pressurization margin and by the finned airframe's aerodynamic stability through apogee. Shock-cord and parachute sizing (1/8″ tubular Kevlar cord, 24″ ripstop nylon canopy, with a Nomex blanket shielding the canopy from the ejection gas) are verified against the worst-case deployment scenario, ejection ≈ 0.78 s past apogee while the vehicle still carries ≈ 7.7 m/s of vertical velocity (`we4_ejection_feasibility.py`), giving a harness structural safety factor comfortably above the 2.0 target. A predicted terminal descent rate near 4.8 m/s follows once the canopy is fully open. The bulkhead joint is checked against the ≈ 140 kPa ejection pressure in the structural analysis (`WYVERN_E4_FEA_Structural.md` §4), returning a bending safety factor of ≈ 8× as an upper-bound check if the joint failed to release; the release-force sizing pass itself (§4 of the same document) remains an open item ahead of flight.

---

## 7. Safety and Regulatory Compliance

All flights use a single Estes F15-4 motor (49.6 N·s total impulse, 60 g propellant, F-class, four-second ejection delay), and the fully loaded liftoff mass of 698 g is well under the FAA's Class 1 (model rocket) threshold of 1,500 g loaded weight per motor, requiring no Federal Aviation Administration airworthiness waiver and no NAR/Tripoli high-power certification. Range procedures include remote ignition, a minimum 3 m personnel standoff from both ground-test stands during firing, a fail-safe neutral-gimbal default state on any control-system fault, and standard model-rocketry motor-handling discipline for the motor's integral ejection charge, the igniter is installed last, at the pad, and there are no independent pyrotechnic or electronic ejection circuits in the vehicle to arm or inhibit (recovery is effected solely by the motor's own delay/ejection charge).

---

## 8. Expected Outcomes

The program is expected to produce a quantitative, ground-validated comparison of magnetic-solenoid and servo thrust-vector-control actuators, including bandwidth, slew rate, overshoot, and steady-state error for each, directly informing actuator selection for future closed-loop rocketry programs without requiring a dedicated in-flight A/B comparison. A validated thermally-zoned additive-manufacturing material allocation is expected to demonstrate a measurable dry-mass reduction relative to a uniform heat-rated-material baseline, with no corresponding loss of structural margin, offering a transferable design pattern for hobbyist and academic rocketry programs using FDM fabrication; the jetvane blast-shield screen adds a failure-thickness ranking across six candidate materials under real motor-plume conditions to that same dataset. Direct wind-tunnel measurement of the candidate fin aerofoil sections, cross-checked against the inviscid vortex-panel CFD prediction, is expected to produce a defensible, measured basis for fin-section selection rather than a purely modeled one, and, combined with flight telemetry, a quantified accuracy bound on Barrowman-class stability prediction for short, low-Reynolds-number, additively-manufactured airframes, expressed as the discrepancy between predicted, tunnel-measured, and telemetry-reconstructed static margin and coast-phase drag coefficient. This three-way comparison (analytical, tunnel, flight) will be released as an open dataset, giving subsequent low-cost programs a defensible error bar to design against with or without their own flow facility. Finally, flight-validated comparison of control-loop gain configurations on a consolidated dual-core single-controller architecture is expected to clarify the practical control-authority and determinism benefits of separating real-time control execution from logging and telemetry I/O on a single low-cost microcontroller, relative to either a single-threaded controller or a distributed multi-board avionics stack. All resulting design files, firmware, simulation code, and flight datasets will be released publicly through the National Association of Rocketry in support of the program's open-source objectives.

---

## 9. Project Timeline

The build-to-flight schedule runs on a fixed external constraint: two launch windows bracketing the Thanksgiving holiday, chosen for range availability and family/team scheduling rather than any technical deadline, with all program data required to be collected by December 1. Work is budgeted at three hours per week against this fixed end date (excluding unattended 3D-print run time, but including CAD, print-queue management, and all post-processing), spanning roughly fifteen and a half weeks from proposal finalization to data close-out.

**Fixed dates.** Primary launch window: Saturday–Sunday, November 21–22, 2026 (the weekend preceding Thanksgiving). Contingency/repeat launch window: Saturday–Sunday, November 28–29, 2026 (the weekend following Thanksgiving), held in reserve for weather scrubs, hardware anomalies, or a repeat flight if the primary window leaves a gap in the RQ dataset. All data collection concludes by December 1, 2026, with the final two days held as unscheduled margin rather than committed work.

**Phase structure.** The schedule proceeds through design lock and procurement, electronics bring-up on the custom flight-computer PCB, CAD and fabrication of all four ground-test stands (§5.4), the ground-test campaigns for Research Questions 1 through 3 and the motor-characterization data supporting Research Question 5, airframe fabrication and recovery-system integration, full-stack integration and a dry-run countdown rehearsal, the two launch windows, and a final data-reduction pass cross-checking results against all five research questions. Full week-by-week task allocation is maintained in the program's supporting documentation (`WYVERN_E4_Timeline_3Month.md`) and is not reproduced in full here; the governing principle is that the launch dates are fixed and the schedule bends around them, with slip absorbed by the built-in end-of-schedule buffer before either launch date is moved.

---

## References

Abbott, I. H., & Von Doenhoff, A. E. (1959). *Theory of wing sections: Including a summary of airfoil data.* Dover Publications.

Barrowman, J. S. (1967). *The practical calculation of the aerodynamic characteristics of slender finned vehicles* (NASA NTRS accession 20010047838). https://ntrs.nasa.gov/citations/20010047838

Bosch Sensortec. (n.d.). *BNO085 9-axis absolute orientation IMU, datasheet.*

BPS.space. (n.d.). *Thrust vector control.* Retrieved from https://bps.space/products/thrust-vector-control

Dizon, J. R. C., Espera, A. H., Chen, Q., & Advincula, R. C. (2018). Mechanical characterization of 3D-printed polymers. *Additive Manufacturing, 20,* 44–67. https://doi.org/10.1016/j.addma.2017.12.002

Kuethe, A. M., & Chow, C. Y. (1998). *Foundations of aerodynamics: Bases of aerodynamic design* (5th ed.). Wiley.

Lissaman, P. B. S. (1983). Low-Reynolds-number airfoils. *Annual Review of Fluid Mechanics, 15,* 223–239.

Mueller, T. J., & DeLaurier, J. D. (2003). Aerodynamics of small vehicles. *Annual Review of Fluid Mechanics, 35,* 89–111. https://doi.org/10.1146/annurev.fluid.35.101101.161102

National Advisory Committee for Aeronautics. (1951). *Aerodynamic characteristics of NACA 0012 airfoil section at angles of attack from 0° to 180°* (NACA TN 2502).

National Association of Rocketry. (2023). *NAR safety code and motor classification standards.* https://www.nar.org/safety-information/

NASA. (1968). *Thrust-vector control requirements for solid-propellant launch vehicles* (NASA TN D-4971).

OpenRocket Project. (2023). *OpenRocket technical documentation v23.09.* https://openrocket.info/documentation.html

Pérez Gordillo, A., Simplício, P., Iannelli, A., & Marcos, A. (2023). Thrust vector control and state estimation architecture for low-cost small-scale launchers. *arXiv.* https://arxiv.org/pdf/2303.16983

Popescu, D., Zapciu, A., Amza, C., Baciu, F., & Marinescu, R. (2018). FDM process parameters influence over the mechanical properties of polymer specimens. *Polymer Testing, 69,* 157–166. https://doi.org/10.1016/j.polymertesting.2018.05.020

Raspberry Pi Foundation. (2024). *RP2350 datasheet.*

Sahoo, S. (2026, April 11). WYVERN PTD Portal. Skylight Industries. https://gtr70e wyvern.base44.app/

Ziegler, J. G., & Nichols, N. B. (1942). Optimum settings for automatic controllers. *Transactions of the ASME, 64,* 759–768.

# WYVERN-E 4.0 — Structural & Thermal Analysis (first-order FEA)

*First-order analytical margins from `../Simulations/we4_analysis.py` → `plots4/08_fea_loads.png`,
`09_thermal.png`. A 25 N motor on a 70 mm tube is handling/print-limited, not load-limited.*

## 1. Load cases
| Case | Source | Magnitude |
|---|---|---|
| Axial compression | F15 peak thrust | 25.3 N |
| Bending (TVC) | side force at ±8° gimbal | $25.3\sin8°=3.5$ N at the nozzle |
| Bulkhead A | reaction of the gimbal mount | 3.5 N + thrust transfer |
| Gimbal pivot | side load through 2 mm pins | 3.5 N |

## 2. Stresses & margins (yield: ASA ~30 MPa, PC-FR ~60 MPa)
- **Tube axial:** $\sigma=F/(\pi D\,t)=25.3/(\pi\cdot0.070\cdot0.0016)=72$ kPa → SF vs ASA ≈ **400×**.
- **Tube bending:** $M=3.5\times0.16=0.56$ N·m, $\sigma=Mc/I$ with $I=\pi/64(D^4-d^4)$ → ~0.2 MPa → SF **>100×**.
- **Gimbal pivot / Bulkhead A (PC-FR):** SF **>50×**.
- **Minimum SF across the structure: ~340×.** The airframe wall (1.6 mm) is set by printability and
  handling robustness, not flight loads — there is no structural reason to go thicker.

## 3. Thermal (engine-bay PC-FR wall, F15 3.45 s burn)
Lumped-wall transient with a **0.5 mm phenolic motor liner** as the thermal barrier (liner-reduced
inner driving temp ~180 °C, $h\approx120$ W/m²K): engine-bay wall peaks **~47 °C**, well under
PC-FR's ~110 °C HDT — the flight F15 case has wide margin. On the *static stand* (not in the vehicle),
the steel deflector + phenolic liner protect the mounts against the plume.

## 4. Recovery ejection loads & bypass-tube thermal (F15-4 motor ejection)
Recovery is now by the F15-4's built-in ejection charge, routed through the solid-walled bypass
tube (OD15 / ID12 mm, PC-FR) past the *sealed* FC bay into the recovery bay (see
`WYVERN_E4_Recovery.md`). This adds two new load paths.

**4.1 Recovery-side bulkhead (Bulkhead B, PC-FR) — ejection pressure.**
The recovery bay pressurizes to $p\approx140$ kPa (feasibility sim), acting across the sealed
bulkhead B (bore radius $r=33.4$ mm, thickness $t=4$ mm) against the ambient FC bay:
- **Net thrust on bulkhead:** $F=p\,\pi r^2 = 140{,}000\times\pi(0.0334)^2 = \mathbf{491\ N}$.
- **Plate bending (clamped-edge circular plate):** $\sigma_{max}=\tfrac{3pr^2}{4t^2}=\tfrac{3(0.14)(0.0334)^2}{4(0.004)^2}=\mathbf{7.3\ MPa}$ → SF vs PC-FR (60 MPa) ≈ **8.2×**.
- The two M3 through-bolts / bonded shoulder carry the 491 N in shear ($\tau\approx491/(2\cdot\pi\cdot1.5^2)=35$ MPa on nylon → use steel bolts, SF **>10×**). *Both bulkheads are now PC-FR: Bulkhead B is the pressure-bearing member (ejection), Bulkhead A carries the gimbal reaction from §2.*

**4.2 Bypass tube — internal pressure.**
Hoop stress at the 140 kPa peak: $\sigma_\theta=\tfrac{pr}{t}=\tfrac{140{,}000\times0.006}{0.0015}=0.56$ MPa
→ SF vs PC-FR (60 MPa) ≈ **107×**. The tube is pressure-trivial; its wall is set by print
robustness and the thermal duty below, not stress.

**4.3 Bypass-tube thermal (ejection-gas pulse).**
The ejection charge is a single brief (~0.1 s) black-powder pulse. Post-expansion gas entering the
12 mm bore is on the order of 200–300 °C. A lumped thin-wall estimate ($h\approx100$ W/m²K, PC-FR
$\rho c\approx1.9$ MJ/m³K, wall 1.5 mm) gives a Biot number $\ll0.1$ and an inner-wall temperature
rise of only a few °C over the 0.1 s pulse — well under PC-FR's ~110 °C HDT. **Margin item:** a
thin Nomex/Kapton sleeve inside the bore is specified as cheap insurance against soot erosion over
repeated static firings, not because the single-flight thermal case requires it.

## 5. Modal / dynamics note
The body is short (0.74 m) and stiff relative to the loads; first lateral bending mode is well above
the ~10 Hz TVC control bandwidth, so no structural–control coupling. (A full modal FEM is listed as
future work; first-order separation is comfortable.)

## 6. Conclusion
Structurally over-margined on the flight loads (min SF ~340×) and comfortable on the new recovery
loads (bulkhead-B ejection SF ~8×, tube ~107×); thermally safe with a phenolic motor liner and a
Nomex bore sleeve. The design drivers are **mass** (ASA-Aero main construction; PC-FR only where heat or ejection pressure demands it, to keep T/W up) and **CG/control-arm** (TVC authority), not
stress.

# WYVERN-E — Structural & Thermal Analysis (first-order FEA)

*First-order analytical margins from `../Simulations/we4_analysis.py` → `plots4/08_fea_loads.png`,
`09_thermal.png`. A 25 N motor on a 70 mm tube is handling/print-limited, not load-limited.*

## 1. Load cases
| Case | Source | Magnitude |
|---|---|---|
| Axial compression | F15 peak thrust | 25.3 N |
| Bending (TVC) | side force at ±8° gimbal | $25.3\sin8°=3.5$ N at the nozzle |
| Bulkhead A | reaction of the gimbal mount | 3.5 N + thrust transfer |
| Gimbal pivot | side load through 2 mm pins | 3.5 N |

## 2. Stresses & margins (yield: ASA ~30 MPa, PETG-CF ~60 MPa)
- **Tube axial:** $\sigma=F/(\pi D\,t)=25.3/(\pi\cdot0.070\cdot0.0016)=72$ kPa → SF vs ASA ≈ **400×**.
- **Tube bending:** $M=3.5\times0.16=0.56$ N·m, $\sigma=Mc/I$ with $I=\pi/64(D^4-d^4)$ → ~0.2 MPa → SF **>100×**.
- **Gimbal pivot / Bulkhead A (PETG-CF):** SF **>50×**.
- **Minimum SF across the structure: ~340×.** The airframe wall (1.6 mm) is set by printability and
  handling robustness, not flight loads — there is no structural reason to go thicker.

## 3. Thermal (engine-bay PETG-CF wall, F15 3.45 s burn)
Lumped-wall transient with a **0.5 mm phenolic motor liner** as the thermal barrier (liner-reduced
inner driving temp ~180 °C, $h\approx120$ W/m²K): engine-bay wall peaks **~47 °C**, well under
PETG-CF's ~110 °C HDT — the flight F15 case has wide margin. On the *static stand* (not in the vehicle),
the steel deflector + phenolic liner protect the mounts against the plume.

## 4. Recovery ejection loads at the bulkhead separation joint (F15-4 motor ejection) — **needs a real re-pass, flagged below**

**⚠ Design-intent change, not yet re-analyzed.** Recovery is now by the F15-4's built-in ejection
charge separating the two body tubes at a single bulkhead joint (see `WYVERN_E4_Recovery.md`) — the
old solid-walled bypass tube routing gas past a *sealed* FC bay into a separate recovery bay no
longer exists in this design, so **§4.2 and §4.3 below (bypass-tube pressure/thermal) are obsolete**
and retained only as historical record of the prior single-tube analysis. The bulkhead-B numbers in
§4.1 are also now **the wrong design target**: they were sized to make the bulkhead **survive** 491 N
without separating (a sealed pressure-bearing member with bolted retention), but the new architecture
*wants* that joint to **release** at a controlled 50–150 N (per `WYVERN_E4_Recovery.md` §4, matching
the old nose friction-fit spec) — a friction-fit/shear-pin joint calibrated to let go, not bolted
hardware sized to hold. This is a genuine open engineering decision (pin/friction-fit sizing for a
target release force under the ~140 kPa ejection pulse), not something to invent a number for here.

**4.1 Bulkhead joint (PETG-CF) — ejection pressure, old "survive" analysis (superseded target)**
The old calc, kept for reference: the recovery-side volume pressurizes to $p\approx140$ kPa
(feasibility sim, single-tube geometry — not yet re-run for the two-BT volume), acting across a
bulkhead of bore radius $r=33.4$ mm, thickness $t=4$ mm:
- **Net force on bulkhead:** $F=p\,\pi r^2 = 140{,}000\times\pi(0.0334)^2 = \mathbf{491\ N}$.
- **Plate bending (clamped-edge circular plate), if it were bolted to hold:** $\sigma_{max}=\tfrac{3pr^2}{4t^2}=\tfrac{3(0.14)(0.0334)^2}{4(0.004)^2}=\mathbf{7.3\ MPa}$ → SF vs PETG-CF (60 MPa) ≈ **8.2×**.
- The old M3 through-bolt calc ($\tau\approx491/(2\cdot\pi\cdot1.5^2)=35$ MPa) assumed a bolted,
  non-releasing joint — **not applicable** to a joint that's meant to separate. A separating joint
  needs a friction-fit or shear-pin sizing pass against the 50–150 N release-force target instead,
  using the same 491 N/140 kPa pressure figure as the driving load (re-verify against the new two-BT
  volume first).

**4.2 (superseded) Bypass tube — internal pressure.** No longer applicable; there is no bypass tube
in the two-BT design. The gas path is now directly across the bulkhead joint itself.

**4.3 (superseded) Bypass-tube thermal.** No longer applicable for the same reason. The bulkhead
joint is now the direct-gas-exposure surface — a **new thermal check is needed** for the PETG-CF
bulkhead's brief (~0.1 s) exposure to the 200–300 °C ejection pulse, analogous to the old §4.3 lumped-
wall estimate (which found only a few °C inner-wall rise, well under PETG-CF's ~110 °C HDT) but not
yet re-run for the bulkhead's geometry instead of the tube's. Wadding on both bulkhead faces
(`WYVERN_E4_Recovery.md` §7) is the near-term thermal mitigation regardless of what the recalculated
number says.

## 5. Modal / dynamics note
The body is short (0.74 m) and stiff relative to the loads; first lateral bending mode is well above
the ~10 Hz TVC control bandwidth, so no structural–control coupling. (A full modal FEM is listed as
future work; first-order separation is comfortable.)

## 6. Conclusion
Structurally over-margined on the flight loads (min SF ~340×) — that part is unaffected by the bay
split. Recovery loads need a fresh pass: the bulkhead separation joint's release-force sizing (§4.1)
and its direct-gas-exposure thermal check (§4.3) are both open items, not carried over from the old
bypass-tube design. The design drivers remain **mass** (PLA main construction; PETG-CF only where
heat or ejection pressure demands it, to keep T/W up) and **CG/control-arm** (TVC authority) for the
flight-load side; the recovery side is now driven by **release-force calibration**, not survival
strength.

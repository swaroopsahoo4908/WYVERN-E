# WYVERN-E 4.0 — Recovery (motor ejection via bypass tube — no CO2, no pyro bay, no recovery electronics)

Huge simplification: recovery now runs entirely on the **F15-4's own ejection charge**. No CO2 system,
no solenoids/drivers/cartridges/needles, no recovery battery, and the flight computer does **not**
actuate recovery at all — it only logs. The motor's ejection gas is routed through a **solid-walled
bypass tube** that passes the *sealed* flight-computer bay and vents into the recovery bay, popping
the friction-fit nose + chute. Feasibility confirmed in `../Simulations/we4_ejection_feasibility.py`.

## 1. Why F15-4 (not F15-6)
Apogee is at **7.0 s** (burnout 3.45 s + 3.53 s coast), so the ideal ejection delay is ~3.5 s. Of the
off-the-shelf F15 delays, **F15-4** is the closest:

| Motor | Ejection | vs apogee | Deploy speed | Opening load | Verdict |
|---|---|---|---|---|---|
| **F15-4** | 7.45 s | **+0.47 s** | **~4.7 m/s** | ~6 N | near-optimal ✅ |
| F15-6 | 9.45 s | +2.47 s | ~23 m/s | ~147 N | late/hard |
| F15-8 | 11.45 s | +4.47 s | ~39 m/s, only 51 m AGL | ~408 N | unsafe (too low) |

F15-4 deploys just after apogee at a gentle ~4.7 m/s — the softest opening of the three.

## 2. Gas path (the sealed-bay bypass)
```
            FC bay (SEALED between two bulkheads — electronics + gimbal protected)
        /|===========================|\
Engine  |  (solid-walled bypass tube runs past the sealed FC bay)  ==>  Recovery bay --> nose
        \|===========================|/
            FC bay
   motor forward closure --(ejection gas)--> bypass tube --> recovery bay
```
1. At t = 7.5 s the F15-4 fires its forward ejection charge.
2. Gas enters the **bypass tube** at the aft (engine-side) bulkhead and travels the length of the FC
   bay *inside a solid-walled tube* — the FC bay itself stays sealed and gas-free.
3. The tube vents into the **recovery bay** above the forward bulkhead.
4. The bay pressurizes and pushes the **friction-fit nose shoulder** off → chute deploys.

The two bulkheads seal the FC bay on both ends; the bypass tube penetrates both with **epoxied /
O-ring sealed** pass-throughs. A **Nomex blanket** protects the chute from the hot gas.

## 3. Feasibility numbers (`we4_ejection_feasibility.py`)
- **Bypass tube:** a **12 mm ID** tube, ~160 mm long, drops only **0.06 kPa** during the ~0.1 s
  ejection pulse — negligible. (Even a 6 mm tube stays under the 10 kPa ceiling.)
- **Bay pressurization:** the F15 charge delivers **~140 kPa** into the sealed recovery bay against a
  **14–41 kPa** friction-fit release window → **3.4× margin**. Deploys reliably.
- **Deploy dynamics:** ~4.7 m/s at ejection (0.47 s past apogee), ~6 N opening on the 18" chute —
  trivial for the 1/8" Kevlar cord (>1000× SF).

## 4. Sealing & thermal
- **FC-bay integrity:** the whole point of the twin bulkheads + tube is that ejection gas *never*
  contacts the Pico, servos, gimbal, or sensors. Verify both bulkhead pass-throughs are gas-tight.
- **Tube thermal:** ejection gas is hot but the pulse is brief (~0.1 s). **PC-FR** (HDT ~110 °C, and
  the ablative/char behavior of the flame-retardant blend) handles a single brief exposure; a thin
  Nomex or Kapton liner on the tube ID adds margin for repeated flights. See FEA doc §3.

## 5. Ground test (mandatory before flight)
1. **Static ejection test:** fire an F15-4 (or a ground-igniter + a measured BP sim charge) in the
   built airframe on the bench — confirm gas routes through the tube and cleanly pops the nose.
2. **Leak check:** pressurize the FC bay path and confirm the two bulkhead seals hold (no gas into
   the electronics bay).
3. **Nose pull-off:** confirm the friction-fit release force is in the 50–150 N band (see §4 of the
   prior friction-fit note; unchanged — the nose retention method is the same, only the *pressure
   source* changed from CO2 to motor ejection).
4. **Chute pack + Nomex:** verify the blanket fully shields the canopy from the tube outlet.

## 6. Why this is the best recovery for WYVERN-E 4.0
Simplest and lightest path: the motor already carries a perfectly-timed ejection charge, so recovery
needs **zero electronics, zero battery, zero pyro handling, and zero FC involvement** — just a printed
tube, two bulkheads, a chute, cord, and a Nomex blanket. It's the most flight-proven recovery method
in hobby rocketry (every Estes rocket uses motor ejection), and it keeps the flight computer bay fully
sealed so the TVC electronics never see ejection gas. Trade vs CO2/electronic deploy: the delay is
fixed by the motor (can't retune in software) — which is why the F15-4 delay selection above matters.

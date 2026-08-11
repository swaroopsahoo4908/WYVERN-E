# WYVERN-E, Recovery (two-body-tube separation at the bulkhead joint, motor ejection, no pyro/CO2/recovery electronics)

Recovery still runs entirely on the **F15-4's own ejection charge**, no CO2 system, no solenoids/
drivers/cartridges/needles, no recovery battery, and the flight computer does **not** actuate
recovery at all, it only logs. What changed is the airframe: WYVERN-E is now **two separate body
tubes (Lower BT, Upper BT)** joined at a single bulkhead, and the ejection charge's job is now to
**separate the two tubes at that joint**, a traditional dual-deploy-style break, not a friction-fit
nose pop off a single continuous tube.

## 1. Bay layout (nose to tail)

| Body tube | Contents (fwd → aft within the tube) |
|---|---|
| **Upper BT** | Nose cone, Camera, Flight Computer (custom PCB), Battery, Recovery Wadding |
| **Lower BT** | Parachute + Shock Cord, Recovery Wadding, TVC Bay (gimbal, servos, motor mount, motor) |

The two tubes meet at **one bulkhead**, friction-fit/shear-pinned so the ejection charge can part
them cleanly. The bulkhead carries **pass-through holes** for the servo signal/power extensions and
the STEMMA-QT cable running from the flight computer (Upper BT) to the **single external BNO085**
mounted right at the TVC-bay/electronics boundary, the fwd end of the Lower BT / aft end of the
Upper BT, immediately adjacent to the bulkhead joint itself, not deep in the TVC bay on the gimbal
and not in a separate "recovery" IMU position. The custom PCB (rev 2026-08-11, `PCB/`) is a
**circular Ø61 mm board**, sized for the Upper BT: 70 mm OD airframe less the 1.6 mm wall gives a
~66.8 mm ID, so Ø61 mm clears with **~2.9 mm radial clearance per side**.

The custom PCB has exactly **one** STEMMA-QT port, so
there is one external IMU, not the tri-IMU gimbal/body/recovery set the electronics docs elsewhere
in this project still describe (that language predates the custom PCB and hasn't been reconciled, 
see the open item in `WYVERN_E4_BUILD_READINESS.md`). These bulkhead pass-throughs are wiring holes,
not gas seals, so the FC bay is **no longer isolated from ejection gas by a dedicated bypass tube**
the way the old single-tube design was. Wadding is placed on **both** sides of the bulkhead (aft face
of the Upper BT's contents, fwd face of the Lower BT's parachute pack) as thermal/soot protection for
whatever gas reaches either side of the joint during the brief separation event.

**Open question worth flagging:** mounting the external IMU at the bulkhead boundary rather than on
the gimbal itself means it senses vehicle/joint attitude, not gimbal-relative nozzle deflection
directly. If RQ1's magnetic-vs-servo TVC comparison needs a direct gimbal-deflection measurement,
confirm that's still covered by the 3-axis load balance on the ground rigs (per
`WYVERN_E4_GSE_TestStands.md`) rather than assumed from this IMU in flight.

## 2. Why F15-4 (not F15-6), unchanged reasoning

Apogee is at **7.0 s** (burnout 3.45 s + 3.53 s coast), so the ideal ejection delay is ~3.5 s. Of the
off-the-shelf F15 delays, **F15-4** is the closest:

| Motor | Ejection | vs apogee | Deploy speed | Opening load | Verdict |
|---|---|---|---|---|---|
| **F15-4** | 7.45 s | **+0.47 s** | **~4.7 m/s** | ~6 N | near-optimal ✅ |
| F15-6 | 9.45 s | +2.47 s | ~23 m/s | ~147 N | late/hard |
| F15-8 | 11.45 s | +4.47 s | ~39 m/s, only 51 m AGL | ~408 N | unsafe (too low) |

F15-4 deploys just after apogee at a gentle ~4.7 m/s, the softest opening of the three.

## 3. Separation sequence (new, replaces the old bypass-tube gas path)

```
Nose --- Upper BT (Cam / FC / Battery / Wadding) ---[ BULKHEAD JOINT ]--- Lower BT (Wadding / Chute+Cord / TVC Bay) --- Motor
```
1. At t = 7.45 s the F15-4 fires its forward ejection charge, inside the Lower BT's TVC bay region.
2. Gas pressurizes the Lower BT against the recovery wadding and parachute pack, driving pressure up
   against the bulkhead joint.
3. The bulkhead joint releases (friction-fit / shear-pin, sized like the prior single-tube nose
   friction-fit, see §4) and the **two body tubes separate**.
4. The parachute, packed at the fwd end of the Lower BT, deploys into the gap. The shock cord
   tethers both halves (anchored in the Lower BT near the TVC bay, and in the Upper BT past the
   wadding) so the vehicle descends as one linked assembly, not two free-falling halves.
5. The servo extensions and STEMMA-QT cable running through the bulkhead holes carry the separation
   event too, see §5, this is the new open item this design introduces.

## 4. Separation force target, carried over from the old friction-fit spec

The bulkhead joint should release in the same **50–150 N** band the old single-tube nose friction-fit
used, soft enough to separate reliably off the ejection charge's pressure pulse, tight enough that
handling/flight vibration doesn't pop it early. **This is a design item that needs re-verification**,
not a number I'm changing on my own: the old FEA doc (`WYVERN_E4_FEA_Structural.md` §4.1) sized
Bulkhead B's *retention* hardware (M3 bolts) to **survive** 491 N without failing, because that
bulkhead was never meant to separate. A joint meant to separate at 50–150 N is a different structural
target entirely (friction/shear-pin calibrated to release, not bolted to hold), flagged in the FEA
doc, needs a real pass before this flies.

## 5. New open item: cable slack through the bulkhead

The servo extensions are continuous runs from the FC (Upper BT) to the gimbal servos deeper in the
Lower BT; the STEMMA-QT cable runs only as far as the external BNO085 mounted right at the joint
itself (§1), a much shorter run than a gimbal-mounted IMU would need, but it still physically
crosses the separation plane. Two ways to handle the crossing, neither yet decided:
- **Service-loop slack:** enough coiled cable on both sides that the two halves can separate their
  full shock-cord tether length without the harness going taut and yanking a connector loose or
  snapping a wire.
- **Breakaway connector at the bulkhead:** a connector pair mounted flush at the joint that disconnects
  cleanly on separation, so no wire crosses the break at all.
Given TVC control authority is already zero by the time recovery fires (thrust → 0 at burnout, well
before the 7.45 s ejection), the servos don't need to keep functioning post-separation, the only
requirement is that the cable doesn't cause a *mechanical* problem (snag, incomplete separation,
torn connector) during the event. This needs a decision and a bench pull-test before flight.

## 6. Feasibility numbers (`we4_ejection_feasibility.py`), bay-pressurization math still applies

- **Bay pressurization:** the F15 charge delivers **~140 kPa** into the Lower BT against a
  **14–41 kPa** friction-fit release window → **3.4× margin** at the old friction-fit spec. This
  should still hold for a bulkhead-joint release calibrated to the same 50–150 N band (§4), but the
  feasibility sim was written for the single-tube geometry and hasn't been re-run against the new
  two-BT volume/geometry, **flagged for a sim re-run, not silently assumed to still be exactly 3.4×**.
- **Deploy dynamics:** ~4.7 m/s at ejection (0.47 s past apogee), ~6 N opening on the 24" chute, 
  trivial for the 1/8" Kevlar cord (>1000× SF). Unchanged by the bay split.

## 7. Sealing & thermal

The **twin-bulkhead-plus-bypass-tube** concept from the old single-tube design is gone, there is now
one bulkhead, and it's expected to see combustion gas directly (that's what separates it). Wadding on
both faces is the thermal protection, same role Nomex/wadding plays in any traditional dual-deploy
rocket. **PETG-CF bulkhead** (HDT ~110 °C) with the ejection pulse being brief (~0.1 s) should be fine
thermally by the same lumped-wall logic as the old design (see FEA doc §4.3), but that calc was run
for the *bypass tube*, not this bulkhead's new direct-gas-exposure role, needs re-checking, not
assumed to carry over unchanged.

## 8. Ground test (mandatory before flight), updated for the new joint

1. **Static separation test:** fire an F15-4 (or a ground-igniter + measured BP sim charge) in the
   built two-BT airframe on the bench, restrained, confirm the bulkhead joint releases cleanly, the
   chute deploys, and the cable pass-through survives the event without damage.
2. **Cable pull-test:** with the joint held at full shock-cord extension, confirm the servo/STEMMA-QT
   harness (whichever solution from §5 is chosen) doesn't bind, snag, or take load it shouldn't.
3. **Release-force check:** confirm the bulkhead joint's actual release force lands in the 50–150 N
   band (§4), too tight and it won't separate reliably, too loose and flight vibration risks early
   separation.
4. **Chute pack + Nomex/wadding:** verify wadding on both bulkhead faces adequately shields the
   canopy and the Upper BT's electronics bay from the ejection gas.

## 9. Why this is still the best recovery approach for WYVERN-E

The core case is unchanged: the motor already carries a perfectly-timed ejection charge, so recovery
needs zero electronics, zero battery, zero pyro handling, just wadding, a chute, cord, and a joint
sized to release at the right force. Splitting into two BTs to house the custom PCB up top doesn't
change that philosophy, it just moves the separation mechanism from "friction-fit nose on one tube"
to "friction-fit joint between two tubes", a more traditional, well-proven dual-deploy-style break,
at the cost of the two new open items above (§4 release-force re-spec, §5 cable slack) that need
closing before this is flight-ready.

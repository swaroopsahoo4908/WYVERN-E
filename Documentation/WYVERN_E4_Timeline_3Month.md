# GTR70E WYVERN, 3-Month Project Timeline (2026-08-10 → 2026-12-01)

**Authors:** Swaroop Sahoo, Chris Liu, Allison Hong  
**Date:** 2026-08-12  
**Program:** GTR70E WYVERN


*Budget: 3 man-hours/week. Does not include 3D-print run time (parts print unattended in the
background) but does include CAD/design time, starting/queuing prints, and all post-processing
(support removal, heat-set inserts, sanding, assembly fit-checks). ~15.5 weeks, ~46-47 hrs total.*

## Target dates (fixed, work backward from these)
- **Launch weekend 1**: Sat–Sun **Nov 21–22, 2026** (weekend before Thanksgiving)
- **Launch weekend 2**: Sat–Sun **Nov 28–29, 2026** (weekend after Thanksgiving)
- **All data collected by**: **Dec 1, 2026**, with buffer, Nov 30–Dec 1 is slack, not a work day.
- Everything else (electronics, four ground-test stands, four RQ campaigns, airframe, full-stack
  rehearsal) must be done before Nov 21.

## Why two launch weekends, not one
Weekend 1 (Nov 21–22) is the primary flight window, both the F15-4 recovery flight and the RQ
data it produces are the goal. Weekend 2 (Nov 28–29) is the built-in contingency: weather scrub,
hardware anomaly, or a repeat flight if weekend 1's data has a gap. Data collection isn't
"complete" until whichever weekend actually produces a clean flight, hence the Dec 1 cutoff sitting
just past both dates rather than immediately after weekend 1.

## Phase map (15 weeks, 3 hrs/wk = 45 hrs, + ~2 hrs slack folded into weeks with lighter loads)

| Phase | Weeks | Dates | Hrs | Focus |
|---|---|---|---|---|
| 0. Design lock + ordering | 1–2 | Aug 10 – Aug 23 | 6 | Freeze BOM/CAD, place every order (long-lead items first) |
| 1. Electronics bring-up | 2–4 | Aug 17 – Sep 6 | 9 | Custom PCB bring-up, firmware smoke test, sensor bus checks |
| 2. Test-stand CAD + builds | 3–6 | Aug 24 – Sep 20 | 12 | CAD, print-queue, and assemble all 4 ground rigs |
| 3. Ground campaigns | 6–10 | Sep 14 – Oct 18 | 15 | Static-fire/jetvane, servo TVC, magnetic TVC, wind tunnel, data collection starts here |
| 4. Airframe build | 8–12 | Sep 28 – Oct 25 | 9 | Print (background), post-process, assemble both body tubes, recovery, avionics bay |
| 5. Integration + rehearsal | 12–14 | Oct 26 – Nov 15 | 9 | Full-stack integration, SIL/HIL checks, dry-run countdown, go/no-go review |
| 6. Launch weekend 1 | 15 | **Nov 21–22** | on-site | Primary flight, F15-4, RQ1–RQ5 flight data |
| 7. Contingency buffer | 16 (partial) | Nov 23–27 | 3 | Data triage from weekend 1; repack/repair if a repeat flight is needed |
| 8. Launch weekend 2 | 16 | **Nov 28–29** | on-site | Contingency/repeat flight only if weekend 1 left a gap |
| 9. Data reduction + report | 17 | Nov 30 – **Dec 1** | buffer | Final data pull, cross-check against all 5 RQs, close out |

Phases overlap on purpose, CAD for the wind tunnel can start while the static-fire stand is
already collecting data, for instance. The week ranges are outer bounds, not a rigid sequence.

## Week-by-week (3 hrs/week, Mon-anchored weeks)

| Wk | Dates | Hrs | Work |
|---|---|---|---|
| 1 | Aug 10–16 | 3 | Freeze BOM against the live carts; place Amazon/Adafruit/Estes/Bambu orders now, everything downstream waits on parts arriving. |
| 2 | Aug 17–23 | 3 | Order any remaining long-lead items (motors, PCB fab). Start PCB bring-up CAD/wiring checks while boards are in fab. |
| 3 | Aug 24–30 | 3 | PCB arrives → bring-up: power-on, sensor bus scan (BNO085/BME680/INA226/LIS3MDL), microSD logger smoke test. |
| 4 | Aug 31–Sep 6 | 3 | Firmware SIL pass; start CAD on servo TVC + static-fire stands (parallel). |
| 5 | Sep 7–13 | 3 | Queue prints for static-fire + servo TVC stands; CAD the magnetic TVC stand and wind tunnel rig. |
| 6 | Sep 14–20 | 3 | Post-process + assemble static-fire and servo TVC stands. First static-fire calibration firing (E16-4 commissioning). |
| 7 | Sep 21–27 | 3 | Queue prints for magnetic TVC stand + jetvane/coupon set; start servo TVC campaign (F15-0 firings, load-cell logging). |
| 8 | Sep 28–Oct 4 | 3 | Assemble magnetic TVC stand; begin airframe body-tube prints (background), post-process as they finish. |
| 9 | Oct 5–11 | 3 | Magnetic TVC campaign (RQ1 A/B data collection, mirrors week 7's servo runs). Jetvane/coupon firings on static-fire stand. |
| 10 | Oct 12–18 | 3 | Assemble wind tunnel rig; RQ3/RQ4 aerofoil runs start. Continue airframe post-processing. |
| 11 | Oct 19–25 | 3 | Finish wind tunnel campaign; recovery system (bulkhead joint, chute, wadding) assembly + ground-ejection test. |
| 12 | Oct 26–Nov 1 | 3 | Avionics bay integration into upper BT; full-stack power-on with airframe closed up. |
| 13 | Nov 2–8 | 3 | HIL/SIL regression on the integrated stack; PID gain sweep (RQ5) on the servo TVC stand with the actual flight PCB. |
| 14 | Nov 9–15 | 3 | Dry-run countdown procedure end-to-end; fix whatever the rehearsal exposes. |
| 15 | Nov 16–22 | 3 (+ on-site) | Final go/no-go review early in the week; **launch weekend 1, Nov 21–22**. |
| 16 | Nov 23–29 | 3 (+ on-site if flying) | Triage weekend-1 data against all 5 RQs; repair/repack only if needed; **contingency launch weekend 2, Nov 28–29**. |
| 17 | Nov 30–Dec 1 | buffer | Final data pull and cross-check. Nothing new gets built this week, this is pure margin. |

## Explicit buffer accounting
- **Week 16 exists only as contingency**, if weekend 1 (Nov 21–22) produces clean data across all
  five RQs, week 16 and the Nov 28–29 flight are optional, not required, and that time rolls
  straight into slack.
- **Week 17 (Nov 30–Dec 1) is unscheduled on purpose**, it exists so "all data by Dec 1" is true
  even if weekend 2 is the one that actually flies.
- Any phase that slips should eat into week 17's buffer first, then week 16's, before pushing the
  Nov 21 launch date, the launch dates are fixed, the schedule bends around them, not the reverse.

## Cross-reference
- Ground-rig build/test detail: `WYVERN_E4_GSE_TestStands.md` (four stands, RQ mapping, motor counts).
- Scope context: `CONFLICTS.md` §6 (wind tunnel + RQ3/RQ4), and the
  jetvane restoration note in `WYVERN_E4_GSE_TestStands.md` §1 / `WYVERN_E4_Build_Guide.md`.
- Procedure-level detail for any given work session: `WYVERN_E4_Timeline_14Day.md` (the original
  day-by-day plan, use it for *what to actually do* in a session; use this doc for *when*).
- Go/no-go criteria: `FLIGHT_READINESS.md`, `WYVERN_E4_BUILD_READINESS.md`.

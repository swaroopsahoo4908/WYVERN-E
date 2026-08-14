# GTR70E WYVERN, Cart Gap Analysis

**Authors:** Swaroop Sahoo, Chris Liu, Allison Hong  
**Program:** GTR70E WYVERN


### What's in the cart vs. the BOM and the design

Configuration this is checked against: ASA-Aero/PETG-CF/PC-FR zoned materials, 24 in chute, jetvane
blast-shield materials screen in the ground campaign, ground campaign = static thrust curves + MTVC
+ servo TVC + jetvane screen, flight = servo TVC only. `WYVERN_E4_BOM.xlsx` is the authoritative
line-item source; this file is the narrative read of the gaps that matter most.

---

## 1. Still flight-blocking, order today

| # | Missing | Why it stops you | Where | Est. |
|---|---|---|---|---|
| **1** | **BME688 barometer + STEMMA-QT cabling confirmed on hand** | No barometric altitude means **no apogee detection and no RQ3**. The coast-Cd reconstruction in `we4_flight_reduce.py` is computed entirely from baro altitude; `baro.h` initializes it on the shared I2C bus at 0x76. | Adafruit | confirm quantity |
| **2** | **Airframe filament stock** | ASA-Aero (Upper/Lower BT, nose) and PETG-CF (fins, bulkhead) both draw from the current print queue; confirm enough of each is on hand for the full airframe plus at least one reprint margin. | Bambu / any | ~$20–40 |
| **3** | **Estes E16-4 × 3 packs (6 motors)** | **Stand commissioning.** Both stands must be validated against a published curve before any F15-0 data run, or the RQ1 numbers aren't defensible. | Estes | $33.33 ea |
| **4** | **More STARTECH starters** | **6 starters for ~15 firings.** Buy 3–4 packs. Cheapest possible way to lose a test day. | Estes | $6.99 ea |
| **5** | **LiPo balance charger** | Confirm a charger for the 2S packs is on hand and in the BOM. | Amazon | ~$14 |
| **6** | **Decoupling kit, 1000 µF + 100 µF low-ESR + SS34 Schottky** | `COMPATIBILITY.md` §5c calls this **mandatory**. Servos and the Pico share one 5 V rail; a ~1 A servo-stall transient can brown out the flight computer mid-burn without the bulk cap and hold-up diode. | Amazon | ~$10 |
| **7** | **Arming switch/power switch spares** | U13 is the sole arming safety on the flight computer as fabricated; a spare is worth having. | Amazon | ~$2 |
| **8** | **Calibration masses to ~2.5 kg** | The axial load cell is 5 kg (≈49 N); calibrating over a small fraction of range doesn't clear the bench-calibration gate. Anything of *known* mass works, gym plates, water bottles on a kitchen scale. | — | ~$15 |

---

## 2. Confirmed on hand

Confirmed against the *Already Acquired* sheet:

| Item | Status |
|---|---|
| 1010 launch rail | ✅ on hand |
| Anemometer | ✅ on hand |
| Steel blast deflector | ✅ on hand |
| microSD SPI breakouts | ✅ on hand |
| 100 kΩ / 62 kΩ resistors | ✅ on hand |
| PETG-CF filament | ✅ on hand, comfortable margin for fins + bulkhead |
| BME688 | ✅ on hand |
| microSD cards | ✅ on hand |
| **Full magnetic-solenoid actuator (MTVC)** | ✅ on hand, 3 × 12 V electromagnets, 3 × AIRTAK 100 N, IRF520 pack, 1N4007 pack, 12 V 2 A supply — the MTVC half of RQ1 is fully covered |
| STEMMA QT cables ×6 | ✅ on hand |

---

## 3. Schedule risk, the one thing that can still sink the flight

| Item | Delivery | Needed | Status |
|---|---|---|---|
| **M2 linkage rod ends** (uxcell, 4 pcs ×2) | limited stock | Bench gate | **RE-SOURCE, see below** |
| EMAX ES08MA II servos | standard lead time | Bench gate | Find a faster source if you can |

### Re-sourced linkage rod ends

The part is a standard M2 ball-link / rod end; it is not a specialty item and there are Prime-eligible
alternatives with more pieces per pack:

| Option | Pack | Note |
|---|---|---|
| **[10 Pcs Metal M2 Link Tie Rod End Ball Joint](https://www.amazon.com/Buckle-Steering-Pushrod-Upgrade-Accessories/dp/B0F1XK7H11)** | **10** | **Recommended.** Metal, 15 mm, 2 mm bore, same geometry as the uxcell part, 10 per pack, faster ship. |
| [M2 rod ends + pushrod kit](https://www.amazon.com/Plastic-Pushrod-Connector-Airplane-M2x150mm/dp/B0FD8XVF9R) | 2 sets | Includes the pushrod and a linkage stopper, useful if you want the threaded rod as well as the ends. Plastic ends. |
| [M2/M2.5/M3 nylon ball joints (eBay)](https://www.ebay.com/itm/196503474782) | varies | Backup only, shipping timing is not reliable enough for a tight window. |
| [uxcell original](https://www.amazon.com/uxcell-Linkage-Joint-Adapter-Crawler/dp/B07Q2V3R61) | 4 | Keep as a backstop, but don't plan around it alone. |

Buy the 10-pack **and** keep the existing order. They're a few dollars, and the gimbal linkage is a
single point of failure on the bench gate.

---

## 4. Parachute, 24 in

`CHUTE_D = 0.6096 m` is set in `wyvern_datagen/core.py`, `we4_sim.py`, and `we4_validation.py`, and
every descent number is generated from it: descent rate **4.7 m/s**, well inside the 3–7 m/s
safe-landing gate with comfortable margin, shock-cord SF far in excess of what's needed. The
trade against a smaller canopy is drift — a slower descent from the same altitude spends longer in
the wind — worth keeping in mind for range/recovery-area sizing on launch day.

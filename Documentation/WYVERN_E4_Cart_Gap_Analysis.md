---
updated_at: 2026-08-01
---

# WYVERN-E — Cart Gap Analysis

### Cart of 2026-08-01 (Estes + Adafruit + Amazon, subtotal $373.15 Amazon) vs. the BOM and the design

**Short answer: yes, five things are missing that will stop you, and three of them stop you cold.**
Two more are schedule risks that don't show up as missing items at all.

Checked against `WYVERN_E4_BOM.xlsx` (both the to-buy sheet and the *Already Acquired* sheet) and
against what the firmware and simulations actually require.

---

## 1. Flight-blocking — order these today or the build stops

| # | Missing | Why it stops you | Where | Est. |
|---|---|---|---|---|
| **1** | **BMP388 barometer ×2** | No barometric altitude means **no apogee detection and no RQ3**. The entire coast-Cd reconstruction in `we4_flight_reduce.py` is computed from baro altitude — without it the flight produces no RQ3 result at all. `baro.h` initializes it on mux ch3 (0x77) and the self-test gates on `BARO_BMP`. | Adafruit **PID 3966** | $9.95 ea |
| **2** | **microSD SPI breakout** | **This is not in the BOM either — it is a BOM defect, not just a cart omission.** You own the *cards* and a USB card reader, but the Pico 2 W has no SD slot. `sd_logger.h` drives a breakout on SPI0 (SCK GP2 / MOSI GP3 / MISO GP4 / CS GP5). No breakout = no flight log = **no data from any flight.** | Adafruit PID 254, or any SPI microSD module | ~$8 |
| **3** | **Bambu ASA-Aero filament, 2 × 2 kg** | The **primary airframe material** — nose, all three bay tubes, all four fins. It is also the whole point of RQ2's thermal zoning. Your cart has PC-FR, ABS and plain PC, but no ASA-Aero, so the airframe cannot be printed. | Bambu Lab | $93.60 |
| **4** | **100 kΩ / 62 kΩ 1% resistors** | The GP26 battery divider. Without it the ADC pin floats, `BATTERY` fails self-test, and **the state machine never leaves BOOT** — you cannot arm. (This is the divider that was missing from the schematics until this pass.) | Any 1% metal-film pack | ~$7 |
| **5** | **Decoupling kit — 1000 µF + 100 µF low-ESR + SS34 Schottky** | `COMPATIBILITY.md` §5c calls this **mandatory**, not optional. Servos and the Pico share one 5 V rail; a ~1 A servo-stall transient will brown-out the flight computer mid-burn without the bulk cap and hold-up diode. | Amazon | ~$10 |

---

## 2. Will block a specific gate

| # | Missing | Blocks | Est. |
|---|---|---|---|
| 6 | **Estes E16-4 × 3 packs (6 motors)** | **Gate 5, stand commissioning.** Both stands must be validated against a known published curve before any F15-0 data run. Without these you'd be taking RQ1 data on an uncommissioned stand — the data would not be defensible. | $33.33 × 3 |
| 7 | **More STARTECH starters** | You have **6 starters for 15 planned firings.** Buy 3–4 packs. This is the cheapest possible way to lose a test day. | $6.99 ea |
| 8 | **LiPo balance charger** | You cannot charge the OVONIC 2S packs. In the BOM (HTRC B3), not in the cart. | ~$14 |
| 9 | **Arming switch ×2 (SPDT slide)** | The RBF / arming switch. `PIN_RBF` (GP22) needs something to sense. | ~$2 |
| 10 | **Larger calibration masses** | Your UCEC set tops out at **211 g ≈ 2 N**. The axial cell is 5 kg (≈49 N) and the static-stand cell is 20 kg. Calibrating a 5 kg cell over 4% of its range will not pass **Gate 4**. Add known masses to ~2.5 kg (gym plates, water bottles weighed on a kitchen scale — they just need to be *known*). | ~$15 |

---

## 3. Missing from the BOM entirely — design needs them, nobody costed them

These are not cart omissions; the BOM never had them. Worth fixing in the BOM as well as buying.

| # | Item | Why |
|---|---|---|
| 11 | **1010 launch rail / pad** | You have the Pro Series II *controller* and printed 1010 rail buttons — but no rail. Rail-exit velocity is a validation gate (6.1 m/s off a 1 m rail); you need the actual rail. |
| 12 | **Nomex chute protector** | The design specifies a Nomex blanket shielding the canopy from ejection gas. Recovery wadding is **not** equivalent for a bypass-tube gas path that dumps directly into the recovery bay. |
| 13 | **Anemometer / handheld wind meter** | `we4_flight_reduce.py --wind` needs the **measured** surface wind at launch, and the range go/no-go has a hard 5 m/s limit. Without a real measurement, RQ3's margin reconstruction cannot be run and the weather call is a guess. |
| 14 | **Steel blast deflector plate** | The static stand design calls for steel. The CAD generates a printable one, but PC-FR will not survive an F15 plume at close range. A cut mild-steel or stainless plate is fine. |

---

## 4. Schedule risks hiding inside the cart

Two items in the cart have delivery dates that land **after** the days they are needed:

| Item | Delivery | Needed | Problem |
|---|---|---|---|
| **uxcell M2 linkage rod ends** | **Aug 12–21** | Day 7 bench (Aug 8) | **Worst risk in the cart.** These connect the servo to the nozzle. Arriving Aug 12–21 puts them at or past the flight window, and the listing says *"only 2 left in stock."* **Source these locally or from a second supplier today.** |
| **EMAX ES08MA II servos** | **Aug 8–14** | Day 7 bench (Aug 8) | Best case they arrive the morning you need them; worst case Day 7 slips to Day 13. Find a faster source. |
| 5 kg load cell + HX711 kit | Aug 8–13 | Day 7 stand cal | Same shape of problem, but Track B has more slack than Track A. |

Everything else in the cart is Aug 6 delivery, which fits.

---

## 5. Good news — these are already covered

I checked the *Already Acquired* sheet before calling anything missing. You own:

- **PCA9548A I2C mux** — the whole I2C0 architecture depends on it (two BNO085s share address 0x4A and must be mux-isolated). Not in the cart, and doesn't need to be.
- **BME688 ×2** — the second barometer.
- **microSD cards ×2** + USB reader (you still need the SPI *breakout*, item 2).
- **The entire magnetic-solenoid actuator for RQ1** — 3 × 12 V electromagnets, 3 × AIRTAK 100 N solenoids, IRF520 pack, 1N4007 flyback pack, 12 V 2 A supply. **RQ1's A/B comparison is fully covered.** I flagged this because without it there would be no "A" in the A/B.
- STEMMA QT cables ×6, epoxy, PTFE grease, shim stock, breadboard/jumpers.

And in the cart itself, correctly covered: Pico 2 W ×2 (with spare), BNO085 ×4 (3 + spare), all load cells,
HX711 ×5 (need 4), bearings, servo extensions, perma-proto, solder, polyimide tape, PC-FR, ABS, PC,
camera, F15-4 ×4, F15-0 ×10, engine plugs, launch controller, aramid cord, wadding.

---

## 6. One thing you got right that the BOM had wrong

Your cart has **4 × 18-inch parachutes**. The BOM line says *24 inch × 3*.

**The 18 inch is correct.** Every simulation uses `CHUTE_D = 0.4572 m` (18 in) and the 6.2 m/s
descent rate, the shock-cord safety factor and the landing-dispersion results all follow from it. A
24 in canopy would descend near 4.5 m/s and drift proportionally further — different numbers than
every document quotes. The BOM line is a defect; I've corrected it to 18 in and kept 24 in as an
explicitly-labelled spare.

---

## 7. What to add to the order right now

**Flight-blocking (items 1–5) — nothing works without these:**

- [ ] BMP388 × 2 — Adafruit PID 3966 — $19.90
- [ ] microSD SPI breakout × 1 — ~$8
- [ ] Bambu ASA-Aero 2 kg × 2 — $93.60
- [ ] 100 kΩ + 62 kΩ 1% resistor pack — ~$7
- [ ] Decoupling kit: 1000 µF + 100 µF low-ESR + SS34 — ~$10

**Gate-blocking (items 6–10):**

- [ ] Estes E16-4 2-ct × 3 — ~$100
- [ ] STARTECH starters × 3 more packs — ~$21
- [ ] LiPo balance charger — ~$14
- [ ] SPDT slide switch × 2 — ~$2
- [ ] Calibration masses to ~2.5 kg — ~$15

**Never costed (items 11–14):**

- [ ] 1010 launch rail — ~$30
- [ ] Nomex chute protector — ~$8
- [ ] Handheld anemometer — ~$20
- [ ] Steel blast deflector plate — ~$15

**Today, separately from the order:** re-source the **M2 linkage rod ends** and the **servos** from a
supplier that can deliver by Aug 7. These two parts sit directly on the Day-7 bench gate, and no
amount of schedule discipline recovers a part that arrives on Aug 21.

Rough addition: **~$365** on top of the current cart.

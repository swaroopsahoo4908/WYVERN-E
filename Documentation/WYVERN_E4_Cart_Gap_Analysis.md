---
updated_at: 2026-08-01
---

# WYVERN-E — Cart Gap Analysis (rev 2)

### Cart of 2026-08-01 vs. the BOM and the design, **after** the 2026-08 scope + material change

**Rev 2 supersedes rev 1.** The scope change (jetvane dropped, materials moved to PLA / PETG-CF)
removed four items from the missing list and made three items in your cart unnecessary. Net effect:
**you need less than rev 1 said, and about $98 of what you already have in the cart should come out.**

Configuration this is checked against: PLA primary structure, PETG-CF for the ejection-gas path and
TVC assemblies, 24 in chute, no jetvane, ground campaign = static thrust curves + MTVC + servo TVC,
flight = servo TVC only.

---

## 1. Still flight-blocking — order today

| # | Missing | Why it stops you | Where | Est. |
|---|---|---|---|---|
| **1** | **BMP388 barometer ×2** | No barometric altitude means **no apogee detection and no RQ3**. The coast-Cd reconstruction in `we4_flight_reduce.py` is computed entirely from baro altitude. `baro.h` initializes it on mux ch3 (0x77); the self-test gates on `BARO_BMP`. | Adafruit **PID 3966** | $9.95 ea |
| **2** | **More PLA filament** | You own 2 kg (1 white + 1 black). The airframe needs ~195 g of PLA, but that is *finished part* mass — with supports, brims, purge and at least one reprint of something, 2 kg is thin for a two-week build with no second shipping cycle. Add 1–2 kg. | Bambu / any | ~$20–40 |
| **3** | **Estes E16-4 × 3 packs (6 motors)** | **Gate 5, stand commissioning.** Both stands must be validated against a published curve before any F15-0 data run, or the RQ1 numbers aren't defensible. | Estes | $33.33 ea |
| **4** | **More STARTECH starters** | **6 starters for ~15 firings.** Buy 3–4 packs. Cheapest possible way to lose a test day. | Estes | $6.99 ea |
| **5** | **LiPo balance charger** | You cannot charge the OVONIC 2S packs. In the BOM (HTRC B3), not in the cart. | Amazon | ~$14 |
| **6** | **Decoupling kit — 1000 µF + 100 µF low-ESR + SS34 Schottky** | `COMPATIBILITY.md` §5c calls this **mandatory**. Servos and the Pico share one 5 V rail; a ~1 A servo-stall transient browns out the flight computer mid-burn without the bulk cap and hold-up diode. | Amazon | ~$10 |
| **7** | **Arming switch ×2 (SPDT slide)** | `PIN_RBF` (GP22) needs something to sense. | Amazon | ~$2 |
| **8** | **Calibration masses to ~2.5 kg** | Your UCEC set tops out at **211 g ≈ 2 N**. The axial cell is 5 kg (≈49 N). Calibrating over 4% of range will not pass **Gate 4**. Anything of *known* mass works — gym plates, water bottles on a kitchen scale. | — | ~$15 |

---

## 2. Now resolved — you have these

Confirmed against the *Already Acquired* sheet:

| Item | Status |
|---|---|
| 1010 launch rail | ✅ you have it |
| Anemometer | ✅ you have it |
| Steel blast deflector | ✅ you have it |
| microSD SPI breakouts | ✅ you have them |
| 100 kΩ / 62 kΩ resistors | ✅ you have them |
| **PETG-CF filament** | ✅ **4 kg owned** (Bambu Black, 2 × 2 kg). Needs ~300 g. Comfortable. |
| PCA9548A I2C mux | ✅ owned |
| BME688 ×2 | ✅ owned |
| microSD cards ×2 | ✅ owned |
| **Full magnetic-solenoid actuator (MTVC)** | ✅ owned — 3 × 12 V electromagnets, 3 × AIRTAK 100 N, IRF520 pack, 1N4007 pack, 12 V 2 A supply. **The MTVC half of RQ1 is fully covered.** |
| STEMMA QT cables ×6 | ✅ owned |

---

## 3. Take these OUT of the cart — the scope change made them unnecessary

| Item | In cart | Why it's no longer needed |
|---|---|---|
| **kexcelled PC-FR filament** | $49.99 | PC-FR is gone from the design. PETG-CF replaces it in the gas path and TVC assemblies, and you already own 4 kg. |
| **SUNLU PC (natural white)** | $30.39 | Was for pure-PC material coupons in the old zoned-materials RQ2. That RQ is now PLA vs PETG-CF. |
| **Creality ABS** | $17.48 | Was **jetvane coupon material only**. Jetvane testing is dropped entirely. |
| 18 in parachutes ×4 | $36.00 | Superseded — see §5. Keep if you want spares, but the flight canopy is 24 in. |

**Removing the three filaments saves $97.86.**

---

## 4. Schedule risk — the one thing that can still sink the flight

| Item | Delivery | Needed | Status |
|---|---|---|---|
| **M2 linkage rod ends** (uxcell, 4 pcs ×2) | **Aug 12–21**, *"only 2 left in stock"* | Day 7 bench, Aug 8 | **RE-SOURCE — see below** |
| EMAX ES08MA II servos | Aug 8–14 | Day 7 bench | Find a faster source if you can |

### Re-sourced linkage rod ends

The part is a standard M2 ball-link / rod end; it is not a specialty item and there are Prime-eligible
alternatives with more pieces per pack:

| Option | Pack | Note |
|---|---|---|
| **[10 Pcs Metal M2 Link Tie Rod End Ball Joint](https://www.amazon.com/Buckle-Steering-Pushrod-Upgrade-Accessories/dp/B0F1XK7H11)** | **10** | **Recommended.** Metal, 15 mm, 2 mm bore — same geometry as the uxcell part, 10 per pack instead of 4, and not on a 3-week ship. |
| [M2 rod ends + pushrod kit](https://www.amazon.com/Plastic-Pushrod-Connector-Airplane-M2x150mm/dp/B0FD8XVF9R) | 2 sets | Includes the pushrod and a linkage stopper — useful if you want the threaded rod as well as the ends. Plastic ends. |
| [M2/M2.5/M3 nylon ball joints (eBay)](https://www.ebay.com/itm/196503474782) | varies | Backup only — eBay SpeedPAK timing is not reliable enough for this window. |
| [uxcell original](https://www.amazon.com/uxcell-Linkage-Joint-Adapter-Crawler/dp/B07Q2V3R61) | 4 | Keep the order as a backstop, but do not plan around it. |

Buy the 10-pack **and** keep the existing order. They're a few dollars, and the gimbal linkage is a
single point of failure on the bench gate.

---

## 5. Parachute — 24 in confirmed

**Switched to 24 in and the simulations now match.** `CHUTE_D = 0.6096 m` is set in
`wyvern_datagen/core.py`, `we4_sim.py` and `we4_validation.py`, and every descent number has been
regenerated from it:

| | 18 in (old) | **24 in (now)** |
|---|---|---|
| Descent rate | 6.7 m/s | **5.0 m/s** |
| Shock-cord SF | 152× | 133× |
| Drift per m/s of wind | higher rate, less time | lower rate, ~35% more drift |

5.0 m/s is inside the 3–7 m/s safe-landing gate with more margin than the 18 in canopy had. The cost
is drift: a slower descent from the same altitude spends longer in the wind, so the landing-dispersion
figures grew. That trade is worth it on a vehicle carrying a camera and a flight computer.

Your cart has 2 × 24 in — **order 2 more** so you have a spare per flight.

---

## 6. Revised order

**Add:**

- [ ] BMP388 × 2 — Adafruit PID 3966 — $19.90
- [ ] PLA filament × 1–2 kg — ~$20–40
- [ ] Estes E16-4 2-ct × 3 — ~$100
- [ ] STARTECH starters × 3 packs — ~$21
- [ ] LiPo balance charger — ~$14
- [ ] Decoupling kit (1000 µF + 100 µF + SS34) — ~$10
- [ ] SPDT slide switch × 2 — ~$2
- [ ] Calibration masses to ~2.5 kg — ~$15
- [ ] 24 in parachute × 2 more — ~$20
- [ ] **M2 ball-link 10-pack (the re-sourced one)** — ~$10

**Remove:** PC-FR filament (−$49.99), SUNLU PC (−$30.39), ABS (−$17.48), and the 18 in chutes if you
don't want them as spares (−$36.00).

**Net: roughly +$134 of additions against −$98 of removals ≈ +$36 on the current cart.**

The scope change paid for most of what was missing.

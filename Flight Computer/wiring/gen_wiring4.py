#!/usr/bin/env python3
"""WYVERN-E 4.0 wiring — KiCad-7 .kicad_sch (flat netlist via global labels). Raspberry Pi Pico 2 W
flight harness + 3-axis TVC-balance harness. No symbol library required (documentation schematic)."""
import os
HERE=os.path.dirname(os.path.abspath(__file__))
def esc(s): return s.replace('"','\\"')
def box(x,y,title,pins,w=48):
    h=6+len(pins)*4; it=[f'(rectangle (start {x} {y}) (end {x+w} {y+h}) (stroke (width 0.25)(type solid)) (fill (type none)))',
        f'(text "{esc(title)}" (at {x+2} {y+3} 0)(effects (font (size 1.6 1.6))(justify left)))']
    yy=y+8
    for p in pins:
        it.append(f'(text "{esc(p)}" (at {x+2} {yy} 0)(effects (font (size 1.2 1.2))(justify left)))')
        it.append(f'(global_label "{esc(p.split(":")[0].strip())}" (shape input)(at {x+w} {yy} 0)(effects (font (size 1.0 1.0))(justify left)))')
        it.append(f'(wire (pts (xy {x+w-1} {yy}) (xy {x+w} {yy})) (stroke (width 0.15)(type solid)))'); yy+=4
    return it,h
def sch(title,mods):
    items=[]; x=20; y=20
    for m in mods:
        t,h=box(x,y,m[0],m[1],m[2] if len(m)>2 else 48); items+=t; y+=h+8
        if y>250: y=20; x+=72
    body="\n  ".join(items)
    return f'''(kicad_sch (version 20230121) (generator "wyvern4_wiring") (paper "A2")
  (title_block (title "{esc(title)}") (company "Skylight Rocketry") (rev "4.0"))
  (lib_symbols)
  {body}
  (sheet_instances (path "/" (page "1"))))'''
FLIGHT=[
 ("POWER",["VBAT: 2S LiPo 7.4V +","GND","V5: 5V UBEC -> Pico VSYS/cam/servos","decouple: 1000uF@servos, 100uF+SS34->VSYS"],54),
 ("RPi PICO 2 W (RP2350, FC + real-time TVC)",["VSYS: 5V in","3V3: sensor rail","GND",
   "GP16/GP17: I2C0 -> PCA9548A mux","GP18/GP19: I2C1 -> gimbal BNO085","GP2/3/4/5: SPI0 microSD (SCK/MOSI/MISO/CS)",
   "GP14: PWM servo1 (pitch)","GP15: PWM servo2 (yaw)","GP8: CAM_EN gate","GP7: LAUNCH_IRQ","GP22: RBF sense","GP6/GP1: spare (RRC3 removed — motor ejection)","GP9: LED  GP10: buzzer","CYW43: WiFi/BLE bench telemetry"],68),
 ("PCA9548A I2C MUX (0x70, on I2C0)",["ch0: body BNO085 0x4A","ch1: recovery BNO085 0x4A (vote)","ch2: BME688 0x76","ch3: BMP388 0x77 (Adafruit 3966, 3V3)","ch4: spare (unpopulated)"],58),
 ("IMUs x3 (BNO085, Game Rotation Vector)",["gimbal: 0x4A I2C1 (dedicated)","body(FC): 0x4A mux ch0","recovery: 0x4A mux ch1 (vote)"],56),
 ("STORAGE — microSD (SPI0 breakout)",["SCK GP2 / MOSI GP3 / MISO GP4 / CS GP5","3V3 / GND: full-rate flight log"],52),
 ("Action camera (self-contained)",["V5: gated by CAM_EN (GP8)","GND: records to own microSD"],54),
 ("TVC SERVOS (2-axis gimbal)",["S1_SIG: GP14","S2_SIG: GP15","+5V (UBEC rail)","GND"],50),
]
# Ground-rig DAQ MCU is a Raspberry Pi Pico / Pico 2 W (Arduino-Pico core) per wyvern4_gse_servo_rig.ino
# and wyvern4_gse_solenoid_rig.ino, NOT the Arduino Nano/Teensy this file's older revisions specified.
# See CONFLICTS.md item 3 for the record of that supersession. Pin numbers below are pulled directly
# from each sketch's header PIN MAP comment -- keep in sync if the sketches' pin maps ever change.
BAL_SERVO=[
 ("RPi PICO / PICO 2 W — servo-rig DAQ",["3V3: HX711 + BNO085 power","GND","USB: CSV log to host",
   "GP4/GP5: I2C0 -> gimbal BNO085 (0x4A)","GP14: PWM servo1 (pitch)","GP15: PWM servo2 (yaw)",
   "LED_BUILTIN: tare-complete blink"],64),
 ("LOAD CELLS + HX711 x3 (bit-banged)",["Z (axial, 5kg): DT GP10 / SCK GP11","X (lateral, 1kg): DT GP6 / SCK GP7","Y (lateral, 1kg): DT GP8 / SCK GP9"],56),
 ("GIMBAL BNO085 (Game Rotation Vector)",["I2C0 0x4A: nozzle-angle cross-check vs. commanded servo pos"],52),
 ("SERVO GIMBAL UNDER TEST",["S1_SIG: GP14 (pitch)","S2_SIG: GP15 (yaw)","VSERVO","GND"],50),
]
BAL_SOLENOID=[
 ("RPi PICO / PICO 2 W — solenoid-rig DAQ",["3V3: HX711 + BNO085 power","GND","USB: CSV log to host",
   "GP4/GP5: I2C0 -> BNO085 (gimbal attitude)","GP16-19: solenoid PWM -> 4x IRF520 gate"],64),
 ("LOAD CELLS + HX711 x3 (bit-banged)",["Z (axial, 5kg): DT GP10 / SCK GP11","X (lateral, 1kg): DT GP12 / SCK GP13","Y (lateral, 1kg): DT GP14 / SCK GP15"],56),
 ("GIMBAL BNO085 (I2C0 0x4A)",["Game Rotation Vector + gyro rate -> deflection from 3-axis load balance"],48),
 ("SOLENOIDS x4 via IRF520 (+ 1N4007 flyback EACH)",["PITCH+: GP16","PITCH-: GP17","YAW+: GP18","YAW-: GP19","V12: coil supply"],62),
]
open("WYVERN_E4_flight_harness.kicad_sch","w").write(sch("WYVERN-E 4.0 — RPi Pico 2 W flight harness",FLIGHT))
open("WYVERN_E4_tvc_balance_servo_harness.kicad_sch","w").write(sch("WYVERN-E 4.0 — TVC balance harness (servo rig, Pico)",BAL_SERVO))
open("WYVERN_E4_tvc_balance_solenoid_harness.kicad_sch","w").write(sch("WYVERN-E 4.0 — TVC balance harness (solenoid rig, Pico)",BAL_SOLENOID))
for f in ("WYVERN_E4_flight_harness.kicad_sch","WYVERN_E4_tvc_balance_servo_harness.kicad_sch","WYVERN_E4_tvc_balance_solenoid_harness.kicad_sch"):
    s=open(f).read(); print(f,"parens",s.count("("),"==",s.count(")"),"OK" if s.count("(")==s.count(")") else "BAD")

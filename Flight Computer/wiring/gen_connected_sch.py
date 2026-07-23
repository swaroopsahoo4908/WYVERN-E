#!/usr/bin/env python3
"""WYVERN-E 4.0 — fully-routed flight wiring schematic (KiCad-7 .kicad_sch).
Unlike the flat-netlist harness, every component is DRAWN and PHYSICALLY WIRED pin-to-pin with
orthogonal wire segments, junctions, power rails (2S LiPo -> 5V UBEC / 3V3 / GND) and net labels.
No symbol library required — components are documentation rectangles with real pin stubs + wires."""
import os
HERE=os.path.dirname(os.path.abspath(__file__))
S=[]  # s-expr item accumulator
def esc(x): return str(x).replace('"','\\"')
def rect(x,y,w,h):
    S.append(f'(rectangle (start {x} {y}) (end {x+w} {y+h}) (stroke (width 0.25)(type solid)) (fill (type none)))')
def text(x,y,t,sz=1.3,j="left"):
    S.append(f'(text "{esc(t)}" (at {x} {y} 0)(effects (font (size {sz} {sz}))(justify {j})))')
def wire(x1,y1,x2,y2,w=0.15):
    S.append(f'(wire (pts (xy {x1} {y1}) (xy {x2} {y2})) (stroke (width {w})(type solid)))')
def poly(pts,w=0.15):
    for i in range(len(pts)-1): wire(*pts[i],*pts[i+1],w)
def junc(x,y):
    S.append(f'(junction (at {x} {y}) (diameter 0.9) (color 0 0 0 0))')
def label(x,y,t,sz=1.0,ang=0):
    S.append(f'(label "{esc(t)}" (at {x} {y} {ang})(effects (font (size {sz} {sz}))(justify left)))')

PINLEN=3.0
class Comp:
    def __init__(self,name,x,y,w,h,subtitle=""):
        self.x,self.y,self.w,self.h=x,y,w,h; self.an={}
        rect(x,y,w,h); text(x+1.5,y+3.5,name,1.5)
        if subtitle: text(x+1.5,y+h-1.5,subtitle,1.0)
    def pins(self,side,names,y0=None,y1=None,x0=None,x1=None):
        n=len(names)
        for i,nm in enumerate(names):
            f=(i+1)/(n+1)
            if side=="L":
                py=self.y+self.h*f; px=self.x
                a=(px-PINLEN,py); wire(px,py,a[0],a[1]); text(px+1,py-0.6,nm,1.0)
            elif side=="R":
                py=self.y+self.h*f; px=self.x+self.w
                a=(px+PINLEN,py); wire(px,py,a[0],a[1]); text(px-1,py-0.6,nm,1.0,"right")
            elif side=="T":
                px=self.x+self.w*f; py=self.y
                a=(px,py-PINLEN); wire(px,py,a[0],a[1]); text(px,py-PINLEN-0.6,nm,0.9)
            elif side=="B":
                px=self.x+self.w*f; py=self.y+self.h
                a=(px,py+PINLEN); wire(px,py,a[0],a[1]); text(px,py+PINLEN+2.0,nm,0.9)
            self.an[nm]=a
    def p(self,nm): return self.an[nm]

# ---------------- layout ----------------
# Power rails (horizontal): single 5V UBEC rail top (feeds VSYS + servos), 3V3 below, GND at bottom.
# 2S LiPo -> one 5V/6V UBEC (set 5V) -> single 5V rail (servos run at 5V).
RAIL_5V=18; RAIL_3V3=26; RAIL_GND=286
RX0,RX1=40,395
for ry,nm in [(RAIL_5V,"+5V"),(RAIL_3V3,"+3V3"),(RAIL_GND,"GND")]:
    wire(RX0,ry,RX1,ry,0.3); label(RX0-8,ry-0.6,nm,1.3)

def tap3v3(x): wire(x,RAIL_3V3,x,RAIL_3V3,0.15); junc(x,RAIL_3V3)
def tapV(x,ry): junc(x,ry)

# ----- power source (far left): 2S LiPo -> 5V UBEC -----
batt=Comp("2S LiPo 7.4V",42,55,46,22,"~450mAh flight pack")
batt.pins("T",["+"] ,); batt.pins("B",["GND"])
# 2S -> arming switch/fuse -> 5V/6V UBEC (set 5V); 1000uF bulk @ servos, 100uF + SS34 Schottky @ VSYS
bec=Comp("SW+fuse / 5V UBEC",42,100,46,22,"5V -> Pico+cam+servos")
bec.pins("L",["VIN","G"]); bec.pins("T",["+5V"]); bec.pins("B",["GND"])

# battery -> UBEC input
poly([batt.p("+"),(batt.p("+")[0],48),(34,48),(34,bec.p("VIN")[1]),bec.p("VIN")])
poly([batt.p("GND"),(batt.p("GND")[0],88),(30,88),(30,bec.p("G")[1]),bec.p("G")])
# UBEC +5V -> 5V rail ; pack/UBEC GND -> GND rail
poly([bec.p("+5V"),(bec.p("+5V")[0],RAIL_5V)]); junc(bec.p("+5V")[0],RAIL_5V)
poly([bec.p("GND"),(bec.p("GND")[0],RAIL_GND)]); junc(bec.p("GND")[0],RAIL_GND)
poly([batt.p("GND"),(batt.p("GND")[0],RAIL_GND)]); junc(batt.p("GND")[0],RAIL_GND)

# ----- Pico 2 W (central hub) -----
pico=Comp("RPi PICO 2 W",150,40,70,200,"RP2350 dual-M33 150MHz · WiFi/BLE")
pico.pins("T",["VSYS","3V3OUT"])
pico.pins("B",["GND"])
pico.pins("R",["GP2 SCK","GP3 MOSI","GP4 MISO","GP5 CS","GP14 S1","GP15 S2",
               "GP16 SDA0","GP17 SCL0","GP18 SDA1","GP19 SCL1",
               "GP6 spare","GP7 IRQ","GP8 CAM","GP9 LED","GP10 BUZ","GP22 RBF","GP1 spare"])
# Pico power to rails
poly([pico.p("VSYS"),(pico.p("VSYS")[0],RAIL_5V)]); junc(pico.p("VSYS")[0],RAIL_5V)
poly([pico.p("3V3OUT"),(pico.p("3V3OUT")[0],RAIL_3V3)]); junc(pico.p("3V3OUT")[0],RAIL_3V3)
poly([pico.p("GND"),(pico.p("GND")[0],RAIL_GND)]); junc(pico.p("GND")[0],RAIL_GND)

# ----- microSD (SPI0) top-right -----
sd=Comp("microSD (SPI0)",250,40,60,40,"flight log")
sd.pins("L",["SCK","MOSI","MISO","CS"]); sd.pins("T",["3V3"]); sd.pins("B",["GND"])

# ----- PCA9548A mux -----
mux=Comp("PCA9548A MUX 0x70",250,95,60,70,"I2C0 trunk -> 5 ch")
mux.pins("L",["SDA","SCL"]); mux.pins("T",["3V3"]); mux.pins("B",["GND"])
mux.pins("R",["c0 SD","c0 SC","c1 SD","c1 SC","c2 SD","c2 SC","c3 SD","c3 SC"])

# ----- gimbal BNO085 (dedicated I2C1) -----
gim=Comp("BNO085 GIMBAL 0x4A",250,178,60,30,"I2C1 · GRV")
gim.pins("L",["SDA","SCL"]); gim.pins("T",["3V3"]); gim.pins("B",["GND"])

# ----- servos -----
sv1=Comp("SERVO 1 (pitch)",250,214,58,18)
sv1.pins("L",["SIG","+5V","GND"])
sv2=Comp("SERVO 2 (yaw)",250,236,58,18)
sv2.pins("L",["SIG","+5V","GND"])

# ----- sensors behind mux (right) -----
body=Comp("BNO085 BODY 0x4A",340,70,58,24,"mux c0 · GRV")
body.pins("L",["SDA","SCL"]); body.pins("T",["3V3"]); body.pins("B",["GND"]); body.pins("R",["INT"])
recov=Comp("BNO085 RECOVERY 0x4A",340,104,58,24,"mux c1 · vote")
recov.pins("L",["SDA","SCL"]); recov.pins("T",["3V3"]); recov.pins("B",["GND"])
bme=Comp("BME688 0x76",340,138,58,24,"mux c2")
bme.pins("L",["SDA","SCL"]); bme.pins("T",["3V3"]); bme.pins("B",["GND"])
bmp=Comp("BMP388 0x77 (3966)",340,170,58,24,"mux c3 · 3V3")
bmp.pins("L",["SDA","SCL"]); bmp.pins("T",["3V3"]); bmp.pins("B",["GND"])

# ----- camera -----
cam=Comp("i3 4K thumb cam",96,40,46,30,"self-contained")
cam.pins("R",["V5_EN","GND"])

# ================= WIRING =================
def chan(a,b,cx,lbl=None,lj=False):
    # route a -> vertical channel cx -> b (orthogonal), optional net label near a
    poly([a,(cx,a[1]),(cx,b[1]),b])
    if lbl: label(a[0]+ (4 if a[0]<cx else -4), a[1]-0.6, lbl)

# SPI0 Pico -> microSD (4 nets), channels 226..234
for pin,sdpin,cx,nm in [("GP2 SCK","SCK",226,"SCK"),("GP3 MOSI","MOSI",229,"MOSI"),
                        ("GP4 MISO","MISO",232,"MISO"),("GP5 CS","CS",235,"CS")]:
    chan(pico.p(pin),sd.p(sdpin),cx,nm)

# servo signals Pico GP14/GP15 -> servo SIG (long run to right), channels 238..241
chan(pico.p("GP14 S1"),sv1.p("SIG"),243,"SERVO1")
chan(pico.p("GP15 S2"),sv2.p("SIG"),246,"SERVO2")

# I2C0 Pico GP16/GP17 -> mux SDA/SCL, channels 226..230 (lower)
chan(pico.p("GP16 SDA0"),mux.p("SDA"),227,"SDA0")
chan(pico.p("GP17 SCL0"),mux.p("SCL"),230,"SCL0")

# I2C1 Pico GP18/GP19 -> gimbal, channels 233..236
chan(pico.p("GP18 SDA1"),gim.p("SDA"),233,"SDA1")
chan(pico.p("GP19 SCL1"),gim.p("SCL"),236,"SCL1")

# GPIO: GP7 IRQ <- body INT ; GP8 CAM -> cam V5_EN ; GP6/GP1 spare (RRC3 removed — motor ejection)
# GP7 IRQ from body INT (body is far right; route along y of GP7)
poly([pico.p("GP7 IRQ"),(244,pico.p("GP7 IRQ")[1]),(244,82),(body.p("INT")[0],82),body.p("INT")])
label(pico.p("GP7 IRQ")[0]+4,pico.p("GP7 IRQ")[1]-0.6,"LAUNCH_IRQ")
# GP8 CAM -> camera V5_EN (camera top-left). route up-left over Pico
poly([pico.p("GP8 CAM"),(246,pico.p("GP8 CAM")[1]),(246,300)])  # placeholder removed below
S.pop()  # drop stray
poly([pico.p("GP8 CAM"),(248,pico.p("GP8 CAM")[1]),(248,33),(cam.p("V5_EN")[0]+8,33),(cam.p("V5_EN")[0]+8,cam.p("V5_EN")[1]),cam.p("V5_EN")])
label(pico.p("GP8 CAM")[0]+4,pico.p("GP8 CAM")[1]-0.6,"CAM_EN")
# GP9 LED, GP10 BUZ, GP22 RBF -> short labeled stubs (local I/O)
for pin,nm in [("GP9 LED","LED"),("GP10 BUZ","BUZZER"),("GP22 RBF","RBF_SAFE")]:
    a=pico.p(pin); poly([a,(a[0]+8,a[1])]); label(a[0]+9,a[1]-0.6,nm)


# mux channels -> sensors (each SDA/SCL pair)
def muxto(csd,csc,comp,cx):
    chan(mux.p(csd),comp.p("SDA"),cx)
    chan(mux.p(csc),comp.p("SCL"),cx+2.5)
muxto("c0 SD","c0 SC",body,318)
muxto("c1 SD","c1 SC",recov,318)
muxto("c2 SD","c2 SC",bme,318)
muxto("c3 SD","c3 SC",bmp,318)


# ----- power-rail taps: 3V3 (top pins up to 3V3 rail), GND (bottom pins down to GND rail) -----
def to3v3(comp,pin="3V3"):
    a=comp.p(pin); poly([a,(a[0],RAIL_3V3)]); junc(a[0],RAIL_3V3)
def toGND(comp,pin="GND"):
    a=comp.p(pin); poly([a,(a[0],RAIL_GND)]); junc(a[0],RAIL_GND)
for c in [sd,mux,gim,body,recov,bme,bmp]: to3v3(c)
for c in [sd,mux,gim,body,recov,bme,bmp]: toGND(c)
# servo +5V/GND to the shared 5V UBEC rail (servos run at 5V, bulk cap at the servos)
for sv in [sv1,sv2]:
    a=sv.p("+5V"); poly([a,(a[0],RAIL_5V)]); junc(a[0],RAIL_5V)
    g=sv.p("GND"); poly([g,(g[0]-2,g[1]),(g[0]-2,RAIL_GND)]); junc(g[0]-2,RAIL_GND)
# camera GND
g=cam.p("GND"); poly([g,(g[0]+12,g[1]),(g[0]+12,RAIL_GND)]); junc(g[0]+12,RAIL_GND)

# title + notes
text(40,8,"WYVERN-E 4.0 — Flight Wiring (fully routed, all components connected)",2.4)
text(40,300,"All sensors GRV (mag off). 3 BNO085 @0x4A: gimbal on dedicated I2C1; body/recovery isolated on PCA9548A ch0/ch1. RP2350 is 3.3V logic; all STEMMA-QT sensors (incl. BMP388) run at 3.3V.",1.1)

body_s="\n  ".join(S)
out=f'''(kicad_sch (version 20230121) (generator "wyvern4_connected") (paper "A1")
  (title_block (title "WYVERN-E 4.0 Flight Wiring — fully routed") (company "Skylight Rocketry") (rev "4.0"))
  (lib_symbols)
  {body_s}
  (sheet_instances (path "/" (page "1"))))'''
p=os.path.join(HERE,"WYVERN_E4_flight_wiring_connected.kicad_sch")
open(p,"w").write(out)
print("wrote",p,"| parens",out.count("("),"==",out.count(")"),"OK" if out.count("(")==out.count(")") else "BAD")
print("wires:",sum(1 for i in S if i.startswith("(wire")),"junctions:",sum(1 for i in S if i.startswith("(junction")),
      "rects:",sum(1 for i in S if i.startswith("(rectangle")))

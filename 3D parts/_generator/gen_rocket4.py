#!/usr/bin/env python3
"""WYVERN-E — 70 mm single-stage FINNED TVC sustainer (F15-4, motor-ejection recovery via bypass tube) + 3-axis TVC balance + static deflector.
Material zoning as of the 2026-08 change:
  PETG-CF: both bulkheads (A+B), bypass tube, and the engine assembly (engine/TVC bay + motor mount
           + gimbal) -- i.e. the whole ejection-gas path and the main TVC assemblies (HDT ~80 C).
  PLA:     nose, body tube, FC/recovery bays, fins (everything outside the gas path).
This supersedes the earlier ASA-Aero / PC-FR zoning; part suffixes are _PLA and _PETGCF."""
import os, json
import math
from wcad import S, cyl, tube, cone, box, sphere, ogive_nose, fin, _revolve, export_step, export_stl
def ellipsoid_nose(R,L,wall=None,z=0):
    n=60; pts=[(max(R*math.sqrt(max(1-(L*i/n/L)**2,0.0)),0.0), L*i/n) for i in range(n+1)]
    return _revolve(pts,z,wall)
ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),"..",".."))
DROCK=os.path.join(ROOT,"3D parts"); DSTAND=os.path.join(ROOT,"Motor Test Stand","TVC Thrust-Vector Balance")
DSTATIC=os.path.join(ROOT,"Motor Test Stand","Static Stand (thrust + deflector)")
for d in (DROCK,DSTAND,DSTATIC): os.makedirs(d,exist_ok=True)
# ================================================================================================
# MATERIAL CHANGE 2026-08 — ASA-Aero / PC-FR  ->  PLA / PETG-CF
# ================================================================================================
# The thermally-zoned foamed-ASA + flame-rated-PC scheme is retired. New allocation:
#
#   PLA      nose, both PLA bay tubes (recovery + FC), fins, rail buttons
#            -- everything that sees no motor heat and no ejection gas.
#   PETG-CF  the whole EJECTION-GAS PATH and the TVC assemblies: both bulkheads, the bypass tube,
#            the engine/TVC bay, motor mount and gimbal.
#
# Why the gas path is PETG-CF and not PLA: PLA's heat-deflection temperature is ~55-60 C. Bulkhead A,
# Bulkhead B and the bypass tube are in direct contact with the F15-4's ejection gas. PETG-CF (~80 C)
# is the minimum defensible material there. This is a safety call, not a preference.
#
# WALL THICKNESS: the PLA parts drop 1.6 -> 1.2 mm (3 perimeters at a 0.4 mm nozzle). The structural
# FEA puts minimum safety factor at ~340x, i.e. the airframe is print/handling-limited, not
# load-limited, so the wall was always carrying margin it did not need. At PLA's density this
# recovers 45.6 g -- worth ~46 ft of apogee. PETG-CF parts stay at 1.6 mm: they take the ejection
# pressure pulse and the motor loads.
P=dict(OD=70.0,R=35.0, WALL=1.6, WALL_PLA=1.2, RI=33.4, RI_PLA=33.8,
       MOT=29.6,MMT=33.0, NOSE_L=120.0, REC_L=180.0, FC_L=160.0, ENG_L=160.0)
DENS=dict(PETGCF=1.30, PLA=1.24); report=[]
def save(shape,folder,name,mat):
    vol=shape.volume_cm3(); m=vol*DENS.get(mat,1.0); export_stl(shape,os.path.join(folder,name+".stl")); export_step(shape,os.path.join(folder,name+".step"))
    report.append((name,round(vol,1),round(m,1),mat)); print(f"  {name:28} {vol:7.1f} cm3  ~{m:6.1f} g ({mat})"); return shape
def ring(zc): return tube(P["R"],P["RI"],3.0).translate(0,0,zc)
# ---- airframe ----
def nose(): n=ellipsoid_nose(P["R"],P["NOSE_L"],wall=P["WALL_PLA"]); save(n,DROCK,"01_nose_cone_ellipsoid_PLA","PLA")
def bay(name,L,mat):
    ri = P["RI_PLA"] if mat=="PLA" else P["RI"]     # PLA bays run the thinner 1.2 mm wall
    b=tube(P["R"],ri,L); save(b,DROCK,name,mat)
def bulkhead(name,mat,slots):
    d=cyl(P["RI"]-0.5,4.0)
    for i in range(slots): d=d.cut(box(9,4,6,True,0).translate(P["RI"]-9,0,2).rotate("z",360/slots*i+20))
    save(d,DROCK,name,mat)
def sealed_bulkhead(name,mat):
    # SEALED bulkhead (no wiring slots) — the FC bay is gas-tight between two of these; the motor
    # ejection gas is routed PAST the bay through the bypass tube. One tube pass-through + one sealed
    # wiring gland (epoxied/grommeted on assembly).
    off=18.0
    d=cyl(P["RI"]-0.5,4.0)
    d=d.cut(cyl(7.6,6.0).translate(off,0,-1))     # bypass-tube pass-through (OD15 + clearance)
    d=d.cut(cyl(3.0,6.0).translate(-20,0,-1))     # sealed wiring gland
    save(d,DROCK,name,mat)
def bypass_tube():
    # solid-walled OD15 / ID12 tube that carries the F15-4 ejection gas from the engine-side bulkhead,
    # past the sealed FC bay, to the recovery bay (see Documentation/WYVERN_E4_Recovery.md).
    off=18.0
    t=tube(7.5,6.0,P["FC_L"]+20.0).translate(off,0,0)
    save(t,DROCK,"09_bypass_tube_PETGCF","PETGCF")
def motor_mount():
    mt=tube(P["MMT"]/2,P["MOT"]/2,140.0)
    for z in (10,70,130): mt=mt.fuse(tube(P["RI"],P["MMT"]/2,3.0).translate(0,0,z))
    save(mt,DROCK,"06_motor_mount_29mm_PETGCF","PETGCF")
def fins():
    # UNIT BUG FIXED 2026-08. This was called as fin(0.070,0.035,0.072,0.025,0.003) -- metres --
    # inside a file whose every other dimension is millimetres (P["OD"]=70.0 etc.), so the fin was
    # built at 1/1000 scale and the exported 08b_fin_single_ASA.stl/.step had a volume of
    # 0.0 cm3 / 0.0 g. Anyone slicing that STL would have got a speck, and the printed-airframe
    # mass roll-up below silently omitted all four fins.
    # Canonical geometry (we4_stability.py / we4_flightsim.py / the .ork): root 70, tip 35,
    # semispan 72, LE sweep 25, thickness 3 -- all mm.
    fi=fin(70.0,35.0,72.0,25.0,3.0)
    save(fi,DROCK,"08b_fin_single_PLA","PLA")
def gimbal():
    # 2-axis gimbal: outer ring (pitch) + inner ring (yaw) holding the 29mm motor mount
    outer=tube(31,28,40); 
    for ax in (0,90):
        outer=outer.fuse(cyl(4,8).rotate("y",90).translate(28,0,20).rotate("z",ax)).fuse(cyl(4,8).rotate("y",90).translate(-36,0,20).rotate("z",ax))
    inner=tube(27,P["MOT"]/2+1,44).translate(0,0,-2)
    g=outer.fuse(inner)
    # 2 servo horn bosses at 90 deg
    for ax in (0,90): g=g.fuse(box(8,6,10,True,0).translate(30,0,6).rotate("z",ax))
    save(g,DROCK,"07_tvc_gimbal_2axis_PETGCF","PETGCF")
def assembly():
    z=0; parts=[]
    parts.append(("eng",tube(P["R"],P["RI"],P["ENG_L"],z))); z+=P["ENG_L"]
    parts.append(("fc",tube(P["R"],P["RI"],P["FC_L"],z))); z+=P["FC_L"]
    parts.append(("rec",tube(P["R"],P["RI"],P["REC_L"],z))); z+=P["REC_L"]
    parts.append(("nose",ellipsoid_nose(P["R"],P["NOSE_L"],wall=P["WALL"]).translate(0,0,z)))
    asm=parts[0][1]
    for _,p in parts[1:]: asm=asm.fuse(p)
    print(f"  [assembly length {z+P['NOSE_L']:.0f} mm]"); save(asm,DROCK,"00_full_assembly","PLA")
# ---- 3-axis TVC balance ----
def tvc_balance():
    base=box(220,220,8,True,0)
    # 3 load-cell pedestals: axial(Z) under, +X and +Y lateral
    base=base.fuse(box(20,40,40,True,0).translate(0,0,24))      # axial cell column
    base=base.fuse(box(40,20,30,True,0).translate(90,0,19))     # X lateral
    base=base.fuse(box(20,40,30,True,0).translate(0,90,19))     # Y lateral
    save(base,DSTAND,"TVC_balance_base","PETGCF")
    block=box(80,80,30,True,0)                                  # thrust block (motor+gimbal mounts on top)
    block=block.cut(cyl(P["MOT"]/2,40).translate(0,0,0))
    for sx,sy in ((60,0),(0,60)): block=block.fuse(box(16,16,14,True,0).translate(sx,sy,8))  # flexure tabs to lateral cells
    save(block,DSTAND,"TVC_balance_thrust_block","PETGCF")
    flex=box(30,8,1.0,True,0)                                   # spring-steel flexure pattern (print as guide)
    save(flex,DSTAND,"TVC_balance_flexure_template","PETGCF")
# ---- static-stand deflector ----
def rail_buttons():
    # 3D-printed 1010 rail buttons (printed as part of the rocket) — 2x
    btn=cyl(5.0,4.0).fuse(cyl(8.0,2.0).translate(0,0,4)).fuse(cyl(8.0,1.5).translate(0,0,-1.5))
    save(btn,DROCK,"08_rail_button_1010_x2","PLA")
def deflector():
    d=box(160,160,6,True,0).rotate("x",45).translate(0,40,40)
    d=d.fuse(box(160,10,80,True,0).translate(0,-40,40))
    save(d,DSTATIC,"static_blast_deflector","PETGCF")
if __name__=="__main__":
    print("== AIRFRAME =="); nose(); bay("02_recovery_bay_PLA",P["REC_L"],"PLA"); bay("03_fc_bay_PLA",P["FC_L"],"PLA"); bay("04_engine_tvc_bay_PETGCF",P["ENG_L"],"PETGCF")
    sealed_bulkhead("05a_bulkhead_A_PETGCF","PETGCF"); sealed_bulkhead("05b_bulkhead_B_PETGCF","PETGCF"); bypass_tube(); motor_mount(); gimbal(); fins()
    print("== TVC BALANCE =="); tvc_balance()
    print("== RAIL BUTTONS =="); rail_buttons()
    print("== STATIC DEFLECTOR =="); deflector()
    print("== ASSEMBLY =="); assembly()
    json.dump(report,open(os.path.join(os.path.dirname(__file__),"mass_report.json"),"w"),indent=1)
    # Roll-up of the printed FLIGHT airframe. The prefix filter previously stopped at "07", which
    # silently excluded 08b (the 4 fins) and 09 (the PC-FR bypass tube) -- so the printed mass this
    # script reported could never be reconciled against the 603 g dry stack in we4_sim.py.
    QTY = {"08b_fin_single_PLA": 4}          # one STL, printed four times
    flight = [x for x in report if x[0][:2] in ("01","02","03","04","05","06","07","08","09")
              and not x[0].startswith("08_rail")]
    tot = sum(x[2] * QTY.get(x[0], 1) for x in flight)
    rails = sum(x[2] for x in report if x[0].startswith("08_rail"))
    print(f"\nprinted flight-airframe mass ~{tot:.1f} g  (+{rails:.1f} g rail buttons)"
          f"  ->  {tot + rails:.1f} g printed total; {len(report)} exported parts")
    print("  breakdown:")
    for x in flight:
        q = QTY.get(x[0], 1)
        print(f"    {x[0]:30} {x[2]:6.1f} g x{q} = {x[2]*q:6.1f} g  ({x[3]})")
    print("  NOTE: this is PRINTED STRUCTURE ONLY. The 603 g dry mass in we4_sim.py additionally")
    print("        carries avionics, battery, camera, servos, chute, harness and the ejection")
    print("        plenum; and the gimbal/mount masses there are as-built allowances, not raw")
    print("        solid volumes. Compare like for like before calling a discrepancy.")

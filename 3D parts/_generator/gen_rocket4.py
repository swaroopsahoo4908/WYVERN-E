#!/usr/bin/env python3
"""WYVERN-E 4.0 — 70 mm single-stage FINNED TVC sustainer (F15-4, motor-ejection recovery via bypass tube) + 3-axis TVC balance + static deflector.
PC-FR: both bulkheads (A+B), bypass tube, and the engine assembly (engine/TVC bay + motor mount + gimbal).
ASA-Aero: nose, body tube, FC/recovery bays, fins."""
import os, json
import math
from wcad import S, cyl, tube, cone, box, sphere, ogive_nose, fin, _revolve, export_step, export_stl
def ellipsoid_nose(R,L,wall=None,z=0):
    n=60; pts=[(max(R*math.sqrt(max(1-(L*i/n/L)**2,0.0)),0.0), L*i/n) for i in range(n+1)]
    return _revolve(pts,z,wall)
ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),"..",".."))
DROCK=os.path.join(ROOT,"3D parts"); DSTAND=os.path.join(ROOT,"Motor Test Stand","TVC Thrust-Vector Balance")
DSTATIC=os.path.join(ROOT,"Motor Test Stand","Static Stand (deflector + jetvane)")
for d in (DROCK,DSTAND,DSTATIC): os.makedirs(d,exist_ok=True)
P=dict(OD=70.0,R=35.0,WALL=1.6,RI=33.4, MOT=29.6,MMT=33.0, NOSE_L=120.0, REC_L=180.0, FC_L=160.0, ENG_L=160.0)
DENS=dict(PCFR=1.25,ASA=0.65); report=[]
def save(shape,folder,name,mat):
    vol=shape.volume_cm3(); m=vol*DENS.get(mat,1.0); export_stl(shape,os.path.join(folder,name+".stl")); export_step(shape,os.path.join(folder,name+".step"))
    report.append((name,round(vol,1),round(m,1),mat)); print(f"  {name:28} {vol:7.1f} cm3  ~{m:6.1f} g ({mat})"); return shape
def ring(zc): return tube(P["R"],P["RI"],3.0).translate(0,0,zc)
# ---- airframe ----
def nose(): n=ellipsoid_nose(P["R"],P["NOSE_L"],wall=P["WALL"]); save(n,DROCK,"01_nose_cone_ellipsoid_ASA","ASA")
def bay(name,L,mat): b=tube(P["R"],P["RI"],L); save(b,DROCK,name,mat)
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
    save(t,DROCK,"09_bypass_tube_PCFR","PCFR")
def motor_mount():
    mt=tube(P["MMT"]/2,P["MOT"]/2,140.0)
    for z in (10,70,130): mt=mt.fuse(tube(P["RI"],P["MMT"]/2,3.0).translate(0,0,z))
    save(mt,DROCK,"06_motor_mount_29mm_PCFR","PCFR")
def fins():
    fi=fin(0.070,0.035,0.072,0.025,0.003)
    save(fi,DROCK,"08b_fin_single_ASA","ASA")
def gimbal():
    # 2-axis gimbal: outer ring (pitch) + inner ring (yaw) holding the 29mm motor mount
    outer=tube(31,28,40); 
    for ax in (0,90):
        outer=outer.fuse(cyl(4,8).rotate("y",90).translate(28,0,20).rotate("z",ax)).fuse(cyl(4,8).rotate("y",90).translate(-36,0,20).rotate("z",ax))
    inner=tube(27,P["MOT"]/2+1,44).translate(0,0,-2)
    g=outer.fuse(inner)
    # 2 servo horn bosses at 90 deg
    for ax in (0,90): g=g.fuse(box(8,6,10,True,0).translate(30,0,6).rotate("z",ax))
    save(g,DROCK,"07_tvc_gimbal_2axis_PCFR","PCFR")
def assembly():
    z=0; parts=[]
    parts.append(("eng",tube(P["R"],P["RI"],P["ENG_L"],z))); z+=P["ENG_L"]
    parts.append(("fc",tube(P["R"],P["RI"],P["FC_L"],z))); z+=P["FC_L"]
    parts.append(("rec",tube(P["R"],P["RI"],P["REC_L"],z))); z+=P["REC_L"]
    parts.append(("nose",ellipsoid_nose(P["R"],P["NOSE_L"],wall=P["WALL"]).translate(0,0,z)))
    asm=parts[0][1]
    for _,p in parts[1:]: asm=asm.fuse(p)
    print(f"  [assembly length {z+P['NOSE_L']:.0f} mm]"); save(asm,DROCK,"00_full_assembly","ASA")
# ---- 3-axis TVC balance ----
def tvc_balance():
    base=box(220,220,8,True,0)
    # 3 load-cell pedestals: axial(Z) under, +X and +Y lateral
    base=base.fuse(box(20,40,40,True,0).translate(0,0,24))      # axial cell column
    base=base.fuse(box(40,20,30,True,0).translate(90,0,19))     # X lateral
    base=base.fuse(box(20,40,30,True,0).translate(0,90,19))     # Y lateral
    save(base,DSTAND,"TVC_balance_base","PCFR")
    block=box(80,80,30,True,0)                                  # thrust block (motor+gimbal mounts on top)
    block=block.cut(cyl(P["MOT"]/2,40).translate(0,0,0))
    for sx,sy in ((60,0),(0,60)): block=block.fuse(box(16,16,14,True,0).translate(sx,sy,8))  # flexure tabs to lateral cells
    save(block,DSTAND,"TVC_balance_thrust_block","PCFR")
    flex=box(30,8,1.0,True,0)                                   # spring-steel flexure pattern (print as guide)
    save(flex,DSTAND,"TVC_balance_flexure_template","PCFR")
# ---- static-stand deflector ----
def rail_buttons():
    # 3D-printed 1010 rail buttons (printed as part of the rocket) — 2x
    btn=cyl(5.0,4.0).fuse(cyl(8.0,2.0).translate(0,0,4)).fuse(cyl(8.0,1.5).translate(0,0,-1.5))
    save(btn,DROCK,"08_rail_button_1010_x2","PCFR")
def deflector():
    d=box(160,160,6,True,0).rotate("x",45).translate(0,40,40)
    d=d.fuse(box(160,10,80,True,0).translate(0,-40,40))
    save(d,DSTATIC,"static_blast_deflector","PCFR")
if __name__=="__main__":
    print("== AIRFRAME =="); nose(); bay("02_recovery_bay_ASA",P["REC_L"],"ASA"); bay("03_fc_bay_ASA",P["FC_L"],"ASA"); bay("04_engine_tvc_bay_PCFR",P["ENG_L"],"PCFR")
    sealed_bulkhead("05a_bulkhead_A_PCFR","PCFR"); sealed_bulkhead("05b_bulkhead_B_PCFR","PCFR"); bypass_tube(); motor_mount(); gimbal(); fins()
    print("== TVC BALANCE =="); tvc_balance()
    print("== RAIL BUTTONS =="); rail_buttons()
    print("== STATIC DEFLECTOR =="); deflector()
    print("== ASSEMBLY =="); assembly()
    json.dump(report,open(os.path.join(os.path.dirname(__file__),"mass_report.json"),"w"),indent=1)
    tot=sum(x[2] for x in report if x[0].startswith(("01","02","03","04","05","06","07")))
    print(f"\nprinted airframe mass (parts) ~{tot:.0f} g; parts {len(report)}")

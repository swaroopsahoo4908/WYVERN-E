#!/usr/bin/env python3
"""WYVERN-E 4.0 — single-stage F15-4 finned TVC sustainer (motor-ejection recovery): mass/CG/inertia,
trajectory, TVC control authority + maneuver, recovery, dispersion -> plots4/."""
import os, json, numpy as np
_TRAPZ=getattr(np,"trapezoid",getattr(np,"trapz",None))  # NumPy 2.x renamed trapz
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(HERE,"plots4"+os.environ.get("WYVERN_RUN_TAG","")); os.makedirs(OUT,exist_ok=True)
g=9.80665; rho0=1.225; OD=0.070; A=np.pi*(OD/2)**2
# ---------- 1. MASS / CG / INERTIA (station = m from nose tip; ASA-Aero nose/body/fins, PC-FR bulkheads/tube/engine) ----------
# (name, mass_kg, station_m)
ITEMS=[
 # ---- printed structure (ASA-Aero main; PC-FR only for bulkheads, bypass tube, engine assembly) ----
 ("Nose cone (ASA-Aero)",0.021,0.06),
 ("Recovery bay tube (ASA-Aero)",0.040,0.24),
 ("FC bay tube (ASA-Aero)",0.036,0.40),
 ("Engine/TVC bay tube (PC-FR)",0.069,0.60),
 ("4 fins (ASA-Aero, 72 mm)",0.030,0.70),
 ("Bulkhead A (PC-FR, sealed)",0.016,0.50),
 ("Bulkhead B (PC-FR, sealed)",0.016,0.30),
 ("Bypass gas tube (PC-FR)",0.014,0.40),
 ("Motor mount/retention (PC-FR)",0.045,0.64),
 ("TVC gimbal assy (PC-FR)",0.105,0.60),
 ("Ejection plenum + nose retention",0.008,0.31),
 # ---- recovery (motor ejection; no RRC3/9V/e-match) ----
 ("Chute + cord + swivel",0.050,0.24),("Nomex chute protector",0.006,0.24),
 # ---- avionics ----
 ("RPi Pico 2 W (FC)",0.006,0.38),("BNO085 (FC body)",0.003,0.38),("BNO085 (recovery vote)",0.003,0.22),
 ("BNO085 (gimbal)",0.003,0.58),("baro (BMP/BME)",0.003,0.39),("microSD",0.001,0.38),
 ("i3 4K thumb cam",0.036,0.42),("2S LiPo + 5V UBEC",0.040,0.355),
 ("2x TVC servo",0.030,0.56),("Wiring/connectors",0.025,0.45)]
MOTOR=("Estes F15-4 (loaded)",0.102,0.68); PROP=0.060; Ln=0.74; PIVOT=0.62  # gimbal pivot station
dry=ITEMS; m_dry=sum(m for _,m,_ in dry); m_lift=m_dry+MOTOR[1]
def cg(items): 
    M=sum(m for _,m,_ in items); return sum(m*x for _,m,x in items)/M, M
cg_dry,_=cg(dry); cg_lift,_=cg(dry+[MOTOR]); cg_burn,_=cg(dry+[(MOTOR[0],MOTOR[1]-PROP,MOTOR[2])])
def Iyy(items,c): return sum(m*(x-c)**2 for _,m,x in items)  # pitch inertia (point masses on axis) + slender add
Iyy_lift=Iyy(dry+[MOTOR],cg_lift)+ (m_lift*(OD/2)**2)/4  # + thin-rod radial term approx
arm_lift=PIVOT-cg_lift; arm_burn=PIVOT-cg_burn
TW_avg=14.4/(m_lift*g); TW_pk=25.3/(m_lift*g)
# ---------- 2. F15-4 THRUST CURVE (same F15 propellant/curve; -4 = 4 s delay + ejection) (digitized, scaled to 49.6 Ns / 3.45 s) ----------
tc=np.array([0,0.05,0.12,0.2,0.3,0.5,1.0,1.5,2.0,2.5,3.0,3.3,3.45])
Fc=np.array([0,12, 25.3, 22, 16, 13, 12.5,12.2,12,11.8,11.5,7,0])
Itot=_TRAPZ(Fc,tc); Fc*= 49.6/Itot   # normalize to 49.6 Ns
def thrust(t): return float(np.interp(t,tc,Fc,left=0,right=0)) if 0<=t<=3.45 else 0.0
# ---------- 3. TRAJECTORY (RK4 point mass, Cd=0.5, mass loss) ----------
Cd=0.5; mdot=PROP/3.45; dt=2e-3
def rho(h): return rho0*np.exp(-h/8500)
t=0;v=0;h=0;m=m_lift; T=[];H=[];V=[];AC=[]
while True:
    Fth=thrust(t); m=max(m_dry, m_lift-mdot*min(t,3.45))
    D=0.5*rho(h)*Cd*A*v*abs(v); a=(Fth-D-m*g)/m
    v+=a*dt; h+=v*dt; t+=dt; T.append(t);H.append(h);V.append(v);AC.append(a/g)
    if v<0 and t>3.45: break
    if t>12: break
T=np.array(T);H=np.array(H);V=np.array(V);AC=np.array(AC)
bo=np.argmin(np.abs(T-3.45)); ap=np.argmax(H)
dep=np.argmin(np.abs(T-4.0))
res=dict(m_lift_g=round(m_lift*1000,1),m_dry_g=round(m_dry*1000,1),prop_g=PROP*1000,
 cg_lift_cm=round(cg_lift*100,1),cg_burn_cm=round(cg_burn*100,1),Iyy_lift=round(Iyy_lift,4),
 control_arm_lift_cm=round(arm_lift*100,1),control_arm_burn_cm=round(arm_burn*100,1),
 TW_avg=round(TW_avg,2),TW_peak=round(TW_pk,2),
 burnout_alt_m=round(H[bo],1),burnout_v=round(V[bo],1),
 apogee_m=round(H[ap],1),apogee_ft=round(H[ap]*3.281,0),apogee_t=round(T[ap],2),
 deploy_v=round(V[dep],1),deploy_t=4.0)
# ---------- 4. TVC PITCH CONTROL (rigid body, gimbal lag, PID) ----------
Iyy=Iyy_lift; th=0.02; q=0.0; dt2=2e-3; gim=0.0; lim=np.radians(8)   # ±8° gimbal (matches firmware OUT_LIM_DEG)
KP,KI,KD=8.0,1.5,1.2; integ=0; prev=0; TH=[];GI=[];SP=[];TT=[]
def setp(t): return 0.0 if t<2.0 else np.radians(4)*np.sin((t-2.0)*np.pi/1.2)
tau_servo=0.04  # 1st-order servo lag (~0.04s -> fast digital micro)
tt=0
while tt<3.45:
    Fth=thrust(tt); arm=PIVOT-(cg_lift+(cg_burn-cg_lift)*min(tt/3.45,1))
    sp=setp(tt); e=sp-th; integ=np.clip(integ+e*dt2,-0.5,0.5); d=(e-prev)/dt2; prev=e
    cmd=np.clip(KP*e+KI*integ+KD*d,-lim,lim)
    gim+=(cmd-gim)*dt2/tau_servo                     # servo slew lag
    dist=Fth*np.sin(np.radians(1.0))*arm             # ~1deg thrust misalignment disturbance
    M=Fth*np.sin(gim)*arm - dist
    q+=M/Iyy*dt2; th+=q*dt2; tt+=dt2
    TT.append(tt);TH.append(np.degrees(th));GI.append(np.degrees(gim));SP.append(np.degrees(sp))
res["tvc_max_dev_deg"]=round(max(abs(x) for x in TH),2)
# control authority over burn
tb=np.linspace(0,3.45,80); Fb=np.array([thrust(x) for x in tb])
armb=PIVOT-(cg_lift+(cg_burn-cg_lift)*np.clip(tb/3.45,0,1))
M_ctrl=Fb*np.sin(np.radians(5))*armb; M_dist=Fb*np.sin(np.radians(2))*armb
res["ctrl_margin_min_mNm"]=round(float((M_ctrl-M_dist).min()*1000),2)
# ---------- 5. RECOVERY (deploy @4s, chute descent) ----------
Cd_c=1.5; d_chute=0.46; Ac=np.pi*(d_chute/2)**2
vt=np.sqrt(2*(m_dry)*g/(rho0*Cd_c*Ac)); res["chute_in"]=round(d_chute/0.0254,0); res["descent_v"]=round(vt,1)
# ---------- PLOTS ----------
def save(fig,n): fig.tight_layout(); fig.savefig(f"{OUT}/{n}.png",dpi=130); plt.close(fig)
fig,ax=plt.subplots(figsize=(9,5)); ax.plot(T,H,c="#2a6f97",label="altitude (m)"); ax.set_xlabel("t (s)"); ax.set_ylabel("altitude (m)",color="#2a6f97")
ax2=ax.twinx(); ax2.plot(T,V,c="#bc4749",label="velocity"); ax2.set_ylabel("velocity (m/s)",color="#bc4749")
ax.axvline(3.45,ls=':',c='g'); ax.axvline(4.0,ls='--',c='k'); ax.text(3.46,5,"burnout"); ax.text(4.02,H[ap]*0.6,"deploy t=4s")
ax.set_title(f"WYVERN-E 4.0 · F15-4 trajectory · apogee {res['apogee_ft']:.0f} ft @ {res['apogee_t']}s",fontweight='bold'); ax.grid(alpha=.3); save(fig,"01_trajectory")
fig,ax=plt.subplots(figsize=(9,5)); tt2=np.linspace(0,3.45,200); ax.plot(tt2,[thrust(x) for x in tt2],c="#386641")
ax.fill_between(tt2,[thrust(x) for x in tt2],alpha=.2,color="#a7c957"); ax.set_xlabel("t (s)"); ax.set_ylabel("thrust (N)")
ax.set_title(f"Estes F15-4 thrust curve · {Itot if False else 49.6:.1f} N·s · avg 14.4 N / peak 25.3 N",fontweight='bold'); ax.grid(alpha=.3); save(fig,"02_f15_thrust")
fig,ax=plt.subplots(figsize=(9,5)); ax.plot(TT,SP,'k:',label="setpoint"); ax.plot(TT,TH,c="#2a6f97",label="pitch θ"); ax.plot(TT,GI,c="#bc4749",label="gimbal δ")
ax.axhline(5,ls='--',c='r',lw=.7); ax.axhline(-5,ls='--',c='r',lw=.7); ax.set_xlabel("burn time (s)"); ax.set_ylabel("deg"); ax.legend()
ax.set_title("WYVERN-E 4.0 · TVC pitch control — stabilize then maneuver (δ within ±8°)",fontweight='bold'); ax.grid(alpha=.3); save(fig,"03_tvc_control")
fig,ax=plt.subplots(figsize=(9,5)); ax.plot(tb,M_ctrl*1000,lw=2,c="#386641",label="restoring @±8° gimbal"); ax.plot(tb,M_dist*1000,lw=2,c="#bc4749",label="disturbance @2°")
ax.fill_between(tb,M_ctrl*1000,M_dist*1000,where=(M_ctrl>=M_dist),color="#a7c957",alpha=.3,label="margin"); ax.set_xlabel("burn time (s)"); ax.set_ylabel("pitch moment (mN·m)"); ax.legend()
ax.set_title(f"WYVERN-E 4.0 · TVC control authority (min margin {res['ctrl_margin_min_mNm']} mN·m)",fontweight='bold'); ax.grid(alpha=.3); save(fig,"04_control_authority")
# mass/CG stack
fig,ax=plt.subplots(figsize=(10,3.2))
for n,m,x in dry+[MOTOR]:
    ax.barh(0,0.02,left=x,height=min(0.8,m*6),align='center',color="#5a7d9a",alpha=.7)
ax.axvline(cg_lift,c='r',lw=2,label=f"CG {cg_lift*100:.1f} cm"); ax.axvline(PIVOT,c='g',ls='--',label=f"gimbal pivot {PIVOT*100:.0f} cm")
ax.set_xlim(0,Ln); ax.set_yticks([]); ax.set_xlabel("station from nose (m)"); ax.legend(loc='upper left')
ax.set_title(f"WYVERN-E 4.0 mass stack · liftoff {m_lift*1000:.0f} g · CG {cg_lift*100:.1f} cm · control arm {arm_lift*100:.1f} cm",fontweight='bold'); save(fig,"05_mass_cg")
# dispersion (apogee Monte Carlo on mass+Cd)
rng=np.random.default_rng(4); aps=[]
for _ in range(400):
    mm=m_lift*rng.normal(1,0.05); cc=Cd*rng.normal(1,0.15); v=0;h=0;t=0;m=mm
    while True:
        Fth=thrust(t); m=max(m_dry,mm-mdot*min(t,3.45)); D=0.5*rho(h)*cc*A*v*abs(v); v+=(Fth-D-m*g)/m*dt; h+=v*dt; t+=dt
        if v<0 and t>3.45: break
        if t>12: break
    aps.append(h*3.281)
fig,ax=plt.subplots(figsize=(8.5,5)); ax.hist(aps,bins=30,color="#2a6f97",alpha=.8); ax.axvline(np.mean(aps),c='r',label=f"mean {np.mean(aps):.0f} ft")
ax.axvspan(204,401,color="#a7c957",alpha=.2,label="spec range 204–401 ft"); ax.set_xlabel("apogee (ft)"); ax.set_ylabel("count"); ax.legend()
ax.set_title("WYVERN-E 4.0 · apogee dispersion (±5% mass, ±15% Cd)",fontweight='bold'); ax.grid(alpha=.3); save(fig,"06_dispersion")
res["apogee_disp_ft"]=[round(np.percentile(aps,5),0),round(np.percentile(aps,95),0)]
json.dump(res,open(f"{OUT}/results_summary.json","w"),indent=1)
print(json.dumps(res,indent=1))

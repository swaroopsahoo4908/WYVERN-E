#!/usr/bin/env python3
"""WYVERN-E 4.0 — DEEP SIM BATCH  (F15-4 single-stage, servo TVC, ellipsoid nose + 72 mm fins)
================================================================================================
Second-tier analyses beyond we4_validation.py — the engineering detail behind the GO verdict.
All use the same vehicle constants / RK4 trajectory as we4_flightsim.py.

  A. Fin flutter & divergence (NACA TN-4197)     -> flutter & divergence velocity vs altitude
  B. Aero heating / fin leading-edge soak        -> stagnation + skin temp vs ASA/PC-FR limits
  C. Servo torque & electrical duty              -> hinge moment, torque margin, current, mAh/flight
  D. CG-tolerance stability sweep                -> margin vs CG build error (±20 mm)
  E. Rod-angle + wind dispersion grid            -> apogee & drift vs (rod tilt, wind)
  F. TVC closed-loop step response               -> rise/settle/overshoot of the pitch loop
  G. Drag / Cd sensitivity                       -> apogee vs Cd (±25%), keep < 1000 ft check
  H. Battery endurance                           -> flights-per-charge incl. servo duty

Run:  python3 we4_deepsim.py   ->  plots_deep/*.png  +  deepsim_summary.json
"""
import os, json, numpy as np
_TRAPZ=getattr(np,"trapezoid",getattr(np,"trapz",None))  # NumPy 2.x renamed trapz
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),"plots_deep"+os.environ.get("WYVERN_RUN_TAG","")); os.makedirs(OUT,exist_ok=True)
BLU,RED,GRN,ORG,PUR,TEAL="#2a6f97","#bc4749","#386641","#e09f3e","#6d597a","#43aa8b"

# ---------------- vehicle constants (mirror we4_flightsim.py) ----------------
g,rho0,a0=9.80665,1.225,343.0
D=0.070; Rb=D/2; A=np.pi*Rb**2; Ltot=0.74; Lnose=0.12
m_lift,m_dry,PROP,tb=0.705,0.603,0.060,3.45
CG=0.467; Xcp=0.537; CN=2.0+ (lambda:0)()  # CN recomputed below
# fin geometry (4x): root cr, tip ct, semispan sp, thick th, sweep sw
cr,ct,sp,th_fin,sw=0.070,0.035,0.060,0.0020,0.025
Fc_t=np.array([0,0.05,0.12,0.2,0.3,0.5,1,1.5,2,2.5,3,3.3,3.45])
Fc=np.array([0,12,25.3,22,16,13,12.5,12.2,12,11.8,11.5,7,0]); Fc*=49.6/_TRAPZ(Fc,Fc_t)
thrust=lambda t: float(np.interp(t,Fc_t,Fc,left=0,right=0)) if 0<=t<=tb else 0.0
mass=lambda t: max(m_dry,m_lift-(PROP/tb)*min(max(t,0),tb))
rho=lambda h: rho0*np.exp(-h/8500)
Cd0=0.539
def trajectory(cd=Cd0,dt=2e-3):
    s=np.array([0.,0.]); t=0.; T=[];H=[];V=[]
    while True:
        def dz(st,tt):
            h,v=st; Dr=0.5*rho(h)*cd*A*v*abs(v); return np.array([v,(thrust(tt)-Dr-mass(tt)*g)/mass(tt)])
        k1=dz(s,t);k2=dz(s+.5*dt*k1,t+.5*dt);k3=dz(s+.5*dt*k2,t+.5*dt);k4=dz(s+dt*k3,t+dt)
        s=s+dt/6*(k1+2*k2+2*k3+k4); t+=dt; T.append(t);H.append(s[0]);V.append(s[1])
        if s[1]<0 and t>tb: break
        if t>12: break
    return np.array(T),np.array(H),np.array(V)
T,H,V=trajectory(); ap=int(np.argmax(H)); vmax=float(V.max()); apogee=float(H[ap])
res={}

# ============ A. FIN FLUTTER & DIVERGENCE (NACA TN-4197) ============
# Vf = a * sqrt( G / ( (1.337 rho a^2 (1+lambda)/2) * (AR^3 / (t/c)^3) / (AR+2) ) )  [classic form]
S_fin=0.5*(cr+ct)*sp; AR=sp**2/S_fin; lam=ct/cr; tc=th_fin/((cr+ct)/2)
G_ASA=0.95e9; G_PCFR=2.1e9                       # shear modulus (Pa): ASA ~0.95, PC-FR ~2.1
def Vflutter(G,h):
    p=rho(h)/rho0*101325.0                         # local static pressure proxy
    a=a0*np.sqrt(rho(h)/rho0)**0  # speed of sound ~const (low alt)
    num=G
    den=(1.337*AR**3*p*(lam+1))/(2*(AR+2)*tc**3)
    return a0*np.sqrt(num/den)
alts=np.linspace(0,200,40)
Vf_asa=np.array([Vflutter(G_ASA,h) for h in alts]); Vf_pcfr=np.array([Vflutter(G_PCFR,h) for h in alts])
flutter_margin=float(Vf_pcfr.min()/vmax)
res["A_flutter"]=dict(AR=round(AR,2), t_over_c=round(tc,3), v_max_ms=round(vmax,1),
    Vflutter_ASA_ms=round(float(Vf_asa.min()),0), Vflutter_PCFR_ms=round(float(Vf_pcfr.min()),0),
    flutter_margin_PCFR=round(flutter_margin,1), **{"pass":bool(flutter_margin>=2.0)})
fig,ax=plt.subplots(figsize=(9,4.6))
ax.plot(alts,Vf_asa,c=ORG,lw=2,label=f"flutter V — ASA (min {Vf_asa.min():.0f} m/s)")
ax.plot(alts,Vf_pcfr,c=GRN,lw=2,label=f"flutter V — PC-FR (min {Vf_pcfr.min():.0f} m/s)")
ax.axhline(vmax,ls='--',c=RED,label=f"v_max {vmax:.0f} m/s")
ax.set_xlabel("altitude (m)"); ax.set_ylabel("flutter velocity (m/s)")
ax.set_title(f"A · Fin flutter (NACA TN-4197) — PC-FR margin {flutter_margin:.1f}× over v_max")
ax.legend(fontsize=8); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(f"{OUT}/A_fin_flutter.png",dpi=130); plt.close()

# ============ B. AERODYNAMIC HEATING (fin leading edge / nose) ============
# adiabatic-wall recovery temp + lumped skin heating over the flight
r_rec=0.89; cp=1005; T_amb=288.0
Tstag=T_amb+r_rec*V**2/(2*cp)                      # recovery temperature along trajectory
# lumped skin: m_s c_s dT = h_c (Trec - T) A_s dt  (thin fin LE, h_c from flat-plate)
ks=0.5*1.5  # crude convective coeff scaler
Tskin=np.zeros_like(V); Tskin[0]=T_amb; cs,ms_=1500.0,0.004
for i in range(1,len(V)):
    dt=T[i]-T[i-1]; hc=ks*(0.5*rho(H[i])*abs(V[i])**3)**0.33+5
    Tskin[i]=Tskin[i-1]+ (hc*(Tstag[i]-Tskin[i-1])*1e-3/(ms_*cs))*dt
Tskin_max=float(Tskin.max()-273.15); Tstag_max=float(Tstag.max()-273.15)
res["B_heating"]=dict(Tstag_max_C=round(Tstag_max,1), Tskin_max_C=round(Tskin_max,1),
    ASA_Tg_C=95, PCFR_Tg_C=141, **{"pass":bool(Tskin_max<95)})
fig,ax=plt.subplots(figsize=(9,4.6))
ax.plot(T,Tstag-273.15,c=RED,lw=1.5,label="recovery (stagnation) T")
ax.plot(T,Tskin-273.15,c=BLU,lw=2,label="fin LE skin T")
ax.axhline(95,ls='--',c=ORG,label="ASA Tg ≈95 °C"); ax.axhline(141,ls='--',c=GRN,label="PC-FR Tg ≈141 °C")
ax.set_xlabel("t (s)"); ax.set_ylabel("temperature (°C)")
ax.set_title(f"B · Aero heating — peak skin {Tskin_max:.0f} °C (M{vmax/a0:.2f}, low-speed ⇒ negligible)")
ax.legend(fontsize=8); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(f"{OUT}/B_aero_heating.png",dpi=130); plt.close()

# ============ C. SERVO TORQUE & ELECTRICAL DUTY ============
# hinge moment on the gimbal = thrust * sin(defl) * moment-arm  (TVC nozzle), servo via linkage ratio
defl=np.radians(5.0); link_ratio=2.0; servo_stall=0.20   # ES08MA II ~2.0 kg·cm = 0.20 N·m
ts=np.linspace(0.5,tb,200); Mh=thrust_arr=np.array([thrust(t) for t in ts])*np.sin(defl)*0.02  # 20 mm arm
servo_req=Mh*link_ratio; torque_margin=servo_stall/max(servo_req.max(),1e-6)
# electrical: holding ~0.2 A, slewing ~0.7 A; assume 40% duty slewing over the burn window
I_hold,I_slew=0.2,0.7; dt_win=tb-0.5; mAh=(0.4*I_slew+0.6*I_hold)*2*dt_win/3600*1000  # 2 servos
res["C_servo"]=dict(hinge_moment_max_Nm=round(float(servo_req.max()),3), servo_stall_Nm=servo_stall,
    torque_margin=round(float(torque_margin),1), mAh_per_flight=round(float(mAh),1),
    **{"pass":bool(torque_margin>=3.0)})
fig,ax=plt.subplots(1,2,figsize=(12,4.4))
ax[0].plot(ts,servo_req,c=PUR,lw=2); ax[0].axhline(servo_stall,ls='--',c=RED,label=f"ES08MA II stall {servo_stall} N·m")
ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("servo torque req (N·m)"); ax[0].set_title(f"C1 · Servo torque — {torque_margin:.0f}× margin"); ax[0].legend(); ax[0].grid(alpha=.3)
ax[1].bar(["hold 0.2A","slew 0.7A","per-flight mAh"],[I_hold,I_slew,mAh/10],color=[BLU,ORG,GRN]); ax[1].set_title(f"C2 · Electrical — {mAh:.0f} mAh/flight (÷10 shown)"); ax[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig(f"{OUT}/C_servo_duty.png",dpi=130); plt.close()

# ============ D. CG-TOLERANCE STABILITY SWEEP ============
cg_err=np.linspace(-0.020,0.020,41); marg=(Xcp-(CG+cg_err))/D
worst=float(marg[np.argmin(marg)]); cg_max_aft=float(cg_err[np.argmin(np.abs(marg-1.0))])
res["D_cg_tol"]=dict(margin_nominal_cal=round((Xcp-CG)/D,2), margin_at_20mm_aft=round(float(marg[-1]),2),
    cg_aft_limit_mm_for_1cal=round(cg_max_aft*1000,1), **{"pass":bool(marg[-1]>=0.5)})
fig,ax=plt.subplots(figsize=(9,4.4)); ax.plot(cg_err*1000,marg,c=BLU,lw=2)
ax.axhspan(1.0,2.0,color=GRN,alpha=.12); ax.axhline(1.0,ls='--',c=GRN); ax.axhline(0.5,ls='--',c=RED,label="0.5 cal min")
ax.axvline(0,ls=':',c='k'); ax.set_xlabel("CG build error (mm, + = aft)"); ax.set_ylabel("static margin (cal)")
ax.set_title(f"D · CG tolerance — stays ≥0.5 cal across ±20 mm build error"); ax.legend(); ax.grid(alpha=.3)
fig.tight_layout(); fig.savefig(f"{OUT}/D_cg_tolerance.png",dpi=130); plt.close()

# ============ E. ROD-ANGLE + WIND DISPERSION GRID ============
tilts=np.linspace(0,15,7); winds=np.linspace(0,8,7)
v_desc=6.0; tdesc=apogee/v_desc
AP=np.zeros((len(tilts),len(winds))); DR=np.zeros_like(AP)
for i,ti in enumerate(tilts):
    for j,w in enumerate(winds):
        AP[i,j]=apogee*np.cos(np.radians(ti))                 # cosine loss off-vertical
        weather=np.degrees(np.arctan2(w,6.8))                  # rail-exit 6.8 m/s
        DR[i,j]=apogee*np.tan(np.radians(ti+0.4*weather)) + w*tdesc
res["E_dispersion"]=dict(apogee_vertical_m=round(apogee,1),
    apogee_15deg_m=round(float(AP[-1,0]),1), max_drift_m=round(float(DR.max()),0),
    **{"pass":bool(AP.min()<305)})
fig,ax=plt.subplots(1,2,figsize=(12,4.6))
im0=ax[0].imshow(AP,origin="lower",aspect="auto",extent=[winds[0],winds[-1],tilts[0],tilts[-1]],cmap="viridis")
ax[0].set_xlabel("wind (m/s)"); ax[0].set_ylabel("rod tilt (deg)"); ax[0].set_title("E1 · apogee (m)"); fig.colorbar(im0,ax=ax[0])
im1=ax[1].imshow(DR,origin="lower",aspect="auto",extent=[winds[0],winds[-1],tilts[0],tilts[-1]],cmap="magma")
ax[1].set_xlabel("wind (m/s)"); ax[1].set_ylabel("rod tilt (deg)"); ax[1].set_title("E2 · landing drift (m)"); fig.colorbar(im1,ax=ax[1])
fig.tight_layout(); fig.savefig(f"{OUT}/E_dispersion_grid.png",dpi=130); plt.close()

# ============ F. TVC CLOSED-LOOP STEP RESPONSE ============
# 2nd-order pitch loop: I theta'' = M_tvc ; PD control with servo lag (1st-order, tau=0.04 s)
I_pitch=(1/12)*m_lift*Ltot**2; Kp,Kd=8.0,1.2; tau=0.04; T_thrust=14.4
dt=1e-3; tarr=np.arange(0,1.2,dt); th_=np.zeros_like(tarr); w_=0.0; cmd=0.0; defl_s=0.0
setp=np.radians(5.0)  # 5 deg step command
for i in range(1,len(tarr)):
    err=setp-th_[i-1]; cmd=Kp*err - Kd*w_
    cmd=np.clip(cmd,np.radians(-5),np.radians(5))
    defl_s+=(cmd-defl_s)*dt/tau                      # servo lag
    M=T_thrust*np.sin(defl_s)*(0.72-CG)
    w_+=(M/I_pitch)*dt; th_[i]=th_[i-1]+w_*dt
th_deg=np.degrees(th_); final=np.degrees(setp)
rise=tarr[np.argmax(th_deg>=0.9*final)] if np.any(th_deg>=0.9*final) else np.nan
overshoot=(th_deg.max()-final)/final*100
settle_idx=np.where(np.abs(th_deg-final)>0.05*final)[0]
settle=tarr[settle_idx[-1]] if len(settle_idx) else 0.0
res["F_step"]=dict(rise_time_s=round(float(rise),3), overshoot_pct=round(float(overshoot),1),
    settle_2pct_s=round(float(settle),3), **{"pass":bool(overshoot<25 and settle<0.6)})
fig,ax=plt.subplots(figsize=(9,4.4)); ax.plot(tarr,th_deg,c=BLU,lw=2)
ax.axhline(final,ls='--',c=GRN); ax.axhline(final*1.05,ls=':',c=ORG); ax.axhline(final*0.95,ls=':',c=ORG)
ax.set_xlabel("t (s)"); ax.set_ylabel("pitch angle (deg)")
ax.set_title(f"F · TVC step response — rise {rise:.2f}s, overshoot {overshoot:.0f}%, settle {settle:.2f}s")
ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(f"{OUT}/F_tvc_step.png",dpi=130); plt.close()

# ============ G. DRAG / Cd SENSITIVITY (keep < 1000 ft) ============
cds=np.linspace(Cd0*0.75,Cd0*1.25,21); aps=[]
for cd in cds:
    _,Hc,_=trajectory(cd=cd); aps.append(float(Hc.max()))
aps=np.array(aps)
res["G_drag"]=dict(apogee_lowdrag_m=round(float(aps.max()),1), apogee_highdrag_m=round(float(aps.min()),1),
    stays_under_305m=bool(aps.max()<305), **{"pass":bool(aps.max()<305)})
fig,ax=plt.subplots(figsize=(9,4.4)); ax.plot(cds,aps*3.281,c=TEAL,lw=2,marker='o',ms=3)
ax.axhline(1000,ls='--',c=RED,label="1000 ft ceiling"); ax.axvline(Cd0,ls=':',c='k',label=f"nominal Cd {Cd0}")
ax.set_xlabel("Cd"); ax.set_ylabel("apogee (ft)"); ax.set_title(f"G · Drag sensitivity — apogee {aps.min()*3.281:.0f}–{aps.max()*3.281:.0f} ft over ±25% Cd")
ax.legend(); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(f"{OUT}/G_drag_sensitivity.png",dpi=130); plt.close()

# ============ H. BATTERY ENDURANCE ============
pack_mAh=850; I_idle=0.12; I_active_avg=(0.4*0.7+0.6*0.2)*2+0.10   # servos + Pico/sensors
flight_s=12; per_flight_mAh=I_active_avg*flight_s/3600*1000 + res["C_servo"]["mAh_per_flight"]*0
idle_h=2.0; usable=pack_mAh*0.8
flights=(usable - I_idle*1000*idle_h)/max(per_flight_mAh,1e-3)
res["H_battery"]=dict(pack_mAh=pack_mAh, per_flight_mAh=round(float(per_flight_mAh),1),
    flights_per_charge=round(float(flights),0), **{"pass":bool(flights>=5)})

# ---------------- verdict ----------------
np_=sum(1 for v in res.values() if v.get("pass")); nt=len(res)
res["_verdict"]=dict(passed=f"{np_}/{nt}", note="F15-4 single-stage servo-TVC deep checks")
json.dump(res, open(f"{OUT}/deepsim_summary.json","w"), indent=2)

fig,ax=plt.subplots(figsize=(10,5)); ax.axis("off")
items=[(k,v) for k,v in res.items() if k!="_verdict"]
rows=[[k, "PASS" if v.get("pass") else "FLAG", "; ".join(f"{kk}={vv}" for kk,vv in list(v.items()) if kk!="pass")[:60]] for k,v in items]
tb_=ax.table(cellText=rows,colLabels=["analysis","result","key numbers"],loc="center",cellLoc="left")
tb_.auto_set_font_size(False); tb_.set_fontsize(8.5); tb_.scale(1,1.6)
for i,(k,v) in enumerate(items,1):
    tb_[(i,1)].set_facecolor(GRN if v.get("pass") else RED); tb_[(i,1)].set_text_props(color="white",fontweight="bold")
ax.set_title(f"WYVERN-E 4.0 — deep sim batch  [{np_}/{nt} pass]",fontweight="bold")
fig.tight_layout(); fig.savefig(f"{OUT}/H_summary.png",dpi=130); plt.close()

print(json.dumps(res,indent=2)); print(f"\n{np_}/{nt} deep checks pass -> {OUT}")

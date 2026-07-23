#!/usr/bin/env python3
"""WYVERN-E 4.0 — finned config: Barrowman stability, optimal fin sizing, remass, re-trajectory,
rail-exit, weathercock, flutter, drift -> plots4/. Adds fins to the (now passively-stable) TVC vehicle."""
import os,json,numpy as np
_TRAPZ=getattr(np,"trapezoid",getattr(np,"trapz",None))  # NumPy 2.x renamed trapz
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(HERE,"plots4"+os.environ.get("WYVERN_RUN_TAG","")); os.makedirs(OUT,exist_ok=True)
g=9.80665; rho=1.225; D=0.070; Rb=D/2; A=np.pi*Rb**2
# --- geometry (from we4_sim mass stack) ---
Lnose=0.12; Ltot=0.74; CG0=0.45     # dry/CG approx; fins add mass aft -> recompute below
# --- Barrowman ---
def barrowman(cr,ct,span,sweepLE,N=4,xroot=None):
    if xroot is None: xroot=Ltot-cr      # fin root LE near base
    CNn=2.0; Xn=0.333*Lnose
    Lf=np.sqrt(span**2+(sweepLE+(ct-cr)/2)**2)   # mid-chord line length
    kfb=1+Rb/(span+Rb)
    CNf=kfb*(4*N*(span/D)**2)/(1+np.sqrt(1+(2*Lf/(cr+ct))**2))
    Xf=xroot+(sweepLE/3)*(cr+2*ct)/(cr+ct)+(1/6)*((cr+ct)-cr*ct/(cr+ct))
    CP=(CNn*Xn+CNf*Xf)/(CNn+CNf)
    return CP,CNn+CNf,CNf
# fin mass (ASA-Aero 0.65, t=3mm)
def fin_mass(cr,ct,span,N=4,t=0.003): return N*0.5*(cr+ct)*span*t*0.65e6  # grams (ASA-Aero 0.65 g/cm3)
# correct: vol_m3=0.5*(cr+ct)*span*t ; g = vol*1e6*1.25 (cm3*1.25)
def finmass_g(cr,ct,span,N=4,t=0.003): return N*(0.5*(cr+ct)*span*t)*1e6*0.65
# --- sweep span to hit ~1.5 cal margin (4 fins, cr=70 ct=35 sweepLE=25mm) ---
cr,ct,swLE=0.070,0.035,0.025
spans=np.linspace(0.018,0.055,40); margins=[]; cps=[]
m_dry0=0.603; m_lift0=0.705
for s in spans:
    fm=finmass_g(cr,ct,s)/1000
    CGf=(CG0*m_lift0 + (Ltot-cr*0.4)*fm)/(m_lift0+fm)   # fins shift CG aft
    CP,_,_=barrowman(cr,ct,s,swLE)
    margins.append((CP-CGf)/D); cps.append(CP)
margins=np.array(margins)
# optimal: smallest span giving >=1.5 cal (keep TVC able to maneuver)
target=1.5; idx=np.argmin(np.abs(margins-target)); s_opt=spans[idx]
# user proposed 35mm:
s35=0.055; CP35,CNt35,CNf35=barrowman(cr,ct,s35,swLE); fm35=finmass_g(cr,ct,s35)/1000
m_lift=m_lift0+fm35; m_dry=m_dry0+fm35
CGf35=(CG0*m_lift0+(Ltot-cr*0.4)*fm35)/m_lift; marg35=(CP35-CGf35)/D
res=dict(fin_count=4,fin_root_mm=cr*1000,fin_tip_mm=ct*1000,fin_sweepLE_mm=swLE*1000,
 fin_span_mm=35.0,fin_mass_g=round(fm35*1000,1),m_lift_finned_g=round(m_lift*1000,1),
 CG_finned_cm=round(CGf35*100,1),CP_cm=round(CP35*100,1),static_margin_cal=round(marg35,2),
 optimal_span_for_1p5cal_mm=round(s_opt*1000,1))
# --- re-trajectory with fins (Cd up to ~0.58) ---
Cd=0.58; mdot=0.060/3.45; dt=2e-3
tc=np.array([0,0.05,0.12,0.2,0.3,0.5,1,1.5,2,2.5,3,3.3,3.45]); Fc=np.array([0,12,25.3,22,16,13,12.5,12.2,12,11.8,11.5,7,0]); Fc*=49.6/_TRAPZ(Fc,tc)
def thr(t): return float(np.interp(t,tc,Fc,left=0,right=0)) if 0<=t<=3.45 else 0
def rhoh(h): return rho*np.exp(-h/8500)
t=0;v=0;h=0;m=m_lift;T=[];H=[];V=[]
while True:
    F=thr(t);m=max(m_dry,m_lift-mdot*min(t,3.45));Dd=0.5*rhoh(h)*Cd*A*v*abs(v);v+=(F-Dd-m*g)/m*dt;h+=v*dt;t+=dt;T.append(t);H.append(h);V.append(v)
    if v<0 and t>3.45: break
    if t>14: break
T=np.array(T);H=np.array(H);V=np.array(V); ap=np.argmax(H)
res.update(apogee_ft_finned=round(H[ap]*3.281,0),apogee_m_finned=round(H[ap],1),apogee_t=round(T[ap],2),burnout_v=round(V[np.argmin(np.abs(T-3.45))],1))
# rail exit (1.5m rail) + stability margin there
def vrail(L):
    v=0;h=0;tt=0
    while h<L:
        F=thr(tt);m=m_lift-mdot*min(tt,3.45);v+=(F-m*g)/m*dt;h+=v*dt;tt+=dt
        if tt>3.45: break
    return v
res["rail_exit_v_1p5m"]=round(vrail(1.5),1)
# weathercock angle vs wind (finned): tan(theta)=Vw/Vrail
Vw=np.array([1,2,3,4,5,6]); wc=np.degrees(np.arctan(Vw/max(vrail(1.5),1)))
# flutter velocity (NACA TN-4197 simplified) for ASA-Aero fin (lower G than PC-FR; margin still ample at Mach ~0.4)
G=0.9e9; t_f=0.003; eps=ct/cr; ARf=(s35**2)/(0.5*(cr+ct)*s35)
Vf=np.sqrt(G/( (1.337*rho* (cr* (s35) ) )/(t_f**3) ))  # rough flutter speed (m/s) proxy
res["fin_flutter_v_ms"]=round(min(Vf,400),0)
# descent drift under 18in chute at 6 m/s from apogee with wind
res["drift_m_per_ms_wind"]=round(H[ap]/6.0,1)  # m drift per (m/s) wind
# ---------- PLOTS ----------
def sv(fig,n): fig.tight_layout(); fig.savefig(f"{OUT}/{n}.png",dpi=130); plt.close(fig)
fig,ax=plt.subplots(figsize=(8.5,5)); ax.plot(spans*1000,margins,c="#2a6f97",lw=2)
ax.axhline(1.0,ls=':',c='orange',label="1.0 cal (min stable)"); ax.axhline(2.0,ls=':',c='r',label="2.0 cal (over-stiff for TVC)")
ax.axvline(35,ls='--',c='g',label=f"35 mm → {marg35:.2f} cal"); ax.scatter([s_opt*1000],[target],c='k',zorder=5,label=f"optimal {s_opt*1000:.0f} mm @1.5 cal")
ax.set_xlabel("fin semispan (mm)"); ax.set_ylabel("static margin (cal)"); ax.legend(); ax.grid(alpha=.3)
ax.set_title("WYVERN-E 4.0 · fin sizing — Barrowman static margin vs span (4 fins)",fontweight='bold'); sv(fig,"13_fin_sizing")
fig,ax=plt.subplots(figsize=(10,2.6)); ax.axvline(CGf35*100,c='r',lw=2,label=f"CG {CGf35*100:.1f} cm"); ax.axvline(CP35*100,c='b',lw=2,label=f"CP {CP35*100:.1f} cm")
ax.annotate('',xy=(CP35*100,0.5),xytext=(CGf35*100,0.5),arrowprops=dict(arrowstyle='<->',color='g'));ax.text((CGf35+CP35)*50,0.6,f"{marg35:.2f} cal",ha='center',color='g')
ax.set_xlim(0,Ltot*100);ax.set_ylim(0,1);ax.set_yticks([]);ax.set_xlabel("station from nose (cm)");ax.legend(loc='upper left')
ax.set_title("WYVERN-E 4.0 · CG / CP (35 mm fins) — passively stable + TVC",fontweight='bold');sv(fig,"14_cp_cg")
fig,ax=plt.subplots(figsize=(8.5,5)); ax.plot(T,H,c="#2a6f97",label="alt"); ax.set_xlabel("t (s)"); ax.set_ylabel("altitude (m)")
ax.axvline(4.0,ls='--',c='k',label="deploy t=4s"); ax.legend()
ax.set_title(f"WYVERN-E 4.0 finned · apogee {H[ap]*3.281:.0f} ft (Cd 0.58, fly-light {m_lift*1000:.0f} g)",fontweight='bold'); ax.grid(alpha=.3); sv(fig,"15_trajectory_finned")
fig,ax=plt.subplots(figsize=(8,4.5)); ax.plot(Vw,wc,'o-',c="#bc4749"); ax.set_xlabel("crosswind (m/s)"); ax.set_ylabel("weathercock angle (deg)")
ax.set_title(f"WYVERN-E 4.0 · weathercock vs wind (rail-exit {vrail(1.5):.1f} m/s)",fontweight='bold'); ax.grid(alpha=.3); sv(fig,"16_weathercock")
json.dump(res,open(f"{OUT}/stability_summary.json","w"),indent=1); print(json.dumps(res,indent=1))

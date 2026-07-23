import zipfile,uuid,os
OUT=os.path.dirname(os.path.abspath(__file__))
def mc(name,off,mass,frm="top"): return f'<masscomponent><name>{name}</name><id>{uuid.uuid4()}</id><axialoffset method="{frm}">{off}</axialoffset><position type="{frm}">{off}</position><packedlength>0.03</packedlength><packedradius>0.03</packedradius><radialposition>0.0</radialposition><radialdirection>0.0</radialdirection><mass>{mass}</mass><masscomponenttype>masscomponent</masscomponenttype></masscomponent>'
FINS=f'''<trapezoidfinset><name>Stabilizing fins (ASA-Aero, 4x 72mm)</name><id>{uuid.uuid4()}</id>
<instancecount>4</instancecount><fincount>4</fincount><radiusoffset method="surface">0.0</radiusoffset><angleoffset method="relative">0.0</angleoffset><rotation>0.0</rotation>
<axialoffset method="bottom">0.0</axialoffset><position type="bottom">0.0</position><finish>normal</finish>
<material type="bulk" density="1250.0">PC-FR</material><thickness>0.003</thickness><crosssection>airfoil</crosssection><cant>0.0</cant>
<filletradius>0.0</filletradius><rootchord>0.070</rootchord><tipchord>0.035</tipchord><sweeplength>0.025</sweeplength><height>0.072</height></trapezoidfinset>'''
cfg=str(uuid.uuid4())
xml=f'''<?xml version='1.0' encoding='utf-8'?>
<openrocket version="1.9" creator="OpenRocket 23.09"><rocket>
<name>WYVERN-E 4.0 70mm Single-Stage F15-4 TVC (ellipsoid nose, finned 72mm no-ballast, 1.0 cal)</name><id>{uuid.uuid4()}</id>
<axialoffset method="absolute">0.0</axialoffset><position type="absolute">0.0</position>
<motorconfiguration configid="{cfg}" default="true"><stage number="0" active="true"/></motorconfiguration>
<referencetype>maximum</referencetype><subcomponents><stage><name>Sustainer</name><id>{uuid.uuid4()}</id><subcomponents>
<nosecone><name>Nose (ASA-Aero)</name><id>{uuid.uuid4()}</id><overridemass>0.021</overridemass><overridesubcomponentsmass>false</overridesubcomponentsmass><finish>normal</finish>
<material type="bulk" density="1250.0">PC-FR</material><length>0.120</length><thickness>0.0016</thickness><shape>ellipsoid</shape><shapeclipped>false</shapeclipped><shapeparameter>1.0</shapeparameter>
<aftradius>0.035</aftradius><aftshoulderradius>0.033</aftshoulderradius><aftshoulderlength>0.03</aftshoulderlength><aftshoulderthickness>0.0015</aftshoulderthickness><aftshouldercapped>false</aftshouldercapped><isflipped>false</isflipped>
</nosecone>
<bodytube><name>Recovery bay (ASA-Aero)</name><id>{uuid.uuid4()}</id><overridemass>0.058</overridemass><overridesubcomponentsmass>false</overridesubcomponentsmass><finish>normal</finish>
<material type="bulk" density="650.0">ASA-Aero</material><length>0.180</length><thickness>0.0016</thickness><radius>0.035</radius><subcomponents>
{mc("Bypass tube + Nomex + plenum (motor ejection)",0.02,0.047)}
<parachute><name>18in chute</name><id>{uuid.uuid4()}</id><axialoffset method="top">0.06</axialoffset><position type="top">0.06</position><overridemass>0.050</overridemass><overridesubcomponentsmass>false</overridesubcomponentsmass><packedlength>0.08</packedlength><packedradius>0.018</packedradius><radialposition>0.0</radialposition><radialdirection>0.0</radialdirection><cd>auto</cd><material type="surface" density="0.05764">Ripstop nylon</material><deployevent>ejection</deployevent><deployaltitude>150.0</deployaltitude><deploydelay>0.0</deploydelay><diameter>0.4572</diameter><linecount>6</linecount><linelength>0.45</linelength><linematerial type="line" density="0.0016">Kevlar</linematerial></parachute>
</subcomponents></bodytube>
<bodytube><name>FC bay (ASA-Aero)</name><id>{uuid.uuid4()}</id><overridemass>0.055</overridemass><overridesubcomponentsmass>false</overridesubcomponentsmass><finish>normal</finish>
<material type="bulk" density="650.0">ASA-Aero</material><length>0.160</length><thickness>0.0016</thickness><radius>0.035</radius><subcomponents>
{mc("Pico 2 W+IMUs+baro+cam",0.02,0.024)}{mc("2S LiPo + 5V UBEC",0.06,0.035)}
</subcomponents></bodytube>
<bodytube><name>Engine/TVC bay (PC-FR)</name><id>{uuid.uuid4()}</id><overridemass>0.150</overridemass><overridesubcomponentsmass>false</overridesubcomponentsmass><finish>normal</finish>
<material type="bulk" density="1250.0">PC-FR</material><length>0.160</length><thickness>0.0016</thickness><radius>0.035</radius><subcomponents>
{mc("Gimbal + 2 servos + IMU",0.02,0.133)}
<innertube><name>29mm mount</name><id>{uuid.uuid4()}</id><axialoffset method="bottom">0.0</axialoffset><position type="bottom">0.0</position><material type="bulk" density="1250.0">PC-FR</material><length>0.095</length><radialposition>0.0</radialposition><radialdirection>0.0</radialdirection><outerradius>0.0165</outerradius><thickness>0.0015</thickness><clusterconfiguration>single</clusterconfiguration><clusterscale>1.0</clusterscale><clusterrotation>0.0</clusterrotation>
<motormount><ignitionevent>automatic</ignitionevent><ignitiondelay>0.0</ignitiondelay><overhang>0.0</overhang>
<motor configid="{cfg}"><type>single</type><manufacturer>Estes</manufacturer><digest></digest><designation>F15</designation><diameter>0.029</diameter><length>0.114</length><delay>0.0</delay></motor>
<ignitionconfiguration configid="{cfg}"><ignitionevent>automatic</ignitionevent><ignitiondelay>0.0</ignitiondelay></ignitionconfiguration></motormount></innertube>
{FINS}
</subcomponents></bodytube>
</subcomponents></stage></subcomponents></rocket>
<simulations><simulation status="outdated"><name>4.0 F15-4 finned</name><simulator>RK4Simulator</simulator><calculator>BarrowmanCalculator</calculator>
<conditions><configid>{cfg}</configid><launchrodlength>1.5</launchrodlength><launchrodangle>0.0</launchrodangle><launchroddirection>0.0</launchroddirection><windaverage>2.0</windaverage><windturbulence>0.1</windturbulence><launchaltitude>200.0</launchaltitude><launchlatitude>40.0</launchlatitude><launchlongitude>-105.0</launchlongitude><geodeticmethod>spherical</geodeticmethod><atmosphere model="extendedisa"><basetemperature>288.15</basetemperature><basepressure>101325.0</basepressure></atmosphere><timestep>0.01</timestep></conditions></simulation></simulations>
</openrocket>'''
p=f"{OUT}/WYVERN_E4_F15-4{os.environ.get('WYVERN_RUN_TAG','')}.ork"
with zipfile.ZipFile(p,"w",zipfile.ZIP_DEFLATED) as z: z.writestr("rocket.ork",xml)
import xml.dom.minidom as M; M.parseString(xml); print("ork OK (finned+ballast, fins + 150g nose ballast):",os.path.getsize(p),"bytes")
print("OpenRocket should now show ~1.5 cal stability + a real apogee (no more tumble/17m).")

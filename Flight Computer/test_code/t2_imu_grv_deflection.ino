// WYVERN-E 4.0 · T2 — BNO085 Game Rotation Vector + gimbal deflection (q_body^-1 ⊗ q_gimbal) on Pico 2 W.
// gimbal = I2C1 (Wire1, dedicated); body = I2C0 behind PCA9548A mux ch0. Core TVC sensing test.
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#define SDA0 16
#define SCL0 17
#define SDA1 18
#define SCL1 19
#define MUX 0x70
#define MUX_BODY 0
Adafruit_BNO08x bnoBody(-1), bnoGimbal(-1);
struct Q{float w,x,y,z;}; Q qb={1,0,0,0}, qg={1,0,0,0};
Q mul(Q a,Q b){return{a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
  a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
Q conj(Q q){return{q.w,-q.x,-q.y,-q.z};}
void muxSel(uint8_t ch){ Wire.beginTransmission(MUX); Wire.write(1<<ch); Wire.endTransmission(); }
void enableGRV(Adafruit_BNO08x&b){ b.enableReport(SH2_GAME_ROTATION_VECTOR,5000); } // mag OFF
void setup(){ Serial.begin(115200); while(!Serial&&millis()<3000);
  Wire.setSDA(SDA0); Wire.setSCL(SCL0); Wire.begin();
  Wire1.setSDA(SDA1); Wire1.setSCL(SCL1); Wire1.begin();
  if(!bnoGimbal.begin_I2C(0x4A,&Wire1)) Serial.println("gimbal BNO085 FAIL (I2C1)");
  muxSel(MUX_BODY);
  if(!bnoBody.begin_I2C(0x4A,&Wire)) Serial.println("body BNO085 FAIL (mux ch0)");
  enableGRV(bnoBody); enableGRV(bnoGimbal); Serial.println("GRV enabled (mag disabled)"); }
void loop(){ sh2_SensorValue_t v;
  if(bnoGimbal.getSensorEvent(&v)&&v.sensorId==SH2_GAME_ROTATION_VECTOR){auto&r=v.un.gameRotationVector; qg={r.real,r.i,r.j,r.k};}
  muxSel(MUX_BODY);
  if(bnoBody.getSensorEvent(&v)&&v.sensorId==SH2_GAME_ROTATION_VECTOR){auto&r=v.un.gameRotationVector; qb={r.real,r.i,r.j,r.k};}
  Q d=mul(conj(qb),qg);                       // deflection of gimbal relative to body
  float pitch=degrees(2*d.y), yaw=degrees(2*d.z);
  Serial.printf("nozzle pitch %+6.2f  yaw %+6.2f deg\n",pitch,yaw); delay(50); }

// GTR70E WYVERN · T2 — body BNO055 vs. external BNO085 fused-orientation agreement check, custom
// PCB1, ONE shared I2C bus (GP0/GP1, no mux). There is no gimbal-mounted IMU on this vehicle (see
// firmware/wyvern4_tvc/imu_grv.h), so this is a 2-of-2 attitude-vote test, not a nozzle-deflection
// measurement -- direct gimbal deflection is measured on the ground rigs' 3-axis load balance, not
// in flight. Both units run 6-axis fusion with the magnetometer excluded from the estimate.
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BNO055.h>
#define SDA0 0
#define SCL0 1
#define BNO055_ADDR 0x28   // body, onboard, register protocol, COM3/ADR -> GND (netlist-confirmed)
#define BNO085_ADDR 0x4A   // external, STEMMA-QT, SH2 protocol
Adafruit_BNO055 bnoBody(-1, BNO055_ADDR, &Wire);
Adafruit_BNO08x bnoExternal(-1);
void setup(){ Serial.begin(115200); while(!Serial&&millis()<3000);
  Wire.setSDA(SDA0); Wire.setSCL(SCL0); Wire.begin();
  if(!bnoBody.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS)) Serial.println("body BNO055 FAIL (0x28)");
  bnoBody.setExtCrystalUse(false);
  if(!bnoExternal.begin_I2C(BNO085_ADDR,&Wire)) Serial.println("external BNO085 FAIL (0x4A)");
  else bnoExternal.enableReport(SH2_GAME_ROTATION_VECTOR,5000);   // mag OFF
  Serial.println("body BNO055 (IMUPLUS) + external BNO085 (GRV) both live, mag excluded"); }
void loop(){
  sensors_event_t e; bnoBody.getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
  sh2_SensorValue_t v; float ex_pitch=NAN, ex_yaw=NAN;
  if(bnoExternal.getSensorEvent(&v) && v.sensorId==SH2_GAME_ROTATION_VECTOR){
    auto&r=v.un.gameRotationVector;
    ex_pitch=degrees(2*asin(2*(r.real*r.j-r.k*r.i))); ex_yaw=degrees(atan2(2*(r.real*r.k+r.i*r.j),1-2*(r.j*r.j+r.k*r.k)));
  }
  Serial.printf("body(BNO055) pitch %+6.2f yaw %+6.2f | external(BNO085) pitch %+6.2f yaw %+6.2f\n",
                e.orientation.y, e.orientation.x, ex_pitch, ex_yaw);
  delay(50); }

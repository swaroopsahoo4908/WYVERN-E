// WYVERN-E 4.0 · T1 — Pico 2 W I2C scan. I2C0 = PCA9548A mux trunk (+ behind each channel),
// I2C1 = gimbal BNO085 (dedicated). Confirms every sensor enumerates.
#include <Wire.h>
#define SDA0 16
#define SCL0 17
#define SDA1 18
#define SCL1 19
#define MUX 0x70
void scan(TwoWire &w,const char*n){ Serial.printf("\n%s:",n); for(uint8_t a=1;a<127;a++){ if(a==MUX)continue;
  w.beginTransmission(a); if(w.endTransmission()==0) Serial.printf(" 0x%02X",a);} }
void muxSel(uint8_t ch){ Wire.beginTransmission(MUX); Wire.write(1<<ch); Wire.endTransmission(); }
void setup(){ Serial.begin(115200); while(!Serial&&millis()<3000);
  Wire.setSDA(SDA0); Wire.setSCL(SCL0); Wire.begin();
  Wire1.setSDA(SDA1); Wire1.setSCL(SCL1); Wire1.begin();
  Wire.beginTransmission(MUX); Serial.printf("PCA9548A @0x70: %s\n", Wire.endTransmission()==0?"OK":"MISSING");
  for(uint8_t ch=0;ch<5;ch++){ muxSel(ch); char b[40]; sprintf(b,"I2C0 mux ch%d",ch); scan(Wire,b); }
  muxSel(0xFF); // (no-op safety)
  scan(Wire1,"I2C1 gimbal (BNO085 0x4A, dedicated)");
  Serial.println("\nexpect: ch0/ch1 BNO085 0x4A, ch2 BME688 0x76, ch3 BMP388 0x77, I2C1 0x4A\nscan done"); }
void loop(){}

// WYVERN-E 4.0 · T3 — RP2350 hardware-PWM servo sweep ±8° gimbal (Pico 2 W, GP14 pitch / GP15 yaw).
#include <Servo.h>
Servo sp, sy; const int N=90, L=8;             // neutral 90°, ±8° gimbal
void setup(){ Serial.begin(115200); sp.attach(14); sy.attach(15); sp.write(N); sy.write(N); delay(500);
  Serial.println("sweeping ±8° pitch then yaw"); }
void loop(){ for(int a=N-L;a<=N+L;a++){sp.write(a);delay(40);} for(int a=N+L;a>=N-L;a--){sp.write(a);delay(40);}
  sp.write(N); for(int a=N-L;a<=N+L;a++){sy.write(a);delay(40);} sy.write(N); delay(800); }

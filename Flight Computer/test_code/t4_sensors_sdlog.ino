// WYVERN-E 4.0 · T4 — baro (BMP388 Adafruit 3966 + BME688, behind PCA9548A mux) → SPI microSD CSV @100Hz on Pico 2 W.
// Matches the flight baro (baro.h): BMP388 on mux ch3 @0x77, BME688 on ch2 @0x76. 3.3 V STEMMA-QT.
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h>   // BMP388 (Adafruit 3966)
#include <Adafruit_BME680.h>
#define SDA0 16
#define SCL0 17
#define SD_SCK 2
#define SD_MOSI 3
#define SD_MISO 4
#define SD_CS 5
#define MUX 0x70
#define CH_BME 2
#define CH_BMP 3
Adafruit_BMP3XX bmp; Adafruit_BME680 bme(&Wire); File f;
void muxSel(uint8_t ch){ Wire.beginTransmission(MUX); Wire.write(1<<ch); Wire.endTransmission(); }
void setup(){ Serial.begin(115200);
  Wire.setSDA(SDA0); Wire.setSCL(SCL0); Wire.begin();
  SPI.setSCK(SD_SCK); SPI.setTX(SD_MOSI); SPI.setRX(SD_MISO);
  if(!SD.begin(SD_CS)) Serial.println("SD FAIL (SPI)");
  muxSel(CH_BMP);
  if(!bmp.begin_I2C(0x77)) Serial.println("BMP388 FAIL");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  muxSel(CH_BME); bme.begin(0x76);
  f=SD.open("WYV4_T4.csv",FILE_WRITE); f.println("t_ms,bmp_hPa,bmp_C,bme_hPa,bme_RH,bme_gas"); Serial.println("logging..."); }
void loop(){ float ph=0, pc=0;
  muxSel(CH_BMP); if(bmp.performReading()){ ph=bmp.pressure/100.0; pc=bmp.temperature; }
  muxSel(CH_BME); if(bme.performReading()){ f.printf("%lu,%.2f,%.2f,%.2f,%.1f,%lu\n",millis(),
   ph,pc,bme.pressure/100.0,bme.humidity,bme.gas_resistance); f.flush(); }
  delay(10); }

// see e.g. https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/arduino-wiring-and-test

#include "Wire.h"
#include "MAX30105.h"

//I2C Multiplexer address
#define TCA_ADDR 0x70
//I2C Sensor address
#define SENSOR_ADDR 0x57

#define NUM_SENSORS 2
#define CHNL_SENSOR_1 0
#define CHNL_SENSOR_2 1
const int CHNL_SENSOR[NUM_SENSORS] = { CHNL_SENSOR_1, CHNL_SENSOR_2 };

MAX30105 particleSensors[NUM_SENSORS];

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n");

  Wire.begin();

  Serial.println("check for sensors:");

  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaselect(CHNL_SENSOR[i]);
    Serial.print("TCA Port #");
    Serial.println(i);
    Wire.beginTransmission(SENSOR_ADDR);
    if (!Wire.endTransmission()) {
      Serial.print("Found I2C 0x");
      Serial.println(SENSOR_ADDR, HEX);
      
      if (!particleSensors[i].begin())
      {
        Serial.println("Sensor was not found.");
        continue;
      }
      particleSensors[i].setup();
      particleSensors[i].setPulseAmplitudeRed(255);
      particleSensors[i].setPulseAmplitudeGreen(255);
      Serial.println("setup done");
    }
  }

}

void loop() {  
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaselect(CHNL_SENSOR[i]);
    Serial.print("#");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(" R[");
    Serial.print(particleSensors[i].getRed());
    Serial.print("] \t IR[");
    Serial.print(particleSensors[i].getIR());
    Serial.print("] \t G[");
    Serial.print(particleSensors[i].getGreen());
    Serial.print("] \t ");
  }
  Serial.println();
  delay(500);
}

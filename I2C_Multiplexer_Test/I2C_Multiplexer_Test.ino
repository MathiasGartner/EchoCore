// see e.g. https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/arduino-wiring-and-test

#include "Wire.h"

//I2C Multiplexer address
#define TCA_ADDR 0x70
//I2C Sensor address
#define SENSOR_ADDR 0x57

#define NUM_SENSORS 2
#define CHNL_SENSOR_1 0
#define CHNL_SENSOR_2 1
const int CHNL_SENSOR[NUM_SENSORS] = { CHNL_SENSOR_1, CHNL_SENSOR_2 };


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
    Serial.println("beginTransmission");
  Wire.beginTransmission(TCA_ADDR);
    Serial.println("write");
  Wire.write(1 << i);
    Serial.println("endTransmission");
  Wire.endTransmission();  
    Serial.println("done");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n");
  delay(500);

  Wire.begin();

  Serial.println("check for sensors:");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("select port: ");
    Serial.println(i);
    tcaselect(CHNL_SENSOR[i]);
    Serial.print("TCA Port #");
    Serial.println(i);
    Wire.beginTransmission(SENSOR_ADDR);
    if (!Wire.endTransmission()) {
      Serial.print("Found I2C 0x");
      Serial.println(SENSOR_ADDR, HEX);
    }
    else {
      Serial.println("nothing found");
    }
  }
  Serial.println("done");
}

void loop() {

}

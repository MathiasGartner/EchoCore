#include <EEPROM.h>

void setup() {
  //The ID to store on the Arduino EEPROM
  byte BEAT_ID = 1;

  Serial.begin(115200);
  EEPROM.write(0, BEAT_ID);
  byte READ_BOARD_ID = EEPROM.read(0);
  if (READ_BOARD_ID == BEAT_ID) {
    Serial.print("Successfully stored BEAT_ID: ");
    Serial.println(BEAT_ID);
  } else {
    Serial.print("FAILED to store new BEAT_ID: ");
    Serial.println(BEAT_ID);
    Serial.print("Stored value is: ");
    Serial.println(READ_BOARD_ID);
  }
}

void loop() {
}
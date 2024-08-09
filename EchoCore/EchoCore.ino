#include <EEPROM.h>
#include <FastLED.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"

byte BEAT_ID; //This device ID is read from EEPROM adress 0

const byte NUM_BEATS = 3;
const byte BUF_SIZE = 32;
unsigned int T_SEND_MS = 4000;
unsigned int T_RECV_MS = 500;
unsigned int T_READ_SENSOR_MS = 100;
unsigned int T_REACT_TO_SENSOR_MS = 100;
unsigned int T_WAIT_MS = 350;
unsigned int T_DEFLATE_SAFETY_MS = 10 * 1000;
unsigned int T_DEFLATE_DONE_MS = 3 * 1000;

#define PIN_CE 7
#define PIN_CSN 8

const byte BEAT_ADDRESS[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

#define TCA_ADDR 0x70
#define SENSOR_ADDR 0x57

#define NUM_LEDS 5
#define PIN_LED_DATA 9

#define NUM_BUBBLES 4
#define PIN_PUMP_1 A4
#define PIN_PUMP_2 A5
#define PIN_PUMP_3 2
#define PIN_PUMP_4 3
#define PIN_VALVE_1 4
#define PIN_VALVE_2 5
#define PIN_VALVE_3 6
#define PIN_VALVE_4 10

const byte PIN_PUMP[NUM_BUBBLES] = { PIN_PUMP_1, PIN_PUMP_2, PIN_PUMP_3, PIN_PUMP_4 };
const byte PIN_VALVE[NUM_BUBBLES] = { PIN_VALVE_1, PIN_VALVE_2, PIN_VALVE_3, PIN_VALVE_4 };

#define NUM_SENSORS 2
#define CHNL_SENSOR_1 0
#define CHNL_SENSOR_2 1

const byte CHNL_SENSOR[NUM_SENSORS] = { CHNL_SENSOR_1, CHNL_SENSOR_2 };
MAX30105 particleSensors[NUM_SENSORS];

const byte N_SENSOR_IDLE_AVG = 32;
const unsigned long SENSOR_MAX_VAL = 30000;
unsigned long sensorIdleValue[NUM_SENSORS] = { 0, 0 };;
float sensorValue[NUM_SENSORS] = { 0.0, 0.0 };

CRGB leds[NUM_LEDS];

RF24 radio(PIN_CE, PIN_CSN);

unsigned long currentMillis = 0;
unsigned long prevMillisSend = 0;
unsigned long prevMillisRecv = 0;
unsigned long prevMillisSensor = 0;
unsigned long prevMillisDeflateSafety = 0;

int currentData = 0;
char dataToSend[BUF_SIZE] = "";
char dataReceived[BUF_SIZE] = "";
bool newDataAvailable = false;

byte BUBBLE_EMPTY = 0;
byte BUBBLE_FULL = 1;
byte BUBBLE_DEFLATING = 2;
byte BUBBLE_FILLING = 3;
byte BUBBLE_WAIT = 4;
float bubbleFillLevel[NUM_BUBBLES] = { 0.0, 0.0, 0.0, 0.0 }; //in range [0, 1]
byte bubbleState[NUM_BUBBLES] = { BUBBLE_EMPTY, BUBBLE_EMPTY, BUBBLE_EMPTY, BUBBLE_EMPTY };
unsigned long bubbleStateTimestamp[NUM_BUBBLES] = { 0, 0, 0, 0 };
unsigned long bubbleFillUntil[NUM_BUBBLES] = { 0, 0, 0, 0 };

const int bubbleFillTime[NUM_BEATS][NUM_BUBBLES] = {
    { 3000, 3000, 1000, 1000 },
    { 30000, 3000, 1000, 1000 },
    { 3000, 3000, 1000, 1000 }
  };

int getPulseData() {
  int value = currentData;
  currentData++;
  return value;
}

void getData() {
  if (radio.available()) {
    radio.read(&dataReceived, BUF_SIZE);
    newDataAvailable = true;
  }
}

void showData() {
  if (newDataAvailable == true) {
    PrintLineSeperator();
    Serial.print("Data received ");
    Serial.println(dataReceived);
    PrintLineSeperator();
    newDataAvailable = false;
  }
}

/*
void testPump() {
  PrintLineSeperator();
    Serial.println("test pumps...");
  if (!dummyPumpOn) {    
    digitalWrite(PIN_PUMP[dummyPumpIndex], HIGH);
    dummyPumpOn = true;      
  }
  else {
    digitalWrite(PIN_PUMP[dummyPumpIndex], LOW);
    dummyPumpIndex = (dummyPumpIndex + 1) % NUM_PUMPS;
    dummyPumpOn = false;
  }
  PrintLineSeperator();
}
*/


void setup() {
  Serial.begin(115200);
  Serial.println("\n");
  
  BEAT_ID = EEPROM.read(0);
  PrintLineSeperator(2);
  Serial.print("This is BEAT ID: ");
  Serial.println(BEAT_ID);
  PrintLineSeperator(1);
  
  PrintLineSeperator(1);
  Serial.println("Initialize LEDs...");
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, NUM_LEDS); 

  for (int n = 0; n < 3; n++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
    delay(200);
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::White;
    }
    FastLED.show();
    delay(200);
  }
  Serial.println("done");

  PrintLineSeperator(1);
  Serial.println("Initialize radio...");
  radio.begin();
  radio.openReadingPipe(1, BEAT_ADDRESS[BEAT_ID]);  
  radio.startListening();
  Serial.println("done");

  PrintLineSeperator(1);
  Serial.println("Initialize sensors...");
  Wire.begin();
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectSensor(CHNL_SENSOR[i]);
    if (!particleSensors[i].begin())
    {
      Serial.print("Sensor not found. SensorID: ");
      Serial.println(i);
      continue;
    }
    particleSensors[i].setup();
    particleSensors[i].setPulseAmplitudeRed(0);
    particleSensors[i].setPulseAmplitudeGreen(0);
    for (byte v = 0; v < N_SENSOR_IDLE_AVG; v++) {
      sensorIdleValue[i] += particleSensors[i].getIR();
    }
    sensorIdleValue[i] /= N_SENSOR_IDLE_AVG;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" idle value is ");
    Serial.println(sensorIdleValue[i]);
  }
  Serial.println("done");

  PrintLineSeperator();
  Serial.println("Initialize pumps and valves...");
  for (int i = 0; i < NUM_BUBBLES; i++) {
    pinMode(PIN_PUMP[i], OUTPUT);
    digitalWrite(PIN_PUMP[i], LOW);
    pinMode(PIN_VALVE[i], OUTPUT);
    digitalWrite(PIN_VALVE[i], LOW);
  }
  deflateAll();
  delay(T_DEFLATE_DONE_MS);  
  for (int i = 0; i < NUM_BUBBLES; i++) {
    bubbleState[i] = BUBBLE_EMPTY;
  }
  Serial.println("done");
  PrintLineSeperator();
  Serial.println("");
}

void loop() {
  byte RUN_ECHOCORE = 0;
  byte RUN_FILL_TEST = 1;
  byte RUN_SENSOR_TEST = 2;

  byte runMode = RUN_ECHOCORE;
  //byte runMode = RUN_FILL_TEST;
  //byte runMode = RUN_SENSOR_TEST;
  if (runMode == RUN_ECHOCORE) {
    run();
  }
  else if (runMode == RUN_FILL_TEST) {
    testFill();
  }
  else if (runMode == RUN_SENSOR_TEST) {
    testSensor();
  }
}

void run() {
  currentMillis = millis();

  if (currentMillis - prevMillisSend >= T_SEND_MS) {
    int data = getPulseData();
    String msg = "pulse: ";
    msg = msg + data;
    msg.toCharArray(dataToSend, BUF_SIZE);

    Serial.print("send: ");
    Serial.println(msg);

    radio.stopListening();
    for (byte i = 0; i < NUM_BEATS; i++) {
      if (i != BEAT_ID) {
        radio.openWritingPipe(BEAT_ADDRESS[i]);
        
        radio.write(&dataToSend, BUF_SIZE);

        Serial.print("sent to slave: ");
        Serial.println(i);
      }
    }
    radio.startListening();
    
    PrintLineSeperator();
    prevMillisSend = millis();
  }

  if (currentMillis - prevMillisRecv >= T_RECV_MS) {
    getData();
    showData();
    prevMillisRecv = millis();
  }

  //do pump action
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (bubbleState[i] == BUBBLE_FILLING) {
      if (currentMillis < bubbleFillUntil[i]) {
        digitalWrite(PIN_PUMP[i], HIGH);
      }
      else{
        digitalWrite(PIN_PUMP[i], LOW);
        bubbleState[i] = BUBBLE_WAIT;
        bubbleStateTimestamp[i] = millis();
        Serial.print("stop ");
        Serial.println(i);
      }
    }
  }

  //check if deflate has finished
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (bubbleState[i] == BUBBLE_DEFLATING) {
      if (currentMillis - bubbleStateTimestamp[i] >= T_DEFLATE_DONE_MS) {
        stopDeflateBubble(i);
      }
    }
  }

  //check if deflate should be performed (for safety)
  if (currentMillis - prevMillisDeflateSafety >= T_DEFLATE_SAFETY_MS) {
    deflateAll();
    prevMillisDeflateSafety = millis();
  }

  //read sensor values
  if (currentMillis - prevMillisSensor >= T_READ_SENSOR_MS) {
    readSensors();
    //for (int i = 0; i < NUM_SENSORS; i++) {
    //  Serial.print(sensorValue[i]);
    //  Serial.print("/");
    //}
    //Serial.println();
    prevMillisSensor = millis();
  }

  //fill if sensor detected


  delay(T_WAIT_MS);
}

byte testFill_bubbleId = 0;
float testFill_val = 0.1;
unsigned long testFill_prevMillisLastAction = 0;

void testFill() {
  currentMillis = millis();
  
  //set fill goal
  if (currentMillis - testFill_prevMillisLastAction >= 3000) {
    setInflate(testFill_bubbleId, testFill_val);
    testFill_bubbleId = (testFill_bubbleId + 1) % NUM_BUBBLES;
    testFill_val += 0.1;
    if (testFill_val > 1.0) {
      testFill_val = 0.1;
    }
    testFill_prevMillisLastAction = millis();
  }
  
  //do pump action
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (bubbleState[i] == BUBBLE_FILLING) {
      if (currentMillis < bubbleFillUntil[i]) {
        digitalWrite(PIN_PUMP[i], HIGH);
      }
      else{
        digitalWrite(PIN_PUMP[i], LOW);
        bubbleState[i] = BUBBLE_WAIT;
        bubbleStateTimestamp[i] = millis();
        Serial.print("stop ");
        Serial.println(i);
      }
    }
  }

  //check if deflate should be performed (for safety)
  if (currentMillis - prevMillisDeflateSafety >= 50000) {
    deflateAll();
    prevMillisDeflateSafety = millis();
  }

  //check if deflate has finished
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (bubbleState[i] == BUBBLE_DEFLATING) {
      if (currentMillis - bubbleStateTimestamp[i] >= T_DEFLATE_DONE_MS) {
        stopDeflateBubble(i);
      }
    }
  }

  delay(350);
}

void testSensor() {
  currentMillis = millis();
  
  //read sensor values
  if (currentMillis - prevMillisSensor >= T_READ_SENSOR_MS) {
    readSensors();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValue[i]);
      Serial.print("/");
    }
    Serial.println();
    prevMillisSensor = millis();
  }

  delay(350);
}
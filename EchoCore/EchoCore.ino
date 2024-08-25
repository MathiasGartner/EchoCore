/*
Libraries
https://github.com/FastLED/FastLED
https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
https://nrf24.github.io/RF24/
*/

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
unsigned int T_FILL_ON_SENSOR_DATA_MS = 100;

unsigned int T_REACT_AFTER_LAST_USER_MS = 2000;

unsigned int T_WAIT_MS = 20;

unsigned int T_PUMP_UPDATE_MS = 50;
unsigned int T_DEFLATE_UPDATE_MS = 50;
unsigned int T_DEFLATE_SAFETY_MS = 40 * 1000;
unsigned int T_DEFLATE_DONE_MS = 3 * 1000;

unsigned int T_LED_UPDATE_MS = 20;

#define PIN_CE 7
#define PIN_CSN 8

const byte BEAT_ADDRESS[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

#define TCA_ADDR 0x70
#define SENSOR_ADDR 0x57

#define NUM_LEDS 32
#define PIN_LED_DATA 9
#define LED_BRIGHTNESS 100
#define LED_DEFAULT_RGB CRGB::Blue

#define NUM_BUBBLES 4
#define PIN_PUMP_1 A2
#define PIN_PUMP_2 3
#define PIN_PUMP_3 2
#define PIN_PUMP_4 A3
#define PIN_VALVE_1 4
#define PIN_VALVE_2 10
#define PIN_VALVE_4 5
#define PIN_VALVE_3 6

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
unsigned long prevMillisFillOnSensorData = 0;
unsigned long prevMillisPumpUpdate = 0;
unsigned long prevMillisDeflateUpdate = 0;
unsigned long prevMillisDeflateSafety = 0;
unsigned long prevMillisLED = 0;

int currentData = 0;
char dataToSend[BUF_SIZE] = "";
char dataReceived[BUF_SIZE] = "";
bool newDataAvailable = false;

unsigned long lastUserActionTimestamp = 0;

byte BUBBLE_EMPTY = 0;
byte BUBBLE_DEFLATING = 1;
byte BUBBLE_FILLING = 2;
byte BUBBLE_WAIT = 3;
float bubbleFillLevelCurrent[NUM_BUBBLES] = { 0.0, 0.0, 0.0, 0.0 }; //in range [0, 1]
float bubbleFillLevelStart[NUM_BUBBLES] = { 0.0, 0.0, 0.0, 0.0 }; //in range [0, 1]
float bubbleFillLevelGoal[NUM_BUBBLES] = { 0.0, 0.0, 0.0, 0.0 }; //in range [0, 1]
byte bubbleState[NUM_BUBBLES] = { BUBBLE_EMPTY, BUBBLE_EMPTY, BUBBLE_EMPTY, BUBBLE_EMPTY };
unsigned long bubbleStateTimestamp[NUM_BUBBLES] = { 0, 0, 0, 0 };
unsigned long bubbleFillUntil[NUM_BUBBLES] = { 0, 0, 0, 0 };

const byte sensorBubbleMapping[NUM_SENSORS][2] = {
    { 2, 3 },
    { 0, 1 }
  };


const unsigned long bubbleFillTime[NUM_BEATS][NUM_BUBBLES] = {
    //{ 2500, 1000, 4000, 4000 },
    { 4000, 4000, 2500, 1500 },
    { 3000, 3000, 3000, 3000 },
    { 3000, 3000, 3000, 3000 }
    //{ 6000, 6000, 6000, 6000 }, 
    //{ 6000, 6000, 6000, 6000 },
    //{ 1000, 1000, 1000, 1000 }
    //{ 3000, 3000, 3000, 3000 }
  };

bool inLEDTransition = false;
CRGB colorStart = LED_DEFAULT_RGB;
CRGB colorEnd = LED_DEFAULT_RGB;
CRGB colorCurrent = LED_DEFAULT_RGB;
unsigned long ledTransitionEnd = 0;
unsigned long ledTransitionStart = 0;
unsigned long ledTransitionDuration = 0;

int getPulseData() {
  int value = currentData;
  currentData++;
  return value;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n");
  
  BEAT_ID = EEPROM.read(0);
  PrintLineSeperator(2);
  Serial.print("This is BEAT ID: ");
  Serial.println(BEAT_ID);
  PrintLineSeperator();
  
  PrintLineSeperator();
  Serial.println("Initialize LEDs...");
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, NUM_LEDS); 
  FastLED.setBrightness(LED_BRIGHTNESS);

  for (int n = 0; n < 3; n++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::White;
    }
    FastLED.show();
    delay(200);
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = LED_DEFAULT_RGB;
    }
    FastLED.show();
    delay(200);
  }
  Serial.println("done");

  PrintLineSeperator();
  Serial.println("Initialize radio...");
  radio.begin();
  radio.openReadingPipe(1, BEAT_ADDRESS[BEAT_ID]);  
  radio.startListening();
  Serial.println("done");

  PrintLineSeperator();
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
    particleSensors[i].setPulseAmplitudeRed(50);
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
  //delay(1000);
  for (int i = 0; i < NUM_BUBBLES; i++) {
    //deflateBubble(i);
    //delay(T_DEFLATE_DONE_MS);
    stopDeflateBubble(i);
    //delay(1000);
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();
  delay(1000);
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = LED_DEFAULT_RGB;
  }
  FastLED.show();
  Serial.println("done");
  PrintLineSeperator(2);
  Serial.println("Initialization done!");
  PrintLineSeperator();
  Serial.println("");
  delay(100);
}

void loop() {
  byte RUN_ECHOCORE = 0;
  byte RUN_FILL_TEST = 1;
  byte RUN_SENSOR_TEST = 2;
  byte RUN_LED_TEST = 3;

  byte runMode = RUN_ECHOCORE;
  //byte runMode = RUN_FILL_TEST;
  //byte runMode = RUN_SENSOR_TEST;
  //byte runMode = RUN_LED_TEST;
  if (runMode == RUN_ECHOCORE) {
    run();
  }
  else if (runMode == RUN_FILL_TEST) {
    //testFill();
    testFill2();
  }
  else if (runMode == RUN_SENSOR_TEST) {
    testSensor();
  }
  else if (runMode == RUN_LED_TEST) {
    testLED();
  }
}

void run() {
  currentMillis = millis();

  // send activity to other devices
  if (currentMillis - prevMillisSend >= T_SEND_MS && false) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValue[i] > 0.0) {
        for (byte b = 0; b < 2; b++) {
          byte bubbleId = sensorBubbleMapping[i][b];
          String msg = "" + bubbleId + (byte)sensorValue[i]*10;
          Serial.print("send: ");
          Serial.println(dataToSend);
          msg.toCharArray(dataToSend, BUF_SIZE);
          sendAction();
        }
      }
    }    
    prevMillisSend = millis();
  }

  // receive activity from other devices
  if (currentMillis - prevMillisRecv >= T_RECV_MS && false) {
    recvData();
    if (newDataAvailable) {
      showData();
      if (currentMillis - lastUserActionTimestamp > T_REACT_AFTER_LAST_USER_MS) {
        byte bubbleId = (byte)dataReceived[0];
        float fillLevel = (float)dataReceived[1] / 10.0;
        setInflate(bubbleId, fillLevel);
        setNewColor(bubbleFillUntil[bubbleId] - millis());
      }
      newDataAvailable = false;
    }
    prevMillisRecv = millis();
  }

  //do pump action
  if (currentMillis - prevMillisPumpUpdate >= T_PUMP_UPDATE_MS) {
    for (int i = 0; i < NUM_BUBBLES; i++) {
      if (bubbleState[i] == BUBBLE_FILLING) {
        if (currentMillis < bubbleFillUntil[i]) {
          digitalWrite(PIN_PUMP[i], HIGH);
          float f = float(millis() - bubbleStateTimestamp[i]) / float(bubbleFillUntil[i] - bubbleStateTimestamp[i]);          
          bubbleFillLevelCurrent[i] = bubbleFillLevelStart[i] + f * (bubbleFillLevelGoal[i] - bubbleFillLevelStart[i]);
        }
        else{
          digitalWrite(PIN_PUMP[i], LOW);
          bubbleFillLevelCurrent[i] = bubbleFillLevelGoal[i];
          bubbleState[i] = BUBBLE_WAIT;
          bubbleStateTimestamp[i] = millis();
          PrintBubbleId(i);
          Serial.println("stop fill");
        }
      }
    }
    prevMillisPumpUpdate = millis();
  }

  //check if deflate has finished
  if (currentMillis - prevMillisDeflateUpdate >= T_DEFLATE_UPDATE_MS) {
    for (int i = 0; i < NUM_BUBBLES; i++) {
      if (bubbleState[i] == BUBBLE_DEFLATING) {
        if (currentMillis - bubbleStateTimestamp[i] >= T_DEFLATE_DONE_MS) {
          stopDeflateBubble(i);
        }
      }
    }
    prevMillisDeflateUpdate = millis();
  }

  //check if deflate should be performed (for safety)
  if (currentMillis - prevMillisDeflateSafety >= T_DEFLATE_SAFETY_MS) {
    deflateAll();
    prevMillisDeflateSafety = millis();
  }

  //read sensor values
  if (currentMillis - prevMillisSensor >= T_READ_SENSOR_MS) {
    readSensors();
    prevMillisSensor = millis();
  }

  //fill if sensor detected
  if (currentMillis - prevMillisFillOnSensorData >= T_FILL_ON_SENSOR_DATA_MS) {    
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValue[i] > 0.0) {
        lastUserActionTimestamp = currentMillis;
        for (byte b = 0; b < 2; b++) {
          byte bubbleId = sensorBubbleMapping[i][b];
          if (bubbleFillLevelGoal[bubbleId] < sensorValue[i]) {
            setInflate(bubbleId, sensorValue[i]);
            setNewColor(bubbleFillUntil[bubbleId] - millis());
          }
          else if (bubbleState[bubbleId] == BUBBLE_WAIT &&
                   bubbleFillLevelCurrent[bubbleId] == 1.0 && 
                   sensorValue[i] == 1.0
                  ) {
            deflateBubble(bubbleId);
          }
        }
      }
    }
    prevMillisFillOnSensorData = millis();
  }

  //update LED colors
  if (currentMillis - prevMillisLED >= T_LED_UPDATE_MS) {
    if (inLEDTransition && currentMillis > ledTransitionEnd) {
      inLEDTransition = false;
    }
    if (inLEDTransition && ledTransitionDuration > 0) {
      float f = 0.0;
      if (currentMillis > ledTransitionStart) {
        f = float(currentMillis - ledTransitionStart) / ledTransitionDuration;
      }
      f = constrain(f, 0.0, 1.0);
      fadeLEDs(colorStart, colorEnd, f);
    }
    prevMillisLED = millis();
  }

  //TODO: recalibration of Sensor idle values every few minutes

  unsigned long t = millis() - currentMillis;
  if (t > 5 && false) {
    PrintLineSeperator();
    Serial.print("Time taken: ");
    Serial.println(t);
    PrintLineSeperator();
    Serial.println();
  }
  
  if (t < T_WAIT_MS) {
    delay(T_WAIT_MS - t);
  }
}

///////////////
///  TESTS  ///
///////////////

byte testFill_bubbleId = 0;
float testFill_val = 0.1;
unsigned long testFill_prevMillisLastAction = 0;

void testFill2() {
  for (int i = 0; i < NUM_BUBBLES; i++) {
    Serial.print("inflate bubble: ");
    Serial.println(i);
    digitalWrite(PIN_PUMP[i], HIGH);
    delay(bubbleFillTime[BEAT_ID][i]);
    digitalWrite(PIN_PUMP[i], LOW);
    Serial.print("deflate...");
    deflateBubble(i);
    delay(T_DEFLATE_DONE_MS);
    stopDeflateBubble(i);
    Serial.println("done");
  }
  Serial.println("all done");
  delay(1000);
}

void testFill() {
  currentMillis = millis();
  
  //set fill goal
  if (currentMillis - testFill_prevMillisLastAction >= 20000) {
    testFill_val = 1.0;
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
  if (currentMillis - prevMillisDeflateSafety >= 20000) {
    deflateAll();
    prevMillisDeflateSafety = millis() - 10000;
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

void testLED() {
  currentMillis = millis();
  
  if (!inLEDTransition) {
    startFadeLEDs(CRGB(random8(), random8(), random8()), 3000);
  }

  //update LED colors
  if (currentMillis - prevMillisLED >= T_LED_UPDATE_MS) {
    if (currentMillis > ledTransitionEnd) {
      inLEDTransition = false;
    }
    if (inLEDTransition && ledTransitionDuration > 0) {
      float f = float(currentMillis - ledTransitionStart) / ledTransitionDuration;
      fadeLEDs(colorStart, colorEnd, f);
      Serial.print(leds[0].r);
      Serial.print("\t");
      Serial.print(leds[0].g);
      Serial.print("\t");
      Serial.print(leds[0].b);
      Serial.print("\t");
      Serial.println(f);
    }
    prevMillisLED = millis();
  }
}


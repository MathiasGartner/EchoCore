#include <EEPROM.h>
#include <FastLED.h>
#include <RF24.h>
#include <SPI.h>

byte BEAT_ID; //This device ID is read from EEPROM adress 0

const byte NUM_BEATS = 3;
const byte BUF_SIZE = 32;
unsigned long T_SEND_MS = 4000;
unsigned long T_RECV_MS = 500;
unsigned long T_WAIT_MS = 350;
unsigned long T_DEFLATE_MS = 6 * 1000;

#define PIN_CE 7
#define PIN_CSN 8

const byte BEAT_ADDRESS[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

#define NUM_LEDS 5
#define PIN_LED_DATA 9

#define NUM_PUMPS 3
#define PIN_PUMP_1 2
#define PIN_PUMP_2 3
#define PIN_PUMP_3 4
#define PIN_VALVE_1 5
#define PIN_VALVE_2 6
#define PIN_VALVE_3 10

const int PIN_PUMP[NUM_PUMPS] = { PIN_PUMP_1, PIN_PUMP_2, PIN_PUMP_3 };
const int PIN_VALVE[NUM_PUMPS] = { PIN_VALVE_1, PIN_VALVE_2, PIN_VALVE_3 };

CRGB leds[NUM_LEDS];

RF24 radio(PIN_CE, PIN_CSN);

unsigned long currentMillis = 0;
unsigned long prevMillisSend = 0;
unsigned long prevMillisRecv = 0;
unsigned long prevMillisDeflate = 0;

int currentData = 0;
char dataToSend[BUF_SIZE] = "";
char dataReceived[BUF_SIZE] = "";
bool newDataAvailable = false;

int pumpCounter[NUM_PUMPS] = { 0, 0, 0 };

void PrintLineSeperator(byte n=1);
void PrintLineSeperator(byte n) {  
  for (byte i = 0; i < n; i++) {
    Serial.println("===========");
  }
}

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

void deflateAll() {
  PrintLineSeperator();
  Serial.println("Deflate all...");
  PrintLineSeperator();
  for (int i = 0; i < NUM_PUMPS; i++) {
    digitalWrite(PIN_VALVE[i], HIGH);
    pumpCounter[i] = 0;
  }
  //TODO: lock enabling pumps for a few seconds in order to be sure that all air is released...
  //      do this also by setting a timestamp variable
}


///////////////////////////
void setup() {
  Serial.begin(115200);
  
  BEAT_ID = EEPROM.read(0);
  PrintLineSeperator(2);
  Serial.print("This is BEAT ID: ");
  Serial.println(BEAT_ID);
  PrintLineSeperator(2);
  
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, NUM_LEDS); 

  radio.begin();
  radio.openReadingPipe(1, BEAT_ADDRESS[BEAT_ID]);
  radio.startListening();

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

  for (int i = 0; i < NUM_PUMPS; i++) {
    pinMode(PIN_PUMP[i], OUTPUT);
    digitalWrite(PIN_PUMP[i], LOW);
    pinMode(PIN_VALVE[i], OUTPUT);
    digitalWrite(PIN_VALVE[i], LOW);
  }
}

///////////////////////////
void loop() {
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

  if (currentMillis - prevMillisDeflate >= T_DEFLATE_MS) {
    deflateAll();
    prevMillisDeflate = millis();
  }
  
  delay(T_WAIT_MS);
}

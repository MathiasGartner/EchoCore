#include <EEPROM.h>
#include <RF24.h>
#include <SPI.h>

byte BEAT_ID; //This device ID is read from EEPROM adress 0

const byte NUM_BEATS = 3;
const byte BUF_SIZE = 32;
unsigned long T_SEND_MS = 4000;
unsigned long T_RECV_MS = 500;
unsigned long T_WAIT_MS = 350;

#define PIN_CE 7
#define PIN_CSN 8

const byte beat_address[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

RF24 radio(PIN_CE, PIN_CSN);

unsigned long currentMillis = 0;
unsigned long prevMillisSend = 0;
unsigned long prevMillisRecv = 0;

int currentData = 0;
char dataToSend[BUF_SIZE] = "";
char dataReceived[BUF_SIZE] = "";
bool newDataAvailable = false;

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

///////////////////////////
void setup() {
  Serial.begin(115200);
  
  BEAT_ID = EEPROM.read(0);
  PrintLineSeperator(2);
  Serial.print("This is BEAT ID: ");
  Serial.println(BEAT_ID);
  PrintLineSeperator(2);

  radio.begin();
  radio.openReadingPipe(1, beat_address[BEAT_ID]);
  radio.startListening();
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
    for (byte n = 0; n < NUM_BEATS; n++) {
      if (n != BEAT_ID) {
        radio.openWritingPipe(beat_address[n]);
        
        radio.write(&dataToSend, BUF_SIZE);

        Serial.print("sent to slave: ");
        Serial.println(n);
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
  
  delay(T_WAIT_MS);
}

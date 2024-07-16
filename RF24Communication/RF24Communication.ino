#include <RF24.h>
#include <SPI.h>

#define BEAT_ID 0

#define NUM_BEATS 3
#define T_SEND_MS 1000
#define T_RECV_MS 1000
#define BUF_SIZE 32

RF24 radio(7, 8); // (CE, CSN)

const byte beat_address[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

unsigned long currentMillis;
unsigned long prevMillis;

int currentData = 0;
char dataToSend[BUF_SIZE] = "";
char dataReceived[BUF_SIZE] = "";
bool newData = false;

void PrintLineSeperator() {  
    Serial.println("===========");
}

int getPulseData() {
  int value = currentData;
  currentData++;
  return value;
}

///////////////////////////
void setup() {
  currentMillis = 0;
  prevMillis = 0;

  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(1, beat_address[BEAT_ID]);
  radio.startListening();
}

void getData() {
  if (radio.available()) {
    radio.read(&dataReceived, BUF_SIZE);
    newData = true;
  }
}

void showData() {
  if (newData == true) {
    PrintLineSeperator();
    Serial.print("Data received ");
    Serial.println(dataReceived);
    PrintLineSeperator();
    newData = false;
  }
}

///////////////////////////
void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= T_SEND_MS) {
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
    prevMillis = millis();
    
    PrintLineSeperator();
  }

  currentMillis = millis();
  if (currentMillis - prevMillis >= T_RECV_MS) {
    getData();
    showData();
  }

  delay(100);
  /*
  int action = random(0, 2);
  if (action == 0) { // send data
    int data = getPulseData();
    Serial.print("Pulse Data: ");
    Serial.println(data);
    radio.write()
  }
  else {

  }
  */
}
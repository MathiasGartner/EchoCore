#include <RF24.h>
#include <SPI.h>

#define BEAT_ID 0

#define NUM_BEATS 3
#define T_SEND_MS 1000
#define T_RECV_MS 100
#define BUF_SIZE 32

RF24 radio(7, 8); // (CE, CSN)

const byte beat_address[NUM_BEATS][5] = { "BEAT1", "BEAT2", "BEAT3" };

unsigned long currentMillis;
unsigned long prevMillisSend;
unsigned long prevMillisRecv;

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
  prevMillisSend = 0;
  prevMillisRecv = 0;

  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
  radio.setChannel(124); // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, beat_address[BEAT_ID]);
  radio.startListening();
}

void getData() {
  if (radio.available()) {
    Serial.println("radio available");
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

  currentMillis = millis();
  if (currentMillis - prevMillisRecv >= T_RECV_MS) {
    getData();
    showData();
    prevMillisRecv = millis();
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

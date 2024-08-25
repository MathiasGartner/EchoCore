
#define NUM_BUBBLES 4
#define PIN_PUMP_1 A2
#define PIN_PUMP_2 A3
#define PIN_PUMP_3 2
#define PIN_PUMP_4 3
#define PIN_VALVE_1 6
#define PIN_VALVE_2 10
#define PIN_VALVE_3 4
#define PIN_VALVE_4 5

const byte PIN_PUMP[NUM_BUBBLES] = { PIN_PUMP_1, PIN_PUMP_2, PIN_PUMP_3, PIN_PUMP_4 };
const byte PIN_VALVE[NUM_BUBBLES] = { PIN_VALVE_1, PIN_VALVE_2, PIN_VALVE_3, PIN_VALVE_4 };

void setup() {
  Serial.begin(115200);
  Serial.println("\n");

  Serial.println("Initialize pumps and valves...");
  for (int i = 0; i < NUM_BUBBLES; i++) {
    pinMode(PIN_PUMP[i], OUTPUT);
    digitalWrite(PIN_PUMP[i], LOW);
    pinMode(PIN_VALVE[i], OUTPUT);
    digitalWrite(PIN_VALVE[i], LOW);
  }
  Serial.println("done");
}

bool pump = false;
bool high = true;

void loop() {
  int k = 2;
  //for (int i = k; i <= k; i++) {
  for (int i = 0; i < NUM_BUBBLES; i++) {
    Serial.print(i);
    if (pump) {
      if (high) {
        Serial.println(" pump on");
        digitalWrite(PIN_PUMP[i], HIGH);
      }
      else {
        Serial.println(" pump off");
        digitalWrite(PIN_PUMP[i], LOW);
      }
    }
    else {
      if (high) {
        Serial.println(" valve on");
        digitalWrite(PIN_VALVE[i], HIGH);
      }
      else {
        Serial.println(" valve off");
        digitalWrite(PIN_VALVE[i], LOW);        
      }
    }
    delay(1000);
  }
  pump = !pump;
  if (!pump)
    high = !high;
}

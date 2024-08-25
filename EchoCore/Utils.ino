///////////////
///  UTILS  ///
///////////////

void PrintLineSeperator(byte n) {  
  for (byte i = 0; i < n; i++) {
    Serial.println("===========");
  }
}
void PrintLineSeperator() {
  PrintLineSeperator(1);
}

void PrintBubbleId(int id) {
  Serial.print("#");
  Serial.print(id);
  Serial.print(": ");
}

///////////////////////
///  COMMUNICATION  ///
///////////////////////

void recvData() {
  if (radio.available()) {
    radio.read(&dataReceived, BUF_SIZE);
    newDataAvailable = true;
  }
}

void showData() {
  Serial.print("Data received ");
  Serial.println(dataReceived);
}

void sendAction() {
  radio.stopListening();
  for (byte i = 0; i < NUM_BEATS; i++) {
    if (i != BEAT_ID) {
      radio.openWritingPipe(BEAT_ADDRESS[i]);
      
      radio.write(&dataToSend, BUF_SIZE);

      Serial.print("send: ");
      Serial.print(dataToSend);
      Serial.print(" to Beat No: ");
      Serial.println(i);
    }
  }
  radio.startListening();
}

//////////////
///  LEDS  ///
//////////////

void fadeLEDs(const CRGB& start, const CRGB& end, float f) {
  CRGB cur;
  cur.r = int(start.r + (end.r - start.r) * f);
  cur.g = int(start.g + (end.g - start.g) * f);
  cur.b = int(start.b + (end.b - start.b) * f);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = cur;
  }
  colorCurrent = cur;
  FastLED.show();
}

void startFadeLEDs(const CRGB& end, unsigned long duration) {
  colorStart = colorCurrent;
  colorEnd = end;
  inLEDTransition = true;
  ledTransitionStart = millis();
  ledTransitionEnd = ledTransitionStart + duration;
  ledTransitionDuration = duration;
}

void printColor(const CRGB& c) {  
  Serial.print("[");
  Serial.print(c.r);
  Serial.print("/");
  Serial.print(c.g);
  Serial.print("/");
  Serial.print(c.b);
  Serial.print("]");
}

void setNewColor() {
  setNewColor(6000);
}

void setNewColor(unsigned long duration) {
  float sum = 0.0;
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (bubbleState[i] == BUBBLE_DEFLATING || bubbleState[i] == BUBBLE_EMPTY) {
      sum += 0.0;
    }
    else {
      sum += bubbleFillLevelGoal[i];
    }
  }
  sum /= NUM_BUBBLES;
  CRGB col;
  uint8_t val;
  val = 255 * sum;
  col.r = constrain(val, 0, 255);
  col.g = random8() * 0;
  val = 255 * (1 - sum);
  col.b = constrain(val, 0, 255);
  startFadeLEDs(col, duration);
  //printColor(col);
  //Serial.println();
}

/////////////////
///  SENSORS  ///
/////////////////

void selectSensor(uint8_t i) {  
  if (i > 7) return; 
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectSensor(CHNL_SENSOR[i]);
    unsigned long val = particleSensors[i].getIR();
    float valAdjusted = (float(val) - sensorIdleValue[i]) / SENSOR_MAX_VAL;
    if (valAdjusted < 0.1) valAdjusted = 0.0; // use a 10% threshold
    if (valAdjusted > 1.0) valAdjusted = 1.0;
    sensorValue[i] = valAdjusted;
    //if(i == 0) {
    //sensorValue[i] = 0.0;  
    //}
  }
}

////////////////////////
///  PUMPS & VALVES  ///
////////////////////////

void stopDeflateBubble(byte id) {
  PrintBubbleId(id);
  Serial.println("stop deflate");
  digitalWrite(PIN_VALVE[id], LOW);
  bubbleState[id] = BUBBLE_EMPTY;
  bubbleFillLevelCurrent[id] = 0;
  bubbleFillLevelStart[id] = 0;
  bubbleFillLevelGoal[id] = 0;
  bubbleStateTimestamp[id] = millis();
}

void deflateBubble(byte id) {
  digitalWrite(PIN_VALVE[id], HIGH);
  digitalWrite(PIN_PUMP[id], LOW);
  bubbleState[id] = BUBBLE_DEFLATING;
  bubbleStateTimestamp[id] = millis();

  PrintBubbleId(id);
  Serial.println("deflate");
  setNewColor(T_DEFLATE_DONE_MS);
}

void deflateAll() {
  Serial.println("Deflate all...");
  for (int i = 0; i < NUM_BUBBLES; i++) {
    deflateBubble(i);
  }
}

void setInflate(byte id, float val) {
  // Serial.print("try fill bubble ");
  // Serial.print(id);
  // Serial.print(" from ");
  // Serial.print(bubbleFillLevelCurrent[id]);
  // Serial.print(" to val ");
  // Serial.print(val);
  // Serial.print(" in state ");
  // Serial.println(bubbleState[id]);
  
  if (val > 1) 
    //Serial.println("val > 1");
    return;
  if (!(bubbleState[id] == BUBBLE_EMPTY || bubbleState[id] == BUBBLE_FILLING || bubbleState[id] == BUBBLE_WAIT))
    //Serial.println(bubbleState[id]);
    return;
  if (val <= bubbleFillLevelGoal[id])
    //Serial.println("val < bubbleFillLevelGoal[id]");
    return;

  float fillDuration = 0.0;
  fillDuration = (val - bubbleFillLevelCurrent[id]) * bubbleFillTime[BEAT_ID][id];
  bubbleState[id] = BUBBLE_FILLING;
  bubbleStateTimestamp[id] = millis();  
  bubbleFillUntil[id] = bubbleStateTimestamp[id] + fillDuration;
  bubbleFillLevelStart[id] = bubbleFillLevelCurrent[id];
  bubbleFillLevelGoal[id] = val;
  
  PrintBubbleId(id);
  Serial.print("fill from ");
  Serial.print(bubbleFillLevelStart[id]);
  Serial.print(" to val ");
  Serial.print(val);
  Serial.print("(for ");
  Serial.print(fillDuration);
  Serial.print("ms) -> until ");
  Serial.print(bubbleFillUntil[id]);
  Serial.print(" / ");
  Serial.print(bubbleStateTimestamp[id]);  
  Serial.println();
}
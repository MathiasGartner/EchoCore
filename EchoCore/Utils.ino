
void PrintLineSeperator(byte n) {  
  for (byte i = 0; i < n; i++) {
    Serial.println("===========");
  }
}
void PrintLineSeperator() {
  PrintLineSeperator(1);
}


void selectSensor(uint8_t i) {  
  if (i > 7) return; 
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void stopDeflateBubble(byte id) {
  Serial.print("stop deflate ");
  Serial.println(id);
  digitalWrite(PIN_VALVE[id], LOW);
  bubbleState[id] = BUBBLE_EMPTY;
  bubbleFillLevel[id] = 0;
  bubbleStateTimestamp[id] = millis();
}

void deflateBubble(byte id) {
  digitalWrite(PIN_VALVE[id], HIGH);
  digitalWrite(PIN_PUMP[id], LOW);
  bubbleState[id] = BUBBLE_DEFLATING;
  bubbleStateTimestamp[id] = millis();

  Serial.print("deflate bubble ");
  Serial.println(id);
}

void deflateAll() {
  Serial.println("Deflate all...");
  for (int i = 0; i < NUM_BUBBLES; i++) {
    deflateBubble(i);
  }
}

void setInflate(byte id, float val) {
  /*
  Serial.print("try fill bubble ");
  Serial.print(id);
  Serial.print(" from ");
  Serial.print(bubbleFillLevel[id]);
  Serial.print(" to val ");
  Serial.print(val);
  Serial.print(" in state ");
  Serial.println(bubbleState[id]);
  */

  if (val > 1) 
    //Serial.println("val > 1");
    return;
  if (!(bubbleState[id] == BUBBLE_EMPTY || bubbleState[id] == BUBBLE_FILLING || bubbleState[id] == BUBBLE_WAIT))
    //Serial.println(bubbleState[id]);
    return;
  if (val <= bubbleFillLevel[id])
    //Serial.println("val < bubbleFillLevel[id]");
    return;

  bubbleState[id] = BUBBLE_FILLING;
  bubbleStateTimestamp[id] = millis();  
  bubbleFillUntil[id] = bubbleStateTimestamp[id] + (val - bubbleFillLevel[id]) * bubbleFillTime[BEAT_ID][id];
  bubbleFillLevel[id] = val;
  
  Serial.print("fill bubble ");
  Serial.print(id);
  Serial.print(" to val ");
  Serial.print(val);
  Serial.print(" until ");
  Serial.print(bubbleFillUntil[id]);
  Serial.print(" / ");
  Serial.println(bubbleStateTimestamp[id]);
}

void readSensors() {  
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectSensor(CHNL_SENSOR[i]);
    unsigned long val = particleSensors[i].getIR();
    float valAdjusted = (float(val) - sensorIdleValue[i]) / SENSOR_MAX_VAL;
    if (valAdjusted < 0.0) valAdjusted = 0.0;
    if (valAdjusted > 1.0) valAdjusted = 1.0;
    sensorValue[i] = valAdjusted;
  }
}
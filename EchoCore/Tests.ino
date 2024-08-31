
///////////////
///  TESTS  ///
///////////////

byte testFill_bubbleId = 0;
float testFill_val = 0.1;
unsigned long testFill_prevMillisLastAction = 0;

void testFill2() {
  for (int i = 0; i < NUM_BUBBLES; i++) {
    if (i != 2) continue;
    PrintBubbleId(i);
    Serial.println("inflate");
    digitalWrite(PIN_PUMP[i], HIGH);
    delay(bubbleFillTime[BEAT_ID][i]);
    digitalWrite(PIN_PUMP[i], LOW);
    deflateBubble(i);
    delay(bubbleDeflateTime[BEAT_ID][i]);
    stopDeflateBubble(i);
    Serial.println("done");
  }
  Serial.println("all done");
  //delay(10000);
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
      if (currentMillis - bubbleStateTimestamp[i] >= bubbleDeflateTime[BEAT_ID][i]) {
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
    readSensors(true);
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

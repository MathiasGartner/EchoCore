#include <FastLED.h>

#define NUM_LEDS 5

CRGB leds[NUM_LEDS];

int led_id;

void setup() { 
  led_id = 0;
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, 9>(leds, NUM_LEDS); 
}

void loop() {

	leds[led_id] = CRGB::White; 
  FastLED.show(); 
  delay(300);
  Serial.println("test");
	leds[led_id] = CRGB::Red; 
  FastLED.show(); 
  delay(300);

  led_id += 1;
  led_id = led_id % NUM_LEDS;
}
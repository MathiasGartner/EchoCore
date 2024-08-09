#include <FastLED.h>

#define NUM_LEDS 70
#define PIN_LED_DATA 9

CRGB leds[NUM_LEDS];

int led_id;

void setup() { 
  led_id = 0;
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, NUM_LEDS); 
}

void loop() {
	leds[led_id] = CRGB::Red; 
  FastLED.show(); 
  delay(100);
  Serial.println("test");
	leds[led_id] = CRGB::White; 
  FastLED.show(); 
  delay(100);

  led_id += 1;
  led_id = led_id % NUM_LEDS;
}
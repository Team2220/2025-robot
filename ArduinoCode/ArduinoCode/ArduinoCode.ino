#include <FastLED.h>

#define LED_PIN 7
#define NUM_LEDS 100
// number of leds on the strip
// FastLED.show(); (ill just keep this here)

typedef struct ColorData
{
  byte red = 0;
  byte green = 0;
  byte blue = 0;
} ColorData;

ColorData getColor(void);

CRGB leds[NUM_LEDS];
byte value = -1;
const unsigned long interval = 150; // 1 second interval in milliseconds
const unsigned int msgStart = 0x30;
const unsigned int wordBytes = 4;
unsigned long previousMillis = 0;
bool toggleState = false; // Track which command is active


void setup()
{
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(12, OUTPUT);
}

ColorData getColor(void)
{
  unsigned int inWord[wordBytes];
  ColorData commandedColor;

  for (unsigned int n = 0; n < wordBytes; n++)
  {
    inWord[n] = Serial.read();
  }

  if (inWord[0] == msgStart)
  {
    commandedColor.red = inWord[1];
    commandedColor.green = inWord[2];
    commandedColor.blue = inWord[3];
  }

  return commandedColor;
}

void loop()
{
  static ColorData currentColor;
  static int LEDStat = HIGH;

  // if a word is in the serial buffer
  if (Serial.available() == 4)
  {
    // decode color data from serial
    currentColor = getColor();
    LEDStat = ~LEDStat;
  }

  digitalWrite(13, LEDStat);
  fill_solid(leds, NUM_LEDS, CRGB(currentColor.red, currentColor.green, currentColor.blue));
  FastLED.show();
}

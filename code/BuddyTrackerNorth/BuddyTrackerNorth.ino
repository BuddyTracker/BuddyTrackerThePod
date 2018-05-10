#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define NEO_DATA_PIN 6
#define STRIP_LENGTH 7
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(STRIP_LENGTH, NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);
uint32_t LED_COLOR = strip.Color(255, 0, 255);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
    #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    #endif
    // End of trinket special code
    
    
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    
    Serial.begin(115200);
    
    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(1000);
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);
    
    int LEDNorth = orientation2LED((float)event.orientation.x);
    
    clearStrip();
    if(LEDNorth > -1){
        strip.setPixelColor(LEDNorth, LED_COLOR);
    }
    strip.show();
    
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

int orientation2LED(float orientation){
    const int fov = 180;
    const int degreesPerLED = fov / STRIP_LENGTH;
    int offsetOrientation = orientation + fov / 2;

    while(offsetOrientation > 360){
        offsetOrientation -= 360;
    }
    if(offsetOrientation > fov){
        return -1;
    }

    return STRIP_LENGTH - offsetOrientation / degreesPerLED;
}

void clearStrip(){
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, 0);
    }
}


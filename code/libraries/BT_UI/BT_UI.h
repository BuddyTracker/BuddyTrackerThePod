#ifndef BT_UI_H
#define BT_UI_H

#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS  8
#define LED_OFF     0
#define LED_ON      255

class BT_UI {
    public:
        BT_UI(uint8_t pin);

        void
            begin(void),
            setBrightness(uint8_t b),
            test(void),
            show(void),
            clear(void);
        uint32_t
            Color(uint8_t r, uint8_t g, uint8_t b),
            Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

        void setBuddyLight(uint8_t light, uint32_t color);
        void setWaypointLight(uint8_t light);
        uint8_t orientation2LED(float orientation);
    protected:
        Adafruit_NeoPixel strip;
        uint8_t stripPin;
        uint32_t buddyColor;
        uint32_t waypointColor;
        uint8_t fov;
};

#endif
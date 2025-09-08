#ifndef NEOPIXEL_LED_H
#define NEOPIXEL_LED_H

#include <Adafruit_NeoPixel.h>
#include "system_config.h"

// =============================================================================
// NEOPIXEL LED WRAPPER CLASS
// =============================================================================

class NeoPixelLED {
public:
    NeoPixelLED();
    ~NeoPixelLED();
    
    bool initialize();
    void cleanup();
    
    // Color control methods
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setColor(uint32_t color);
    void setBrightness(uint8_t brightness);
    void off();
    
    // Status indication methods
    void showUSBInit();
    void showUSBReady();
    void showUSBError();
    void showWaitingForAgent();
    void showAgentFound();
    void showConnected();
    void showDisconnected();
    
    // Utility methods
    void blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t onTime, uint16_t offTime);
    void fade(uint8_t red, uint8_t green, uint8_t blue, uint16_t duration);
    
private:
    Adafruit_NeoPixel* strip;
    bool initialized;
    uint8_t currentBrightness;
    
    // Color constants
    static const uint32_t COLOR_RED = 0xFF0000;
    static const uint32_t COLOR_GREEN = 0x00FF00;
    static const uint32_t COLOR_BLUE = 0x0000FF;
    static const uint32_t COLOR_YELLOW = 0xFFFF00;
    static const uint32_t COLOR_CYAN = 0x00FFFF;
    static const uint32_t COLOR_MAGENTA = 0xFF00FF;
    static const uint32_t COLOR_WHITE = 0xFFFFFF;
    static const uint32_t COLOR_OFF = 0x000000;
};

// Global instance
extern NeoPixelLED statusLED;

#endif // NEOPIXEL_LED_H

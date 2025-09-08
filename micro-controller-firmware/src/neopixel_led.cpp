#include "neopixel_led.h"

// Global instance
NeoPixelLED statusLED;

// =============================================================================
// NEOPIXEL LED IMPLEMENTATION
// =============================================================================

NeoPixelLED::NeoPixelLED() 
    : strip(nullptr), initialized(false), currentBrightness(50) {
}

NeoPixelLED::~NeoPixelLED() {
    cleanup();
}

bool NeoPixelLED::initialize() {
    if (initialized) {
        return true;
    }
    
    // Create NeoPixel strip
    strip = new Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
    if (strip == nullptr) {
        return false;
    }
    
    // Initialize the strip
    strip->begin();
    strip->setBrightness(currentBrightness);
    strip->clear();
    strip->show();
    
    initialized = true;
    return true;
}

void NeoPixelLED::cleanup() {
    if (strip != nullptr) {
        strip->clear();
        strip->show();
        delete strip;
        strip = nullptr;
    }
    initialized = false;
}

void NeoPixelLED::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    if (!initialized || strip == nullptr) return;
    
    strip->setPixelColor(0, strip->Color(red, green, blue));
    strip->show();
}

void NeoPixelLED::setColor(uint32_t color) {
    if (!initialized || strip == nullptr) return;
    
    strip->setPixelColor(0, color);
    strip->show();
}

void NeoPixelLED::setBrightness(uint8_t brightness) {
    if (!initialized || strip == nullptr) return;
    
    currentBrightness = brightness;
    strip->setBrightness(brightness);
    strip->show();
}

void NeoPixelLED::off() {
    setColor(COLOR_OFF);
}

// Status indication methods
void NeoPixelLED::showUSBInit() {
    setColor(COLOR_BLUE);  // Blue = USB initialization starting
}

void NeoPixelLED::showUSBReady() {
    // Quick blink sequence to show USB ready
    setColor(COLOR_GREEN);
    delay(100);
    setColor(COLOR_OFF);
    delay(100);
    setColor(COLOR_GREEN);
    delay(100);
    setColor(COLOR_OFF);
}

void NeoPixelLED::showUSBError() {
    setColor(COLOR_RED);  // Red = USB error
}

void NeoPixelLED::showWaitingForAgent() {
    // Slow blink blue = waiting for agent
    static uint32_t lastBlink = 0;
    uint32_t now = millis();
    
    if (now - lastBlink >= 1000) {  // 1 second cycle
        static bool blinkState = false;
        blinkState = !blinkState;
        setColor(blinkState ? COLOR_BLUE : COLOR_OFF);
        lastBlink = now;
    }
}

void NeoPixelLED::showAgentFound() {
    // Fast blink yellow = agent found, creating entities
    static uint32_t lastBlink = 0;
    uint32_t now = millis();
    
    if (now - lastBlink >= 200) {  // 200ms cycle
        static bool blinkState = false;
        blinkState = !blinkState;
        setColor(blinkState ? COLOR_YELLOW : COLOR_OFF);
        lastBlink = now;
    }
}

void NeoPixelLED::showConnected() {
    setColor(COLOR_GREEN);  // Solid green = connected
}

void NeoPixelLED::showDisconnected() {
    setColor(COLOR_OFF);  // Off = disconnected
}

// Utility methods
void NeoPixelLED::blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t onTime, uint16_t offTime) {
    setColor(red, green, blue);
    delay(onTime);
    setColor(COLOR_OFF);
    delay(offTime);
}

void NeoPixelLED::fade(uint8_t red, uint8_t green, uint8_t blue, uint16_t duration) {
    if (!initialized || strip == nullptr) return;
    
    uint16_t steps = duration / 10;  // 10ms per step
    uint8_t stepRed = red / steps;
    uint8_t stepGreen = green / steps;
    uint8_t stepBlue = blue / steps;
    
    for (uint16_t i = 0; i < steps; i++) {
        setColor(stepRed * i, stepGreen * i, stepBlue * i);
        delay(10);
    }
    setColor(red, green, blue);
}

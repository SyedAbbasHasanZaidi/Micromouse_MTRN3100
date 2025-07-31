#include "OLEDDisplay.hpp"

OledDisplay::OledDisplay() : display(128, 64, &Wire) {}

void OledDisplay::begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED init failed!");
        while (true);  // Halt if OLED fails
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void OledDisplay::clear() {
    display.clearDisplay();
}

void OledDisplay::displayText(int x, int y, const String& text, int size) {
    display.setTextSize(size);
    display.setCursor(x, y);
    display.println(text);
}

void OledDisplay::update() {
    display.display();
}

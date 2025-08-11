#ifndef OLED_DISPLAY_HPP
#define OLED_DISPLAY_HPP

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class OledDisplay {
public:
    OledDisplay();

    void begin();
    void clear();
    void displayText(int x, int y, const String& text, int size = 1);
    void update();

private:
    Adafruit_SSD1306 display;
};

#endif

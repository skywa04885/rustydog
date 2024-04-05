#pragma once

#include <Adafruit_SSD1306.h>

#define DISPLAY_MOD_UPDATE_INTERVAL 200

class DisplayMod
{
private:
    Adafruit_SSD1306 display;
    unsigned long lastUpdate;
public:
    DisplayMod();

    void setup();

    void loop();

private:
    void update();

    void drawServo(int n);
};

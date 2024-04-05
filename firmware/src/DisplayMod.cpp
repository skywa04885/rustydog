#include "DisplayMod.hpp"
#include "Ethernet.h"
#include "ServoMod.hpp"

extern void errorHandler();

extern ServoMod servos[];
extern int numberOfServos;

DisplayMod::DisplayMod()
    : display(128, 64),
      lastUpdate(0)
{
}

void DisplayMod::setup()
{
    // Begin the display.
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("E: SSD1306 allocation failed"));
        errorHandler();
    }

    // Clear the display and write it.
    display.clearDisplay();
    display.display();

    // Update the display.
    this->update();
}

void DisplayMod::loop()
{
    this->update();
}

void DisplayMod::update()
{
    if (millis() - this->lastUpdate < DISPLAY_MOD_UPDATE_INTERVAL)
    {
        return;
    }

    this->display.clearDisplay();

    // Draw the status bar.
    if (Ethernet.linkStatus() == EthernetLinkStatus::LinkON)
    {
        const IPAddress ip = Ethernet.localIP();

        String ipString = "";
        ipString += ip[0];
        ipString += ".";
        ipString += ip[1];
        ipString += ".";
        ipString += ip[2];
        ipString += ".";
        ipString += ip[3];

        this->display.setTextColor(WHITE);
        this->display.setTextSize(1);
        this->display.setCursor(0, 0);
        this->display.print(ipString);
    }

    for (int i = 0; i < numberOfServos; ++i)
    {
        this->drawServo(i);
    }

    this->display.display();

    this->lastUpdate = millis();
}

void DisplayMod::drawServo(int n)
{
    ServoMod &servo = servos[n];

    const int topOffset = 18;

    const int radius = 5;
    const int padding = 3;

    const int circleX = n * radius * 2 + radius + n * padding + padding;
    const int circleY = topOffset + radius;

    this->display.fillCircle(circleX, circleY, radius, WHITE);

    const int charX = circleX - radius + 3;
    const int charY = circleY - radius + 2;

    const char nChar = '0' + static_cast<char>(n); // Allowed, not more than 9 servos.
    this->display.drawChar(charX, charY, nChar, BLACK, WHITE, 1);

    const int lineRangeStartY = topOffset + radius * 2 + padding;
    const int lineRangeEndY = 64;
    const int lineRangeCenterY = lineRangeStartY + (lineRangeEndY - lineRangeStartY) / 2;
    const int lineX = circleX;

    this->display.fillCircle(lineX, lineRangeCenterY, 1, WHITE);

    const int angle = servo.getAngle();

    int angleLineStartY = 0;
    int angleLineEndY = 0;

    if (angle > 0)
    {
        const int lineHeight = static_cast<int>((static_cast<float>(lineRangeCenterY - lineRangeStartY) / 180.0f) * static_cast<float>(angle));

        angleLineStartY = lineRangeCenterY;
        angleLineEndY = lineRangeCenterY - lineHeight;
    }
    else if (angle < 0)
    {
        const int lineHeight = static_cast<int>((static_cast<float>(lineRangeEndY - lineRangeCenterY) / 180.0f) * static_cast<float>(-angle));

        angleLineStartY = lineRangeCenterY;
        angleLineEndY = lineRangeCenterY + lineHeight;
    }

    if (angleLineStartY != angleLineEndY)
    {
        this->display.drawLine(lineX, angleLineStartY, lineX, angleLineEndY, WHITE);
    }
}
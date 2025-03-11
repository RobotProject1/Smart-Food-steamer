#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include "VEGA_MLX90614.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

VEGA_MLX90614 mlx(18, 19);
Adafruit_SSD1306 display(4);

// Pin
const int touchpad1 = 0;
const int touchpad2 = 1;
const int touchpad3 = 2;

// Initial values
int valtp1 = 0;
int stat1 = 0;
int valtp3 = 0;
int stat3 = 0;

void setup() {
  Serial.begin(9600);

  // Pin modes
  pinMode(touchpad1, INPUT);
  pinMode(touchpad2, INPUT);
  pinMode(touchpad3, INPUT);
}

void loop() {
  checkTouchpad1();     // Check touchpad and toggle state
  checkTouchpad3();     // Check touchpad and toggle state
  displaySetup();       // Setup for OLED display
  delay(300);            // Main loop delay
}

void drawMode() {
  display.fillRect(0, 20, 50, 20, SSD1306_BLACK);  // Clear previous mode text
  display.setCursor(5, 10);
  display.print("MODE:");
  display.setCursor(5, 30);
  display.print(stat1 == 0 ? "MANUAL" : "AUTO");
  display.display();
}

void drawLightBulb() {
  int x = 110, y = 12;
  display.fillRect(100, 0, 28, 25, SSD1306_BLACK); // Clear previous lightbulb
  if (stat3 == 1) {
    // Light ON - Draw glowing bulb
    display.drawCircle(x, y, 9, SSD1306_WHITE);
    display.fillCircle(x, y, 6, SSD1306_WHITE);
    display.drawLine(x, 18, x, 24, SSD1306_WHITE);
    display.drawLine(x - 3, 24, x + 3, 24, SSD1306_WHITE);
  } else {
    // Light OFF - Only draw bulb outline
    display.drawCircle(x, y, 9, SSD1306_WHITE);
    display.drawLine(x, 18, x, 24, SSD1306_WHITE);
    display.drawLine(x - 3, 24, x + 3, 24, SSD1306_WHITE);
  }
  display.display();
}

void checkTouchpad1() {
  valtp1 = digitalRead(touchpad1);
  if (valtp1 == 1) {
    stat1 = !stat1;
    drawMode();
    delay(100);
  }
}

void checkTouchpad3() {
  valtp3 = digitalRead(touchpad3);
  if (valtp3 == 1) {
    stat3 = !stat3;
    drawMode();
    delay(100);
  }
}

// Function to update OLED display (placeholder)
void displaySetup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  drawMode();
  drawLightBulb();
  // Add OLED display code here (e.g., using SSD1306 library)
  // Example: display temperature, servo position, or status
}
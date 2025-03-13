#include <OneWire.h>
#include <DS18B20.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_LEDBackpack.h"
#include "VEGA_MLX90614.h"

VEGA_MLX90614 mlx(18, 19);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
Adafruit_SSD1306 display(4);
DS18B20 ds(4);

void setup() {
  Serial.begin(9600);
  
  // Initialize the 7-segment display
  alpha4.begin();
  display.clearDisplay();
  display.display();
}

void loop(){
  void updatesevensegdisplay();
}

void updatesevensegdisplay() {
  float tempP = ds.getTempC();
  
  char buffer[5]; // Buffer to hold "25.0"
  dtostrf(tempP, 4, 1, buffer);  // Convert float to string with 1 decimal place
  
  for (int i = 0; i < 4; i++) {
    alpha4.writeDigitAscii(i, buffer[i]);
  }
  
  alpha4.writeDisplay(); // Update display
}

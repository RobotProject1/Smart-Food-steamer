#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include "VEGA_MLX90614.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Servo myservo;
VEGA_MLX90614 mlx(18, 19);
Adafruit_SSD1306 display(4);

// Pin
const int touchpad1 = 0;
const int touchpad2 = 1;
const int touchpad3 = 2;
const int buzz = 3;

// Initial values
int valtp1 = 0;
int stat1 = 0;
int valtp3 = 0;
int stat3 = 0;

int valtp2 = 0;
int stat2 = 0;
int seropen = 90;
int serclose = 0;

const int SAMPLES = 10;
float s_val[SAMPLES];
float kt = 2;

// // LID PID variables and setup
// double lSetpoint, lInput, lOutput;
// double lKp = 2, lKi = 5, lKd = 1;
// PID lidPID(&lInput, &lOutput, &lSetpoint, lKp, lKi, lKd, DIRECT);

// TEMP PID variables and setup
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float threshold = 0.0;

void setup() {
  Serial.begin(9600);

  // Servo setup
  myservo.attach(6);
  myservo.write(serclose);

  // Pin modes
  pinMode(touchpad1, INPUT);
  pinMode(touchpad2, INPUT);
  pinMode(touchpad3, INPUT);
  pinMode(buzz, OUTPUT);

  // Initial PID inputs
  Input = mlx.mlx90614ReadTargetTempC(); // Temp input
  lInput = myservo.read();               // Servo current position

  // PID mode activation
  lidPID.SetMode(AUTOMATIC);
  tempPID.SetMode(AUTOMATIC);
}

void loop() {
  checkTouchpad2();       // Check touchpad and toggle state
  updateServoState();    // Update servo position based on state
  updateTempPID();       // Placeholder for temp PID logic
  checkTouchpad1();     // Check touchpad and toggle state
  checkTouchpad3();     // Check touchpad and toggle state
  displaySetup();       // Setup for OLED display
  delay(300);            // Main loop delay
}

// Function to check touchpad and toggle state
void checkTouchpad2() {
  valtp2 = digitalRead(touchpad2);
  if (valtp2 == 1) {
    tone(buzz, 3000, 100);
    stat2 = !stat2;
    delay(100);
  }
}

void moveServo(int targetPos) {
  int currentPos = myservo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      myservo.write(pos);
      delay(20);
      
      float cur = readCurrent();
      if (cur > threshold) {
        Serial.println("⚠️ Obstacle detected! Stopping servo.");
        return;
      }
    }
  } 
  else if (currentPos > targetPos) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      myservo.write(pos);
      delay(20);
      
      float cur = readCurrent();
      if (cur > threshold) {
        Serial.println("⚠️ Obstacle detected! Stopping servo.");
        return;
      }
    }
  }
}

// float calibratethreshold(float mean, float sd) {
//   float threshold = mean + (kt * sd);
//   return threshold;
// }

// float readCurrent() {
//   int adc = analogRead(A1);
//   float voltage = adc * 5.0 / 1023.0;
//   float current = (voltage - 2.5) / 0.185;
//   return current;
// }

// float calculateMean(float arr[], int size) {
//   float sum = 0;
//   for (int i = 0; i < size; i++) {
//     sum += arr[i];
//   }
//   return sum / size;
// }

// float calculateSD(float arr[], int size, float mean) {
//   float sumSquaredDiffs = 0;
//   for (int i = 0; i < size; i++) {
//     sumSquaredDiffs += pow(arr[i] - mean, 2);
//   }
//   return sqrt(sumSquaredDiffs / size);  // Population SD (use (size-1) for sample SD)
// }

// Function to update servo state based on stat
void updateServoState() {
  float cur = readCurrent();  // Read the current value

<<<<<<< HEAD
  if (stat2 == 1) { // Lid is open
    if (cur > threshold && myservo.read() != serclose) {
      Serial.println("Closing lid due to obstacle...");
      moveServo(serclose);
      stat2 = 0;
    } 
    else if (myservo.read() != seropen) {
      moveServo(seropen);
    }
  } 
  else { // Lid is closed
    if (cur > threshold && myservo.read() != seropen) {
      Serial.println("Opening lid due to obstacle...");
      moveServo(seropen);
      stat2 = 1;
    } 
    else if (myservo.read() != serclose) {
      moveServo(serclose);
=======
  // Check the condition for opening or closing the lid based on the threshold
  if (stat2 == 1) {  // Lid is currently open
    if (cur > threshold) {
      if (myservo.read() != serclose) {  // If servo isn't already closed
        moveServo(serclose);  // Close the lid
        stat2 = 0;  // Update state to closed
      }
    }
    else {
      if (myservo.read() != seropen) {  // If servo isn't already open
        moveServo(seropen);  // Open the lid
      }
    }
  }
  else {  // Lid is currently closed
    if (cur > threshold) {
      if (myservo.read() != seropen) {  // If servo isn't already open
        moveServo(seropen);  // Open the lid
        stat2 = 1;  // Update state to open
      }
    }
    else {
      if (myservo.read() != serclose) {  // If servo isn't already closed
        moveServo(serclose);  // Close the lid
      }
>>>>>>> f7e03ddded479c2ff5559147a9869b2c61eedae5
    }
  }
}

// Function to handle temperature PID (placeholder)
void updateTempPID() {
  Input = mlx.mlx90614ReadTargetTempC();
  tempPID.Compute();
  // Add logic here to use Output (e.g., control a heater or fan)
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
    tone(buzz, 3000, 100);
    stat1 = !stat1;
    drawMode();
    delay(100);
  }
}

void checkTouchpad3() {
  valtp3 = digitalRead(touchpad3);
  if (valtp3 == 1) {
    tone(buzz, 3000, 100);
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
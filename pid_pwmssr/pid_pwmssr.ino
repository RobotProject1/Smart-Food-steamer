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
int seropen = 135;
int serclose = 15;

const int SAMPLES = 10;
float s_val[SAMPLES];
float kt = 2;

int stovePin = 13;

// // LID PID variables and setup
// double lSetpoint, lInput, lOutput;
// double lKp = 2, lKi = 5, lKd = 1;
// PID lidPID(&lInput, &lOutput, &lSetpoint, lKp, lKi, lKd, DIRECT);

// TEMP PID variables and setup
double Setpoint, Input, Output;
double Kp = 2, Ki = 0, Kd = 0;
PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float threshold = 1.2; // ไว้มาแก้
int WindowSize = 5000;
unsigned long windowStartTime;

void setup() {
  Serial.begin(9600);

  pinMode(stovePin, OUTPUT);

  // Initial PID inputs // Temp input
  tempPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  Setpoint = 30;
  // lInput = myservo.read();               // Servo current position

  // PID mode activation
  // lidPID.SetMode(AUTOMATIC);

}

void loop() {
  updateTempPID();
  Serial.print("CurrentTemp : ");Serial.println(Input);
  Serial.print("signal : ");Serial.println(Output);
  delay(500);
}

// Function to check touchpad and toggle state
void checkTouchpad2() {
  valtp2 = digitalRead(touchpad2);
  if (valtp2 == 1) {
    //tone(buzz, 3000, 100);
    stat2 = !stat2;
    delay(100);
  }
}

// Function to handle temperature PID (placeholder)
void updateTempPID() {
  Input = mlx.mlx90614ReadTargetTempC();
  tempPID.Compute();
  pwm(Output);
}


void pwm(int sig) {
  sig = constrain(sig, 0, 255);  // Ensure PWM value is within range

  int onTime = map(sig, 0, 255, 0, 10);  // Convert 0-255 to 0-10 ms
  int offTime = 10 - onTime;  // Complementary OFF time

  digitalWrite(stovePin, HIGH);
  delay(onTime);  // Keep SSR ON for 'onTime'

  digitalWrite(stovePin, LOW);
  delay(offTime);  // Keep SSR OFF for 'offTime'
}


// void checkTouchpad3() {
//   valtp3 = digitalRead(touchpad3);
//   if (valtp3 == 1) {
//     tone(buzz, 3000, 100);
//     stat3 = !stat3;
//     drawMode();
//     delay(100);
//   }
// }

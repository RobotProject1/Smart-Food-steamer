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
bool stat2;
int seropen = 135;
int serclose = 10;

const int SAMPLES = 10;
float s_val[SAMPLES];
float kt = 2;

// // LID PID variables and setup
// double lSetpoint, lInput, lOutput;
// double lKp = 2, lKi = 5, lKd = 1;
// PID lidPID(&lInput, &lOutput, &lSetpoint, lKp, lKi, lKd, DIRECT);

// // TEMP PID variables and setup
// double Setpoint, Input, Output;
// double Kp = 2, Ki = 5, Kd = 1;
// PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float thresholdup = 3; // ไว้มาแก้
float thresholddown = 3.2; // ไว้มาแก้

void setup() {
  stat2 = 0;
  Serial.begin(9600);

  // Servo setup
  myservo.attach(6);
  moveServo(serclose);


  // Pin modes
  pinMode(touchpad1, INPUT);
  pinMode(touchpad2, INPUT);
  // pinMode(touchpad3, INPUT);
  // pinMode(buzz, OUTPUT);

  // // Initial PID inputs
  // Input = mlx.mlx90614ReadTargetTempC(); // Temp input
  // // lInput = myservo.read();               // Servo current position

  // // PID mode activation
  // // lidPID.SetMode(AUTOMATIC);
  // tempPID.SetMode(AUTOMATIC);
}

void loop() {
  checkTouchpad2();       // Check touchpad and toggle state
  updateServoState();    // Update servo position based on state
  //Serial.println(stat2);
}

// // Function to check touchpad and toggle state
// void checkTouchpad2() {
//   valtp2 = digitalRead(touchpad2);
//   if (valtp2 == 1) {
//     tone(buzz, 3000, 100);
//     stat2 = !stat2;
//     delay(100);
//   }
// }

void checkTouchpad2() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read input until newline
    command.trim(); // Remove extra spaces/newlines

    if (command == "toggle") { // Simulate button press
      tone(buzz, 3000, 100);
      stat2 =!stat2;
      Serial.print("Button Press Simulated! stat2 is : ");
      Serial.println(stat2);
    }
  }
}

void moveServo(int targetPos) {
  int currentPos = myservo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      myservo.write(pos);
    }
  } 
  else if (currentPos > targetPos-10) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      myservo.write(pos);
      delay(20);
    }
  }
}


// float calibratethreshold(float mean, float sd) {
//   float threshold = mean + (kt * sd);
//   return threshold;
// }

// Function to read current from sensor
float readCurrent() {
  int adc = analogRead(A1);
  float voltage = adc * 5.0 / 1023.0;
  float current = (voltage - 2.5) / 0.185;  // Adjust 0.185 based on your sensor specs
  return current;
}

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
// void updateServoState() {
//   float cur = readCurrent();  // Read the current value

//   if (stat2 == 1) { // Lid is open
//     if (abs(cur) < thresholddown && myservo.read() != serclose) {
//       Serial.println("Closing lid");
//       moveServo(serclose);
//     } 
//     else if (myservo.read() != seropen) {
//       Serial.println("Opening lid due to obstacle...");
//       moveServo(seropen);
//       stat2 = !stat2;
//     }
//   } 
//   else if (stat2 == 0) { // Lid is closed
//     if (abs(cur) < thresholdup && myservo.read() != seropen) {
//       Serial.println("Opening lid");
//       moveServo(seropen);
//     } 
//     else if (myservo.read() != serclose) {
//       Serial.println("Closing lid due to obs");
//       moveServo(serclose);
//       stat2 = !stat2;
//     }
//   }
// }

void updateServoStatenocurrent() {
  float cur = readCurrent();  // Read the current value

  if (stat2 == 0) {
    Serial.println("Closing lid");
    moveServo(serclose);
    // else if (myservo.read() != seropen) {
    //   Serial.println("Opening lid due to obstacle...");
    //   moveServo(seropen);
    //   stat2 = !stat2;
    // }
  } 
  else if (stat2 == 1) {
    Serial.println("Opening lid");
    moveServo(seropen);
    // else if (myservo.read() != serclose) {
    //   Serial.println("Closing lid due to obs");
    //   moveServo(serclose);
    //   stat2 = !stat2;
    // }
  }
}

void updateServoState() {
  float cur = readCurrent();  // Read the current value
  int pos = myservo.read();
  if (stat2 == 1) {
    for ( ; pos < seropen-10; pos += 3) {
      //Serial.println("Lid is opening...");
      myservo.write(pos);
      int pos = myservo.read();
      delay(8);
      Serial.println(readCurrent());
      if (float cur = abs(readCurrent())>thresholdup) {
        Serial.print("obstacle detect, lid closing :");
        Serial.println(cur);
        moveServo(serclose);
        stat2=0;
        return;
      }
    }
  } 
  else if (stat2 == 0) {
    for ( ; pos != serclose; pos = pos-1) {
      //Serial.println("Lid is closing...");
      Serial.println(readCurrent());
      myservo.write(pos);
      int pos = myservo.read();
      delay(20);
      if (float cur = abs(readCurrent())>thresholddown) {
        Serial.println(readCurrent());
        Serial.print("obstacle detect, lid open");
        moveServo(seropen);
        stat2=1;
        return;
      }
    }
  }
}
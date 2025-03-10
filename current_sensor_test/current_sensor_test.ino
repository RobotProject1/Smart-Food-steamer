#include <PID_v1.h>
#include <Servo.h>
#include "VEGA_MLX90614.h"

// Object initialization
Servo myservo;
VEGA_MLX90614 mlx(18, 19);

// Pin
const int touchpad1 = 0;
const int touchpad2 = 1;
const int touchpad3 = 2;
const int buzz = 3;

// Initial values
int valtp1 = 0;
int stat = 0;
int seropen = 90;
int serclose = 0;
float kt = 2;

const int SAMPLES = 10;
float s_val[SAMPLES];

void setup() {
  Serial.begin(9600);  // Start serial communication
  myservo.attach(6);  // Attach the servo to the defined pin
}

void loop() {
  if (cur > threshold) {
    if (myservo.read() != serclose) {  // If servo isn't already closed
      moveServo(serclose);  // Close the lid
      stat = 0;  // Update state to closed
    }
  }
  else {
    if (myservo.read() != seropen) {  // If servo isn't already open
      moveServo(seropen);  // Open the lid
    }
  }
}

float readCurrent() {
  int adc = analogRead(A5);
  float voltage = adc * 5.0 / 1023.0;
  float current = (voltage - 2.5) / 0.185;
  return current;
}

float calculateMean(float arr[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

float calculateSD(float arr[], int size, float mean) {
  float sumSquaredDiffs = 0;
  for (int i = 0; i < size; i++) {
    sumSquaredDiffs += pow(arr[i] - mean, 2);
  }
  return sqrt(sumSquaredDiffs / size);  // Population SD (use (size-1) for sample SD)
}

void moveServo(int targetPos) {
  int currentPos = myservo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) { 
      // Collect SAMPLES before processing
      for (int i = 0; i < SAMPLES; i++) {
        myservo.write(pos);
        delay(20);
        s_val[i] = readCurrent();  // Store current reading
      }

      float mean = calculateMean(s_val, SAMPLES);
      float sd = calculateSD(s_val, SAMPLES, mean);
      float threshold = calibratethreshold(mean, sd);
      memset(s_val, 0, sizeof(s_val));
    }
  }

  else if (currentPos > targetPos) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      for (int i = 0; i < SAMPLES; i++) {
        myservo.write(pos);
        delay(20);
        s_val[i] = readCurrent();  // Store current reading
      }
      
      float mean = calculateMean(s_val, SAMPLES);
      float sd = calculateSD(s_val, SAMPLES, mean);
      float threshold = calibratethreshold(mean, sd);
      memset(s_val, 0, sizeof(s_val));
    }
  }
}
float calibratethreshold(float mean, float sd) {
  float threshold = mean + (kt * sd);
  return threshold;
}



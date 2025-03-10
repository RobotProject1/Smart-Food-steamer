#include <PID_v1.h>
#include <Servo.h>
#include "VEGA_MLX90614.h"

// Object initialization
Servo myservo;
VEGA_MLX90614 mlx(18, 19);

// Pin assignments
const int buzz = 3;  // Buzzer pin

// Initial values
int stat = 0;
const int seropen = 130;
const int serclose = 0;
const float kt = 5;

const int SAMPLES = 10;
float s_val[SAMPLES];
float threshold = 0.0;

void setup() {
  Serial.begin(9600);
  myservo.attach(6);
  pinMode(buzz, OUTPUT);
}

void loop() {
  moveServo(seropen);
  delay(2000);  // Wait for 2 seconds
  moveServo(serclose);
  delay(2000);  // Wait for 2 seconds
}

float readCurrent() {
  int adc = analogRead(A1);
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
  return sqrt(sumSquaredDiffs / size);
}

void moveServo(int targetPos) {
  int currentPos = myservo.read();
  int step = (currentPos < targetPos) ? 1 : -1;

  for (int pos = currentPos; pos != targetPos + step; pos += step) {
    myservo.write(pos);
    delay(10);

    // Collect SAMPLES for current measurement
    for (int i = 0; i < SAMPLES; i++) {
      s_val[i] = readCurrent();
    }
    float cur = readCurrent();
    float mean = calculateMean(s_val, SAMPLES);
    float sd = calculateSD(s_val, SAMPLES, mean);
    threshold = calibrateThreshold(mean, sd);

    if (cur > threshold) {
      Serial.println("Obstacle detected!");
      Serial.println(threshold);
      Serial.println(cur);
      return;  // Stop movement if obstacle is found
    }
  }

  // Ensure final position is set
  myservo.write(targetPos);
}

float calibrateThreshold(float mean, float sd) {
  return mean + (kt * sd);
}

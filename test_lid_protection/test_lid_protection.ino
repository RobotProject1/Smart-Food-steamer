#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include "VEGA_MLX90614.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BUFFER_SIZE 10
float bufferUp[BUFFER_SIZE]={0};
float bufferDown[BUFFER_SIZE]={0};
int bufferIndexUp=0;
int bufferIndexDown=0;
float kl=1.3;

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

  updateCurrentBuffer(cur);  // Store new value in both buffers

  float dynamicThreshUp = calculateThreshold(bufferUp,bufferIndexUp);  // Adjust margin as needed
  float dynamicThreshDown = calculateThreshold(bufferDown,bufferIndexDown);  // Adjust margin

  if (stat2 == 1) {
    if (pos==seropen-10) {memset(bufferUp, 0, sizeof(bufferUp));}
    for ( ; pos < seropen-10; pos += 3) {
      //Serial.println("Lid is opening...");
      myservo.write(pos);
      int pos = myservo.read();
      delay(8);
      cur = readCurrent();
      updateCurrentBuffer(cur);
      Serial.print("Current: "); Serial.println(cur);
      Serial.print("ThreshUp: "); Serial.println(dynamicThreshUp);
      if (cur < dynamicThreshUp) {
        Serial.print("obstacle detect, lid closing :");
        Serial.println(cur);
        moveServo(serclose);
        stat2=0;
        return;
      }
    }
  } 
  else if (stat2 == 0) {
    if (pos==serclose) {memset(bufferDown, 0, sizeof(bufferDown));}
    for ( ; pos != serclose; pos = pos-1) {
      //Serial.println("Lid is closing...");
      myservo.write(pos);
      int pos = myservo.read();
      delay(20);
      cur = readCurrent();
      updateCurrentBuffer(cur);

      Serial.print("Current: "); Serial.println(cur);
      Serial.print("ThreshDown: "); Serial.println(dynamicThreshDown);

      if (cur<dynamicThreshDown) {
        Serial.print("obstacle detect, lid open");
        Serial.println(cur);
        moveServo(seropen);
        stat2=1;
        return;
      }
    }
  }
}

float calculateThreshold(float arr[], int count) {
  if (count < BUFFER_SIZE) {
    // // Use mean if buffer is not full yet
    // float sum = 0;
    // for (int i = 0; i < count; i++) {
    //   sum += arr[i];
    // }
    // return (sum / count) + 0.5;  // Adjust margin as needed
    return 100.0;
  } else {
    float min = 0;
    for (int i=1; i<6; i++){
      if (arr[i] < min){
      min = arr[i];
      }
    }
    // Use median after 10 readings
    return calculateMedian(arr) - 0.5;
    // return calculateMedian(arr) - kl*(calculateMedian(arr)-min);
    // return calculateMean(arr,10) - kl*calculateSD(arr,10,calculateMean(arr,10));
  }
}

float calculateMedian(float arr[]) {
  float temp[BUFFER_SIZE];
  memcpy(temp, arr, sizeof(temp));  // Copy array to avoid modifying original

  // Insertion Sort (since BUFFER_SIZE is small, this is efficient)
  for (int i = 1; i < BUFFER_SIZE; i++) {
    float key = temp[i];
    int j = i - 1;

    // Move elements that are greater than key
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j = j - 1;
    }
    temp[j + 1] = key;
  }

  // Return the median value
  if (BUFFER_SIZE % 2 == 0) {
    return (temp[BUFFER_SIZE / 2 - 1] + temp[BUFFER_SIZE / 2]) / 2.0;
  } else {
    return temp[BUFFER_SIZE / 2];
  }
}

void updateCurrentBuffer(float value) {
  bufferUp[bufferIndexUp] = value;
  bufferDown[bufferIndexDown] = value;

  bufferIndexUp = (bufferIndexUp + 1) % BUFFER_SIZE;  // Circular buffer
  bufferIndexDown = (bufferIndexDown + 1) % BUFFER_SIZE;
}
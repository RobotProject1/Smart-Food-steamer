#include <Servo.h>

// Object initialization
Servo myservo;

// Pin assignments
const int buzz = 3;  // Buzzer pin
const int touchpad2 = 1;

// Servo positions
const int seropen = 130;
const int serclose = 0;

// Stall detection threshold (adjust based on your servo specs)
const float threshold = 1.2;

// Moving average for current reading
const int numReadings = 10;
float readings[numReadings] = {0};  
int readIndex = 0;
float total = 0;
float maxCurrent = 0;
unsigned long lastPrintTime = 0;

// Touchpad state
int stat2 = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(6);
  pinMode(buzz, OUTPUT);
  pinMode(touchpad2, INPUT);

  Serial.println("Avg Current (A), Max Current (A)");

  // Initialize current readings array
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  checkTouchpad2();       // Check touchpad input
  updateServoState();     // Control servo based on state
}

// Function to read current from sensor
float readCurrent() {
  int adc = analogRead(A1);
  float voltage = adc * 5.0 / 1023.0;
  float current = (voltage - 2.5) / 0.185;  // Adjust 0.185 based on your sensor specs
  return current;
}

// Function to move servo with stall current detection
void moveServo(int targetPos) {
  int currentPos = myservo.read();
  float stallCurrentup = 0;  // Track max current during movement
  float stallCurrentdown = 0;

  Serial.println("Moving servo...");

  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      myservo.write(pos);
      delay(20);

      float cur = readCurrent();
      if (cur > stallCurrent) {
        stallCurrentup = cur;  // Capture peak stall current
      }
    }
  } 
  else if (currentPos > targetPos) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      myservo.write(pos);
      delay(20);

      float cur = readCurrent();
      if (cur > stallCurrent) {
        stallCurrentdown = cur;
      }
    }
  }

  // Log stall current
  Serial.print("Stall Current (A): ");
  Serial.println(stallCurrent, 3);
}

// Function to check touchpad input and toggle state
void checkTouchpad2() {
  if (digitalRead(touchpad2) == HIGH) {
    tone(buzz, 3000, 100);
    stat2 = !stat2;
    delay(100);
  }
}

// Function to control servo based on touchpad state
void updateServoState() {
  float cur = readCurrent();

  if (stat2 == 1) { // Lid is open
    if (cur < threshold && myservo.read() != seropen) {
      Serial.println("Closing lid...");
      moveServo(serclose);
      stat2 = 0;
    } else if (myservo.read() != seropen) {
      moveServo(seropen);
    }
  } else { // Lid is closed
    if (cur < threshold && myservo.read() != serclose) {
      Serial.println("Opening lid...");
      moveServo(seropen);
      stat2 = 1;
    } else if (myservo.read() != serclose) {
      moveServo(serclose);
    }
  }
}

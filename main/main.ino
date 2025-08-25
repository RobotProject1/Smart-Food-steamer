#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
//#include "VEGA_MLX90614.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include "Adafruit_LEDBackpack.h"
#include <OneWire.h>
#include <DS18B20.h>

// Serial communication
//#define SERIAL_BAUD 115200

// Relay and fan control
#define F_RELAY_PIN 7  // Fan relay (digital pin 7)
//#define H_RELAY_PIN 11  // Heater relay
int H_RELAY_PIN = 13;
#define HUMIDITY_THRESHOLD 95.0

// BME280 sensor (temperature, pressure, humidity)
BME280I2C bme;  // Default settings: forced mode, standby time = 1000 ms
                // Oversampling: pressure ×1, temperature ×1, humidity ×1, filter off

// RGB LED pins
#define RED_PIN 9
#define GREEN_PIN 10#define LED 2

void setup() {
  // Set pin mode
  pinMode(LED,OUTPUT);
}

void loop() {
  delay(50);
// you can set the delay time by adjusting the parameter of delay();
  digitalWrite(LED,HIGH);
  delay(50);
  digitalWrite(LED,LOW);
}
// #define BLUE_PIN

// Light control
#define LIGHT 12
int statL = 0;

// Servo setup
Servo myservo;
int seropen = 135;
int serclose = 7;

// Temperature sensor (infrared)
//VEGA_MLX90614 mlx(18, 19);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Displays
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
Adafruit_SSD1306 display(4);

// Touchpad and buzzer
const int touchpad1 = 5;
const int touchpad2 = 2;
const int touchpad3 = 3;
const int buzz = 11;

// Touchpad state variables
int valtp1 = 0, stat1 = 0;
int valtp2 = 0, stat2 = 0;
int valtp3 = 0, stat3 = 0;

// OLED
bool isIdle = false;
long lastPressTime = 0;

// Sensor sampling
const int SAMPLES = 10;
float s_val[SAMPLES];
float kt = 2;

// // LID PID variables and setup (commented out)
// double lSetpoint, lInput, lOutput;
// double lKp = 2, lKi = 5, lKd = 1;
// PID lidPID(&lInput, &lOutput, &lSetpoint, lKp, lKi, lKd, DIRECT);

// Temperature PID control
double Setpoint = 75, Input, Output;
double Kp = 15, Ki = 0, Kd = 0;
PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Environmental data
float temp, hum, pres;

// DS18B20 temperature sensor
DS18B20 ds(8);

// Threshold value (to be adjusted)
float threshold = 1.2;  // ไว้มาแก้


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

  // Initialize PID input
  pinMode(H_RELAY_PIN, OUTPUT);
  digitalWrite(H_RELAY_PIN, LOW);

  mlx.begin();
  Input = mlx.readObjectTempC();  // Read target temperature from MLX90614

  // Activate PID control
  tempPID.SetMode(AUTOMATIC);

  // Humidity and fan setup
  Wire.begin();
  pinMode(F_RELAY_PIN, OUTPUT);
  digitalWrite(F_RELAY_PIN, LOW);  // Fan relay (active high)
  bme.begin();

  // LED in box
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, LOW);

  // OLED display setup
  displaySetup();

  // 7-segment display setup
  alpha4.begin(0x70);
}

void loop() {
  //check
  // update system
  // main
  checkTouchpad1();  // Check touchpad and toggle state
  checkTouchpad2();  // Check touchpad and toggle state
  checkTouchpad3();  // Check touchpad and toggle state
  idleOLED();
  Ventilator_control();            // Control ventilator based on humidity
  updateServoStatenoProtection();  // Update servo position based on state
  updateSystem();                  // Update system for PID and Manual
  // statusUpdate();       // Check if food's ready
  updatesevensegdisplay();  // update 7segment display
  //printdata();
  // delay(300);            // Main loop delay
}

// void printdata() {
//   // IRobjecttemp,IRambienttemp,Probetemp,BMEtemp,BMEhumidity,BMEpressure,ServoAngle,CurrentA
//   Serial.print(mlx.readObjectTempC());Serial.print(",");
//   Serial.print(mlx.readAmbientTempC());Serial.print(",");
//   Serial.print(ds.getTempC());Serial.print(",");
//   Serial.print(temp);Serial.print(",");
//   Serial.print(hum);Serial.print(",");
//   Serial.print(pres);Serial.print(",");
//   Serial.println(myservo.read());//Serial.print(",");
//   //Serial.println(readCurrent());
// }

// void check() {
//   Serial.print("oled touchpad1 state : "); Serial.println(stat1);
//   Serial.print("lid touchpad2 state : "); Serial.println(stat2);
//   Serial.print("light touchpad1 state : "); Serial.println(stat1);
//   delay(2000);

//   Serial.print("lid degree : "); Serial.println(myservo.read());
//   Serial.print("temp from probe : "); Serial.println(ds.getTempC());
//   Serial.print("object temp from IR : "); Serial.println(mlx.readObjectTempC());
//   Serial.print("ambient temp from IR : "); Serial.println(mlx.readAmbientTempC());
//   BMEread(temp, hum, pres);
//   Serial.print("humidity from bme : ");Serial.println(hum);
//   //Serial.print("Current from current sensor : "); Serial.println(readCurrent());
//   delay(2000);

//   Serial.println("relay fan on");
//   digitalWrite(F_RELAY_PIN, HIGH);
//   delay(5000);
//   Serial.println("relay fan off");
//   digitalWrite(F_RELAY_PIN, LOW);
//   delay(5000);
//   Serial.println("relay light on");
//   digitalWrite(LIGHT, HIGH);
//   delay(5000);
//   Serial.println("relay light off");
//   digitalWrite(LIGHT, LOW);
//   delay(5000);
//   Serial.println("relay heater on");
//   digitalWrite(H_RELAY_PIN, HIGH);
//   delay(5000);
//   Serial.println("relay heater off");
//   digitalWrite(H_RELAY_PIN, LOW);
//   delay(5000);

//   Serial.println("Opening lid :");
//   moveServo(seropen);
//   delay(500);
//   Serial.println("Closing lid :");
//   moveServo(serclose);
//   delay(500);
//   Serial.println("7 SEG CHECK");
//   updatesevensegdisplay();
//   // delay(2000);
//   Serial.println("buzzer");
//   tone(buzz, 3000, 5000);
//   delay(5000);
// }

void moveServo(int targetPos) {
  int currentPos = myservo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      myservo.write(pos);
      delay(8);
    }
  } else if (currentPos > targetPos) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      myservo.write(pos);
      delay(20);
    }
  }
}

// float readCurrent() {
//   int adc = analogRead(A3);
//   float voltage = adc * 5.0 / 1023.0;
//   float current = (voltage - 2.5) / 0.185;
//   return current;
// }

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, 255 - red);  // Invert สีสำหรับ Common Anode
  analogWrite(GREEN_PIN, 255 - green);
  //analogWrite(BLUE_PIN, 255-blue);
}

// Function to update servo state based on stat
// void updateServoState() {
//   float cur = readCurrent();  // Read the current value

//   if (stat2 == 1) { // Lid is open
//     if (cur < threshold && myservo.read() != seropen) {
//       Serial.println("Closing lid due to obstacle...");
//       moveServo(serclose);
//       stat2 = 0;
//     }
//     else if (myservo.read() != seropen) {
//       moveServo(seropen);
//     }
//   }
//   else { // Lid is closed
//     if (cur < threshold && myservo.read() != serclose) {
//       Serial.println("Opening lid due to obstacle...");
//       moveServo(seropen);
//       stat2 = 1;
//     }
//     else if (myservo.read() != serclose) {
//       moveServo(serclose);
//     }
//   }
// }

void updateServoStatenoProtection() {
  //float cur = readCurrent();  // Read the current value

  if (stat2 == 0) {
    //Serial.println("Closing lid");
    moveServo(serclose);
  } else if (stat2 == 1) {
    //Serial.println("Opening lid");
    moveServo(seropen);
  }
}

// Function to handle temperature PID (placeholder)
void updateTempPID() {
  Input = ds.getTempC();
  if (Setpoint - Input >= 8) {
    pwm(255);
    return;
  }
  tempPID.Compute();
  pwm(Output);
}

void pwm(float sig) {
  sig = constrain(sig, 0, 255);  // Ensure PWM value is within range

  int onTime = map(sig, 0, 255, 0, 5000);  // Convert 0-255 to 0-10 ms
  int offTime = 5000 - onTime;             // Complementary OFF time

  digitalWrite(H_RELAY_PIN, HIGH);
  delay(onTime);  // Keep SSR ON for 'onTime'

  digitalWrite(H_RELAY_PIN, LOW);
  delay(offTime);  // Keep SSR OFF for 'offTime'
}

void idleOLED() {
  if (millis() - lastPressTime > 10000) {
    display.clearDisplay();
    display.setCursor(10, 3);
    display.setTextSize(3);
    display.print(ds.getTempC(), 1);
    display.print(" C");
    display.fillRect(88, 3, 5, 2, SSD1306_WHITE);
    display.fillRect(85, 5, 3, 3, SSD1306_WHITE);
    display.fillRect(93, 5, 3, 3, SSD1306_WHITE);
    display.fillRect(88, 8, 5, 2, SSD1306_WHITE);  // degree symbol (°)
    display.display();
    isIdle = true;
  }
}

void wakeUpOLED() {
  if (isIdle) {
    display.clearDisplay();  // Clear screen ONCE
    drawMode();              // Redraw mode section
    drawLightBulb();         // Redraw light bulb section
    display.display();       // Ensure buffer update
    isIdle = false;          // Reset idle status
  }
}

void drawMode() {
  display.fillRect(0, 0, 80, 40, SSD1306_BLACK);
  display.setCursor(5, 0);
  display.setTextSize(2);
  display.print("Mode:");
  display.setCursor(5, 17);
  display.print(stat1 == 0 ? "ON" : "AUTO");
  display.display();
}

void drawLightBulb() {
  display.setTextSize(2);
  display.fillRect(75, 0, 55, 80, SSD1306_BLACK);

  display.fillRect(99, 24, 12, 7, SSD1306_WHITE);
  display.fillRect(101, 25, 8, 5, SSD1306_BLACK);
  display.drawLine(101, 26, 109, 26, SSD1306_WHITE);
  display.drawLine(101, 28, 109, 28, SSD1306_WHITE);  // Base
  display.drawLine(98, 4, 111, 4, SSD1306_WHITE);     // Top

  display.drawLine(98, 24, 95, 19, SSD1306_WHITE);  // Left bottom1
  display.drawLine(95, 19, 90, 12, SSD1306_WHITE);  // Left bottom2
  display.drawLine(90, 12, 90, 9, SSD1306_WHITE);   // Left side
  display.drawLine(90, 9, 92, 6, SSD1306_WHITE);    // Left top1
  display.drawLine(92, 6, 97, 5, SSD1306_WHITE);    // Left top2

  display.drawLine(111, 24, 114, 19, SSD1306_WHITE);  // Right bottom1
  display.drawLine(114, 19, 119, 12, SSD1306_WHITE);  // Right bottom2
  display.drawLine(119, 12, 119, 9, SSD1306_WHITE);   // Right side
  display.drawLine(119, 9, 117, 6, SSD1306_WHITE);    // Right top1
  display.drawLine(117, 6, 112, 5, SSD1306_WHITE);    // Right top2

  if (stat3 == 1) {
    display.fillRect(101, 26, 8, 4, SSD1306_WHITE);
    display.fillRect(100, 6, 10, 2, SSD1306_WHITE);
    display.drawLine(98, 7, 112, 7, SSD1306_WHITE);
    display.fillCircle(104, 14, 7, SSD1306_WHITE);
    display.fillCircle(105, 14, 7, SSD1306_WHITE);
    display.fillRect(95, 8, 20, 1, SSD1306_WHITE);
    display.fillRect(94, 9, 22, 3, SSD1306_WHITE);
    display.fillRect(95, 12, 20, 2, SSD1306_WHITE);
    display.drawLine(96, 14, 113, 14, SSD1306_WHITE);
    display.drawLine(101, 21, 109, 21, SSD1306_WHITE);
    display.drawLine(101, 22, 108, 22, SSD1306_WHITE);  // Fill

    display.drawLine(92, 1, 95, 2, SSD1306_WHITE);
    display.drawLine(117, 1, 114, 2, SSD1306_WHITE);
    display.drawLine(82, 11, 86, 11, SSD1306_WHITE);
    display.drawLine(123, 11, 127, 11, SSD1306_WHITE);
    display.drawLine(88, 21, 91, 20, SSD1306_WHITE);
    display.drawLine(121, 21, 118, 20, SSD1306_WHITE);  // Sparkles
  }
  display.display();
}

void checkTouchpad1() {
  valtp1 = digitalRead(touchpad1);
  if (valtp1 == 1) {
    wakeUpOLED();
    tone(buzz, 3000, 100);
    stat1 = !stat1;
    drawMode();
    display.display();
    lastPressTime = millis();
    delay(100);
  }
}

void checkTouchpad2() {
  valtp2 = digitalRead(touchpad2);
  if (valtp2 == 1) {
    tone(buzz, 3000, 100);
    stat2 = !stat2;
    delay(100);
  }
}

void checkTouchpad3() {
  valtp3 = digitalRead(touchpad3);
  if (valtp3 == 1) {
    wakeUpOLED();
    tone(buzz, 3000, 100);
    stat3 = !stat3;
    statL = !statL;
    drawLightBulb();
    display.display();
    lastPressTime = millis();
    digitalWrite(LIGHT, statL == 1 ? HIGH : LOW);
    delay(100);
  }
}

void displaySetup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  drawMode();
  drawLightBulb();
  // Add OLED display code here (e.g., using SSD1306 library)
  // Example: display temperature, servo position, or status
}

void updateSystem() {
  if (stat1 == 1) {
    tempPID.SetMode(AUTOMATIC);  // PID on
    updateTempPID();
  } else {
    tempPID.SetMode(MANUAL);  // PID off
    digitalWrite(H_RELAY_PIN, HIGH);
  }
}

void statusUpdate() {
  if (Input == Setpoint) {
    setColor(255, 0, 0);
  } else {
    setColor(0, 0, 0);
  }
}

void updatesevensegdisplay() {
  float tempP = mlx.readObjectTempC();
  char buffer[5];
  dtostrf(tempP, 4, 1, buffer);  // Convert float to string with 1 decimal place
  for (int i = 0; i < 4; i++) {
    alpha4.writeDigitAscii(i, buffer[i]);
  }
  alpha4.writeDisplay();  // Update display
}

void Ventilator_control() {
  BMEread(temp, hum, pres);
  if (ds.getTempC() > 55) {
    digitalWrite(F_RELAY_PIN, HIGH);  // Turn ON ventilator
    // Serial.println("Ventilator ON");
  } else {
    digitalWrite(F_RELAY_PIN, LOW);  // Turn OFF ventilator
    // Serial.println("Ventilator OFF");
  }
}

void BMEread(float &temp, float &hum, float &pres) {
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
}

// void printBME280Data(float temp, float hum, float pres) {
//   BMEread(temp, hum, pres);
//   Serial.print("Temp: ");
//   Serial.print(temp);
//   Serial.print(" °C\t");

//   Serial.print("Humidity: ");
//   Serial.print(hum);
//   Serial.print("% RH\t");

//   Serial.print("Pressure: ");
//   Serial.print(pres);
//   Serial.println(" Pa");
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

// float calibratethreshold(float mean, float sd) {
//   float threshold = mean + (kt * sd);
//   return threshold;
// }
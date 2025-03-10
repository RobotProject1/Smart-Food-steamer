int touchpad1 = 2;
// int touchpad2 = 4;
int valtp1 = 0;
// int valtp2 = 0;
int stat = 0;
const int buzz = 9;
#include <Servo.h>
Servo myservo;
int seropen = 60;
int serclose = 0;

void setup() {
  pinMode (touchpad1, INPUT);
  pinMode(buzz, OUTPUT); 
  myservo.attach(4);
  // pinMode (touchpad2,INPUT);
  Serial.begin(9600);
  myservo.write(serclose);
}

void loop() {
  valtp1 = digitalRead(touchpad1);

  if (valtp1 == 1) {
    tone(buzz,3000,100);
    stat = !stat; 
    delay(100);   
  }

  if (stat == 1) {
    myservo.write(seropen);
  }
  else {
    myservo.write(serclose);
  }

  Serial.print("Status: ");
  Serial.print("status -> ");
  Serial.println(stat);
delay(300);

}
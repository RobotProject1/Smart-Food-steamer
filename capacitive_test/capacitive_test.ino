int touchpad1 = 2;
// int touchpad2 = 4;
int valtp1 = 0;
// int valtp2 = 0;
int stat = 0;

void setup() {
  pinMode (touchpad1, INPUT);
  // pinMode (touchpad2,INPUT);
  Serial.begin(9600);
}

void loop() {
  valtp1 = digitalRead(touchpad1);
  // valtp2 = digitalRead(touchpad2);
  //Serial.println(valtp);

  // Serial.print("Status: ");
  // Serial.print("1 -> ");
  // Serial.println(valtp1 == 1 ? "ON" : "OFF");
  // Serial.print(" | 2 -> ");
  // Serial.println(valtp2 == 1 ? "ON" : "OFF");
  
  if (valtp1 == 1) {
    stat = !stat; // Toggle status
    delay(200);   // Simple debounce delay to prevent multiple toggles
  }

  Serial.print("Status: ");
  Serial.print("status -> ");
  Serial.println(stat);
delay(300);

}
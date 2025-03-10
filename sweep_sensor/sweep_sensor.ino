int wave = A1;
int wavetp;

void setup() {
  pinMode (wave, INPUT);
  Serial.begin(9600);
}

void loop() {
  wavetp = digitalRead(wave);
  //Serial.println(valtp);
  Serial.println(wavetp);
  delay(300);

}
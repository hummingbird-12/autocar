#define PIN3 3

void setup() {
  pinMode(PIN3, INPUT);
  Serial.begin(115200);
}


void loop() {
  unsigned long duration;
  float duration_msec;
  duration = pulseIn(PIN3, HIGH);
  duration_msec = duration / 1000.0f;

  Serial.print("Pulse width = ");
  Serial.print(duration_msec);
  Serial.println("msec");
  delay(500);
}


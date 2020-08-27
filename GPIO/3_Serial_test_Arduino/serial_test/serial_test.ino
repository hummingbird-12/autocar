void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char a;

    a = Serial.read();
    if (a == 'A') {
      Serial.print("test : A");
    }
    else {
      Serial.print("test : not A");
    }
  }
}





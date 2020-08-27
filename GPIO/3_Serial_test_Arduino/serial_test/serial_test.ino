void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char a;

    a = Serial.read(); // Get output from serial (written by Raspberry Pi)
    
    // Write to serial (read by Raspberry Pi)
    if (a == 'A') {
      Serial.print("test : A");
    }
    else {
      Serial.print("test : not A");
    }
  }
}

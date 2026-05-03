void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    
    int ex, ey;
    sscanf(msg.c_str(), "%d,%d", &ex, &ey);
    Serial.printf("Error x-axis: %d, Error y-axis: %d\n", ex, ey); // temp
  }
}

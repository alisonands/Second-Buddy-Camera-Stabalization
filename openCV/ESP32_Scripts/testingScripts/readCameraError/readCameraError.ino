void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    
    int ex, ey, mode;
    sscanf(msg.c_str(), "%d,%d,%d", &ex, &ey, &mode);
    // Serial.printf("Error x-axis: %d, Error y-axis: %d\n | Mode is: %d", ex, ey, mode); // debug
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    
    float ex, ey;
    sscanf(msg.c_str(), "%f,%f", &ex, &ey);
    Serial.printf("Error x-axis: %.2f, Error y-axis: %.2f\n", ex, ey); // temp
  }
}

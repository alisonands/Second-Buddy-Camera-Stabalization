const int pinXaxis = 34;
const int pinYaxis = 35;
const int pinButtn = 32;

// centering constants
int xCenter = 2048;
int yCenter = 2048;

void setup() {
  Serial.begin(115200); 

  // ADC pins are input by default, but explicit doesn't hurt 
  pinMode(pinXaxis, INPUT);
  pinMode(pinYaxis, INPUT);

  pinMode(pinButtn, INPUT_PULLUP);

  // quick calibration
  long xSum = 0, ySum = 0;

  for (int i = 0; i < 100; i++) {
    xSum += analogRead(pinXaxis);
    ySum += analogRead(pinYaxis);
    delay(5);
  }

  xCenter = xSum / 100;
  yCenter = ySum / 100;
}

void loop() {
  int xVal = analogRead(pinXaxis) - xCenter;   // 0–4095
  int yVal = analogRead(pinYaxis) - yCenter;   // 0–4095
  int btnVal = digitalRead(pinButtn); // HIGH or LOW

  // =========================
  // DEAD ZONE (±300)
  // =========================
  if (abs(xVal) < 50) xVal = 0;
  if (abs(yVal) < 50) yVal = 0;

  Serial.print("X: ");
  Serial.print(xVal);

  Serial.print(" | Y: ");
  Serial.print(yVal);

  Serial.print(" | Button: ");
  Serial.println(btnVal == HIGH ? "Released" : "Pressed");

  delay(100);
}

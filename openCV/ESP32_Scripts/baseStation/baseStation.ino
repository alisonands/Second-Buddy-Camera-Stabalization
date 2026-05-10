// baseStation.ino

#include <WiFi.h>
#include <esp_now.h>

// receiver (blue) MAC address: 4C:11:AE:7E:DC:84
// esp32-cam MAC address: 1C:C3:AB:D2:3A:04
uint8_t receiverMACAddress[] = {0x4C, 0x11, 0xAE, 0x7E, 0xDC, 0x84};

struct ControlPacket {
  int16_t x;
  int16_t y;
  uint8_t stab;
  uint8_t mode;
};
ControlPacket pkt;

// define pins
const int pinXaxis = 34;
const int pinYaxis = 35;
const int pinButtn = 32;
const int pinLED = 4;

// joystick centering constants
int xCenter;
int yCenter;

// openCV commands
int ex = 0;
int ey = 0;
int mode = 0;

// ================================================
//                      SETUP
// ================================================
void setup() {
  Serial.begin(115200);
  // initialize pins
  pinMode(pinLED, INPUT_PULLUP);
  
  // initialize esp-now protocol
  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW protocol");
    return;
  }
  else
  {
    Serial.println("Successfully initialized ESP-NOW protocol");
  }

  // pair with other ESP32
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Successfully added peer");
  }

  // Joystick Initialization

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

// ================================================
//                     MAIN LOOP
// ================================================
void loop() {
  // read LED button
  int LED = !digitalRead(pinLED);

  // read computer inputs
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    sscanf(msg.c_str(), "%d,%d,%d", &ex, &ey, &mode);
    // Serial.printf("Error x-axis: %d, Error y-axis: %d\n | Mode is: %d", ex, ey, mode); // debug
  }
  pkt.mode = mode;
  
  if (mode == 0){
    // joystick control loop
    pkt = getControl(pinXaxis, pinYaxis, xCenter, yCenter);
  }
  else {
    // openCV control loop
    pkt.x = ex;
    pkt.y = ey;
  }

  // send data
  esp_now_send(receiverMACAddress, (uint8_t *) &pkt, sizeof(pkt));

  if (LED == 1) {
    Serial.println(LED);
  }

  delay(30);
}


// ================================================
//               SUPPORTING FUNCTIONS
// ================================================
ControlPacket getControl(int pinX, int pinY, int xCenter, int yCenter) {
  ControlPacket pkt;

  int x = analogRead(pinX) - xCenter;
  int y = analogRead(pinY) - yCenter;

  // =========================
  // DEAD ZONE (±100)
  // =========================
  if (abs(x) < 750) x = 0; else x = constrain(x,-1,1);
  if (abs(y) < 750) y = 0; else y = constrain(y,-1,1);

  pkt.x = x;
  pkt.y = y;

  int stab = digitalRead(pinButtn);
  if (stab == HIGH) pkt.stab = 0; else pkt.stab = 1;
  
  return pkt;
}

// esp32-cam has madr 1C:C3:AB:D2:3A:04
// green esp has madr 80:7D:3A:8E:F0:44
//  blue esp has madr 4C:11:AE:7E:DC:84
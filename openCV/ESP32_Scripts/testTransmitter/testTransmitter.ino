// testTransmitter.ino

#include <WiFi.h>
#include <esp_now.h>

// receiver (blue) MAC address: 4C:11:AE:7E:DC:84
// esp32-cam MAC address: 1C:C3:AB:D2:3A:04
uint8_t receiverMACAddress[] = {0x1C, 0xC3, 0xAB, 0xD2, 0x3A, 0x04};

// define data packets
struct PacketData
{
  byte switch1Value;
  byte switch2Value; 
};
PacketData data;

struct ControlPacket {
  int16_t x;
  int16_t y;
  uint8_t btn;
};
ControlPacket pkt;

// define pins
const int pinXaxis = 34;
const int pinYaxis = 35;
const int pinButtn = 32;

// joystick centering constants
int xCenter;
int yCenter;

/*
void onDataSend(const wifi_tx_info_t * mac, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}
*/


// ================================================
//                      SETUP
// ================================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  
  // initialize esp-now protocol
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

  // set callback function when sending data
  // esp_now_register_send_cb(onDataSend);

  // =========================
  // Joystick Initialization
  // =========================
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
  // read joystick
  pkt = getControl(pinXaxis, pinYaxis, xCenter, yCenter);

  // send data
  esp_now_send(receiverMACAddress, (uint8_t *) &pkt, sizeof(pkt));
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

  int btn = digitalRead(pinButtn);
  if (btn == HIGH) pkt.btn = 0; else pkt.btn = 1;
  
  return pkt;
}

// esp32-cam has madr 1C:C3:AB:D2:3A:04
// green esp has madr 80:7D:3A:8E:F0:44
//  blue esp has madr 4C:11:AE:7E:DC:84
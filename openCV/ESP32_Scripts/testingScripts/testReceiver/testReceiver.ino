// testReceiver.ino

#include <WiFi.h>
#include <esp_now.h>

// define data packet
struct ControlPacket {
  int16_t x;
  int16_t y;
  uint8_t btn;
};
ControlPacket pkt;

// define LED pins
const int uLED = 12;
const int dLED = 13;
const int lLED = 15;
const int rLED = 14;

void onDataRecv(const esp_now_recv_info * info, const uint8_t * incomingData, int len)
{
  memcpy(&pkt, incomingData, sizeof(pkt));
  signalPacketData();
}

void signalPacketData()
{
  // x-axis
  if (pkt.x > 0) {
    digitalWrite(uLED, 1);
  }
  else if (pkt. x < 0) {
    digitalWrite(dLED, 1);
  }
  else{
    digitalWrite(uLED, 0);
    digitalWrite(dLED, 0);
  }

  // y-axis
  if (pkt.y > 0) {
    digitalWrite(rLED, 1);
  }
  else if (pkt. y < 0) {
    digitalWrite(lLED, 1);
  }
  else{
    digitalWrite(rLED, 0);
    digitalWrite(lLED, 0);
  }

  // button
  if (pkt.btn == 1) {
    digitalWrite(uLED, 1);
    digitalWrite(dLED, 1);
    digitalWrite(lLED, 1);
    digitalWrite(rLED, 1);
  }
  else {
    digitalWrite(uLED, 0);
    digitalWrite(dLED, 0);
    digitalWrite(lLED, 0);
    digitalWrite(rLED, 0);
  }
}

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

  // set callback function when receiving data
  esp_now_register_recv_cb(onDataRecv);

  pinMode(uLED, OUTPUT);
  pinMode(dLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  pinMode(lLED, OUTPUT);
}

void loop() {
  signalPacketData();
}



// green esp has madr 80:7D:3A:8E:F0:44
//  blue esp has madr 4C:11:AE:7E:DC:84
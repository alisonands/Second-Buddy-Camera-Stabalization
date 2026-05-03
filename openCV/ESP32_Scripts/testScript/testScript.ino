#include "WiFi.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(WiFi.macAddress());
}



// green esp has madr 80:7D:3A:8E:F0:44
//  blue esp has madr 4C:11:AE:7E:DC:84
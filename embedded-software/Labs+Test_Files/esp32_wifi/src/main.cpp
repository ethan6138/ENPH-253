
#include <Arduino.h>

#include <WiFi.h>

const char *AP_SSID = "ESP32_Dash";
const char *AP_PASS = "12345678";

void setup() { Serial.begin(115200); 

    delay(1000); 

    WiFi.mode(WIFI_AP); 

    bool works {WiFi.softAP(AP_SSID, AP_PASS)}; 

    Serial.println(works ? "AP started" : "AP failed");
    Serial.println(WiFi.localIP()); 



}   


void loop() {}
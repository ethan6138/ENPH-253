#include <Arduino.h>
#include <ArduinoJson.h>

// -------------------- K230 --------------------------------
constexpr int PIN_UART_K230_RX{5};
constexpr int PIN_UART_K230_TX{6};
constexpr uint32_t k230BaudRate{9600};

HardwareSerial K230Serial(1);

constexpr int rxBufferSize{32768};
char rxBuffer[rxBufferSize];
int rxIndex{0};

struct Detection {
  String label{};
  int classId{};
  int x1{}, y1{}, x2{}, y2{};
  float score{};
};

constexpr int maxDetections{8};
Detection latestDetections[maxDetections];
int latestDetectionCount{0};

void parsePacket(const char *json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("JSON PARSE FAILED: ");
    Serial.println(error.c_str());
    return;
  }

  // K230 sends a bare array, e.g. [{"label":"Rock",...}, {...}]
  // NOT wrapped in a "detections" key.
  JsonArray detections = doc.as<JsonArray>();
  if (detections.isNull()) {
    Serial.println("Payload is not a JSON array");
    return;
  }

  latestDetectionCount = 0;
  for (JsonObject det : detections) {
    if (latestDetectionCount >= maxDetections)
      break;

    Detection &d = latestDetections[latestDetectionCount];
    d.label = det["label"].as<String>();
    d.classId = det["class_id"].as<int>();
    d.x1 = det["x1"].as<int>();
    d.y1 = det["y1"].as<int>();
    d.x2 = det["x2"].as<int>();
    d.y2 = det["y2"].as<int>();
    d.score = det["score"].as<float>();
    latestDetectionCount++;
  }

  // Empty frame (no detections) is valid — just nothing to print.
  if (latestDetectionCount == 0) {
    return;
  }

  // Debug print, confirm parsing worked
  for (int i = 0; i < latestDetectionCount; i++) {
    Serial.print("Detection ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(latestDetections[i].label);
    Serial.print(" score=");
    Serial.print(latestDetections[i].score);
    Serial.print(" box=(");
    Serial.print(latestDetections[i].x1);
    Serial.print(",");
    Serial.print(latestDetections[i].y1);
    Serial.print(")-(");
    Serial.print(latestDetections[i].x2);
    Serial.print(",");
    Serial.print(latestDetections[i].y2);
    Serial.println(")");
  }
}

void setup() {
  Serial.begin(115200);
  K230Serial.begin(k230BaudRate, SERIAL_8N1, PIN_UART_K230_RX,
                   PIN_UART_K230_TX);
  delay(2500);
  Serial.println("K230 UART JSON Test good to go");
}

void loop() {
  while (K230Serial.available()) {
    char incomingByte = K230Serial.read();
    Serial.write(incomingByte);
    if (incomingByte == '\n') {
      rxBuffer[rxIndex] = '\0';
      parsePacket(rxBuffer);
      rxIndex = 0;
    } else if (rxIndex < rxBufferSize - 1) {
      rxBuffer[rxIndex++] = incomingByte;
    } else {
      rxIndex = 0;
    }
  }
}
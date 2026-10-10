#include "K230.h"

#include <error_handler.h>
#include <pins.h>

#include <Arduino.h>
#include <ArduinoJson.h>

namespace {
constexpr uint32_t k230BaudRate{9600};
HardwareSerial K230Serial(1);

constexpr int rxBufferSize{32768};
char rxBuffer[rxBufferSize];
int rxIndex{0};

Detection latestDetections[maxDetections];
int latestDetectionCount{0};
SemaphoreHandle_t detectionMutex;

// teletubby watch state
bool teletubbyWatchActive{false};
uint32_t teletubbyFramesTotal{0};
uint32_t teletubbyFramesWithHit{0};
float teletubbyScoreSum{0.0f};

// Margins used to decide whether a detected teletubby is positioned behind
// the rock in the same frame, expressed as a fraction of the rock's own
// bounding box size so the margin scales with distance during alignment.
float teletubbyXMarginFraction{0.05f};
float teletubbyYMarginFraction{0.05f};

// Gating for the case where the robot is driving over the rock and the rock
// itself may no longer be detected, but the teletubby - now close and
// directly ahead - should dominate the frame. "Foreground" is judged by
// bounding box area, "middle" by how close the box's center is to
// teletubbyMiddleXTargetPx. TODO: tune all three against real footage of the
// robot driving over a rock.
float teletubbyForegroundMinAreaPx{5500.0f};
int teletubbyMiddleXTargetPx{400};
int teletubbyMiddleXBandPx{150};

// Finds the largest-area detection matching `label` among dets[0..count).
// Mirrors the "largest area wins" heuristic K230Align uses to pick its
// alignment target, so this reflects the rock the robot is currently
// aligning to.
bool findLargestDetection(const Detection *dets, int count, const char *label,
                           Detection &best) {
  bool found = false;
  long bestArea = -1;

  for (int i = 0; i < count; i++) {
    if (!dets[i].label.equals(label))
      continue;

    long width = dets[i].x2 - dets[i].x1;
    long height = dets[i].y2 - dets[i].y1;
    long area = width * height;

    if (area > bestArea) {
      bestArea = area;
      best = dets[i];
      found = true;
    }
  }

  return found;
}

// A teletubby is only considered "behind" the rock if it horizontally
// overlaps the rock's footprint (padded by a margin) and doesn't extend
// below the rock's own bottom edge (padded by a margin). This assumes a
// forward-facing, downward-tilted camera, where a farther-away object
// projects higher in the frame than a nearer one at the same lateral
// position - consistent with how K230Align already infers distance from the
// rock's bottom edge moving down-frame as the robot approaches.
bool isTeletubbyBehindRock(const Detection &teletubby, const Detection &rock) {
  int rockWidth = rock.x2 - rock.x1;
  int rockHeight = rock.y2 - rock.y1;
  int xMargin = static_cast<int>(teletubbyXMarginFraction * rockWidth);
  int yMargin = static_cast<int>(teletubbyYMarginFraction * rockHeight);

  // Serial.print("xMargin: ");
  // Serial.println(xMargin);
  // Serial.print("yMargin: ");
  // Serial.println(yMargin);
  bool xOverlap = teletubby.x1 <= rock.x2 + xMargin &&
                  teletubby.x2 >= rock.x1 - xMargin;
  if (!xOverlap)
    return false;

  if (teletubby.y2 > rock.y2 + yMargin)
    return false;

  return true;
}

// A teletubby is "foreground and centered" if its box is large enough (close
// to the camera) and roughly centered on X - how it should look once the
// robot is driving over the rock, even if the rock itself has dropped out of
// frame by then.
bool isTeletubbyForegroundAndCentered(const Detection &teletubby) {
  long width = teletubby.x2 - teletubby.x1;
  long height = teletubby.y2 - teletubby.y1;
  long area = width * height;
  // Serial.print("area: ");
  // Serial.println(area);
  if (area < teletubbyForegroundMinAreaPx)
    return false;

  int centerX = (teletubby.x1 + teletubby.x2) / 2;
  int dx = centerX - teletubbyMiddleXTargetPx;
  if (dx < 0)
    dx = -dx;
  if (dx > teletubbyMiddleXBandPx)
    return false;

  return true;
}

void parsePacket(const char *json) {
  // Serial.print("K230 received JSON: ");
  // Serial.println(json);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print("K230 JSON parse error: ");
    Serial.println(error.c_str());
    error_handler();
    Serial.print("deserialization error");
    return;
  }

  JsonArray detections = doc.as<JsonArray>(); // K230 sends a bare array
  if (detections.isNull()) {
    // error_handler();
    // Serial.print("");
    return;
  }

  Detection parsed[maxDetections];
  int count = 0;

  for (JsonObject det : detections) {
    if (count >= maxDetections)
      break;
    parsed[count].label = det["label"].as<String>();
    parsed[count].classId = det["class_id"].as<int>();
    parsed[count].x1 = det["x1"].as<int>();
    parsed[count].y1 = det["y1"].as<int>();
    parsed[count].x2 = det["x2"].as<int>();
    parsed[count].y2 = det["y2"].as<int>();
    parsed[count].score = det["score"].as<float>();
    count++;
  }

  if (xSemaphoreTake(detectionMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < count; i++)
      latestDetections[i] = parsed[i];
    latestDetectionCount = count;

    if (teletubbyWatchActive) {
      teletubbyFramesTotal++;

      // A rock may or may not be visible depending on where in the
      // approach/drive-over this frame lands, so this is allowed to fail -
      // isTeletubbyBehindRock() below is simply skipped when it does.
      Detection rock{};
      bool rockFound = findLargestDetection(parsed, count, "Rock", rock);

      float bestScore = 0.0f;
      bool hit = false;
      bool hitBehindRock = false;
      bool hitForeground = false;
      for (int i = 0; i < count; i++) {
        if (!parsed[i].label.equals("Teletubby"))
          continue;

        bool behindRock = rockFound && isTeletubbyBehindRock(parsed[i], rock);
        bool foregroundCentered = isTeletubbyForegroundAndCentered(parsed[i]);
        if (!behindRock && !foregroundCentered)
          continue;

        if (parsed[i].score > bestScore) {
          bestScore = parsed[i].score;
          hit = true;
          hitBehindRock = behindRock;
          hitForeground = foregroundCentered;
        }
      }

      if (hit) {
        teletubbyFramesWithHit++;
        teletubbyScoreSum += bestScore;

        // TURN THIS ON WHEN TUNING
        // Serial.printf("K230: teletubby hit (score %.2f) [behindRock=%d foregroundCentered=%d]\n",
        //               bestScore, hitBehindRock, hitForeground);
      }
    }

    xSemaphoreGive(detectionMutex);
  }
}

void k230Task(void *pvParameters) {
  for (;;) {
    while (K230Serial.available()) {
      char incomingByte = K230Serial.read();
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
} // namespace

void k230_init(void) {
  K230Serial.begin(k230BaudRate, SERIAL_8N1, PIN_UART_K230_RX,
                   PIN_UART_K230_TX);
  detectionMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(k230Task, "k230Task", 4096, nullptr, 1, nullptr, 1);
}

int getDetectionCount() {
  int result{0};
  if (xSemaphoreTake(detectionMutex, portMAX_DELAY) == pdTRUE) {
    result = latestDetectionCount;
    xSemaphoreGive(detectionMutex);
  }

  return result;
}

Detection getDetection(int index) {
  Detection result{};

  if (index < 0 || index >= maxDetections)
    return result;

  if (xSemaphoreTake(detectionMutex, portMAX_DELAY) == pdTRUE) {
    result = latestDetections[index];
    xSemaphoreGive(detectionMutex);
  }

  return result;
}

void k230_setPhase(int phase) {
  JsonDocument doc;
  doc["cmd"] = "set_phase";
  doc["phase"] = phase;

  String output;
  serializeJson(doc, output);
  output += '\n';

  K230Serial.print(output);

  // Serial.print("K230 file sent command: ");
  // Serial.print(output);
}

void k230_blinkLight() {
  JsonDocument doc;
  doc["cmd"] = "bl";

  String output;
  serializeJson(doc, output);
  output += '\n';

  K230Serial.print(output);

  // Serial.print("K230 file sent command: ");
  // Serial.print(output);
}

void k230_startTeletubbyWatch() {
  if (xSemaphoreTake(detectionMutex, portMAX_DELAY) == pdTRUE) {
    teletubbyFramesTotal = 0;
    teletubbyFramesWithHit = 0;
    teletubbyScoreSum = 0.0f;
    teletubbyWatchActive = true;
    xSemaphoreGive(detectionMutex);
  }

  // Serial.println("K230: started watching for teletubbies");
}

float k230_stopTeletubbyWatch() {
  uint32_t framesTotal{0};
  uint32_t framesWithHit{0};
  float scoreSum{0.0f};

  if (xSemaphoreTake(detectionMutex, portMAX_DELAY) == pdTRUE) {
    teletubbyWatchActive = false;
    framesTotal = teletubbyFramesTotal;
    framesWithHit = teletubbyFramesWithHit;
    scoreSum = teletubbyScoreSum;
    xSemaphoreGive(detectionMutex);
  }

  if (framesTotal == 0) {
    // Serial.println("teletubby watch stopped, no frames were received");
    return 0.0f;
  }

  // confidence score is (percentage of sightings) * (sighting confidence) added
  // up over sightings
  float hitRate =
      static_cast<float>(framesWithHit) / static_cast<float>(framesTotal);
  float avgHitScore = (framesWithHit > 0) ? (scoreSum / framesWithHit) : 0.0f;
  float confidence = hitRate * avgHitScore * 100.0f;
  if (confidence > 100.0f)
    confidence = 100.0f;

  // Serial.printf("K230: teletubby watch stopped - %u/%u frames had a hit, "
  //               "confidence %.1f%%\n",
  //               framesWithHit, framesTotal, confidence);

  return confidence;
}

void k230_setTeletubbyXMarginFraction(float frac) {
  teletubbyXMarginFraction = frac;
}

void k230_setTeletubbyYMarginFraction(float frac) {
  teletubbyYMarginFraction = frac;
}

void k230_setTeletubbyForegroundMinAreaPx(float areaPx) {
  teletubbyForegroundMinAreaPx = areaPx;
}

void k230_setTeletubbyMiddleXTargetPx(int px) { teletubbyMiddleXTargetPx = px; }

void k230_setTeletubbyMiddleXBandPx(int px) { teletubbyMiddleXBandPx = px; }

#include <WiFi.h>
#include <WebServer.h>

namespace Config {
  const char* WIFI_SSID = "Gogosaru";
  const char* WIFI_PASSWORD = "!ileana!";
  constexpr uint32_t USB_SERIAL_BAUD = 115200;
  constexpr uint32_t MEGA_SERIAL_BAUD = 38400;
  constexpr int RX2_PIN = 16;
  constexpr int TX2_PIN = 17;
  constexpr uint16_t HTTP_PORT = 80;
  constexpr uint32_t WIFI_RETRY_MS = 10000;
  constexpr uint32_t MEGA_STALE_MS = 6000;
}

WebServer server(Config::HTTP_PORT);

struct RoverData {
  uint32_t sequence = 0;
  float temperature = 0;
  float humidity = 0;
  int mq135 = 0;
  int mq7 = 0;
  int mq8 = 0;
  float dust = 0;
  float pressure = 0;
  String mode = "unknown";
  String state = "unknown";
  uint32_t receivedAt = 0;
  bool valid = false;
} rover;

String megaLine;
uint32_t lastWifiAttempt = 0;
bool wifiWasConnected = false;

void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store");
}

void sendJson(int code, const String &json) {
  addCorsHeaders();
  server.send(code, "application/json; charset=utf-8", json);
}

void handleOptions() {
  addCorsHeaders();
  server.send(204, "text/plain", "");
}

String jsonEscape(const String &input) {
  String output;
  output.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if (c == '"' || c == '\\') output += '\\';
    if (c >= 32) output += c;
  }
  return output;
}

bool megaOnline() {
  return rover.valid && (uint32_t)(millis() - rover.receivedAt) <= Config::MEGA_STALE_MS;
}

void handleData() {
  String json;
  json.reserve(320);
  json += F("{\"online\":");
  json += megaOnline() ? F("true") : F("false");
  json += F(",\"seq\":"); json += rover.sequence;
  json += F(",\"temp\":"); json += String(rover.temperature, 1);
  json += F(",\"hum\":"); json += String(rover.humidity, 1);
  json += F(",\"mq135\":"); json += rover.mq135;
  json += F(",\"mq7\":"); json += rover.mq7;
  json += F(",\"mq8\":"); json += rover.mq8;
  json += F(",\"dust\":"); json += String(rover.dust, 1);
  json += F(",\"pressure\":"); json += String(rover.pressure, 1);
  json += F(",\"mode\":\""); json += jsonEscape(rover.mode); json += '"';
  json += F(",\"state\":\""); json += jsonEscape(rover.state); json += '"';
  json += F(",\"ageMs\":"); json += rover.valid ? millis() - rover.receivedAt : 0;
  json += '}';
  sendJson(200, json);
}

bool sendCommandToMega(String command) {
  command.toLowerCase();
  String frame;
  if (command == "auto") {
    frame = F("MODE,AUTO");
    rover.mode = "auto";
  } else if (command == "manual") {
    frame = F("MODE,MANUAL");
    rover.mode = "manual";
  } else if (command == "forward") {
    frame = F("MOVE,FWD");
  } else if (command == "backward") {
    frame = F("MOVE,BACK");
  } else if (command == "left") {
    frame = F("MOVE,LEFT");
  } else if (command == "right") {
    frame = F("MOVE,RIGHT");
  } else if (command == "stop") {
    frame = F("MOVE,STOP");
  } else {
    return false;
  }
  Serial2.println(frame);
  return true;
}

void handleControl() {
  if (!server.hasArg("cmd")) {
    sendJson(400, F("{\"ok\":false,\"error\":\"missing cmd\"}"));
    return;
  }
  const String command = server.arg("cmd");
  if (!sendCommandToMega(command)) {
    sendJson(400, F("{\"ok\":false,\"error\":\"invalid command\"}"));
    return;
  }
  sendJson(200, String(F("{\"ok\":true,\"cmd\":\"")) + jsonEscape(command) + F("\"}"));
}

String fieldAt(const String &line, uint8_t wanted) {
  int start = 0;
  uint8_t field = 0;
  for (int i = 0; i <= (int)line.length(); ++i) {
    if (i == (int)line.length() || line[i] == ',') {
      if (field == wanted) return line.substring(start, i);
      start = i + 1;
      ++field;
    }
  }
  return "";
}

void parseMegaData(const String &line) {
  if (!line.startsWith("DATA,")) return;
  // DATA,seq,temp,hum,mq135,mq7,mq8,dust,pressure,mode,state
  if (fieldAt(line, 10).length() == 0) return;
  rover.sequence = fieldAt(line, 1).toInt();
  rover.temperature = fieldAt(line, 2).toFloat();
  rover.humidity = fieldAt(line, 3).toFloat();
  rover.mq135 = fieldAt(line, 4).toInt();
  rover.mq7 = fieldAt(line, 5).toInt();
  rover.mq8 = fieldAt(line, 6).toInt();
  rover.dust = fieldAt(line, 7).toFloat();
  rover.pressure = fieldAt(line, 8).toFloat();
  rover.mode = fieldAt(line, 9);
  rover.state = fieldAt(line, 10);
  rover.receivedAt = millis();
  rover.valid = true;
}

void readMegaSerial() {
  while (Serial2.available()) {
    const char c = (char)Serial2.read();
    if (c == '\n') {
      megaLine.trim();
      if (megaLine.length()) {
        Serial.print(F("[UART Mega] "));
        Serial.println(megaLine);
      }
      parseMegaData(megaLine);
      megaLine = "";
    } else if (c != '\r' && megaLine.length() < 220) {
      megaLine += c;
    } else if (megaLine.length() >= 220) {
      megaLine = "";
    }
  }
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if ((uint32_t)(millis() - lastWifiAttempt) < Config::WIFI_RETRY_MS && lastWifiAttempt != 0) return;
  lastWifiAttempt = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() {
    sendJson(200, F("{\"device\":\"EcoRover ESP32\",\"routes\":[\"/data\",\"/control?cmd=\"]}"));
  });
  server.on("/data", HTTP_GET, handleData);
  server.on("/data", HTTP_OPTIONS, handleOptions);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/control", HTTP_OPTIONS, handleOptions);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) handleOptions();
    else sendJson(404, F("{\"ok\":false,\"error\":\"not found\"}"));
  });
}

void setup() {
  Serial.begin(Config::USB_SERIAL_BAUD);
  Serial2.begin(Config::MEGA_SERIAL_BAUD, SERIAL_8N1, Config::RX2_PIN, Config::TX2_PIN);
  megaLine.reserve(220);
  WiFi.setSleep(false);
  connectWifi();
  setupRoutes();
  server.begin();
}

void loop() {
  readMegaSerial();
  connectWifi();
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wifiWasConnected) {
    Serial.print(F("[WiFi] EcoRover IP: "));
    Serial.println(WiFi.localIP());
  }
  wifiWasConnected = connected;
  server.handleClient();
}

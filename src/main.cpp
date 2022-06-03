//$ Include Library
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

#define RELAY_IN1 16
#define RELAY_IN2 17
#define RELAY_IN3 18
#define RELAY_IN4 19
#define RELAY_IN5 21

#define SWITCH1 32
#define SWITCH2 33
#define SWITCH3 25
#define SWITCH4 27
#define SWITCH5 14

#define OPT1 13
#define OPT2 4
#define OPT3 26
#define OPT4 22
#define OPT5 23

//* Device Name
const String deviceName = "plug-1";
const String centerName = "center";

int pinout[5] = {
    RELAY_IN1, RELAY_IN2, RELAY_IN3, RELAY_IN4, RELAY_IN5,
};

int buttons[5] = {
    SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5,
};

int optocouplers[5] = {
    OPT1, OPT2, OPT3, OPT4, OPT5,
};

int buttonsCondition[5] = {
    HIGH, HIGH, HIGH, HIGH, HIGH,
};

bool relayConditions[5] = {
    false, false, false, false, false,
};

bool feedbacks[5] = {
    false, false, false, false, false,
};

int lastButtonsCondition[5] = {
    HIGH, HIGH, HIGH, HIGH, HIGH,
};

bool lastFeedback[5] = {
    false, false, false, false, false,
};

unsigned long lastDebounceTime[5] = {
    0, 0, 0, 0, 0,
};

bool hasRising[5] = {
    false, false, false, false, false,
};

unsigned long prevMillis[5] = {
    0, 0, 0, 0, 0,
};

//$ Kalkulasi panjang array
int pinoutLength = sizeof(pinout) / sizeof(pinout[0]);

//$ Kalkulasi panjang array
int switchsLength = sizeof(buttons) / sizeof(buttons[0]);

//$ Kalkulasi panjang array
int relayConditionLength = sizeof(relayConditions) / sizeof(relayConditions[0]);

//$ Kalkulasi panjang array
int optocouplersLength = sizeof(optocouplers) / sizeof(optocouplers[0]);

int debounceDelay = 50;
unsigned long interval = 1000;
unsigned long lastScan = 0;

String socket = "";
bool plugCondition = false;

WebSocketsClient webSocket;

DynamicJsonDocument data(1024);
DynamicJsonDocument receivedData(1024);

String getValue(String data, char separator, int index);
void sendMessage();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED && millis() <= 5000) {
    Serial.print(".");
    delay(500);
  }

  for (int i = 0; i < pinoutLength; i++) {
    pinMode(pinout[i], OUTPUT);
    digitalWrite(pinout[i], HIGH);
  }

  for (int i = 0; i < switchsLength; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }

  for (int i = 0; i < optocouplersLength; i++) {
    pinMode(optocouplers[i], INPUT_PULLUP);
  }

  webSocket.begin("192.168.5.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();

  if (millis() - lastScan >= 2138) {
    lastScan = millis();

    if (webSocket.isConnected()) {
      Serial.println("WebSocket Connected");
    } else {
      Serial.println("Connecting (Please Connect)");
      webSocket.begin("192.168.5.1", 80, "/ws");
    }
  }

  for (int i = 0; i < relayConditionLength; i++) {
    int reading = digitalRead(buttons[i]);

    if (reading != lastButtonsCondition[i]) {
      lastDebounceTime[i] = millis();
    }

    if (millis() - lastDebounceTime[i] > debounceDelay) {
      if (reading != buttonsCondition[i]) {
        buttonsCondition[i] = reading;

        if (buttonsCondition[i] == LOW) {
          relayConditions[i] = !relayConditions[i];
        }
      }
    }

    lastButtonsCondition[i] = reading;

    if (hasRising[i] == false) {
      feedbacks[i] = digitalRead(optocouplers[i]);

      if (feedbacks[i] == true && relayConditions[i] == true) {
        feedbacks[i] = true;
        socket = "socket-" + String(i + 1);
        plugCondition = feedbacks[i];
        hasRising[i] = true;
        sendMessage();
        delay(500);
      }
    }

    if (hasRising[i] == true) {
      feedbacks[i] = digitalRead(optocouplers[i]);

      if (feedbacks[i] == false && relayConditions[i] == false) {
        feedbacks[i] = false;
        socket = "socket-" + String(i + 1);
        plugCondition = feedbacks[i];
        hasRising[i] = false;
        sendMessage();
        delay(500);
      }
    }

    digitalWrite(pinout[i], !relayConditions[i]);
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void sendMessage() {
  data["from"] = deviceName;
  data["socket"] = socket;
  data["to"] = centerName;
  data["condition"] = plugCondition;
  String msg;
  serializeJson(data, msg);
  webSocket.sendTXT(msg);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    deserializeJson(receivedData, payload);

    String myData;
    serializeJson(receivedData, myData);
    String from = receivedData["from"].as<String>();
    String to = receivedData["to"].as<String>();
    String condition = receivedData["condition"].as<String>();

    if (from == centerName) {
      Serial.println("Data from center!");
      String deviceName = getValue(to, '/', 0);
      String deviceSubName = getValue(to, '/', 1);

      if (deviceName == "plug-1") {
        Serial.println("Data is for this device!");
        if (deviceSubName == "socket-1") {
          socket = "socket-1";
          if (condition == "1") {
            relayConditions[0] = true;
            Serial.println("Socket 1 => True!");
          } else {
            relayConditions[0] = false;
            Serial.println("Socket 1 => False!");
          }
        } else if (deviceSubName == "socket-2") {
          socket = "socket-2";
          if (condition == "1") {
            relayConditions[1] = true;
            Serial.println("Socket 2 => True!");
          } else {
            relayConditions[1] = false;
            Serial.println("Socket 2 => False!");
          }
        } else if (deviceSubName == "socket-3") {
          socket = "socket-3";
          if (condition == "1") {
            relayConditions[2] = true;
            Serial.println("Socket 3 => True!");
          } else {
            relayConditions[2] = false;
            Serial.println("Socket 3 => False!");
          }
        } else if (deviceSubName == "socket-4") {
          socket = "socket-4";
          if (condition == "1") {
            relayConditions[3] = true;
            Serial.println("Socket 4 => True!");
          } else {
            relayConditions[3] = false;
            Serial.println("Socket 4 => False!");
          }
        } else if (deviceSubName == "socket-5") {
          socket = "socket-5";
          if (condition == "1") {
            relayConditions[4] = true;
            Serial.println("Socket 5 => True!");
          } else {
            relayConditions[4] = false;
            Serial.println("Socket 5 => False!");
          }
        } else if (deviceSubName == "all") {
          socket = "all";
          if (condition == "1") {
            for (int i = 0; i < 5; i++) {
              relayConditions[i] = true;
            }
          } else {
            for (int i = 0; i < 5; i++) {
              relayConditions[i] = false;
            }
          }
        }
      }
    }
  }
}
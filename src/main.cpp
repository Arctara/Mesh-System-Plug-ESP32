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

String child = "";
bool plugCondition = false;

WebSocketsClient webSocket;

DynamicJsonDocument data(1024);
DynamicJsonDocument receivedData(1024);

void sendMessage();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED && millis() <= 15000) {
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

      if (feedbacks[i] == true) {
        feedbacks[i] = true;
        child = "plug-" + String(i + 1);
        plugCondition = feedbacks[i];
        hasRising[i] = true;
        sendMessage();
      }
    }

    if (hasRising[i] == true && relayConditions[i] == false) {
      feedbacks[i] = digitalRead(optocouplers[i]);

      if (feedbacks[i] == false) {
        feedbacks[i] = false;
        child = "plug-" + String(i + 1);
        plugCondition = feedbacks[i];
        hasRising[i] = false;
        sendMessage();
      }
    }

    digitalWrite(pinout[i], !relayConditions[i]);
  }
}

void sendMessage() {
  data["from"] = "plugFamily";
  data["child"] = child;
  data["to"] = "center";
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

    if (from == "center") {
      Serial.println("Data from center!");
      if (to == "plug-1") {
        Serial.println("Data is for this device!");
        child = "plug-1";
        if (condition == "1") {
          // relayConditions[0] = true;
          Serial.println("True!");
        } else {
          // relayConditions[0] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-2") {
        Serial.println("Data is for this device!");
        child = "plug-2";
        if (condition == "1") {
          // relayConditions[1] = true;
          Serial.println("True!");
        } else {
          // relayConditions[1] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-3") {
        Serial.println("Data is for this device!");
        child = "plug-3";
        if (condition == "1") {
          // relayConditions[2] = true;
          Serial.println("True!");
        } else {
          // relayConditions[2] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-4") {
        Serial.println("Data is for this device!");
        child = "plug-4";
        if (condition == "1") {
          // relayConditions[3] = true;
          Serial.println("True!");
        } else {
          // relayConditions[3] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-5") {
        Serial.println("Data is for this device!");
        child = "plug-5";
        if (condition == "1") {
          // relayConditions[4] = true;
          Serial.println("True!");
        } else {
          // relayConditions[4] = false;
          Serial.println("False!");
        }
      }
    }
  }
}
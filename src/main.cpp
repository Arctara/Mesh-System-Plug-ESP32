/*
    PINOUT
      PIN 2 = RELAY 1
      PIN 0 = RELAY 2
      PIN 16 = RELAY 3
      PIN 4 = RELAY 4
      PIN 1 = RELAY 5

      PIN 13 = SWITCH 1
      PIN 3 = SWITCH 2
      PIN 5 = SWITCH 3
      PIN 12 = SWITCH 4
      PIN 14 = SWITCH 5
*/

//$ Include Library
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
// #include <painlessMesh.h>

//$ Mesh Configuration
// #define MESH_PREFIX "ALiVe_MESH"
// #define MESH_PASSWORD "TmlhdCBzZWthbGkgYW5kYSBtZW5kZWNvZGUgaW5pIC1NZXJ6YQ=="
// #define MESH_PORT 5555

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

#define RELAY_IN1 16  // 2 = RELAY 2 = RELAY 4 <== RELAY 1
#define RELAY_IN2 4   // 0 = RELAY 3 = RELAY 3 <== RELAY 2
#define RELAY_IN3 0   // 16 = RELAY 5 = RELAY 1 <== RELAY 3
#define RELAY_IN4 2   // 4 = RELAY 4 = RELAY 2 <== RELAY 4
#define RELAY_IN5 1   // 1 = RELAY 1 = RELAY 5 <== RELAY 5

#define SWITCH1 5   // 13
#define SWITCH2 3   // 3
#define SWITCH3 13  // 5
#define SWITCH4 12  // 12
#define SWITCH5 14  // 14

int pinout[5] = {
    RELAY_IN1, RELAY_IN2, RELAY_IN3, RELAY_IN4, RELAY_IN5,
};

int buttons[5] = {
    SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5,
};

int buttonsCondition[5] = {
    HIGH, HIGH, HIGH, HIGH, HIGH,
};

bool relayConditions[5] = {
    false, false, false, false, false,
};

int lastButtonsCondition[5] = {
    HIGH, HIGH, HIGH, HIGH, HIGH,
};

unsigned long lastDebounceTime[5] = {
    0, 0, 0, 0, 0,
};

//$ Kalkulasi panjang array
int pinoutLength = sizeof(pinout) / sizeof(pinout[0]);

//$ Kalkulasi panjang array
int switchsLength = sizeof(buttons) / sizeof(buttons[0]);

//$ Kalkulasi panjang array
int relayConditionLength = sizeof(relayConditions) / sizeof(relayConditions[0]);

unsigned long debounceDelay = 50;

//*Mesh Configuration
// Scheduler userScheduler;
// painlessMesh mesh;
// int nodeNumber = 1;

String child = "";
bool plugCondition = false;

WebSocketsClient webSocket;

DynamicJsonDocument data(1024);
DynamicJsonDocument receivedData(1024);

void sendMessage();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
// Task taskSendMessage(TASK_SECOND * 1, TASK_FOREVER, &sendMessage);

//$ Needed for painless mesh library
// void receivedCallback(uint32_t from, String &msg);
// void newConnectionCallback(uint32_t nodeId);
// void changedConnectionCallback();
// void nodeTimeAdjustedCallback(int32_t offset);

void setup() {
  Serial.begin(115200);
  // mesh.setDebugMsgTypes(ERROR | STARTUP);

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

  webSocket.begin("192.168.5.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  // mesh.onReceive(&receivedCallback);
  // mesh.onNewConnection(&newConnectionCallback);
  // mesh.onChangedConnections(&changedConnectionCallback);
  // mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // userScheduler.addTask(taskSendMessage);
  // taskSendMessage.enable();
}

void loop() {
  // mesh.update();
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
          child = "plug-" + String(i + 1);
          plugCondition = relayConditions[i];
          sendMessage();
        }
      }
    }

    digitalWrite(pinout[i], !relayConditions[i]);

    lastButtonsCondition[i] = reading;
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
  // mesh.sendBroadcast(msg);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    deserializeJson(receivedData, payload);

    String myData;
    serializeJson(receivedData, myData);
    String from = receivedData["from"].as<String>();
    String to = receivedData["to"].as<String>();
    String condition = receivedData["condition"].as<String>();

    Serial.println(myData);
    Serial.println(from);
    Serial.println(to);
    Serial.println(condition);

    if (from == "center") {
      Serial.println("Data from center!");
      if (to == "plug-1") {
        Serial.println("Data is for this device!");
        child = "plug-1";
        if (condition == "1") {
          relayConditions[0] = true;
          Serial.println("True!");
        } else {
          relayConditions[0] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-2") {
        Serial.println("Data is for this device!");
        child = "plug-2";
        if (condition == "1") {
          relayConditions[1] = true;
          Serial.println("True!");
        } else {
          relayConditions[1] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-3") {
        Serial.println("Data is for this device!");
        child = "plug-3";
        if (condition == "1") {
          relayConditions[2] = true;
          Serial.println("True!");
        } else {
          relayConditions[2] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-4") {
        Serial.println("Data is for this device!");
        child = "plug-4";
        if (condition == "1") {
          relayConditions[3] = true;
          Serial.println("True!");
        } else {
          relayConditions[3] = false;
          Serial.println("False!");
        }
      } else if (to == "plug-5") {
        Serial.println("Data is for this device!");
        child = "plug-5";
        if (condition == "1") {
          relayConditions[4] = true;
          Serial.println("True!");
        } else {
          relayConditions[4] = false;
          Serial.println("False!");
        }
      }
    }
  }
}

//$ Needed for painless mesh library
// void receivedCallback(uint32_t from, String &msg) {
//   Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
//   deserializeJson(receivedData, msg);
//   if (data["from"].as<String>() == "center" &&
//       data["to"].as<String>() == "lamp-1") {
//     if (data["condition"].as<String>() == "true") {
//       plugCondition = true;
//       dimmer.setState(ON);
//     } else {
//       plugCondition = false;
//       dimmer.setState(OFF);
//     }
//   }
//   sendMessage();
// }

// void newConnectionCallback(uint32_t nodeId) {
//   Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
// }

// void changedConnectionCallback() { Serial.printf("Changed connections\n"); }

// void nodeTimeAdjustedCallback(int32_t offset) {
//   Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),
//   offset);
// }
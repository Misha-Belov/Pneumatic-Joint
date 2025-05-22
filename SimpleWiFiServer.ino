#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* 2 --- 1
 *  \   /
 *    3
 */
const int BUTTON_PIN = 12;
const int V1_PIN = 26;
const int V2_PIN = 27;
const int V3_PIN = 14;
const int DELAY = 500;

const char* hostname = "joint-control";
const char *ssid = "iPhone (Michael)";
const char *password = "12345678";

Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket wss("/ws");
hw_timer_t *Timer0_Cfg = NULL;

char buffer[1024] = "";
JsonDocument orientation;
volatile bool ppc = false;

const uint64_t debounceTime = 200000;

volatile bool interruptEnabled = true;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR timerISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptEnabled = true;
  portEXIT_CRITICAL_ISR(&timerMux);
  
  int state = digitalRead(BUTTON_PIN);
  updatePpc(state);
}

void IRAM_ATTR onButtonChanged() {
  portENTER_CRITICAL_ISR(&timerMux);
 
  if (interruptEnabled) {
    interruptEnabled = false;
    timerEnd(timer);
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &timerISR);
    timerAlarm(timer, debounceTime, false, 0);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

void updatePpc(int state) {
  if (state == LOW) {
      digitalWrite(V1_PIN, LOW);
      digitalWrite(V2_PIN, LOW);
      digitalWrite(V3_PIN, LOW);
      ppc = false;
      Serial.println("button is low");
    } else {
      digitalWrite(V1_PIN, HIGH);
      digitalWrite(V2_PIN, HIGH);
      digitalWrite(V3_PIN, HIGH);
      Serial.println("button is hight");
      ppc = true;
    }    
}

void onMessage(char *data, size_t len) {
  JsonDocument pos;
  deserializeJson(pos, data, len);

  if (ppc)
    return;

  switch((int) pos["dir"]) {
    case 0:
      digitalWrite(V1_PIN, HIGH);
      digitalWrite(V3_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V1_PIN, LOW);
      digitalWrite(V3_PIN, LOW);
      break;
    case 1:
      digitalWrite(V1_PIN, HIGH);
      digitalWrite(V2_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V1_PIN, LOW);
      digitalWrite(V2_PIN, LOW);
      break;
    case 2:
      digitalWrite(V3_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V3_PIN, LOW);
      break;
    case 3:
      digitalWrite(V2_PIN, HIGH);
      digitalWrite(V3_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V2_PIN, LOW);
      digitalWrite(V3_PIN, LOW);
      break;
    case 4:
      digitalWrite(V1_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V1_PIN, LOW);
      break;
   case 5:
      digitalWrite(V2_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V2_PIN, LOW);
      break;
   case 6:
      digitalWrite(V3_PIN, HIGH);
      delay(DELAY);
      digitalWrite(V3_PIN, LOW);
      break;
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
 void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      onMessage((char*) data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(V1_PIN, OUTPUT);
  pinMode(V2_PIN, OUTPUT);
  pinMode(V3_PIN, OUTPUT);
  
  timer = timerBegin(1000000); // Single parameter for frequency
  timerAttachInterrupt(timer, &timerISR);
  timerAlarm(timer, debounceTime, false, 0);
  timerStop(timer); // Start disabled

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonChanged, CHANGE);

  delay(100);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  delay(100);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  orientation["pitch"] = 0;
  orientation["roll"] = 0;

  SPIFFS.begin(false);

  server.serveStatic("/", SPIFFS, "/");
  wss.onEvent(onEvent);
  server.addHandler(&wss);

  server.begin();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate temperature
  float temperature = temp.temperature / 340.0 + 36.53;

  // Calculate orientation (roll and pitch from accelerometer)
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + 
                      a.acceleration.z * a.acceleration.z)) * 180 / PI;

  orientation["pitch"] = pitch;
  orientation["roll"] = roll;

  serializeJson(orientation, buffer, 1024);
  wss.textAll(buffer);
  wss.cleanupClients();
  
  delay(100);
}

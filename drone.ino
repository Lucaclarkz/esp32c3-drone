#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>

// WiFi Settings
const char* ssid = "ESP32-Drone-C3";
const char* password = "password123";

// GPIOs
const int M1_PIN = 21, M2_PIN = 20, M3_PIN = 10, M4_PIN = 5;
const int LED_PIN = 8;

MPU6050 mpu(Wire);
AsyncWebServer server(80);

// Drone States
int throttle = 0;
float roll_input = 0, pitch_input = 0;
float pid_p = 1.3;

// Web Page with Slider and Joystick
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Drone Control</title>
<style>
  body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; }
  .slider { width: 80%; height: 25px; background: #444; }
  .data-box { background: #333; padding: 10px; border-radius: 10px; margin: 10px auto; width: 80%; }
</style></head>
<body>
  <h1>Drone Master C3</h1>
  <div class="data-box">
    Roll: <span id="roll">0</span> | Pitch: <span id="pitch">0</span>
  </div>
  <h3>Throttle</h3>
  <input type="range" min="0" max="255" value="0" class="slider" oninput="sendThrottle(this.value)">
  <div id="joystick" style="width:200px; height:200px; background:#444; margin: 20px auto; border-radius:50%;">Joystick Space</div>
  <script>
    function sendThrottle(val) { fetch(`/throttle?v=${val}`); }
    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('roll').innerText = d.r;
        document.getElementById('pitch').innerText = d.p;
      });
    }, 200);
  </script>
</body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(7, 6);

  // WiFi AP Mode & Blink LED during setup
  WiFi.softAP(ssid, password);
  for(int i=0; i<10; i++) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  digitalWrite(LED_PIN, HIGH); // WiFi Stable On

  // Motor Attach
  ledcAttach(M1_PIN, 500, 8); ledcAttach(M2_PIN, 500, 8);
  ledcAttach(M3_PIN, 500, 8); ledcAttach(M4_PIN, 500, 8);

  mpu.begin();
  delay(1000);
  mpu.calcOffsets();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/throttle", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("v")) throttle = request->getParam("v")->value().toInt();
    request->send(200);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"r\":" + String(mpu.getAngleX()) + ",\"p\":" + String(mpu.getAngleY()) + "}";
    request->send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  mpu.update();

  float out_roll = pid_p * (mpu.getAngleX() - roll_input);
  float out_pitch = pid_p * (mpu.getAngleY() - pitch_input);

  if (throttle < 20) {
    ledcWrite(M1_PIN, 0); ledcWrite(M2_PIN, 0);
    ledcWrite(M3_PIN, 0); ledcWrite(M4_PIN, 0);
  } else {
    ledcWrite(M1_PIN, constrain(throttle - out_pitch + out_roll, 0, 255));
    ledcWrite(M2_PIN, constrain(throttle - out_pitch - out_roll, 0, 255));
    ledcWrite(M3_PIN, constrain(throttle + out_pitch - out_roll, 0, 255));
    ledcWrite(M4_PIN, constrain(throttle + out_pitch + out_roll, 0, 255));
  }
  delay(10);
}

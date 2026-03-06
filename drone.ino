#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>

#define M1 21
#define M2 20
#define M3 10
#define M4 5
#define LED 8

WebServer server(80);
MPU6050 mpu(Wire);

int throttle = 0;
float roll_set = 0;
float pitch_set = 0;

float kp = 1.8;

const char page[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{background:#111;color:white;text-align:center;font-family:sans-serif}
.slider{width:80%}
</style></head>
<body>
<h2>ESP32 C3 DRONE</h2>
<input type="range" min="0" max="255" value="0"
class="slider" oninput="fetch('/t?v='+this.value)">
<br><br>
<button onclick="fetch('/j?x=-10')">LEFT</button>
<button onclick="fetch('/j?x=10')">RIGHT</button>
<br><br>
<button onclick="fetch('/j?y=10')">FORWARD</button>
<button onclick="fetch('/j?y=-10')">BACK</button>
<br><br>
<button onclick="fetch('/j?x=0&y=0')">CENTER</button>
</body></html>
)rawliteral";


void handleRoot(){ server.send_P(200,"text/html",page); }

void handleThrottle(){
  if(server.hasArg("v")) throttle = server.arg("v").toInt();
  server.send(200,"text/plain","OK");
}

void handleJoy(){
  if(server.hasArg("x")) roll_set = server.arg("x").toFloat();
  if(server.hasArg("y")) pitch_set = server.arg("y").toFloat();
  server.send(200,"text/plain","OK");
}


void setup(){

  Serial.begin(115200);
  pinMode(LED,OUTPUT);

  Wire.begin(7,6);

  WiFi.softAP("ESP32-C3-DRONE","12345678");

  server.on("/",handleRoot);
  server.on("/t",handleThrottle);
  server.on("/j",handleJoy);
  server.begin();

  ledcSetup(0,2000,8);
  ledcSetup(1,2000,8);
  ledcSetup(2,2000,8);
  ledcSetup(3,2000,8);

  ledcAttachPin(M1,0);
  ledcAttachPin(M2,1);
  ledcAttachPin(M3,2);
  ledcAttachPin(M4,3);

  if(mpu.begin()!=0){
    while(1){
      digitalWrite(LED,!digitalRead(LED));
      delay(100);
    }
  }

  delay(1000);
  mpu.calcOffsets();

  digitalWrite(LED,HIGH);
}


void loop(){

  server.handleClient();
  mpu.update();

  float roll  = mpu.getAngleX();
  float pitch = mpu.getAngleY();

  float roll_error  = roll_set  - roll;
  float pitch_error = pitch_set - pitch;

  float roll_corr  = kp * roll_error;
  float pitch_corr = kp * pitch_error;

  int m1 = throttle - pitch_corr - roll_corr;
  int m2 = throttle - pitch_corr + roll_corr;
  int m3 = throttle + pitch_corr + roll_corr;
  int m4 = throttle + pitch_corr - roll_corr;

  m1 = constrain(m1,0,255);
  m2 = constrain(m2,0,255);
  m3 = constrain(m3,0,255);
  m4 = constrain(m4,0,255);

  if(throttle < 25){
    m1 = m2 = m3 = m4 = 0;
  }

  ledcWrite(0,m1);
  ledcWrite(1,m2);
  ledcWrite(2,m3);
  ledcWrite(3,m4);

  delay(5);
}

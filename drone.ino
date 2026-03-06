#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>

const int M1 = 21, M2 = 20, M3 = 10, M4 = 5, LED = 8;
MPU6050 mpu(Wire);
AsyncWebServer server(80);

int throttle = 0;
float roll_in = 0, pitch_in = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { background:#1a1a1a; color:white; font-family:sans-serif; text-align:center; touch-action:none; }
  .slider { width:80%; height:40px; margin:20px; accent-color: #00ff88; }
  .joy-base { width:180px; height:180px; background:#333; border-radius:50%; margin:20px auto; position:relative; border: 2px solid #555; }
  .stick { width:60px; height:60px; background:#00ff88; border-radius:50%; position:absolute; top:60px; left:60px; }
</style></head>
<body>
  <h2>C3 DRONE MASTER</h2>
  <p>Roll: <span id="r">0</span> | Pitch: <span id="p">0</span></p>
  <input type="range" min="0" max="255" value="0" class="slider" oninput="fetch(`/t?v=${this.value}`)">
  <div class="joy-base" id="base"><div class="stick" id="stick"></div></div>
  <script>
    setInterval(() => { fetch('/d').then(r=>r.json()).then(d=>{document.getElementById('r').innerText=d.r;document.getElementById('p').innerText=d.p;}); }, 300);
    const b=document.getElementById('base'), s=document.getElementById('stick');
    b.ontouchmove=(e)=>{
      let x=e.touches[0].clientX-b.offsetLeft-90, y=e.touches[0].clientY-b.offsetTop-90;
      s.style.transform=`translate(${x}px,${y}px)`;
      fetch(`/j?x=${x/5}&y=${-y/5}`);
    };
    b.ontouchend=()=>{ s.style.transform='translate(0,0)'; fetch('/j?x=0&y=0'); };
  </script>
</body></html>)rawliteral";

void setup() {
  Serial.begin(115200); pinMode(LED, OUTPUT); Wire.begin(7, 6);
  WiFi.softAP("ESP32-C3-DRONE", "12345678");
  
  for(int i=0; i<10; i++){ digitalWrite(LED, !digitalRead(LED)); delay(100); }
  digitalWrite(LED, HIGH);

  // ESP32 Core v2.0.x အတွက် PWM setup ပြုလုပ်ခြင်း
  ledcSetup(0, 500, 8); ledcAttachPin(M1, 0);
  ledcSetup(1, 500, 8); ledcAttachPin(M2, 1);
  ledcSetup(2, 500, 8); ledcAttachPin(M3, 2);
  ledcSetup(3, 500, 8); ledcAttachPin(M4, 3);

  mpu.begin(); delay(1000); mpu.calcOffsets();

  server.on("/", [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.on("/t", [](AsyncWebServerRequest *r){ if(r->hasParam("v")) throttle = r->getParam("v")->value().toInt(); r->send(200); });
  server.on("/j", [](AsyncWebServerRequest *r){ if(r->hasParam("x")) roll_in = r->getParam("x")->value().toFloat(); if(r->hasParam("y")) pitch_in = r->getParam("y")->value().toFloat(); r->send(200); });
  server.on("/d", [](AsyncWebServerRequest *r){ r->send(200, "application/json", "{\"r\":"+String(mpu.getAngleX())+",\"p\":"+String(mpu.getAngleY())+"}"); });
  server.begin();
}

void loop() {
  mpu.update();
  float r_out = 1.3 * (mpu.getAngleX() - roll_in);
  float p_out = 1.3 * (mpu.getAngleY() - pitch_in);
  
  if (throttle < 20) {
    ledcWrite(0, 0); ledcWrite(1, 0); ledcWrite(2, 0); ledcWrite(3, 0);
  } else {
    ledcWrite(0, constrain(throttle - p_out + r_out, 0, 255));
    ledcWrite(1, constrain(throttle - p_out - r_out, 0, 255));
    ledcWrite(2, constrain(throttle + p_out - r_out, 0, 255));
    ledcWrite(3, constrain(throttle + p_out + r_out, 0, 255));
  }
  delay(10);
}

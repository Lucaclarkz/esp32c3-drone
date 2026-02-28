/*
  ESP32-C3 SuperMini Plus + MPU6050 Quad Prototype
  - WiFi AP + simple web UI control
  - MPU6050 complementary filter
  - PID roll/pitch/yaw rate stabilization
  - Motor mixing (X frame)

  Wiring:
    MPU6050 SDA=GPIO4, SCL=GPIO5, VCC=3.3V, GND=GND
    Motors (PWM to MOSFET/driver):
      M1=GPIO2, M2=GPIO3, M3=GPIO6, M4=GPIO7
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//////////////////// USER CONFIG ////////////////////
const char* AP_SSID = "C3-DRONE";
const char* AP_PASS = "12345678";

// I2C pins (explicit to avoid conflicts)
static const int I2C_SDA = 4;
static const int I2C_SCL = 5;

// Motor PWM pins
static const int PIN_M1 = 2;
static const int PIN_M2 = 3;
static const int PIN_M3 = 6;
static const int PIN_M4 = 7;

// PWM config (LEDC)
static const int PWM_FREQ = 20000;       // 20kHz for motors
static const int PWM_RES_BITS = 8;       // 8-bit (0..255)
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

// Motor channels
static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

// Control loop
static const float LOOP_HZ = 250.0f;     // 250Hz
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

// Complementary filter
static const float CF_ALPHA = 0.98f;

// Arm / safety
volatile bool armed = false;
volatile uint32_t lastCmdMs = 0;
static const uint32_t CMD_TIMEOUT_MS = 400; // if no command, cut throttle

// Control inputs (from web UI): -1..+1 for roll/pitch/yaw, 0..1 throttle
volatile float cmdThrottle = 0.0f;
volatile float cmdRoll = 0.0f;
volatile float cmdPitch = 0.0f;
volatile float cmdYaw = 0.0f;

//////////////////// PID ////////////////////
struct PID {
  float kp, ki, kd;
  float i;
  float lastErr;
  float outMin, outMax;

  PID(float p, float _i, float d, float omin, float omax)
    : kp(p), ki(_i), kd(d), i(0), lastErr(0), outMin(omin), outMax(omax) {}

  float update(float err, float dt) {
    i += err * dt;
    // simple integral clamp
    if (i > outMax) i = outMax;
    if (i < outMin) i = outMin;

    float dErr = (err - lastErr) / dt;
    lastErr = err;

    float out = kp * err + ki * i + kd * dErr;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    return out;
  }

  void reset() { i = 0; lastErr = 0; }
};

// “စတင်စမ်းဖို့” PID (tune လုပ်ပါ)
PID pidRollRate (0.12f, 0.00f, 0.0020f, -1.0f, 1.0f);
PID pidPitchRate(0.12f, 0.00f, 0.0020f, -1.0f, 1.0f);
PID pidYawRate  (0.10f, 0.00f, 0.0008f, -1.0f, 1.0f);

//////////////////// IMU ////////////////////
Adafruit_MPU6050 mpu;

// Estimated angles (deg)
float estRoll = 0.0f;
float estPitch = 0.0f;

// Gyro offsets (deg/s)
float gOffX = 0, gOffY = 0, gOffZ = 0;

//////////////////// WEB ////////////////////
WebServer server(80);

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>C3 Drone Control</title>
<style>
  body { font-family: sans-serif; margin: 16px; }
  .row { margin: 10px 0; }
  input[type=range]{ width: 100%; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  button { width: 100%; padding: 12px; font-size: 16px; }
  .val { font-weight: 700; }
</style>
</head>
<body>
<h2>C3 Drone Control</h2>
<div class="row grid">
  <button onclick="arm(1)">ARM</button>
  <button onclick="arm(0)">DISARM</button>
</div>

<div class="row">
  Throttle: <span class="val" id="tV">0</span>
  <input id="t" type="range" min="0" max="1000" value="0" />
</div>

<div class="row">
  Roll: <span class="val" id="rV">0</span>
  <input id="r" type="range" min="-1000" max="1000" value="0" />
</div>

<div class="row">
  Pitch: <span class="val" id="pV">0</span>
  <input id="p" type="range" min="-1000" max="1000" value="0" />
</div>

<div class="row">
  Yaw: <span class="val" id="yV">0</span>
  <input id="y" type="range" min="-1000" max="1000" value="0" />
</div>

<div class="row">
  <button onclick="zero()">CENTER R/P/Y</button>
</div>

<script>
let T=0,R=0,P=0,Y=0;

function arm(on){
  fetch("/arm?on=" + (on?1:0));
}
function zero(){
  document.getElementById("r").value=0;
  document.getElementById("p").value=0;
  document.getElementById("y").value=0;
}

function send(){
  const t = +document.getElementById("t").value;
  const r = +document.getElementById("r").value;
  const p = +document.getElementById("p").value;
  const y = +document.getElementById("y").value;

  document.getElementById("tV").textContent=t;
  document.getElementById("rV").textContent=r;
  document.getElementById("pV").textContent=p;
  document.getElementById("yV").textContent=y;

  // Normalize to 0..1 and -1..1
  const Tn = (t/1000).toFixed(3);
  const Rn = (r/1000).toFixed(3);
  const Pn = (p/1000).toFixed(3);
  const Yn = (y/1000).toFixed(3);

  fetch(`/cmd?t=${Tn}&r=${Rn}&p=${Pn}&y=${Yn}`).catch(()=>{});
}

// send at 20Hz
setInterval(send, 50);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send(200, "text/html", FPSTR(INDEX_HTML));
}

void handleArm() {
  if (server.hasArg("on")) {
    armed = (server.arg("on").toInt() == 1);
    if (!armed) {
      cmdThrottle = 0;
      pidRollRate.reset();
      pidPitchRate.reset();
      pidYawRate.reset();
    }
  }
  server.send(200, "text/plain", armed ? "ARMED" : "DISARMED");
}

void handleCmd() {
  if (server.hasArg("t")) cmdThrottle = server.arg("t").toFloat();
  if (server.hasArg("r")) cmdRoll     = server.arg("r").toFloat();
  if (server.hasArg("p")) cmdPitch    = server.arg("p").toFloat();
  if (server.hasArg("y")) cmdYaw      = server.arg("y").toFloat();
  lastCmdMs = millis();
  server.send(200, "text/plain", "OK");
}

//////////////////// MOTOR OUTPUT ////////////////////
inline void motorWrite(int ch, int value) {
  if (value < 0) value = 0;
  if (value > PWM_MAX) value = PWM_MAX;
  ledcWrite(ch, value);
}

void motorsAllStop() {
  motorWrite(CH_M1, 0);
  motorWrite(CH_M2, 0);
  motorWrite(CH_M3, 0);
  motorWrite(CH_M4, 0);
}

//////////////////// IMU HELPERS ////////////////////
void calibrateGyro(uint16_t samples = 800) {
  float sx=0, sy=0, sz=0;
  sensors_event_t a, g, t;
  for (uint16_t i=0;i<samples;i++){
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x;
    sy += g.gyro.y;
    sz += g.gyro.z;
    delay(2);
  }
  // rad/s -> deg/s offsets
  gOffX = (sx / samples) * 57.2958f;
  gOffY = (sy / samples) * 57.2958f;
  gOffZ = (sz / samples) * 57.2958f;
}

void setupMPU() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!mpu.begin()) {
    // if no IMU, stop motors forever
    motorsAllStop();
    while(true) { delay(1000); }
  }

  // Basic config
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(200);
  calibrateGyro();
}

//////////////////// SETUP / LOOP ////////////////////
uint32_t lastLoopUs = 0;

void setup() {
  // PWM setup
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);

  motorsAllStop();

  setupMPU();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/arm", handleArm);
  server.on("/cmd", handleCmd);
  server.begin();

  lastCmdMs = millis();
  lastLoopUs = micros();
}

void loop() {
  server.handleClient();

  // fixed rate loop
  uint32_t nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) / 1000000.0f;
  lastLoopUs = nowUs;

  // command timeout safety
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    cmdThrottle = 0;
    cmdRoll = cmdPitch = cmdYaw = 0;
  }

  // read IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // accel angles (deg)
  // roll about X, pitch about Y (approx)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float accRoll  = atan2f(ay, az) * 57.2958f;
  float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;

  // gyro rates (deg/s)
  float gyroRollRate  = g.gyro.x * 57.2958f - gOffX;
  float gyroPitchRate = g.gyro.y * 57.2958f - gOffY;
  float gyroYawRate   = g.gyro.z * 57.2958f - gOffZ;

  // complementary filter for angles (roll/pitch)
  estRoll  = CF_ALPHA * (estRoll  + gyroRollRate  * dt) + (1.0f - CF_ALPHA) * accRoll;
  estPitch = CF_ALPHA * (estPitch + gyroPitchRate * dt) + (1.0f - CF_ALPHA) * accPitch;

  // ===== Control: we stabilize RATE (gyro), command is desired rate =====
  // Scale desired rates (deg/s)
  float desiredRollRate  = cmdRoll  * 180.0f;   // -180..180 deg/s
  float desiredPitchRate = cmdPitch * 180.0f;
  float desiredYawRate   = cmdYaw   * 120.0f;

  float errRollRate  = desiredRollRate  - gyroRollRate;
  float errPitchRate = desiredPitchRate - gyroPitchRate;
  float errYawRate   = desiredYawRate   - gyroYawRate;

  float uRoll  = pidRollRate.update(errRollRate, dt);    // -1..1
  float uPitch = pidPitchRate.update(errPitchRate, dt);
  float uYaw   = pidYawRate.update(errYawRate, dt);

  // throttle 0..1
  float thr = cmdThrottle;
  if (!armed) thr = 0;

  // map throttle to pwm base (keep a minimum if you want, but for safety start at 0)
  float base = thr; // 0..1

  // ===== Motor mixing (X frame) =====
  // M1: front-left, M2: front-right, M3: back-left, M4: back-right
  float m1 = base + uPitch + uRoll - uYaw;
  float m2 = base + uPitch - uRoll + uYaw;
  float m3 = base - uPitch + uRoll + uYaw;
  float m4 = base - uPitch - uRoll - uYaw;

  // constrain 0..1
  m1 = fminf(1.0f, fmaxf(0.0f, m1));
  m2 = fminf(1.0f, fmaxf(0.0f, m2));
  m3 = fminf(1.0f, fmaxf(0.0f, m3));
  m4 = fminf(1.0f, fmaxf(0.0f, m4));

  // convert to PWM 0..255
  int p1 = (int)(m1 * PWM_MAX);
  int p2 = (int)(m2 * PWM_MAX);
  int p3 = (int)(m3 * PWM_MAX);
  int p4 = (int)(m4 * PWM_MAX);

  // safety: if disarmed or throttle==0 => stop
  if (!armed || thr <= 0.01f) {
    motorsAllStop();
    return;
  }

  motorWrite(CH_M1, p1);
  motorWrite(CH_M2, p2);
  motorWrite(CH_M3, p3);
  motorWrite(CH_M4, p4);
}

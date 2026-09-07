/*
  Quadruped sensor integration + WiFi phone dashboard
  Board: ESP32 (NodeMCU-32S)

  Wiring (matches NodeMCU-32S board labels):
    HC-SR04   VCC->5V  GND->GND  TRIG->P4  ECHO->P16 (through voltage divider!)
    Line sens VCC->3V3 GND->GND  S1->P34 S2->P35 S3->SVP(36) S4->SVN(39) S5->P32
              CLP->P33 (optional cliff sensor)  Near->P13 (optional proximity)
    Motor driver (L298N): Motor A IN1->P25 IN2->P26   Motor B IN1->P27 IN2->P14

  WiFi: ESP32 creates its own WiFi network (Access Point mode).
  Connect your phone to the WiFi network named below, then open
  a browser to http://192.168.4.1 to control the bot.

  Two modes:
    AUTO   - sensor-driven line following / obstacle avoidance (your
             existing logic, unchanged)
    MANUAL - phone dashboard buttons directly drive the motors, and
             auto-stops the motors if no command arrives for 800ms
             (so it doesn't run away if your phone disconnects)
  The dashboard has a button to switch between the two.
*/

#include <WiFi.h>
#include <WebServer.h>

// ---------- WiFi Access Point settings ----------
const char* AP_SSID     = "QuadrupedBot";
const char* AP_PASSWORD = "walkbot123";   // must be 8+ characters

WebServer server(80);

// ---------- Pin definitions ----------
#define TRIG_PIN     4
#define ECHO_PIN     16

const int LINE_PINS[5] = {34, 35, 36, 39, 32}; // S1..S5, left-most to right-most

#define CLP_PIN      33
#define NEAR_PIN     13
#define LINE_ACTIVE LOW

// ---------- L298N motor driver pins ----------
#define MOTOR_A_IN1  25
#define MOTOR_A_IN2  26
#define MOTOR_B_IN1  27
#define MOTOR_B_IN2  14

// ---------- Tunables ----------
const int OBSTACLE_STOP_CM = 10;
const unsigned long SENSOR_PERIOD_MS = 50;
const unsigned long MANUAL_TIMEOUT_MS = 800; // auto-stop if no command received

unsigned long lastRead = 0;
unsigned long lastManualCmd = 0;
bool manualMode = false;

// latest sensor snapshot, shared with the /status web endpoint
long   g_distance = -1;
bool   g_line[5]   = {false,false,false,false,false};
bool   g_clp       = false;
bool   g_near      = false;

// forward declarations
void walkForward(); void walkBackward(); void turnLeft(); void turnRight(); void stopWalking();
void motorA(int dir); void motorB(int dir);
long readDistanceCM(); void readLineSensors(bool state[5]);
void handleRoot(); void handleStatus();
void handleCmd(void (*fn)());

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i < 5; i++) pinMode(LINE_PINS[i], INPUT);
  pinMode(CLP_PIN, INPUT);
  pinMode(NEAR_PIN, INPUT);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  stopWalking();

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect to WiFi '");
  Serial.print(AP_SSID);
  Serial.print("' then browse to http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/forward", [](){ manualMode = true; lastManualCmd = millis(); walkBackward(); server.send(200,"text/plain","OK"); });
  server.on("/backward",[](){ manualMode = true; lastManualCmd = millis(); walkForward();  server.send(200,"text/plain","OK"); });
  server.on("/left",    [](){ manualMode = true; lastManualCmd = millis(); turnLeft();     server.send(200,"text/plain","OK"); });
  server.on("/right",   [](){ manualMode = true; lastManualCmd = millis(); turnRight();    server.send(200,"text/plain","OK"); });
  server.on("/stop",    [](){ manualMode = true; lastManualCmd = millis(); stopWalking();  server.send(200,"text/plain","OK"); });
  server.on("/auto",    [](){ manualMode = false; stopWalking(); server.send(200,"text/plain","OK"); });
  server.begin();
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

void readLineSensors(bool state[5]) {
  for (int i = 0; i < 5; i++) state[i] = (digitalRead(LINE_PINS[i]) == LINE_ACTIVE);
}

void loop() {
  server.handleClient(); // handle web requests every loop, not just every sensor tick

  // safety: if in manual mode and no command received recently, stop
  if (manualMode && (millis() - lastManualCmd > MANUAL_TIMEOUT_MS)) {
    stopWalking();
  }

  if (millis() - lastRead < SENSOR_PERIOD_MS) return;
  lastRead = millis();

  g_distance = readDistanceCM();
  readLineSensors(g_line);
  g_clp  = (digitalRead(CLP_PIN)  == LINE_ACTIVE);
  g_near = (digitalRead(NEAR_PIN) == LINE_ACTIVE);

  Serial.print("Mode: "); Serial.print(manualMode ? "MANUAL" : "AUTO");
  Serial.print(" | Distance: ");
  if (g_distance > 0) Serial.print(g_distance); else Serial.print("---");
  Serial.print(" cm | Line[L->R]: ");
  for (int i = 0; i < 5; i++) Serial.print(g_line[i] ? "1 " : "0 ");
  Serial.print("| CLP: "); Serial.print(g_clp ? "1" : "0");
  Serial.print(" | Near: "); Serial.println(g_near ? "1" : "0");

  if (manualMode) return; // phone is in control, skip autonomous driving

  // ---------- AUTO mode: sensor-driven driving (unchanged logic) ----------
  if (g_distance > 0 && g_distance <= OBSTACLE_STOP_CM) {
    stopWalking();
    turnRight();
    return;
  }
  if (g_line[2] && !g_line[0] && !g_line[1] && !g_line[3] && !g_line[4]) {
    walkForward();
  } else if (g_line[0] || g_line[1]) {
    turnLeft();
  } else if (g_line[3] || g_line[4]) {
    turnRight();
  } else {
    walkForward();
  }
}

// ---------- Low-level motor helpers ----------
void motorA(int dir) {
  digitalWrite(MOTOR_A_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_A_IN2, dir < 0 ? HIGH : LOW);
}
void motorB(int dir) {
  digitalWrite(MOTOR_B_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_B_IN2, dir < 0 ? HIGH : LOW);
}
void walkForward() { motorA(1);  motorB(1);  }
void walkBackward(){ motorA(-1); motorB(-1); }
void turnLeft()     { motorA(-1); motorB(1);  }
void turnRight()    { motorA(1);  motorB(-1); }
void stopWalking()  { motorA(0); motorB(0); }

// ---------- Web dashboard ----------
void handleStatus() {
  String json = "{";
  json += "\"mode\":\""; json += (manualMode ? "manual" : "auto"); json += "\",";
  json += "\"distance\":"; json += String(g_distance); json += ",";
  json += "\"line\":[";
  for (int i = 0; i < 5; i++) { json += (g_line[i] ? "1" : "0"); if (i < 4) json += ","; }
  json += "],";
  json += "\"clp\":"; json += (g_clp ? "1" : "0"); json += ",";
  json += "\"near\":"; json += (g_near ? "1" : "0");
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Quadruped Control</title>
<style>
  body { font-family: sans-serif; text-align: center; background:#111; color:#eee; margin:0; padding:20px; }
  h2 { margin-bottom: 4px; }
  #modeBtn { padding:10px 20px; font-size:16px; margin:10px; border-radius:8px; border:none; background:#444; color:#fff; }
  .pad { display:grid; grid-template-columns: 80px 80px 80px; grid-gap:10px; justify-content:center; margin-top:20px; }
  .pad button { font-size:22px; padding:20px 0; border-radius:12px; border:none; background:#2a6; color:#fff; user-select:none; }
  .pad button:active { background:#184; }
  #stopBtn { background:#a22 !important; }
  #status { margin-top:24px; font-size:14px; color:#9cf; white-space:pre; }
</style></head>
<body>
  <h2>Quadruped Control</h2>
  <button id="modeBtn" onclick="toggleMode()">Mode: AUTO</button>
  <div class="pad">
    <div></div>
    <button ontouchstart="cmd('forward')" ontouchend="cmd('stop')" onmousedown="cmd('forward')" onmouseup="cmd('stop')">FWD</button>
    <div></div>
    <button ontouchstart="cmd('left')" ontouchend="cmd('stop')" onmousedown="cmd('left')" onmouseup="cmd('stop')">LEFT</button>
    <button id="stopBtn" onclick="cmd('stop')">STOP</button>
    <button ontouchstart="cmd('right')" ontouchend="cmd('stop')" onmousedown="cmd('right')" onmouseup="cmd('stop')">RIGHT</button>
    <div></div>
    <button ontouchstart="cmd('backward')" ontouchend="cmd('stop')" onmousedown="cmd('backward')" onmouseup="cmd('stop')">BACK</button>
    <div></div>
  </div>
  <div id="status">loading...</div>
<script>
let manual = false;
function cmd(c) {
  manual = true;
  document.getElementById('modeBtn').innerText = 'Mode: MANUAL';
  fetch('/' + c);
}
function toggleMode() {
  manual = !manual;
  fetch(manual ? '/stop' : '/auto');
  document.getElementById('modeBtn').innerText = manual ? 'Mode: MANUAL' : 'Mode: AUTO';
}
setInterval(() => {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('status').innerText =
      'Mode: ' + d.mode + '\n' +
      'Distance: ' + d.distance + ' cm\n' +
      'Line: ' + d.line.join(' ') + '\n' +
      'CLP: ' + d.clp + '  Near: ' + d.near;
  });
}, 500);
</script>
</body></html>
)rawliteral";
  server.send(200, "text/html", page);
}

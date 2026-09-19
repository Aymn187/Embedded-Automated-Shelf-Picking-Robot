#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// WiFi
// =====================================================
const char* WIFI_SSID     = "Abu Hussain 4G";
const char* WIFI_PASSWORD = "Aa@0508036171";

WebServer server(80);

// =====================================================
// Limit switches
// NC wiring: COM -> GND , NC -> GPIO
// Current logic: FREE = LOW , PRESSED = HIGH
// They are monitored only. They DO NOT stop RUN ALL movement.
// If your real test is opposite, change HIGH to LOW.
// =====================================================
const uint8_t LIMIT_PRESSED_LEVEL = HIGH;
const unsigned long LIMIT_DEBOUNCE_US = 8000;

// =====================================================
// Axis map
// X: STEP13 - DIR15 - LIMIT25
// Y: STEP16 - DIR17 - LIMIT33
// Z: STEP27 - DIR23 - LIMIT32
// =====================================================
struct Axis {
  const char* name;
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t limitPin;

  bool invertJog;
  int8_t defaultBlockJogDir;

  long position;
  unsigned long lastStepMicros;

  bool debouncedLimit;
  bool lastRawLimit;
  unsigned long lastLimitChangeMicros;
  int8_t activeBlockedJogDir;

  long autoTarget;
  bool autoMoving;
};

Axis axes[3] = {
  {"X", 13, 15, 32, false,  1, 0, 0, false, false, 0, 0, 0, false},
  {"Y", 16, 17, 33, true,  -1, 0, 0, false, false, 0, 0, 0, false},
  {"Z", 27, 23, 25, false, -1, 0, 0, false, false, 0, 0, 0, false}
};

#define AX_X 0
#define AX_Y 1
#define AX_Z 2

// Smaller = faster
unsigned int stepIntervalUs = 3500;

// =====================================================
// NEW recorded coordinates
// Z is the IN/OUT axis.
// Z ~= 0 means retracted / safe near the limit.
// =====================================================
struct Waypoint {
  long x;
  long y;
  long z;
};

Waypoint waypoints[] = {
  { -182, -1903, 583 },  // Step 1
  {  544, -2290,  10 },  // Step 2
  {  563, -1354, 636 },  // Step 3
  {  563, -1071,   3 },  // Step 4
  {  563, -2002, 598 },  // Step 5
  { 1204, -2345,  12 },  // Step 6
  { 1279, -1371, 610 },  // Step 7
  { 1279, -1037,   3 },  // Step 8
  { 1279, -1988, 605 },  // Step 9
  { 2273, -2331,  52 },  // Step 10
  { 2582,  -623, 626 },  // Step 11
  { 2582,   -10,   3 }   // Step 12
};

const int TOTAL_STEPS = sizeof(waypoints) / sizeof(waypoints[0]);

// =====================================================
// DYNAMIC SAFE MOVEMENT STRATEGY
// IMPORTANT: no two axes move at the same time.
//
// The order depends on the Z direction of the CURRENT waypoint:
//
// A) Z increases  -> insertion into shelf
//    Y -> X -> Z
//    X and Y must reach the correct position BEFORE Z enters.
//
// B) Z decreases  -> extraction / retraction from shelf
//    Y -> Z -> X
//    Y lifts/positions first while Z is still holding the drawer,
//    then Z retracts completely to the recorded target,
//    and only after Z finishes is X allowed to move horizontally.
//
// C) Z unchanged -> use Y -> X -> Z (Z phase completes immediately).
//
// The decision is made once when each waypoint begins and is kept
// for that waypoint, including after STOP/RESUME.
// =====================================================
enum Phase : uint8_t {
  PH_Y_MOVE   = 0,
  PH_X_MOVE   = 1,
  PH_Z_MOVE   = 2
};

bool running = false;
bool finishedAll = false;
bool errorState = false;
int currentStep = 0;
Phase currentPhase = PH_Y_MOVE;

// +1 = Z increasing (insertion)
// -1 = Z decreasing (retraction/extraction)
//  0 = Z unchanged
int8_t currentZDirection = 0;
bool currentStepPrepared = false;

// =====================================================
// Limit helpers
// =====================================================
bool readRawLimit(Axis& ax) {
  return digitalRead(ax.limitPin) == LIMIT_PRESSED_LEVEL;
}

bool limitHit(Axis& ax) {
  return ax.debouncedLimit;
}

int8_t actualDir(Axis& ax, int8_t rawDir) {
  return ax.invertJog ? -rawDir : rawDir;
}

int8_t rawDirFromRealDir(Axis& ax, int8_t realDir) {
  return ax.invertJog ? -realDir : realDir;
}

bool moveAllowed(Axis& ax, int8_t rawDir) {
  if (!limitHit(ax)) return true;

  int8_t blockedDir = ax.activeBlockedJogDir;
  if (blockedDir == 0) blockedDir = ax.defaultBlockJogDir;

  return rawDir != blockedDir;
}

void stepOnce(Axis& ax, int8_t realDir) {
  digitalWrite(ax.dirPin, realDir > 0 ? HIGH : LOW);

  digitalWrite(ax.stepPin, HIGH);
  delayMicroseconds(4);
  digitalWrite(ax.stepPin, LOW);

  ax.position += realDir;
}

void stopAxis(Axis& ax) {
  ax.autoMoving = false;
  digitalWrite(ax.stepPin, LOW);
}

void stopAllAxes() {
  for (int i = 0; i < 3; i++) stopAxis(axes[i]);
}

void updateLimitSwitch(Axis& ax, unsigned long now) {
  bool raw = readRawLimit(ax);

  if (raw != ax.lastRawLimit) {
    ax.lastRawLimit = raw;
    ax.lastLimitChangeMicros = now;
  }

  if ((now - ax.lastLimitChangeMicros) < LIMIT_DEBOUNCE_US) return;
  if (ax.debouncedLimit == raw) return;

  bool oldState = ax.debouncedLimit;
  ax.debouncedLimit = raw;

  if (!oldState && ax.debouncedLimit) {
    int8_t currentRawDir = 0;

    if (ax.autoMoving && ax.autoTarget != ax.position) {
      int8_t realDir = (ax.autoTarget > ax.position) ? 1 : -1;
      currentRawDir = rawDirFromRealDir(ax, realDir);
    }

    ax.activeBlockedJogDir =
      (currentRawDir != 0) ? currentRawDir : ax.defaultBlockJogDir;

    Serial.print("LIMIT HIT ");
    Serial.println(ax.name);
  }

  if (oldState && !ax.debouncedLimit) {
    ax.activeBlockedJogDir = 0;
    Serial.print("LIMIT RELEASED ");
    Serial.println(ax.name);
  }
}

// =====================================================
// Auto movement helpers
// =====================================================
void armAxisTarget(int idx, long target) {
  // Safety: only one axis may be armed at once.
  stopAllAxes();

  axes[idx].autoTarget = target;
  axes[idx].autoMoving = (axes[idx].position != target);
}

bool axisReached(int idx) {
  return axes[idx].position == axes[idx].autoTarget;
}

bool axisStillMoving(int idx) {
  return axes[idx].autoMoving;
}

void armCurrentPhase() {
  Waypoint &wp = waypoints[currentStep];

  if (currentPhase == PH_Y_MOVE) {
    armAxisTarget(AX_Y, wp.y);
    return;
  }

  if (currentPhase == PH_X_MOVE) {
    armAxisTarget(AX_X, wp.x);
    return;
  }

  // PH_Z_MOVE: move directly to the real recorded Z value.
  armAxisTarget(AX_Z, wp.z);
}

void prepareCurrentStep() {
  Waypoint &wp = waypoints[currentStep];

  long zNow = axes[AX_Z].position;

  if (wp.z > zNow) {
    currentZDirection = 1;   // insertion: Y -> X -> Z
  } else if (wp.z < zNow) {
    currentZDirection = -1;  // extraction: Y -> Z -> X
  } else {
    currentZDirection = 0;   // unchanged: Y -> X -> Z
  }

  currentPhase = PH_Y_MOVE;
  currentStepPrepared = true;

  Serial.print("STEP ");
  Serial.print(currentStep + 1);
  Serial.print(" Z direction: ");

  if (currentZDirection > 0) {
    Serial.println("INCREASE -> order Y, X, Z");
  } else if (currentZDirection < 0) {
    Serial.println("DECREASE -> order Y, Z, X");
  } else {
    Serial.println("SAME -> order Y, X, Z");
  }

  armCurrentPhase();
}

void startSequence() {
  if (finishedAll) {
    currentStep = 0;
    finishedAll = false;
    currentStepPrepared = false;
  }

  errorState = false;
  running = true;

  // New waypoint: decide the movement order from Z direction.
  // Resume: keep the already-decided order for this waypoint.
  if (!currentStepPrepared) {
    prepareCurrentStep();
  } else {
    armCurrentPhase();
  }

  Serial.println("SEQUENCE START/RESUME");
}

void stopSequence() {
  running = false;
  stopAllAxes();

  Serial.println("SEQUENCE STOPPED");
}

void finishCurrentStepAndAdvance() {
  currentStep++;
  currentStepPrepared = false;

  if (currentStep >= TOTAL_STEPS) {
    running = false;
    finishedAll = true;
    stopAllAxes();
    Serial.println("SEQUENCE DONE");
    return;
  }

  prepareCurrentStep();
}

void nextPhaseOrStep() {
  // ---------------------------------------------------
  // INSERTION: Z increases
  // Required order: Y -> X -> Z
  // ---------------------------------------------------
  if (currentZDirection >= 0) {
    if (currentPhase == PH_Y_MOVE) {
      currentPhase = PH_X_MOVE;
      armCurrentPhase();
      return;
    }

    if (currentPhase == PH_X_MOVE) {
      currentPhase = PH_Z_MOVE;
      armCurrentPhase();
      return;
    }

    // Z finished: waypoint complete.
    finishCurrentStepAndAdvance();
    return;
  }

  // ---------------------------------------------------
  // EXTRACTION: Z decreases
  // Required order: Y -> Z -> X
  // ---------------------------------------------------
  if (currentPhase == PH_Y_MOVE) {
    currentPhase = PH_Z_MOVE;
    armCurrentPhase();
    return;
  }

  if (currentPhase == PH_Z_MOVE) {
    currentPhase = PH_X_MOVE;
    armCurrentPhase();
    return;
  }

  // X finished: waypoint complete.
  finishCurrentStepAndAdvance();
}

void updateSequence() {
  if (!running) return;

  int activeAxis = AX_Z;

  if (currentPhase == PH_Y_MOVE) activeAxis = AX_Y;
  if (currentPhase == PH_X_MOVE) activeAxis = AX_X;
  if (currentPhase == PH_Z_MOVE) activeAxis = AX_Z;

  if (axisReached(activeAxis)) {
    nextPhaseOrStep();
    return;
  }

  if (!axisStillMoving(activeAxis)) {
    running = false;
    errorState = true;
    stopAllAxes();

    Serial.print("ERROR: AXIS STOPPED BEFORE TARGET: ");
    Serial.println(axes[activeAxis].name);
  }
}

// =====================================================
// Web UI
// =====================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 - Safe Coordinates Player</title>

<style>
*{box-sizing:border-box}
body{
  font-family:Tahoma,Arial,sans-serif;
  background:#111;
  color:#eee;
  text-align:center;
  margin:0;
  padding:12px;
}
h2,h3{margin:8px 0;overflow-wrap:anywhere}
h2{color:#4fc3f7}
.card{
  width:min(100%,560px);
  background:#1c1c1c;
  border-radius:14px;
  padding:16px;
  margin:12px auto;
  overflow:hidden;
}
.row{
  display:flex;
  flex-wrap:wrap;
  gap:10px;
  justify-content:center;
  margin:10px 0;
}
button{
  flex:1 1 140px;
  min-width:0;
  min-height:56px;
  border:0;
  border-radius:10px;
  padding:10px;
  font-size:18px;
  font-weight:bold;
  color:#111;
  white-space:normal;
  overflow-wrap:anywhere;
  touch-action:manipulation;
}
.start{background:#4caf50;color:#fff}
.stop{background:#e53935;color:#fff}
.statLine{
  display:flex;
  justify-content:space-between;
  gap:12px;
  font-size:14px;
  padding:7px 4px;
  border-bottom:1px solid #2a2a2a;
}
.statLine span{min-width:0;overflow-wrap:anywhere}
.statLine span:first-child{color:#9e9e9e}
#state{
  font-size:20px;
  font-weight:bold;
  margin:6px 0 14px;
  overflow-wrap:anywhere;
}
.stIdle{color:#bbb}
.stRun{color:#4fc3f7}
.stDone{color:#8bc34a}
.stError{color:#ff5252}
.safeNote{
  color:#ffca28;
  font-size:13px;
  line-height:1.6;
}
input[type=range]{width:85%;max-width:100%}
</style>
</head>

<body>

<h2>تشغيل الإحداثيات الجديدة - Safe Mode</h2>

<div class="card">
  <div class="safeNote">
    تشغيل واحد يمشي بالـ 12 نقطة بالترتيب دون تخطي أي نقطة.
    الترتيب يتغير تلقائياً حسب اتجاه Z: إذا كانت قيمة Z الجديدة أكبر
    (دخول للرف) يكون الترتيب Y ثم X ثم Z. وإذا كانت قيمة Z الجديدة أقل
    (خروج من الرف) يكون الترتيب Y ثم Z ثم X، بحيث لا يبدأ X إلا بعد
    خروج Z ووصوله الكامل إلى القيمة المطلوبة. لا يتحرك محوران معاً.
  </div>

  <div id="state" class="stIdle">جاهز</div>

  <div class="row">
    <button class="start" id="startBtn">RUN ALL - تشغيل كل الخطوات</button>
    <button class="stop" id="stopBtn">إيقاف</button>
  </div>
</div>

<div class="card">
  <h3>الحالة الحالية</h3>

  <div class="statLine"><span>الخطوة</span><span id="stepInfo">-- / --</span></div>
  <div class="statLine"><span>المرحلة</span><span id="phaseInfo">--</span></div>
  <div class="statLine"><span>X</span><span id="posX">0</span></div>
  <div class="statLine"><span>Y</span><span id="posY">0</span></div>
  <div class="statLine"><span>Z</span><span id="posZ">0</span></div>
  <div class="statLine"><span>Target X</span><span id="tarX">0</span></div>
  <div class="statLine"><span>Target Y</span><span id="tarY">0</span></div>
  <div class="statLine"><span>Target Z</span><span id="tarZ">0</span></div>
</div>

<div class="card">
  <label>سرعة الحركة: <span id="spdVal">3500</span></label>
  <br><br>
  <input type="range" min="500" max="8000" value="3500" id="spd">
</div>

<script>
async function command(url){
  try{
    const r = await fetch(url, {cache:"no-store"});
    const text = await r.text();
    if(!r.ok) throw new Error(text || "Request failed");
    return text;
  }catch(e){
    return null;
  }
}

document.getElementById("startBtn").addEventListener("click", () => {
  command("/start");
});

document.getElementById("stopBtn").addEventListener("click", () => {
  command("/stop");
});

const spd = document.getElementById("spd");

spd.addEventListener("input", () => {
  document.getElementById("spdVal").textContent = spd.value;
});

spd.addEventListener("change", () => {
  command(`/speed?value=${spd.value}`);
});

let busy = false;

async function refreshStatus(){
  if(busy) return;
  busy = true;

  try{
    const r = await fetch("/status", {cache:"no-store"});
    if(!r.ok) throw new Error("status");

    const d = await r.json();

    document.getElementById("posX").textContent = d.x;
    document.getElementById("posY").textContent = d.y;
    document.getElementById("posZ").textContent = d.z;

    document.getElementById("tarX").textContent = d.tx;
    document.getElementById("tarY").textContent = d.ty;
    document.getElementById("tarZ").textContent = d.tz;

    document.getElementById("stepInfo").textContent =
      d.total > 0 ? `${d.step} / ${d.total}` : "-- / --";

    const phaseNames = [
      "تحريك Y (رفع/تموضع)",
      "تحريك X أفقيًا",
      "تحريك Z (دخول/خروج)"
    ];

    const zMode =
      d.zdir > 0 ? "دخول: Y → X → Z" :
      d.zdir < 0 ? "خروج: Y → Z → X" :
                   "Z ثابت: Y → X → Z";

    document.getElementById("phaseInfo").textContent =
      `${zMode} | الآن: ${phaseNames[d.phase] || "--"}`;

    const stateEl = document.getElementById("state");

    if(d.error){
      stateEl.textContent = "خطأ - محور توقف قبل الوصول";
      stateEl.className = "stError";
    } else if(d.running){
      stateEl.textContent = "جارٍ تنفيذ الحركة الآمنة";
      stateEl.className = "stRun";
    } else if(d.finished){
      stateEl.textContent = "تم تنفيذ جميع الخطوات";
      stateEl.className = "stDone";
    } else {
      stateEl.textContent = "متوقف / جاهز";
      stateEl.className = "stIdle";
    }

  }catch(e){
  }finally{
    busy = false;
  }
}

refreshStatus();
setInterval(refreshStatus, 600);
</script>

</body>
</html>
)HTML";

// =====================================================
// HTTP
// =====================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStart() {
  startSequence();
  server.send(200, "text/plain", "STARTED");
}

void handleStop() {
  stopSequence();
  server.send(200, "text/plain", "STOPPED");
}

void handleSpeed() {
  int v = server.arg("value").toInt();

  if (v >= 500 && v <= 8000) {
    stepIntervalUs = v;
  }

  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  String json;
  json.reserve(340);

  Waypoint &wp = waypoints[currentStep < TOTAL_STEPS ? currentStep : TOTAL_STEPS - 1];

  json += "{\"running\":";
  json += running ? "true" : "false";

  json += ",\"finished\":";
  json += finishedAll ? "true" : "false";

  json += ",\"error\":";
  json += errorState ? "true" : "false";

  json += ",\"step\":";
  json += String((currentStep < TOTAL_STEPS) ? currentStep + 1 : TOTAL_STEPS);

  json += ",\"total\":";
  json += String(TOTAL_STEPS);

  json += ",\"phase\":";
  json += String((int)currentPhase);

  json += ",\"zdir\":";
  json += String((int)currentZDirection);

  json += ",\"x\":";
  json += String(axes[AX_X].position);

  json += ",\"y\":";
  json += String(axes[AX_Y].position);

  json += ",\"z\":";
  json += String(axes[AX_Z].position);

  json += ",\"tx\":";
  json += String(wp.x);

  json += ",\"ty\":";
  json += String(wp.y);

  json += ",\"tz\":";
  json += String(wp.z);

  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32 SAFE COORDS PLAYER START ===");

  for (int i = 0; i < 3; i++) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    pinMode(axes[i].limitPin, INPUT_PULLUP);

    digitalWrite(axes[i].stepPin, LOW);

    bool raw = readRawLimit(axes[i]);

    axes[i].lastRawLimit = raw;
    axes[i].debouncedLimit = raw;
    axes[i].lastLimitChangeMicros = micros();

    axes[i].activeBlockedJogDir =
      raw ? axes[i].defaultBlockJogDir : 0;

    // IMPORTANT:
    // Start the machine physically at Home / Limits before running.
    axes[i].position = 0;
    axes[i].autoTarget = 0;
    axes[i].autoMoving = false;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/speed", handleSpeed);
  server.on("/status", handleStatus);

  server.begin();

  Serial.println("WEB SERVER READY");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  server.handleClient();

  unsigned long now = micros();

  for (int i = 0; i < 3; i++) {
    updateLimitSwitch(axes[i], now);
  }

  // Only one axis is armed at a time by armAxisTarget().
  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    if (!ax.autoMoving) continue;
    if ((now - ax.lastStepMicros) < stepIntervalUs) continue;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
      continue;
    }

    int8_t realDir = (ax.autoTarget > ax.position) ? 1 : -1;

    // IMPORTANT:
    // Limit switches are NOT allowed to stop automatic movement.
    // They may still be read for monitoring, but RUN ALL continues
    // until the programmed coordinate is reached.
    stepOnce(ax, realDir);
    ax.lastStepMicros = now;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
    }
  }

  updateSequence();
}

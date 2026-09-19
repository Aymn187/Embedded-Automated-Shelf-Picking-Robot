#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// WiFi
// =====================================================
const char* WIFI_SSID     = "Abu Hussain 4G";
const char* WIFI_PASSWORD = "Aa@0508036171";

WebServer server(80);

// =====================================================
// Limit switches - نفس التوصيل المستخدم سابقاً
// NC wiring: COM -> GND , NC -> GPIO
// المنطق الحالي: FREE = LOW , PRESSED = HIGH
// لو الاختبار الفعلي طلع عكس كده، غيّر HIGH إلى LOW هنا.
// =====================================================
const uint8_t LIMIT_PRESSED_LEVEL = HIGH;
const unsigned long LIMIT_DEBOUNCE_US = 8000;

// =====================================================
// المحاور - نفس خريطة البنّات المستخدمة سابقاً
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
  {"X", 13, 15, 25, false,  1, 0, 0, false, false, 0, 0, 0, false},
  {"Y", 16, 17, 33, true,  -1, 0, 0, false, false, 0, 0, 0, false},
  {"Z", 27, 23, 32, false, -1, 0, 0, false, false, 0, 0, 0, false}
};

#define AX_X 0
#define AX_Y 1
#define AX_Z 2

// سرعة الحركة (متغيّرة من الواجهة) - قيمة أصغر = أسرع
unsigned int stepIntervalUs = 5000;

// =====================================================
// قائمة الإحداثيات المطلوب تنفيذها بالترتيب.
// عدّل / أضف / احذف خطوات هنا فقط (نفس بيانات box02).
// =====================================================
struct Waypoint {
  long x;
  long y;
  long z;
};

Waypoint waypoints[] = {
  { -906,  -1899,    0 },
  { -906,  -1899,  565 },
  { -906,  -2308,   -4 },
  { -2935,  -704,   -4 },
  { -2935,  -704,  745 },
  { -2935,   5,   -4 }
};

const int TOTAL_STEPS = sizeof(waypoints) / sizeof(waypoints[0]);

// =====================================================
// حالة التشغيل
// القاعدة الأساسية: X و Y يتحركان معاً أولاً، وبعد
// وصولهما التام يتحرك Z بمفرده. Z لا يتحرك أبداً في نفس
// الوقت مع X أو Y.
// =====================================================
enum Phase : uint8_t {
  PH_XY = 0,  // X و Y يتحركان معاً نحو هدف هذه الخطوة
  PH_Z  = 1   // Z يتحرك بمفرده بعد اكتمال X و Y
};

bool running = false;
bool finishedAll = false;
bool errorState = false;
int currentStep = 0;
Phase currentPhase = PH_XY;

// =====================================================
// دوال مساعدة - نفس منطق الحماية من نهايات الشوط (Limit)
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

  // فقط الاتجاه الذي يدخل أكثر داخل نهاية الشوط الضاغط هو الممنوع.
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

  // FREE -> PRESSED
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

  // PRESSED -> FREE
  if (oldState && !ax.debouncedLimit) {
    ax.activeBlockedJogDir = 0;
    Serial.print("LIMIT RELEASED ");
    Serial.println(ax.name);
  }
}

// =====================================================
// تسليح هدف تلقائي لمحور واحد
// =====================================================
void armAxisTarget(int idx, long target) {
  axes[idx].autoTarget = target;
  axes[idx].autoMoving = (axes[idx].position != target);
}

bool xyReachedTarget() {
  return axes[AX_X].position == axes[AX_X].autoTarget &&
         axes[AX_Y].position == axes[AX_Y].autoTarget;
}

bool xyStillMoving() {
  return axes[AX_X].autoMoving || axes[AX_Y].autoMoving;
}

bool zReachedTarget() {
  return axes[AX_Z].position == axes[AX_Z].autoTarget;
}

bool zStillMoving() {
  return axes[AX_Z].autoMoving;
}

// يجهّز حركة المرحلة الحالية (X&Y معاً، أو Z بمفرده) لنفس الخطوة الحالية.
void armStepPhase() {
  Waypoint& wp = waypoints[currentStep];

  if (currentPhase == PH_XY) {
    axes[AX_Z].autoMoving = false;   // Z ثابت تماماً في هذه المرحلة
    armAxisTarget(AX_X, wp.x);
    armAxisTarget(AX_Y, wp.y);
  } else {
    axes[AX_X].autoMoving = false;   // X و Y ثابتان تماماً في هذه المرحلة
    axes[AX_Y].autoMoving = false;
    armAxisTarget(AX_Z, wp.z);
  }
}

void startSequence() {
  if (finishedAll) {
    // إعادة التشغيل من أول خطوة بعد اكتمال التسلسل بالكامل
    currentStep = 0;
    currentPhase = PH_XY;
    finishedAll = false;
  }

  errorState = false;
  running = true;

  armStepPhase();

  Serial.println("SEQUENCE START/RESUME");
}

void stopSequence() {
  running = false;
  stopAllAxes();

  Serial.println("SEQUENCE STOPPED");
}

void updateSequence() {
  if (!running) return;

  if (currentPhase == PH_XY) {
    if (xyReachedTarget()) {
      currentPhase = PH_Z;
      armStepPhase();
      return;
    }

    if (!xyStillMoving()) {
      running = false;
      errorState = true;
      Serial.println("ERROR: X/Y STOPPED BEFORE TARGET (LIMIT?)");
    }

    return;
  }

  // PH_Z
  if (zReachedTarget()) {
    currentStep++;

    if (currentStep >= TOTAL_STEPS) {
      running = false;
      finishedAll = true;
      Serial.println("SEQUENCE DONE");
      return;
    }

    currentPhase = PH_XY;
    armStepPhase();
    return;
  }

  if (!zStillMoving()) {
    running = false;
    errorState = true;
    Serial.println("ERROR: Z STOPPED BEFORE TARGET (LIMIT?)");
  }
}

// =====================================================
// واجهة الويب - بسيطة: تشغيل / إيقاف + حالة + سرعة
// =====================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 - تشغيل الإحداثيات</title>

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
  width:min(100%,520px);
  background:#1c1c1c;
  border-radius:14px;
  padding:16px;
  margin:12px auto;
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
  min-height:56px;
  border:0;
  border-radius:10px;
  font-size:18px;
  font-weight:bold;
  color:#111;
  touch-action:manipulation;
}

.start{background:#4caf50;color:#fff}
.stop{background:#e53935;color:#fff}

.statLine{
  display:flex;
  justify-content:space-between;
  font-size:14px;
  padding:6px 4px;
  border-bottom:1px solid #2a2a2a;
}

.statLine span:first-child{color:#9e9e9e}

#state{
  font-size:20px;
  font-weight:bold;
  margin:6px 0 14px;
}

.stIdle{color:#bbb}
.stRunXY{color:#4fc3f7}
.stRunZ{color:#ffca28}
.stDone{color:#8bc34a}
.stError{color:#ff5252}

input[type=range]{width:85%}
</style>
</head>

<body>

<h2>تشغيل الإحداثيات المسجّلة</h2>

<div class="card">
  <div id="state" class="stIdle">جاهز</div>

  <div class="row">
    <button class="start" id="startBtn">تشغيل</button>
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
</div>

<div class="card">
  <label>سرعة الحركة: <span id="spdVal">5000</span></label>
  <br><br>
  <input type="range" min="500" max="8000" value="5000" id="spd">
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

    document.getElementById("stepInfo").textContent =
      d.total > 0 ? `${d.step} / ${d.total}` : "-- / --";

    document.getElementById("phaseInfo").textContent =
      d.phase === 0 ? "X و Y معاً" : "Z بمفرده";

    const stateEl = document.getElementById("state");

    if(d.error){
      stateEl.textContent = "خطأ - توقف قبل الوصول (تحقق من الليميت)";
      stateEl.className = "stError";
    } else if(d.running && d.phase === 0){
      stateEl.textContent = "جارٍ تحريك X و Y";
      stateEl.className = "stRunXY";
    } else if(d.running && d.phase === 1){
      stateEl.textContent = "جارٍ تحريك Z";
      stateEl.className = "stRunZ";
    } else if(d.finished){
      stateEl.textContent = "تم تنفيذ كل الخطوات";
      stateEl.className = "stDone";
    } else {
      stateEl.textContent = "متوقف / جاهز";
      stateEl.className = "stIdle";
    }

  }catch(e){
    // تجاهل فشل مؤقت في الاتصال
  }finally{
    busy = false;
  }
}

refreshStatus();
setInterval(refreshStatus, 500);
</script>

</body>
</html>
)HTML";

// =====================================================
// معالجات HTTP
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
  json.reserve(220);

  json += "{\"running\":";
  json += running ? "true" : "false";

  json += ",\"finished\":";
  json += finishedAll ? "true" : "false";

  json += ",\"error\":";
  json += errorState ? "true" : "false";

  json += ",\"step\":";
  json += String(currentStep + 1);

  json += ",\"total\":";
  json += String(TOTAL_STEPS);

  json += ",\"phase\":";
  json += String((int)currentPhase);

  json += ",\"x\":";
  json += String(axes[AX_X].position);

  json += ",\"y\":";
  json += String(axes[AX_Y].position);

  json += ",\"z\":";
  json += String(axes[AX_Z].position);

  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32 COORDS PLAYER START ===");

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

    // ملاحظة مهمة: يفترض البرنامج أن العربة عند الطاقة
    // تكون فعلياً عند نقطة الصفر (0,0,0) لكل محور، لأن
    // الإحداثيات المسجّلة في القائمة مطلقة (Absolute).
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

  // تحديث قراءة الليميت سويتش لكل محور
  for (int i = 0; i < 3; i++) {
    updateLimitSwitch(axes[i], now);
  }

  // تحريك أي محور مسلَّح للحركة التلقائية (autoMoving)
  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    if (!ax.autoMoving) continue;
    if ((now - ax.lastStepMicros) < stepIntervalUs) continue;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
      continue;
    }

    int8_t realDir = (ax.autoTarget > ax.position) ? 1 : -1;
    int8_t rawDir = rawDirFromRealDir(ax, realDir);

    if (!moveAllowed(ax, rawDir)) {
      ax.autoMoving = false;
      continue;
    }

    stepOnce(ax, realDir);
    ax.lastStepMicros = now;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
    }
  }

  // تقدّم تسلسل الخطوات (X&Y معاً ثم Z بمفرده)
  updateSequence();
}

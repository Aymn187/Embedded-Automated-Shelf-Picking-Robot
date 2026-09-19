#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// =====================================================
// WiFi
// =====================================================
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

WebServer server(80);
Preferences prefs;

// =====================================================
// Limit switches
// NC wiring used in your project:
// COM -> GND
// NC  -> GPIO
//
// Current logic:
// FREE    = LOW
// PRESSED = HIGH
//
// If your real RAW test shows the opposite, change HIGH to LOW.
// =====================================================
const uint8_t LIMIT_PRESSED_LEVEL = HIGH;
const unsigned long LIMIT_DEBOUNCE_US = 8000;

// =====================================================
// Axis
// =====================================================
struct Axis {
  const char* name;
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t limitPin;

  bool invertJog;
  int8_t defaultBlockJogDir;
  bool useLimit;

  long position;
  int8_t jogDir;

  unsigned long lastStepMicros;

  bool debouncedLimit;
  bool lastRawLimit;
  unsigned long lastLimitChangeMicros;
  int8_t activeBlockedJogDir;

  long autoTarget;
  bool autoMoving;
};

// Current pin map:
// X: STEP13 - DIR15 - LIMIT25
// Y: STEP16 - DIR17 - LIMIT33
// Z: STEP27 - DIR23 - LIMIT32
Axis axes[3] = {
  {"X", 13, 15, 25, false,  1, true, 0, 0, 0, false, false, 0, 0, 0, false},
  {"Y", 16, 17, 33, true,  -1, true, 0, 0, 0, false, false, 0, 0, 0, false},
  {"Z", 27, 23, 32, false, -1, true, 0, 0, 0, false, false, 0, 0, 0, false}
};

// =====================================================
// Saved positions
// 0 = Shelf 1
// 1 = Shelf 2
// 2 = Shelf 3
// 3 = Delivery
// =====================================================
struct SavedPoint {
  long x;
  long y;
  long z;
  bool valid;
};

SavedPoint points[4];

bool homeReady = false;
unsigned int stepIntervalUs = 5000;

// =====================================================
// Recorded step sequence (Teach mode)
// Start from HOMING, then press "SAVE STEP" as many times
// as needed while jogging (X/Y/Z per step) to build an
// ordered list. Press "RUN SEQUENCE" to replay all steps
// in order automatically, including the final return step.
// =====================================================
#define MAX_STEPS 40

struct StepPoint {
  long x;
  long y;
  long z;
};

StepPoint steps[MAX_STEPS];
int stepCount = 0;
int seqIndex = -1;  // current index while a sequence is running, -1 if idle

// =====================================================
// Retrieval job
// GET SHELF:
// 1) Go to shelf
// 2) Then go to delivery
//
// NOTE: this assumes the mechanical drawer pickup happens
// by your existing mechanism when the carriage reaches the shelf.
// If you later add a separate pickup motor/servo, insert its action
// between JOB_TO_SHELF and JOB_TO_DELIVERY.
// =====================================================
enum JobState : uint8_t {
  JOB_IDLE = 0,
  JOB_TO_SHELF = 1,
  JOB_TO_DELIVERY = 2,
  JOB_DONE = 3,
  JOB_ERROR = 4,
  JOB_SEQUENCE = 5
};

JobState jobState = JOB_IDLE;
int8_t activeShelf = -1;

// =====================================================
// Helpers
// =====================================================
int findAxisIndex(const String& name) {
  for (int i = 0; i < 3; i++) {
    if (name.equalsIgnoreCase(axes[i].name)) return i;
  }
  return -1;
}

bool readRawLimit(Axis& ax) {
  return digitalRead(ax.limitPin) == LIMIT_PRESSED_LEVEL;
}

bool limitHit(Axis& ax) {
  if (!ax.useLimit) return false;
  return ax.debouncedLimit;
}

int8_t actualDir(Axis& ax, int8_t rawDir) {
  return ax.invertJog ? -rawDir : rawDir;
}

int8_t rawDirFromRealDir(Axis& ax, int8_t realDir) {
  return ax.invertJog ? -realDir : realDir;
}

bool moveAllowed(Axis& ax, int8_t rawDir) {
  if (!ax.useLimit) return true;
  if (!limitHit(ax)) return true;

  int8_t blockedDir = ax.activeBlockedJogDir;
  if (blockedDir == 0) blockedDir = ax.defaultBlockJogDir;

  // Only the direction INTO the pressed limit is blocked.
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
  ax.jogDir = 0;
  ax.autoMoving = false;
  digitalWrite(ax.stepPin, LOW);
}

void stopAllAxes() {
  for (int i = 0; i < 3; i++) stopAxis(axes[i]);
}

void cancelJob() {
  for (int i = 0; i < 3; i++) axes[i].autoMoving = false;
  jobState = JOB_IDLE;
  activeShelf = -1;
  seqIndex = -1;
}

// =====================================================
// Limit debounce
// =====================================================
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

    if (ax.jogDir != 0) {
      currentRawDir = ax.jogDir;
    }
    else if (ax.autoMoving && ax.autoTarget != ax.position) {
      int8_t realDir = (ax.autoTarget > ax.position) ? 1 : -1;
      currentRawDir = rawDirFromRealDir(ax, realDir);
    }

    ax.activeBlockedJogDir =
      (currentRawDir != 0) ? currentRawDir : ax.defaultBlockJogDir;

    Serial.print("LIMIT HIT ");
    Serial.print(ax.name);
    Serial.print(" | blocked dir = ");
    Serial.println(ax.activeBlockedJogDir);
  }

  // PRESSED -> FREE
  if (oldState && !ax.debouncedLimit) {
    ax.activeBlockedJogDir = 0;

    Serial.print("LIMIT RELEASED ");
    Serial.println(ax.name);
  }
}

// =====================================================
// NVS / Preferences
// =====================================================
void loadSavedPoints() {
  prefs.begin("asrs", false);

  for (int i = 0; i < 4; i++) {
    String base = "p" + String(i);

    points[i].x = prefs.getLong((base + "x").c_str(), 0);
    points[i].y = prefs.getLong((base + "y").c_str(), 0);
    points[i].z = prefs.getLong((base + "z").c_str(), 0);
    points[i].valid = prefs.getBool((base + "v").c_str(), false);
  }
}

void savePointToNVS(int slot) {
  if (slot < 0 || slot > 3) return;

  String base = "p" + String(slot);

  prefs.putLong((base + "x").c_str(), points[slot].x);
  prefs.putLong((base + "y").c_str(), points[slot].y);
  prefs.putLong((base + "z").c_str(), points[slot].z);
  prefs.putBool((base + "v").c_str(), true);
}

void teachPoint(int slot) {
  if (slot < 0 || slot > 3) return;

  points[slot].x = axes[0].position;
  points[slot].y = axes[1].position;
  points[slot].z = axes[2].position;
  points[slot].valid = true;

  savePointToNVS(slot);
}

void clearAllSavedPoints() {
  prefs.clear();

  for (int i = 0; i < 4; i++) {
    points[i].x = 0;
    points[i].y = 0;
    points[i].z = 0;
    points[i].valid = false;
  }

  stepCount = 0;
  seqIndex = -1;
}

// =====================================================
// Step sequence: NVS load / save
// =====================================================
void loadSteps() {
  stepCount = prefs.getInt("stepCount", 0);

  if (stepCount < 0) stepCount = 0;
  if (stepCount > MAX_STEPS) stepCount = MAX_STEPS;

  for (int i = 0; i < stepCount; i++) {
    String base = "s" + String(i);

    steps[i].x = prefs.getLong((base + "x").c_str(), 0);
    steps[i].y = prefs.getLong((base + "y").c_str(), 0);
    steps[i].z = prefs.getLong((base + "z").c_str(), 0);
  }
}

void saveStepToNVS(int idx) {
  String base = "s" + String(idx);

  prefs.putLong((base + "x").c_str(), steps[idx].x);
  prefs.putLong((base + "y").c_str(), steps[idx].y);
  prefs.putLong((base + "z").c_str(), steps[idx].z);
}

// Appends the CURRENT axis positions as the next step in the list.
bool addStep() {
  if (stepCount >= MAX_STEPS) return false;

  steps[stepCount].x = axes[0].position;
  steps[stepCount].y = axes[1].position;
  steps[stepCount].z = axes[2].position;

  saveStepToNVS(stepCount);

  stepCount++;
  prefs.putInt("stepCount", stepCount);

  return true;
}

bool deleteLastStep() {
  if (stepCount <= 0) return false;

  stepCount--;
  prefs.putInt("stepCount", stepCount);

  return true;
}

void clearSteps() {
  stepCount = 0;
  seqIndex = -1;
  prefs.putInt("stepCount", 0);
}

// =====================================================
// Automatic movement
// =====================================================
void startAutoToPoint(int slot) {
  if (slot < 0 || slot > 3) return;
  if (!points[slot].valid) return;

  for (int i = 0; i < 3; i++) axes[i].jogDir = 0;

  axes[0].autoTarget = points[slot].x;
  axes[1].autoTarget = points[slot].y;
  axes[2].autoTarget = points[slot].z;

  axes[0].autoMoving = (axes[0].position != axes[0].autoTarget);
  axes[1].autoMoving = (axes[1].position != axes[1].autoTarget);
  axes[2].autoMoving = (axes[2].position != axes[2].autoTarget);
}

bool allAxesReachedTarget() {
  for (int i = 0; i < 3; i++) {
    if (axes[i].position != axes[i].autoTarget) return false;
  }
  return true;
}

bool anyAxisAutoMoving() {
  for (int i = 0; i < 3; i++) {
    if (axes[i].autoMoving) return true;
  }
  return false;
}

void startRetrieveShelf(int shelfSlot) {
  if (shelfSlot < 0 || shelfSlot > 2) return;

  activeShelf = shelfSlot;
  jobState = JOB_TO_SHELF;

  startAutoToPoint(shelfSlot);

  Serial.print("GET SHELF ");
  Serial.println(shelfSlot + 1);
}

void updateRetrieveJob() {
  if (jobState == JOB_TO_SHELF) {
    if (allAxesReachedTarget()) {
      Serial.println("SHELF REACHED -> GO DELIVERY");

      // If later you add a pickup/gripper action,
      // place it here before startAutoToPoint(3).

      jobState = JOB_TO_DELIVERY;
      startAutoToPoint(3);
      return;
    }

    if (!anyAxisAutoMoving()) {
      jobState = JOB_ERROR;
      Serial.println("AUTO ERROR: SHELF TARGET NOT REACHED");
      return;
    }
  }

  if (jobState == JOB_TO_DELIVERY) {
    if (allAxesReachedTarget()) {
      jobState = JOB_DONE;
      Serial.println("DELIVERY REACHED");
      return;
    }

    if (!anyAxisAutoMoving()) {
      jobState = JOB_ERROR;
      Serial.println("AUTO ERROR: DELIVERY TARGET NOT REACHED");
      return;
    }
  }
}

// =====================================================
// Step sequence playback
// Moves through steps[0..stepCount-1] in the recorded
// order (homing -> step 1 -> step 2 -> ... -> return step).
// =====================================================
void startAutoToStep(int idx) {
  if (idx < 0 || idx >= stepCount) return;

  for (int i = 0; i < 3; i++) axes[i].jogDir = 0;

  axes[0].autoTarget = steps[idx].x;
  axes[1].autoTarget = steps[idx].y;
  axes[2].autoTarget = steps[idx].z;

  axes[0].autoMoving = (axes[0].position != axes[0].autoTarget);
  axes[1].autoMoving = (axes[1].position != axes[1].autoTarget);
  axes[2].autoMoving = (axes[2].position != axes[2].autoTarget);
}

void startRunSequence() {
  if (stepCount <= 0) return;

  jobState = JOB_SEQUENCE;
  seqIndex = 0;

  startAutoToStep(seqIndex);

  Serial.print("SEQUENCE START, STEPS=");
  Serial.println(stepCount);
}

void updateSequenceJob() {
  if (jobState != JOB_SEQUENCE) return;

  if (allAxesReachedTarget()) {
    Serial.print("STEP REACHED #");
    Serial.println(seqIndex + 1);

    seqIndex++;

    if (seqIndex >= stepCount) {
      jobState = JOB_DONE;
      seqIndex = -1;
      Serial.println("SEQUENCE DONE");
      return;
    }

    startAutoToStep(seqIndex);
    return;
  }

  if (!anyAxisAutoMoving()) {
    jobState = JOB_ERROR;
    Serial.println("SEQUENCE ERROR: STEP NOT REACHED");
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
<title>AS/RS Teach & Retrieve</title>

<style>
*{box-sizing:border-box}

body{
  font-family:Tahoma,Arial,sans-serif;
  background:#111;
  color:#eee;
  text-align:center;
  margin:0;
  padding:12px;
  overflow-x:hidden;
}

h2,h3{
  margin:8px 0;
  line-height:1.25;
  overflow-wrap:anywhere;
}

h2{color:#4fc3f7}

.card{
  width:min(100%,680px);
  background:#1c1c1c;
  border-radius:14px;
  padding:14px;
  margin:12px auto;
  overflow:hidden;
}

.axesGrid{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(190px,1fr));
  gap:10px;
  width:100%;
}

.axisBox{
  min-width:0;
  background:#272727;
  border-radius:12px;
  padding:10px;
  overflow:hidden;
}

.row{
  display:flex;
  flex-wrap:wrap;
  gap:8px;
  justify-content:center;
  align-items:stretch;
  width:100%;
  margin:9px 0;
}

button{
  flex:1 1 135px;
  min-width:0;
  max-width:100%;
  min-height:48px;
  border:0;
  border-radius:10px;
  padding:10px;
  font-size:14px;
  line-height:1.2;
  font-weight:bold;
  background:#4fc3f7;
  color:#111;
  white-space:normal;
  overflow-wrap:anywhere;
  touch-action:manipulation;
}

button:active,.jogBtn.pressed{
  background:#0288d1;
  color:#fff;
}

button:disabled{opacity:.35}

.jogBtn{
  flex:1 1 calc(50% - 8px);
  touch-action:none;
}

.jogBtn.blocked{
  background:#555 !important;
  color:#aaa !important;
}

.stop{background:#e53935;color:#fff}
.home{background:#ffb300;color:#111}
.clearHome{background:#ff9800;color:#111}
.clearSaved{background:#b71c1c;color:#fff}
.save{background:#8bc34a;color:#111}
.get{background:#29b6f6;color:#111}
.saveStep{background:#8bc34a;color:#111}
.runSeq{background:#29b6f6;color:#111}
.deleteStep{background:#ff9800;color:#111}
.clearSteps{background:#b71c1c;color:#fff}

.stepRow{
  display:flex;
  justify-content:space-between;
  gap:8px;
  background:#252525;
  border-radius:8px;
  padding:8px 10px;
  margin:6px 0;
  font-size:12px;
  overflow-wrap:anywhere;
}

.stepRow.active{
  background:#0d3a52;
  color:#8be9fd;
  font-weight:bold;
}

.pos{
  color:#b8e35d;
  font-weight:bold;
  overflow-wrap:anywhere;
}

.limit{
  display:inline-block;
  max-width:100%;
  border-radius:7px;
  padding:5px 8px;
  font-size:12px;
  margin:5px 0;
  overflow-wrap:anywhere;
}

.limit.free{background:#2e7d32;color:#fff}
.limit.hit{background:#e53935;color:#fff;font-weight:bold}

.raw{
  font-size:11px;
  color:#bbb;
  margin-bottom:6px;
  overflow-wrap:anywhere;
}

.small{
  font-size:12px;
  color:#bbb;
  line-height:1.5;
  overflow-wrap:anywhere;
}

.point{
  width:100%;
  min-width:0;
  background:#252525;
  border-radius:10px;
  padding:10px;
  margin:8px 0;
  overflow:hidden;
}

.pointInfo{
  font-size:12px;
  color:#ccc;
  margin-top:5px;
  overflow-wrap:anywhere;
}

#message{
  min-height:24px;
  font-size:14px;
  line-height:1.4;
  color:#ffd54f;
  margin:8px 0;
  overflow-wrap:anywhere;
}

#jobStatus{
  margin-top:8px;
  font-weight:bold;
  line-height:1.4;
  overflow-wrap:anywhere;
}

.statusOk{color:#9ccc65}
.statusBad{color:#ff8a80}

input[type=range]{max-width:100%}

@media(max-width:520px){
  .axesGrid{grid-template-columns:1fr}

  button{
    flex-basis:100%;
    font-size:14px;
  }

  .jogBtn{
    flex-basis:calc(50% - 8px);
  }
}
</style>
</head>

<body>

<h2>AS/RS - Teach & Retrieve</h2>

<div class="card">
  <div id="homeState" class="statusBad">HOME غير محدد</div>

  <div class="small">
    ضع المحاور عند نقطة البداية ثم اضغط SET HOME.
    بعد فصل الكهرباء يجب تحديد HOME من جديد.
  </div>

  <div class="row">
    <button class="home" id="setHomeBtn">SET HOME</button>
    <button class="clearHome" id="clearHomeBtn">CLEAR HOME</button>
    <button class="stop" id="stopAllBtn">STOP ALL</button>
  </div>

  <div id="message"></div>
  <div id="jobStatus">AUTO: IDLE</div>
</div>

<div class="card">
  <h3>Teach Mode - التحريك اليدوي</h3>
  <div class="axesGrid" id="axes"></div>
</div>

<div class="card">
  <h3>حفظ المواقع</h3>

  <div class="point">
    <strong>رف 1</strong>
    <div class="row">
      <button class="save" data-save="0">SAVE SHELF 1</button>
    </div>
    <div class="pointInfo" id="point0">غير محفوظ</div>
  </div>

  <div class="point">
    <strong>رف 2</strong>
    <div class="row">
      <button class="save" data-save="1">SAVE SHELF 2</button>
    </div>
    <div class="pointInfo" id="point1">غير محفوظ</div>
  </div>

  <div class="point">
    <strong>رف 3</strong>
    <div class="row">
      <button class="save" data-save="2">SAVE SHELF 3</button>
    </div>
    <div class="pointInfo" id="point2">غير محفوظ</div>
  </div>

  <div class="point">
    <strong>نقطة التسليم</strong>
    <div class="row">
      <button class="save" data-save="3">SAVE DELIVERY</button>
    </div>
    <div class="pointInfo" id="point3">غير محفوظ</div>
  </div>

  <div class="row">
    <button class="clearSaved" id="clearSavedBtn">
      CLEAR SAVED LOCATIONS
    </button>
  </div>
</div>

<div class="card">
  <h3>Auto Retrieve - جلب الدرج</h3>

  <div class="small">
    GET SHELF يذهب أولاً إلى موقع الرف المحفوظ،
    وبعد الوصول ينتقل تلقائياً إلى نقطة التسليم المحفوظة.
  </div>

  <div class="row">
    <button class="get" data-get="0">GET SHELF 1</button>
    <button class="get" data-get="1">GET SHELF 2</button>
    <button class="get" data-get="2">GET SHELF 3</button>
  </div>
</div>

<div class="card">
  <h3>تسجيل تسلسل الخطوات (Teach Sequence)</h3>

  <div class="small">
    ابدأ من SET HOME، ثم حرّك المحاور يدوياً إلى أول موضع واضغط
    "حفظ خطوة" لتسجيلها في القائمة. كرر ذلك لكل موضع تالٍ،
    وفي النهاية حرّك المحاور للرجوع إلى نقطة البداية واضغط
    "حفظ خطوة" مرة أخيرة لتسجيل خطوة الرجوع أيضاً.
    اضغط "تشغيل التسلسل" لتنفيذ كل الخطوات تلقائياً بالترتيب.
  </div>

  <div class="row">
    <button class="saveStep" id="saveStepBtn">حفظ خطوة</button>
    <button class="runSeq" id="runSeqBtn">تشغيل التسلسل</button>
  </div>

  <div class="row">
    <button class="deleteStep" id="deleteLastStepBtn">حذف آخر خطوة</button>
    <button class="clearSteps" id="clearStepsBtn">مسح كل الخطوات</button>
  </div>

  <div class="pointInfo" id="stepsSummary">لا توجد خطوات محفوظة بعد</div>
  <div id="stepsList"></div>
</div>

<div class="card">
  <label>
    سرعة الحركة:
    <span id="spdVal">5000</span>
  </label>

  <br><br>

  <input type="range" min="500" max="8000" value="5000" id="spd" style="width:85%">
</div>

<script>
const axesList = ["X","Y","Z"];
const axesContainer = document.getElementById("axes");
const messageEl = document.getElementById("message");

axesList.forEach(a => {
  axesContainer.insertAdjacentHTML(
    "beforeend",
    `
    <div class="axisBox">
      <h3>Axis ${a}</h3>
      <div>Position: <span class="pos" id="pos${a}">0</span></div>
      <div class="limit" id="limit${a}">LIMIT --</div>
      <div class="raw" id="raw${a}">RAW --</div>

      <div class="row">
        <button class="jogBtn" id="jogNeg${a}" data-axis="${a}" data-dir="-1">Direction -</button>
        <button class="jogBtn" id="jogPos${a}" data-axis="${a}" data-dir="1">Direction +</button>
      </div>
    </div>
    `
  );
});

function showMessage(text, ok=false){
  messageEl.textContent = text;
  messageEl.style.color = ok ? "#9ccc65" : "#ffd54f";
}

async function command(url){
  try{
    const r = await fetch(url, {cache:"no-store"});
    const text = await r.text();

    if(!r.ok){
      throw new Error(text || "Request failed");
    }

    return text;
  }catch(e){
    showMessage("فشل الاتصال بالـ ESP32");
    throw e;
  }
}

// Robust jog buttons: one Pointer Event path only.
document.querySelectorAll(".jogBtn").forEach(btn => {
  let active = false;

  btn.addEventListener("pointerdown", e => {
    e.preventDefault();
    if(active) return;

    active = true;

    try{
      btn.setPointerCapture(e.pointerId);
    }catch(_){}

    btn.classList.add("pressed");

    command(
      `/jog?axis=${btn.dataset.axis}&dir=${btn.dataset.dir}`
    ).catch(()=>{});
  });

  const release = () => {
    if(!active) return;

    active = false;
    btn.classList.remove("pressed");

    command(
      `/stopAxis?axis=${btn.dataset.axis}`
    ).catch(()=>{});
  };

  btn.addEventListener("pointerup", release);
  btn.addEventListener("pointercancel", release);
  btn.addEventListener("lostpointercapture", release);
});

// Safety if page loses focus.
window.addEventListener("blur", () => {
  fetch("/stopAll", {cache:"no-store"}).catch(()=>{});
});

document.addEventListener("visibilitychange", () => {
  if(document.hidden){
    fetch("/stopAll", {cache:"no-store"}).catch(()=>{});
  }
});

document.getElementById("stopAllBtn").addEventListener("click", async () => {
  await command("/stopAll").catch(()=>{});
  showMessage("تم إيقاف جميع المحاور", true);
});

document.getElementById("setHomeBtn").addEventListener("click", async () => {
  const r = await command("/setHome").catch(()=>null);

  if(r !== null){
    showMessage("تم ضبط HOME: X=0, Y=0, Z=0", true);
    refreshStatus();
  }
});

document.getElementById("clearHomeBtn").addEventListener("click", async () => {
  const r = await command("/clearHome").catch(()=>null);

  if(r !== null){
    showMessage("تم مسح HOME. حدده من جديد قبل AUTO.", true);
    refreshStatus();
  }
});

document.getElementById("clearSavedBtn").addEventListener("click", async () => {
  const yes = confirm(
    "هل تريد مسح الرفوف الثلاثة ونقطة التسليم المحفوظة؟"
  );

  if(!yes) return;

  const r = await command("/clearSaved").catch(()=>null);

  if(r !== null){
    showMessage("تم مسح جميع المواقع المحفوظة", true);
    refreshStatus();
  }
});

document.querySelectorAll("[data-save]").forEach(btn => {
  btn.addEventListener("click", async () => {
    const r = await command(
      `/savePoint?slot=${btn.dataset.save}`
    ).catch(()=>null);

    if(r !== null){
      showMessage("تم حفظ الموقع", true);
      refreshStatus();
    }
  });
});

document.querySelectorAll("[data-get]").forEach(btn => {
  btn.addEventListener("click", async () => {
    const shelf = Number(btn.dataset.get) + 1;

    const r = await command(
      `/getShelf?slot=${btn.dataset.get}`
    ).catch(()=>null);

    if(r !== null){
      showMessage(
        `بدأ جلب الرف ${shelf} إلى نقطة التسليم`,
        true
      );

      refreshStatus();
    }
  });
});

document.getElementById("saveStepBtn").addEventListener("click", async () => {
  const r = await command("/saveStep").catch(()=>null);

  if(r !== null){
    showMessage("تم حفظ الخطوة في القائمة", true);
    refreshStatus();
  }
});

document.getElementById("deleteLastStepBtn").addEventListener("click", async () => {
  const yes = confirm("هل تريد حذف آخر خطوة تم تسجيلها؟");

  if(!yes) return;

  const r = await command("/deleteLastStep").catch(()=>null);

  if(r !== null){
    showMessage("تم حذف آخر خطوة", true);
    refreshStatus();
  }
});

document.getElementById("clearStepsBtn").addEventListener("click", async () => {
  const yes = confirm("هل تريد مسح كل خطوات التسلسل المسجلة؟");

  if(!yes) return;

  const r = await command("/clearSteps").catch(()=>null);

  if(r !== null){
    showMessage("تم مسح كل الخطوات", true);
    refreshStatus();
  }
});

document.getElementById("runSeqBtn").addEventListener("click", async () => {
  const r = await command("/runSequence").catch(()=>null);

  if(r !== null){
    showMessage("بدأ تشغيل تسلسل الخطوات من البداية", true);
    refreshStatus();
  }
});

// Speed: send only after releasing slider.
const spd = document.getElementById("spd");

spd.addEventListener("input", () => {
  document.getElementById("spdVal").textContent = spd.value;
});

spd.addEventListener("change", () => {
  command(`/speed?value=${spd.value}`).catch(()=>{});
});

// Slow status polling + no overlapping requests.
let statusBusy = false;

async function refreshStatus(){
  if(statusBusy) return;

  statusBusy = true;

  try{
    const r = await fetch("/status", {cache:"no-store"});

    if(!r.ok){
      throw new Error("status");
    }

    const d = await r.json();

    axesList.forEach(a => {
      const info = d.axes[a];

      document.getElementById("pos"+a).textContent = info.pos;

      const limitEl = document.getElementById("limit"+a);

      limitEl.textContent =
        info.limit ? "LIMIT: PRESSED" : "LIMIT: FREE";

      limitEl.classList.toggle("hit", info.limit);
      limitEl.classList.toggle("free", !info.limit);

      document.getElementById("raw"+a).textContent =
        "GPIO RAW: " + info.raw;

      document.getElementById("jogNeg"+a).classList.toggle(
        "blocked",
        info.limit && info.blockedDir === -1
      );

      document.getElementById("jogPos"+a).classList.toggle(
        "blocked",
        info.limit && info.blockedDir === 1
      );
    });

    const homeState = document.getElementById("homeState");

    homeState.textContent =
      d.homeReady ? "HOME جاهز" : "HOME غير محدد";

    homeState.className =
      d.homeReady ? "statusOk" : "statusBad";

    d.points.forEach((p, i) => {
      const el = document.getElementById("point"+i);

      el.textContent =
        p.valid
        ? `X=${p.x} | Y=${p.y} | Z=${p.z}`
        : "غير محفوظ";

      document.querySelector(`[data-save="${i}"]`).disabled =
        !d.homeReady;
    });

    for(let i=0; i<3; i++){
      document.querySelector(`[data-get="${i}"]`).disabled =
        !d.homeReady ||
        !d.points[i].valid ||
        !d.points[3].valid;
    }

    document.getElementById("saveStepBtn").disabled = !d.homeReady;
    document.getElementById("deleteLastStepBtn").disabled = d.stepCount === 0;
    document.getElementById("clearStepsBtn").disabled = d.stepCount === 0;
    document.getElementById("runSeqBtn").disabled =
      !d.homeReady || d.stepCount === 0;

    const stepsSummary = document.getElementById("stepsSummary");
    const stepsList = document.getElementById("stepsList");

    stepsSummary.textContent =
      d.stepCount === 0
      ? "لا توجد خطوات محفوظة بعد"
      : `عدد الخطوات المحفوظة: ${d.stepCount}`;

    stepsList.innerHTML = d.steps.map((s, i) => {
      const active = (d.jobState === 5 && d.seqIndex === i);

      return `
        <div class="stepRow${active ? " active" : ""}">
          <span>خطوة ${i + 1}${active ? " (جارٍ التنفيذ)" : ""}</span>
          <span>X=${s.x} | Y=${s.y} | Z=${s.z}</span>
        </div>
      `;
    }).join("");

    const job = document.getElementById("jobStatus");

    if(d.jobState === 1){
      job.textContent =
        `AUTO: ذاهب إلى الرف ${d.activeShelf + 1}`;
    }
    else if(d.jobState === 2){
      job.textContent =
        "AUTO: ذاهب إلى نقطة التسليم";
    }
    else if(d.jobState === 5){
      job.textContent =
        `AUTO: تنفيذ التسلسل - خطوة ${d.seqIndex + 1} من ${d.stepCount}`;
    }
    else if(d.jobState === 3){
      job.textContent =
        "AUTO: تم الوصول إلى الهدف";
    }
    else if(d.jobState === 4){
      job.textContent =
        "AUTO: خطأ - توقف قبل الوصول";
    }
    else{
      job.textContent =
        "AUTO: IDLE";
    }

  }catch(e){
    showMessage("الاتصال بالواجهة غير مستقر");
  }finally{
    statusBusy = false;
  }
}

refreshStatus();
setInterval(refreshStatus, 700);
</script>

</body>
</html>
)HTML";

// =====================================================
// HTTP handlers
// =====================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleJog() {
  int idx = findAxisIndex(server.arg("axis"));
  int dir = server.arg("dir").toInt();

  if (idx < 0 || (dir != 1 && dir != -1)) {
    server.send(400, "text/plain", "BAD REQUEST");
    return;
  }

  cancelJob();

  axes[idx].autoMoving = false;
  axes[idx].jogDir = dir;

  server.send(200, "text/plain", "OK");
}

void handleStopAxis() {
  int idx = findAxisIndex(server.arg("axis"));

  if (idx < 0) {
    server.send(400, "text/plain", "BAD AXIS");
    return;
  }

  // Safety: manual stop cancels the automatic retrieval job.
  cancelJob();
  stopAxis(axes[idx]);

  server.send(200, "text/plain", "OK");
}

void handleStopAll() {
  cancelJob();
  stopAllAxes();

  server.send(200, "text/plain", "OK");
}

void handleSetHome() {
  cancelJob();
  stopAllAxes();

  for (int i = 0; i < 3; i++) {
    axes[i].position = 0;
    axes[i].autoTarget = 0;
  }

  homeReady = true;

  Serial.println("HOME SET: X=0 Y=0 Z=0");

  server.send(200, "text/plain", "HOME SET");
}

void handleClearHome() {
  cancelJob();
  stopAllAxes();

  // Saved shelves stay stored. Only the current reference is invalidated.
  homeReady = false;

  Serial.println("HOME CLEARED");

  server.send(200, "text/plain", "HOME CLEARED");
}

void handleClearSaved() {
  cancelJob();
  stopAllAxes();

  clearAllSavedPoints();

  Serial.println("ALL SAVED LOCATIONS CLEARED");

  server.send(200, "text/plain", "SAVED LOCATIONS CLEARED");
}

void handleSavePoint() {
  if (!homeReady) {
    server.send(409, "text/plain", "SET HOME FIRST");
    return;
  }

  int slot = server.arg("slot").toInt();

  if (slot < 0 || slot > 3) {
    server.send(400, "text/plain", "BAD SLOT");
    return;
  }

  teachPoint(slot);

  Serial.print("SAVED POINT ");
  Serial.print(slot);
  Serial.print(" -> X=");
  Serial.print(points[slot].x);
  Serial.print(" Y=");
  Serial.print(points[slot].y);
  Serial.print(" Z=");
  Serial.println(points[slot].z);

  server.send(200, "text/plain", "SAVED");
}

void handleSaveStep() {
  if (!homeReady) {
    server.send(409, "text/plain", "SET HOME FIRST");
    return;
  }

  if (!addStep()) {
    server.send(409, "text/plain", "STEP LIST FULL");
    return;
  }

  Serial.print("STEP SAVED #");
  Serial.print(stepCount);
  Serial.print(" -> X=");
  Serial.print(steps[stepCount - 1].x);
  Serial.print(" Y=");
  Serial.print(steps[stepCount - 1].y);
  Serial.print(" Z=");
  Serial.println(steps[stepCount - 1].z);

  server.send(200, "text/plain", "STEP SAVED");
}

void handleDeleteLastStep() {
  // Safety: don't remove a step while it is being played back.
  cancelJob();
  stopAllAxes();

  if (!deleteLastStep()) {
    server.send(404, "text/plain", "NO STEPS");
    return;
  }

  Serial.print("LAST STEP DELETED, REMAINING=");
  Serial.println(stepCount);

  server.send(200, "text/plain", "STEP DELETED");
}

void handleClearSteps() {
  cancelJob();
  stopAllAxes();

  clearSteps();

  Serial.println("STEP SEQUENCE CLEARED");

  server.send(200, "text/plain", "STEPS CLEARED");
}

void handleRunSequence() {
  if (!homeReady) {
    server.send(409, "text/plain", "SET HOME FIRST");
    return;
  }

  if (stepCount <= 0) {
    server.send(404, "text/plain", "NO STEPS SAVED");
    return;
  }

  cancelJob();
  stopAllAxes();

  startRunSequence();

  server.send(200, "text/plain", "SEQUENCE STARTED");
}

void handleGetShelf() {
  if (!homeReady) {
    server.send(409, "text/plain", "SET HOME FIRST");
    return;
  }

  int slot = server.arg("slot").toInt();

  if (slot < 0 || slot > 2) {
    server.send(400, "text/plain", "BAD SHELF");
    return;
  }

  if (!points[slot].valid) {
    server.send(404, "text/plain", "SHELF NOT SAVED");
    return;
  }

  if (!points[3].valid) {
    server.send(404, "text/plain", "DELIVERY NOT SAVED");
    return;
  }

  cancelJob();
  stopAllAxes();

  startRetrieveShelf(slot);

  server.send(200, "text/plain", "RETRIEVE STARTED");
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
  json.reserve(900);

  json += "{\"homeReady\":";
  json += homeReady ? "true" : "false";

  json += ",\"jobState\":";
  json += String((int)jobState);

  json += ",\"activeShelf\":";
  json += String((int)activeShelf);

  json += ",\"axes\":{";

  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    int8_t blockedDirection = ax.activeBlockedJogDir;
    if (blockedDirection == 0) {
      blockedDirection = ax.defaultBlockJogDir;
    }

    json += "\"";
    json += ax.name;
    json += "\":{";

    json += "\"pos\":";
    json += String(ax.position);

    json += ",\"limit\":";
    json += limitHit(ax) ? "true" : "false";

    json += ",\"raw\":";
    json += String(digitalRead(ax.limitPin));

    json += ",\"blockedDir\":";
    json += String(blockedDirection);

    json += ",\"auto\":";
    json += ax.autoMoving ? "true" : "false";

    json += "}";

    if (i < 2) json += ",";
  }

  json += "},\"points\":[";

  for (int i = 0; i < 4; i++) {
    json += "{";

    json += "\"x\":";
    json += String(points[i].x);

    json += ",\"y\":";
    json += String(points[i].y);

    json += ",\"z\":";
    json += String(points[i].z);

    json += ",\"valid\":";
    json += points[i].valid ? "true" : "false";

    json += "}";

    if (i < 3) json += ",";
  }

  json += "],\"stepCount\":";
  json += String(stepCount);

  json += ",\"seqIndex\":";
  json += String(seqIndex);

  json += ",\"steps\":[";

  for (int i = 0; i < stepCount; i++) {
    json += "{\"x\":";
    json += String(steps[i].x);

    json += ",\"y\":";
    json += String(steps[i].y);

    json += ",\"z\":";
    json += String(steps[i].z);

    json += "}";

    if (i < stepCount - 1) json += ",";
  }

  json += "]}";

  server.send(200, "application/json", json);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== AS/RS TEACH & RETRIEVE START ===");

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

    axes[i].position = 0;
    axes[i].jogDir = 0;
    axes[i].autoTarget = 0;
    axes[i].autoMoving = false;
  }

  loadSavedPoints();
  loadSteps();

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
  server.on("/jog", handleJog);
  server.on("/stopAxis", handleStopAxis);
  server.on("/stopAll", handleStopAll);
  server.on("/setHome", handleSetHome);
  server.on("/clearHome", handleClearHome);
  server.on("/clearSaved", handleClearSaved);
  server.on("/savePoint", handleSavePoint);
  server.on("/saveStep", handleSaveStep);
  server.on("/deleteLastStep", handleDeleteLastStep);
  server.on("/clearSteps", handleClearSteps);
  server.on("/runSequence", handleRunSequence);
  server.on("/getShelf", handleGetShelf);
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

  // Update limit switches.
  for (int i = 0; i < 3; i++) {
    updateLimitSwitch(axes[i], now);
  }

  // Move axes.
  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    if ((now - ax.lastStepMicros) < stepIntervalUs) {
      continue;
    }

    // Manual jog
    if (ax.jogDir != 0) {
      int8_t rawDir = ax.jogDir;

      if (!moveAllowed(ax, rawDir)) {
        ax.jogDir = 0;
        continue;
      }

      int8_t realDir = actualDir(ax, rawDir);

      stepOnce(ax, realDir);
      ax.lastStepMicros = now;

      continue;
    }

    // Automatic target
    if (ax.autoMoving) {
      if (ax.position == ax.autoTarget) {
        ax.autoMoving = false;
        continue;
      }

      int8_t realDir =
        (ax.autoTarget > ax.position) ? 1 : -1;

      int8_t rawDir =
        rawDirFromRealDir(ax, realDir);

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
  }

  // Continue automatic Shelf -> Delivery sequence.
  updateRetrieveJob();

  // Continue automatic recorded step-sequence playback.
  updateSequenceJob();
}

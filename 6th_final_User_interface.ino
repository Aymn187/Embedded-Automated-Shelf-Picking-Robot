#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// =====================================================
// قرارات اتاخدت أثناء الدمج (راجعها قبل الرفع على الهاردوير):
//
// 1) توصيل الليمت سويتش لمحوري X و Z: box001.ino و box03 متفقين
//    على X=GPIO32 و Z=GPIO25، بينما box02.ino وحده كان معكوس
//    (X=GPIO25 و Z=GPIO32). اتفرض هنا إن الغالبية (box001+box03)
//    هي الصح. لو غلط، غيّر limitPin بس تحت في axes[].
//
// 2) الرف 3 (من box03.ino) مفيهوش إحداثيات ثابتة جوه الكود أصلاً،
//    هو مبني على "تعليم" (Teach): تتحرك يدوي بالـjog لحد ما توصل
//    لمكان الرف، وتضغط "Save as Shelf 3"، وكذلك لمكان التسليم
//    وتضغط "Save as Delivery". اتسابت بنفس الفكرة هنا.
//
// 3) زر "Retrieve" (استرجاع) بيرجّع الصندوق: بينفّذ نفس مسار
//    "Fetch" بالظبط لكن بالعكس (من آخر نقطة للأولى).
// =====================================================

// =====================================================
// WiFi
// =====================================================
const char* WIFI_SSID     = "Abu Hussain 4G";
const char* WIFI_PASSWORD = "Aa@0508036171";

WebServer server(80);
Preferences prefs;

// =====================================================
// Limit switches
// NC wiring: COM -> GND , NC -> GPIO
// FREE = LOW , PRESSED = HIGH
// =====================================================
const uint8_t LIMIT_PRESSED_LEVEL = HIGH;
const unsigned long LIMIT_DEBOUNCE_US = 8000;

// =====================================================
// المحاور
// X: STEP13 - DIR15 - LIMIT32
// Y: STEP16 - DIR17 - LIMIT33
// Z: STEP27 - DIR23 - LIMIT25
// =====================================================
struct Axis {
  const char* name;
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t limitPin;

  bool invertJog;
  int8_t defaultBlockJogDir;   // نفس اتجاه الهومنج (الاتجاه اللي بيدخل جوه الليمت)

  long position;
  int8_t jogDir;                // يدوي - يستخدم في تعليم الرف 3 فقط، بيحترم الليمت
  unsigned long lastStepMicros;

  bool debouncedLimit;
  bool lastRawLimit;
  unsigned long lastLimitChangeMicros;
  int8_t activeBlockedJogDir;

  long autoTarget;
  bool autoMoving;               // تنفيذ تلقائي (Fetch/Retrieve/Homing)
};

#define AX_X 0
#define AX_Y 1
#define AX_Z 2

Axis axes[3] = {
  {"X", 13, 15, 32, false,  1, 0, 0, 0, false, false, 0, 0, 0, false},
  {"Y", 16, 17, 33, true,  -1, 0, 0, 0, false, false, 0, 0, 0, false},
  {"Z", 27, 23, 25, false, -1, 0, 0, 0, false, false, 0, 0, 0, false}
};

unsigned int stepIntervalUs = 5000;

// =====================================================
// Waypoints
// =====================================================
struct Waypoint {
  long x;
  long y;
  long z;
};

// الرف 1 - من box001.ino
Waypoint shelf1Waypoints[] = {
  {  -187,  -1893,    0 },
  {  -187,  -1893,  666 },
  {  -187,  -2315,  666 },
  {  -187,  -2315,   -1 },
  {  2545,   -837,   -1 },
  {  2545,   -837,  653 },
  {  2545,    -12,    4 }
};
const int SHELF1_STEPS = sizeof(shelf1Waypoints) / sizeof(shelf1Waypoints[0]);

// الرف 2 - من box02.ino
Waypoint shelf2Waypoints[] = {
  {  -187,  -1893,    0 },
  {  -187,  -1893,  666 },
  {  -187,  -2315,  666 },
  {  -187,  -2315,   99 },
  {  2552,   -837,   -1 },
  {  2577,   -837,  653 },
  {  2577,    -12,    4 }
};
const int SHELF2_STEPS = sizeof(shelf2Waypoints) / sizeof(shelf2Waypoints[0]);

// الرف 3 - Teach mode: نقطتين بس (رف 3 -> تسليم)، بيتبنوا وقت
// التشغيل من shelf3Point و deliveryPoint المتعلّمين.
Waypoint shelf3Waypoints[2];

struct TaughtPoint {
  long x;
  long y;
  long z;
  bool valid;
};

TaughtPoint shelf3Point   = {0, 0, 0, false};
TaughtPoint deliveryPoint = {0, 0, 0, false};

// =====================================================
// حالة عامة
// =====================================================
enum Phase : uint8_t {
  PH_XY = 0,
  PH_Z  = 1
};

bool homeDone = false;   // لازم يتعمل Home مرة على الأقل قبل أي Fetch/Retrieve/تعليم

// =====================================================
// دوال مساعدة - ليمت سويتش
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
  ax.jogDir = 0;
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

    if (ax.jogDir != 0) {
      currentRawDir = ax.jogDir;
    } else if (ax.autoMoving && ax.autoTarget != ax.position) {
      currentRawDir = (ax.autoTarget > ax.position) ? 1 : -1;
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
// Homing - الليمت سويتش يوقف الحركة هنا وفي التعليم اليدوي بس.
// أي تنفيذ تلقائي (Fetch/Retrieve) بيتجاهل الليمت تماماً.
// =====================================================
bool homingActive = false;

enum HomingPhase : uint8_t {
  HOME_IDLE = 0,
  HOME_Z    = 1,
  HOME_XY   = 2,
  HOME_DONE = 3
};

HomingPhase homingPhase = HOME_IDLE;

void startHoming() {
  if (homingActive) return;

  stopAllAxes();

  homingActive = true;
  homingPhase = HOME_Z;

  axes[AX_Z].autoMoving = true;
  axes[AX_X].autoMoving = false;
  axes[AX_Y].autoMoving = false;

  Serial.println("HOMING START");
}

void finishHomingAxis(int idx) {
  axes[idx].position = 0;
  axes[idx].autoTarget = 0;
  axes[idx].autoMoving = false;
}

void updateHoming(unsigned long now) {
  if (!homingActive) return;

  if (homingPhase == HOME_Z) {
    Axis& z = axes[AX_Z];

    if (limitHit(z)) {
      finishHomingAxis(AX_Z);
      homingPhase = HOME_XY;
      axes[AX_X].autoMoving = true;
      axes[AX_Y].autoMoving = true;
      Serial.println("Z HOMED");
      return;
    }

    if ((now - z.lastStepMicros) >= stepIntervalUs) {
      stepOnce(z, z.defaultBlockJogDir);
      z.lastStepMicros = now;
    }
    return;
  }

  if (homingPhase == HOME_XY) {
    Axis& x = axes[AX_X];
    Axis& y = axes[AX_Y];

    if (x.autoMoving) {
      if (limitHit(x)) {
        finishHomingAxis(AX_X);
        Serial.println("X HOMED");
      } else if ((now - x.lastStepMicros) >= stepIntervalUs) {
        stepOnce(x, x.defaultBlockJogDir);
        x.lastStepMicros = now;
      }
    }

    if (y.autoMoving) {
      if (limitHit(y)) {
        finishHomingAxis(AX_Y);
        Serial.println("Y HOMED");
      } else if ((now - y.lastStepMicros) >= stepIntervalUs) {
        stepOnce(y, y.defaultBlockJogDir);
        y.lastStepMicros = now;
      }
    }

    if (!x.autoMoving && !y.autoMoving) {
      homingPhase = HOME_DONE;
      homingActive = false;
      homeDone = true;
      Serial.println("HOMING DONE");
    }
    return;
  }
}

// =====================================================
// تنفيذ تلقائي عام (Fixed Job) - بيشتغل به Fetch/Retrieve
// لأي رف من التلاتة (1 و 2 بمصفوفات ثابتة، 3 بمصفوفة متبنية
// من النقط المتعلّمة وقت الطلب).
// =====================================================
struct FixedJob {
  Waypoint* points;
  int count;
  int8_t direction;   // +1 = Fetch (من الأول للآخر) , -1 = Retrieve (بالعكس)
  int currentIndex;
  Phase phase;
};

FixedJob job;
bool jobActive = false;
bool jobIsFetch = true;
bool jobError = false;
bool jobFinished = false;
int activeShelfNum = 0;

void armAxisTarget(int idx, long target) {
  axes[idx].autoTarget = target;
  axes[idx].autoMoving = (axes[idx].position != target);
}

void armFixedPhase() {
  Waypoint& wp = job.points[job.currentIndex];

  if (job.phase == PH_XY) {
    axes[AX_Z].autoMoving = false;
    armAxisTarget(AX_X, wp.x);
    armAxisTarget(AX_Y, wp.y);
  } else {
    axes[AX_X].autoMoving = false;
    axes[AX_Y].autoMoving = false;
    armAxisTarget(AX_Z, wp.z);
  }
}

bool startFixedJob(Waypoint* pts, int count, int8_t direction, int shelfNum, bool isFetch) {
  if (homingActive || jobActive || !homeDone) return false;

  job.points = pts;
  job.count = count;
  job.direction = direction;
  job.currentIndex = (direction > 0) ? 0 : (count - 1);
  job.phase = PH_XY;

  jobActive = true;
  activeShelfNum = shelfNum;
  jobIsFetch = isFetch;
  jobError = false;
  jobFinished = false;

  armFixedPhase();

  Serial.print(isFetch ? "FETCH" : "RETRIEVE");
  Serial.print(" START shelf=");
  Serial.println(shelfNum);

  return true;
}

void updateFixedJob() {
  if (!jobActive) return;

  if (job.phase == PH_XY) {
    bool xyReached = axes[AX_X].position == axes[AX_X].autoTarget &&
                      axes[AX_Y].position == axes[AX_Y].autoTarget;

    if (xyReached) {
      job.phase = PH_Z;
      armFixedPhase();
      return;
    }

    if (!axes[AX_X].autoMoving && !axes[AX_Y].autoMoving) {
      jobActive = false;
      jobError = true;
      Serial.println("JOB ERROR: XY STOPPED BEFORE TARGET");
    }
    return;
  }

  // PH_Z
  bool zReached = axes[AX_Z].position == axes[AX_Z].autoTarget;

  if (zReached) {
    job.currentIndex += job.direction;

    bool done = (job.direction > 0) ? (job.currentIndex >= job.count)
                                     : (job.currentIndex < 0);

    if (done) {
      jobActive = false;
      jobFinished = true;
      Serial.println("JOB DONE");
      return;
    }

    job.phase = PH_XY;
    armFixedPhase();
    return;
  }

  if (!axes[AX_Z].autoMoving) {
    jobActive = false;
    jobError = true;
    Serial.println("JOB ERROR: Z STOPPED BEFORE TARGET");
  }
}

// =====================================================
// NVS - حفظ نقاط تعليم الرف 3 والتسليم
// =====================================================
void loadTaughtFromNVS() {
  prefs.begin("picking", false);

  shelf3Point.x = prefs.getLong("s3x", 0);
  shelf3Point.y = prefs.getLong("s3y", 0);
  shelf3Point.z = prefs.getLong("s3z", 0);
  shelf3Point.valid = prefs.getBool("s3v", false);

  deliveryPoint.x = prefs.getLong("dvx", 0);
  deliveryPoint.y = prefs.getLong("dvy", 0);
  deliveryPoint.z = prefs.getLong("dvz", 0);
  deliveryPoint.valid = prefs.getBool("dvv", false);
}

void saveTaughtToNVS() {
  prefs.putLong("s3x", shelf3Point.x);
  prefs.putLong("s3y", shelf3Point.y);
  prefs.putLong("s3z", shelf3Point.z);
  prefs.putBool("s3v", shelf3Point.valid);

  prefs.putLong("dvx", deliveryPoint.x);
  prefs.putLong("dvy", deliveryPoint.y);
  prefs.putLong("dvz", deliveryPoint.z);
  prefs.putBool("dvv", deliveryPoint.valid);
}

// =====================================================
// واجهة الويب
// =====================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Embedded Atomated shelf picking robot</title>

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
}

.row{
  display:flex;
  flex-wrap:wrap;
  gap:10px;
  justify-content:center;
  margin:10px 0;
}

button{
  flex:1 1 120px;
  min-height:50px;
  border:0;
  border-radius:10px;
  font-size:16px;
  font-weight:bold;
  color:#111;
  touch-action:manipulation;
}

button:disabled{
  opacity:0.35;
}

.home{background:#4fc3f7;color:#111}
.stop{background:#e53935;color:#fff}
.fetch{background:#4caf50;color:#fff}
.retrieve{background:#ffca28;color:#111}
.teach{background:#ab47bc;color:#fff}
.jog{background:#37474f;color:#fff}

table{
  width:100%;
  border-collapse:collapse;
  font-size:14px;
}

th,td{
  padding:8px 4px;
  border-bottom:1px solid #2a2a2a;
  text-align:center;
}

th{color:#9e9e9e;font-weight:normal}

.jogGrid{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:8px;
  margin:10px 0;
}

.statLine{
  display:flex;
  justify-content:space-between;
  font-size:14px;
  padding:6px 4px;
  border-bottom:1px solid #2a2a2a;
}

.statLine span:first-child{color:#9e9e9e}

#state{
  font-size:18px;
  font-weight:bold;
  margin:6px 0 14px;
}

.stIdle{color:#bbb}
.stRun{color:#4fc3f7}
.stDone{color:#8bc34a}
.stError{color:#ff5252}
.stHoming{color:#4fc3f7}

input[type=range]{width:85%}
</style>
</head>

<body>

<h2>Embedded Atomated shelf picking robot</h2>

<div class="card">
  <div id="state" class="stIdle">جاهز</div>
  <div class="row">
    <button class="home" id="homeBtn">Home</button>
    <button class="stop" id="stopBtn">Stop</button>
  </div>
</div>

<div class="card">
  <h3>Shelves</h3>
  <table>
    <thead>
      <tr><th>#</th><th>Component</th><th>Fetch</th><th>Retrieve</th></tr>
    </thead>
    <tbody>
      <tr>
        <td>1</td><td>Capacitor</td>
        <td><button class="fetch" data-shelf="1">Fetch</button></td>
        <td><button class="retrieve" data-shelf="1">Retrieve</button></td>
      </tr>
      <tr>
        <td>2</td><td>Resistor</td>
        <td><button class="fetch" data-shelf="2">Fetch</button></td>
        <td><button class="retrieve" data-shelf="2">Retrieve</button></td>
      </tr>
      <tr>
        <td>3</td><td>Diode</td>
        <td><button class="fetch" data-shelf="3">Fetch</button></td>
        <td><button class="retrieve" data-shelf="3">Retrieve</button></td>
      </tr>
      <tr><td>4</td><td>-</td><td><button class="fetch" disabled>Fetch</button></td><td><button class="retrieve" disabled>Retrieve</button></td></tr>
      <tr><td>5</td><td>-</td><td><button class="fetch" disabled>Fetch</button></td><td><button class="retrieve" disabled>Retrieve</button></td></tr>
      <tr><td>6</td><td>-</td><td><button class="fetch" disabled>Fetch</button></td><td><button class="retrieve" disabled>Retrieve</button></td></tr>
    </tbody>
  </table>
</div>

<div class="card">
  <h3>الحالة الحالية</h3>
  <div class="statLine"><span>X</span><span id="posX">0</span></div>
  <div class="statLine"><span>Y</span><span id="posY">0</span></div>
  <div class="statLine"><span>Z</span><span id="posZ">0</span></div>
</div>

<div class="card">
  <label>سرعة الحركة: <span id="spdVal">5000</span></label>
  <br><br>
  <input type="range" min="500" max="8000" value="5000" id="spd">
</div>

<div class="card">
  <h3>تعليم موقع الرف 3 والتسليم (Teach Mode)</h3>
  <div class="jogGrid">
    <div></div>
    <button class="jog" data-axis="Z" data-dir="1">Z+</button>
    <div></div>

    <button class="jog" data-axis="X" data-dir="-1">X-</button>
    <button class="jog" data-axis="Y" data-dir="1">Y+</button>
    <button class="jog" data-axis="X" data-dir="1">X+</button>

    <div></div>
    <button class="jog" data-axis="Y" data-dir="-1">Y-</button>
    <div></div>

    <div></div>
    <button class="jog" data-axis="Z" data-dir="-1">Z-</button>
    <div></div>
  </div>

  <div class="row">
    <button class="teach" id="saveShelf3Btn">Save as Shelf 3</button>
    <button class="teach" id="saveDeliveryBtn">Save as Delivery</button>
  </div>

  <div class="statLine"><span>Shelf 3</span><span id="shelf3Info">--</span></div>
  <div class="statLine"><span>Delivery</span><span id="deliveryInfo">--</span></div>
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

document.getElementById("homeBtn").addEventListener("click", () => command("/home"));
document.getElementById("stopBtn").addEventListener("click", () => command("/stop"));

document.querySelectorAll(".fetch[data-shelf]").forEach(btn => {
  btn.addEventListener("click", () => command(`/fetch?shelf=${btn.dataset.shelf}`));
});

document.querySelectorAll(".retrieve[data-shelf]").forEach(btn => {
  btn.addEventListener("click", () => command(`/retrieve?shelf=${btn.dataset.shelf}`));
});

document.querySelectorAll(".jog").forEach(btn => {
  const start = () => command(`/jog?axis=${btn.dataset.axis}&dir=${btn.dataset.dir}`);
  const stop  = () => command(`/jog?axis=${btn.dataset.axis}&dir=0`);

  btn.addEventListener("pointerdown", start);
  btn.addEventListener("pointerup", stop);
  btn.addEventListener("pointerleave", stop);
  btn.addEventListener("pointercancel", stop);
});

document.getElementById("saveShelf3Btn").addEventListener("click", () => command("/saveShelf3"));
document.getElementById("saveDeliveryBtn").addEventListener("click", () => command("/saveDelivery"));

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

    document.getElementById("shelf3Info").textContent =
      d.shelf3Valid ? `X=${d.shelf3.x} Y=${d.shelf3.y} Z=${d.shelf3.z}` : "لسه ما اتسجلش";

    document.getElementById("deliveryInfo").textContent =
      d.deliveryValid ? `X=${d.delivery.x} Y=${d.delivery.y} Z=${d.delivery.z}` : "لسه ما اتسجلش";

    const stateEl = document.getElementById("state");

    if(d.homing){
      stateEl.textContent = "جارٍ الهومنج";
      stateEl.className = "stHoming";
    } else if(d.jobActive){
      stateEl.textContent = (d.jobIsFetch ? "Fetch" : "Retrieve") + " - رف " + d.jobShelf;
      stateEl.className = "stRun";
    } else if(d.jobError){
      stateEl.textContent = "خطأ في آخر عملية";
      stateEl.className = "stError";
    } else if(!d.homed){
      stateEl.textContent = "محتاج هومنج الأول";
      stateEl.className = "stIdle";
    } else if(d.jobFinished){
      stateEl.textContent = "تمت العملية بنجاح";
      stateEl.className = "stDone";
    } else {
      stateEl.textContent = "جاهز";
      stateEl.className = "stDone";
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

void handleHome() {
  startHoming();
  server.send(200, "text/plain", "HOMING");
}

void handleStop() {
  jobActive = false;
  homingActive = false;
  homingPhase = HOME_IDLE;

  for (int i = 0; i < 3; i++) axes[i].jogDir = 0;

  stopAllAxes();

  server.send(200, "text/plain", "STOPPED");
}

void handleFetch() {
  int shelf = server.arg("shelf").toInt();
  bool ok = false;

  if (shelf == 1) {
    ok = startFixedJob(shelf1Waypoints, SHELF1_STEPS, 1, 1, true);
  } else if (shelf == 2) {
    ok = startFixedJob(shelf2Waypoints, SHELF2_STEPS, 1, 2, true);
  } else if (shelf == 3) {
    if (!shelf3Point.valid || !deliveryPoint.valid) {
      server.send(409, "text/plain", "SHELF3 NOT TAUGHT");
      return;
    }
    shelf3Waypoints[0] = { shelf3Point.x, shelf3Point.y, shelf3Point.z };
    shelf3Waypoints[1] = { deliveryPoint.x, deliveryPoint.y, deliveryPoint.z };
    ok = startFixedJob(shelf3Waypoints, 2, 1, 3, true);
  } else {
    server.send(400, "text/plain", "BAD SHELF");
    return;
  }

  server.send(ok ? 200 : 409, "text/plain", ok ? "FETCH STARTED" : "BUSY OR NOT HOMED");
}

void handleRetrieve() {
  int shelf = server.arg("shelf").toInt();
  bool ok = false;

  if (shelf == 1) {
    ok = startFixedJob(shelf1Waypoints, SHELF1_STEPS, -1, 1, false);
  } else if (shelf == 2) {
    ok = startFixedJob(shelf2Waypoints, SHELF2_STEPS, -1, 2, false);
  } else if (shelf == 3) {
    if (!shelf3Point.valid || !deliveryPoint.valid) {
      server.send(409, "text/plain", "SHELF3 NOT TAUGHT");
      return;
    }
    shelf3Waypoints[0] = { shelf3Point.x, shelf3Point.y, shelf3Point.z };
    shelf3Waypoints[1] = { deliveryPoint.x, deliveryPoint.y, deliveryPoint.z };
    ok = startFixedJob(shelf3Waypoints, 2, -1, 3, false);
  } else {
    server.send(400, "text/plain", "BAD SHELF");
    return;
  }

  server.send(ok ? 200 : 409, "text/plain", ok ? "RETRIEVE STARTED" : "BUSY OR NOT HOMED");
}

void handleJog() {
  if (homingActive || jobActive) {
    server.send(409, "text/plain", "BUSY");
    return;
  }

  String axisName = server.arg("axis");
  int dir = server.arg("dir").toInt();

  int idx = -1;
  if (axisName.equalsIgnoreCase("X")) idx = AX_X;
  else if (axisName.equalsIgnoreCase("Y")) idx = AX_Y;
  else if (axisName.equalsIgnoreCase("Z")) idx = AX_Z;

  if (idx < 0) {
    server.send(400, "text/plain", "BAD AXIS");
    return;
  }
  if (dir < -1 || dir > 1) {
    server.send(400, "text/plain", "BAD DIR");
    return;
  }

  axes[idx].jogDir = (int8_t)dir;
  server.send(200, "text/plain", "OK");
}

void handleSaveShelf3() {
  if (homingActive || jobActive || !homeDone) {
    server.send(409, "text/plain", "BUSY OR NOT HOMED");
    return;
  }

  shelf3Point.x = axes[AX_X].position;
  shelf3Point.y = axes[AX_Y].position;
  shelf3Point.z = axes[AX_Z].position;
  shelf3Point.valid = true;

  saveTaughtToNVS();

  server.send(200, "text/plain", "SAVED");
}

void handleSaveDelivery() {
  if (homingActive || jobActive || !homeDone) {
    server.send(409, "text/plain", "BUSY OR NOT HOMED");
    return;
  }

  deliveryPoint.x = axes[AX_X].position;
  deliveryPoint.y = axes[AX_Y].position;
  deliveryPoint.z = axes[AX_Z].position;
  deliveryPoint.valid = true;

  saveTaughtToNVS();

  server.send(200, "text/plain", "SAVED");
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
  json.reserve(500);

  json += "{\"homing\":";
  json += homingActive ? "true" : "false";

  json += ",\"homed\":";
  json += homeDone ? "true" : "false";

  json += ",\"jobActive\":";
  json += jobActive ? "true" : "false";

  json += ",\"jobShelf\":";
  json += String(jobActive ? activeShelfNum : 0);

  json += ",\"jobIsFetch\":";
  json += jobIsFetch ? "true" : "false";

  json += ",\"jobError\":";
  json += jobError ? "true" : "false";

  json += ",\"jobFinished\":";
  json += jobFinished ? "true" : "false";

  json += ",\"x\":";
  json += String(axes[AX_X].position);

  json += ",\"y\":";
  json += String(axes[AX_Y].position);

  json += ",\"z\":";
  json += String(axes[AX_Z].position);

  json += ",\"shelf3Valid\":";
  json += shelf3Point.valid ? "true" : "false";

  json += ",\"shelf3\":{\"x\":";
  json += String(shelf3Point.x);
  json += ",\"y\":";
  json += String(shelf3Point.y);
  json += ",\"z\":";
  json += String(shelf3Point.z);
  json += "}";

  json += ",\"deliveryValid\":";
  json += deliveryPoint.valid ? "true" : "false";

  json += ",\"delivery\":{\"x\":";
  json += String(deliveryPoint.x);
  json += ",\"y\":";
  json += String(deliveryPoint.y);
  json += ",\"z\":";
  json += String(deliveryPoint.z);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== EMBEDDED ATOMATED SHELF PICKING ROBOT ===");

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

  loadTaughtFromNVS();

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
  server.on("/home", handleHome);
  server.on("/stop", handleStop);
  server.on("/fetch", handleFetch);
  server.on("/retrieve", handleRetrieve);
  server.on("/jog", handleJog);
  server.on("/saveShelf3", handleSaveShelf3);
  server.on("/saveDelivery", handleSaveDelivery);
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

  if (homingActive) {
    updateHoming(now);
    return;
  }

  // تعليم يدوي (Jog) - بيحترم الليمت سويتش
  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    if (ax.jogDir == 0) continue;
    if ((now - ax.lastStepMicros) < stepIntervalUs) continue;
    if (!moveAllowed(ax, ax.jogDir)) continue;

    int8_t realDir = actualDir(ax, ax.jogDir);
    stepOnce(ax, realDir);
    ax.lastStepMicros = now;
  }

  // تنفيذ تلقائي (Fetch/Retrieve) - بيتجاهل الليمت تماماً
  for (int i = 0; i < 3; i++) {
    Axis &ax = axes[i];

    if (!ax.autoMoving) continue;
    if ((now - ax.lastStepMicros) < stepIntervalUs) continue;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
      continue;
    }

    int8_t realDir = (ax.autoTarget > ax.position) ? 1 : -1;

    stepOnce(ax, realDir);
    ax.lastStepMicros = now;

    if (ax.position == ax.autoTarget) {
      ax.autoMoving = false;
    }
  }

  updateFixedJob();
}

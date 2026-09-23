/*
  ESP32 AS/RS - RUN ALL + THREE DIRECT GET BUTTONS
  ------------------------------------------------
  Original Run all.ino 12-step route, GPIO map, sequential Z-aware
  phase logic, STEP/DIR pulse generator and NC limit monitoring
  retained. GET2 and GET3 use separately recorded six-waypoint paths.

  GET 1: Shelf 1 -> recorded transit -> Delivery
  GET 2: new user-confirmed six-waypoint path -> Delivery
  GET 3: new user-confirmed six-waypoint path -> Delivery
  RUN ALL: original 12 recorded steps (unchanged)

  GET1 and RUN ALL were reported working on the physical robot.
  Updated GET2/GET3 coordinates are not yet validated in this merged sketch.
  Begin with boxes
  removed and low speed. Original code only MONITORS limit switches:
  it DOES NOT stop automatically if travel exceeds physical limits.
  Have a physical emergency stop and verify all clearances.
  At every power-up physically position the robot at its known HOME.
  Remove each delivered box before starting another program.
*/

#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// WiFi
// =====================================================
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

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

// Indices 0..11: original, successful RUN ALL route; DO NOT MODIFY.
// Indices 12..17: corrected, independently recorded GET2 route.
// Indices 18..23: corrected, independently recorded GET3 route.
Waypoint waypoints[] = {
  { -182, -1903, 583 },  // RUN ALL step 1 (unchanged)
  {  544, -2290,  10 },  // RUN ALL step 2 (unchanged)
  {  563, -1354, 636 },  // RUN ALL step 3 (unchanged)
  {  563, -1071,   3 },  // RUN ALL step 4 (unchanged)
  {  563, -2002, 598 },  // RUN ALL step 5 (unchanged)
  { 1204, -2345,  12 },  // RUN ALL step 6 (unchanged)
  { 1279, -1371, 610 },  // RUN ALL step 7 (unchanged)
  { 1279, -1037,   3 },  // RUN ALL step 8 (unchanged)
  { 1279, -1988, 605 },  // RUN ALL step 9 (unchanged)
  { 2273, -2331,  52 },  // RUN ALL step 10 (unchanged)
  { 2582,  -623, 626 },  // RUN ALL step 11 (unchanged)
  { 2582,   -10,   3 },  // RUN ALL step 12 (unchanged)

  { -906, -1899,   0 },  // GET2 step 1
  { -906, -1899, 565 },  // GET2 step 2
  { -906, -2308,  -4 },  // GET2 step 3
  {-2935,  -704,  -4 },  // GET2 step 4
  {-2935,  -704, 745 },  // GET2 step 5
  {-2935,     5,  -4 },  // GET2 step 6

  {-1652, -1880,   0 },  // GET3 step 1
  {-1652, -1880, 566 },  // GET3 step 2
  {-1652, -2198,   4 },  // GET3 step 3
  {-2948,  -575,   4 },  // GET3 step 4
  {-2948,  -575, 612 },  // GET3 step 5
  {-2948,   -10,   4 }   // GET3 step 6
};

const int TOTAL_STEPS = 12;  // Original RUN ALL count, not entire combined table.

// Preserve original RUN ALL / verified GET1 waypoint selections exactly.
const uint8_t RUN_ALL_PATH[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const uint8_t GET1_PATH[] = {0, 1, 9, 10, 11};
// GET2 and GET3 now point ONLY to their new recorded coordinates.
const uint8_t GET2_PATH[] = {12, 13, 14, 15, 16, 17};
const uint8_t GET3_PATH[] = {18, 19, 20, 21, 22, 23};

enum ProgramMode : uint8_t {
  PROGRAM_NONE = 0,
  PROGRAM_GET1,
  PROGRAM_GET2,
  PROGRAM_GET3,
  PROGRAM_ALL
};

ProgramMode activeProgram = PROGRAM_NONE;
const uint8_t* activePath = nullptr;
int activeCount = 0;

// The prototype has no delivery-presence sensor. Manual acknowledgement only.
bool deliveryOccupied = false;

const char* programName(ProgramMode p) {
  switch (p) {
    case PROGRAM_GET1: return "GET 1";
    case PROGRAM_GET2: return "GET 2";
    case PROGRAM_GET3: return "GET 3";
    case PROGRAM_ALL: return "RUN ALL";
    default: return "NONE";
  }
}

bool configureProgram(ProgramMode p) {
  switch (p) {
    case PROGRAM_GET1:
      activePath = GET1_PATH;
      activeCount = sizeof(GET1_PATH) / sizeof(GET1_PATH[0]);
      break;
    case PROGRAM_GET2:
      activePath = GET2_PATH;
      activeCount = sizeof(GET2_PATH) / sizeof(GET2_PATH[0]);
      break;
    case PROGRAM_GET3:
      activePath = GET3_PATH;
      activeCount = sizeof(GET3_PATH) / sizeof(GET3_PATH[0]);
      break;
    case PROGRAM_ALL:
      activePath = RUN_ALL_PATH;
      activeCount = sizeof(RUN_ALL_PATH) / sizeof(RUN_ALL_PATH[0]);
      break;
    default: return false;
  }
  activeProgram = p;
  return true;
}

Waypoint& currentWaypoint();

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

Waypoint& currentWaypoint() {
  return waypoints[activePath[currentStep]];
}

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
  Waypoint &wp = currentWaypoint();

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
  Waypoint &wp = currentWaypoint();

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

  if (currentStep >= activeCount) {
    running = false;
    finishedAll = true;
    deliveryOccupied = true;
    stopAllAxes();
    Serial.print(programName(activeProgram));
    Serial.println(" DONE - REMOVE BOX AT DELIVERY");
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
// Choose or resume a program without altering RUN ALL logic.
// Do not switch to another route in the middle of a paused move.
// =====================================================
bool launchProgram(ProgramMode requested, String &reason) {
  if (running) {
    reason = "A PROGRAM IS ALREADY RUNNING";
    return false;
  }
  if (deliveryOccupied) {
    reason = "REMOVE DELIVERED BOX, THEN PRESS DELIVERY CLEARED";
    return false;
  }
  if (activeProgram != PROGRAM_NONE && !finishedAll) {
    if (activeProgram != requested) {
      reason = "PAUSED JOB: RESUME THE SAME JOB OR RESTART FROM HOME";
      return false;
    }
    startSequence();   // Same button resumes original waypoint and phase.
    return true;
  }
  if (!configureProgram(requested)) {
    reason = "UNKNOWN PROGRAM";
    return false;
  }
  currentStep = 0;
  finishedAll = false;
  errorState = false;
  currentStepPrepared = false;
  startSequence();
  return true;
}

// =====================================================
// Web UI
// =====================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Shelf Robot</title>
<style>
*{box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#10151b;color:#e9eef4;margin:0;padding:14px}
main{max-width:620px;margin:0 auto}
h1{font-size:22px;text-align:center;margin:16px 0 5px}
.sub{text-align:center;font-size:13px;color:#aab6c5;margin-bottom:20px}
.card{background:#1d2733;border:1px solid #354252;border-radius:12px;padding:16px;margin:12px 0}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
button{border:0;border-radius:9px;padding:18px 8px;color:#fff;font-size:18px;font-weight:bold;cursor:pointer;min-height:65px}
button:disabled{opacity:.4;cursor:not-allowed}
.get{background:#176aaa}.all{background:#1d854f}.stop{background:#c63434}
.clear{background:#6a6072;font-size:14px;min-height:46px;padding:12px}
.badge{padding:10px;background:#111922;border-radius:8px;text-align:center;font-weight:bold}
.error{color:#ff9c9c}.done{color:#97e4aa}.working{color:#8ed5ff}
.row{display:flex;justify-content:space-between;border-bottom:1px solid #354252;padding:8px 0;gap:12px;font-size:14px}
.row:last-child{border-bottom:0}label{font-size:14px}
input{width:100%;margin-top:12px}
.note{font-size:12px;color:#bac5d0;line-height:1.5;margin-top:12px}
@media(max-width:380px){button{font-size:15px}}
</style>
</head>
<body><main>
<h1>Automated Shelf Robot</h1>
<div class="sub">Choose a drawer or run the full 12-step sequence.</div>
<div class="card">
  <div id="state" class="badge">CONNECTING...</div>
  <p id="message" class="note" aria-live="polite"></p>
  <div class="grid">
    <button class="get" data-program="1">GET 1</button>
    <button class="get" data-program="2">GET 2</button>
    <button class="get" data-program="3">GET 3</button>
    <button class="all" data-program="4">RUN ALL</button>
    <button class="stop" id="stop">STOP</button>
    <button class="clear" id="clear">DELIVERY CLEARED</button>
  </div>
  <div class="note">GET 1/2/3: take the chosen shelf box to delivery. Remove the box before starting another job. STOP pauses; press the same job to resume.</div>
</div>
<div class="card">
  <div class="row"><span>Program</span><strong id="job">NONE</strong></div>
  <div class="row"><span>Waypoint</span><strong id="step">--</strong></div>
  <div class="row"><span>Moving</span><strong id="phase">--</strong></div>
  <div class="row"><span>X / Y / Z (steps)</span><strong id="xyz">0 / 0 / 0</strong></div>
  <div class="row"><span>Delivery</span><strong id="delivery">CLEAR</strong></div>
</div>
<div class="card">
  <label for="speed">Step interval: <strong id="speedValue">3500</strong> &micro;s (smaller = faster)</label>
  <input id="speed" type="range" min="500" max="8000" value="3500" step="100">
  <div class="note">Start with the robot physically at its known HOME position after every power cycle. Check updated GET2/GET3 routes on the machine before carrying a box.</div>
</div>
</main><script>
const message=document.getElementById('message');
const state=document.getElementById('state');
async function call(url){
  try{
    const response=await fetch(url,{cache:'no-store'});
    const result=await response.text();
    message.textContent=result;
    message.className='note'+(response.ok?'':' error');
  }catch(e){message.textContent='ESP32 CONNECTION ERROR';message.className='note error';}
}
document.querySelectorAll('[data-program]').forEach(button=>{
  button.addEventListener('click',()=>call('/run?job='+button.dataset.program));
});
document.getElementById('stop').addEventListener('click',()=>call('/stop'));
document.getElementById('clear').addEventListener('click',()=>{
  if(confirm('Confirm the delivered box has been physically removed.')) call('/clearDelivery');
});
const speed=document.getElementById('speed');
speed.addEventListener('input',()=>{document.getElementById('speedValue').textContent=speed.value;});
speed.addEventListener('change',()=>call('/speed?value='+speed.value));
let busy=false;
async function refresh(){
  if(busy)return;
  busy=true;
  try{
    const response=await fetch('/status',{cache:'no-store'});
    if(!response.ok) throw Error('status');
    const d=await response.json();
    state.textContent=d.running?'RUNNING':d.error?'ERROR':d.finished?'COMPLETE':d.job!=='NONE'?'PAUSED':'READY';
    state.className='badge '+(d.error?'error':d.finished?'done':d.running?'working':'');
    document.getElementById('job').textContent=d.job;
    document.getElementById('step').textContent=d.step+' / '+d.total;
    const axis=['Y','X','Z'];
    document.getElementById('phase').textContent=d.running?axis[d.phase]||'--':'--';
    document.getElementById('xyz').textContent=[d.x,d.y,d.z].join(' / ');
    document.getElementById('delivery').textContent=d.deliveryOccupied?'OCCUPIED - REMOVE BOX':'CLEAR';
    if(document.activeElement!==speed){speed.value=d.interval;document.getElementById('speedValue').textContent=d.interval;}
    document.querySelectorAll('[data-program]').forEach(b=>{
      const selected=Number(b.dataset.program);
      b.disabled=d.running||d.deliveryOccupied||(d.job!=='NONE'&&!d.finished&&d.jobId!==selected);
    });
    document.getElementById('clear').disabled=!d.deliveryOccupied;
  }catch(e){state.textContent='OFFLINE';state.className='badge error';}
  finally{busy=false;}
}
refresh();setInterval(refresh,600);
</script></body></html>
)HTML";

// =====================================================
// HTTP
// =====================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleRun() {
  int requested = server.arg("job").toInt();
  if (requested < 1 || requested > 4) {
    server.send(400, "text/plain", "INVALID JOB");
    return;
  }
  String reason;
  if (!launchProgram((ProgramMode)requested, reason)) {
    server.send(409, "text/plain", reason);
    return;
  }
  server.send(200, "text/plain", String(programName(activeProgram)) + " STARTED / RESUMED");
}

void handleStart() {
  String reason;
  if (!launchProgram(PROGRAM_ALL, reason)) {
    server.send(409, "text/plain", reason);
    return;
  }
  server.send(200, "text/plain", "RUN ALL STARTED / RESUMED");
}

void handleStop() {
  stopSequence();
  server.send(200, "text/plain", "STOPPED - PRESS SAME JOB TO RESUME");
}

void handleClearDelivery() {
  if (running || !finishedAll) {
    server.send(409, "text/plain", "CANNOT CLEAR DURING AN ACTIVE OR PAUSED JOB");
    return;
  }
  deliveryOccupied = false; // Manual acknowledgement; no presence sensor installed.
  server.send(200, "text/plain", "DELIVERY CLEAR - READY FOR ANOTHER JOB");
}

void handleSpeed() {
  int v = server.arg("value").toInt();
  if (v < 500 || v > 8000) {
    server.send(400, "text/plain", "VALID STEP INTERVAL: 500..8000 US");
    return;
  }
  stepIntervalUs = v;
  server.send(200, "text/plain", "STEP INTERVAL UPDATED");
}

void handleStatus() {
  String json;
  json.reserve(420);
  Waypoint fallback = {0,0,0};
  Waypoint& wp = (activePath && activeCount > 0)
    ? waypoints[activePath[currentStep < activeCount ? currentStep : activeCount - 1]]
    : fallback;
  json += "{\"running\":";
  json += running ? "true" : "false";
  json += ",\"finished\":";
  json += finishedAll ? "true" : "false";
  json += ",\"error\":";
  json += errorState ? "true" : "false";
  json += ",\"deliveryOccupied\":";
  json += deliveryOccupied ? "true" : "false";
  json += ",\"job\":\"";
  json += programName(activeProgram);
  json += "\",\"jobId\":";
  json += String((int)activeProgram);
  json += ",\"step\":";
  json += String(activeCount > 0 ? (currentStep < activeCount ? currentStep + 1 : activeCount) : 0);
  json += ",\"total\":";
  json += String(activeCount);
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
  json += ",\"interval\":";
  json += String(stepIntervalUs);
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
  server.on("/run", handleRun);
  server.on("/start", handleStart);
  server.on("/clearDelivery", handleClearDelivery);
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

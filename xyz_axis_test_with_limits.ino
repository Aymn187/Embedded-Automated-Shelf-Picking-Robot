#define X_STEP_PIN 13
#define X_DIR_PIN  15

#define Y_STEP_PIN 16
#define Y_DIR_PIN  17

#define Z_STEP_PIN 27
#define Z_DIR_PIN  23

// ===== Limit Switches (NC, common GND) =====
#define X_LIMIT_PIN 25
#define Y_LIMIT_PIN 33
#define Z_LIMIT_PIN 32

// مع مفتاح NC وGND مشترك: في الوضع الطبيعي (غير مضغوط) الدائرة مغلقة
// فيقرأ البن LOW باستخدام INPUT_PULLUP، وعند الضغط (نهاية المشوار) تنفتح
// الدائرة فيرتفع البن إلى HIGH. لذلك: تشغيل الليمت = HIGH.
#define LIMIT_TRIGGERED HIGH

const int stepsToMove = 3000;
const int stepDelayUs = 3000;

bool readLimit(int limitPin) {
  return digitalRead(limitPin) == LIMIT_TRIGGERED;
}

// ===== Homing =====
const int homingStepDelayUs = 1500;   // سرعة الهوملنج
const int backoffSteps = 1000;          // خطوات الابتعاد عن الليمت بعد ملامسته

void homeAxis(int stepPin, int dirPin, int limitPin, const char* axisName) {
  Serial.print("Homing ");
  Serial.print(axisName);
  Serial.println("...");

  // التحرك باتجاه الصفر (LOW) لحد ما يلمس الليمت سويتش
  digitalWrite(dirPin, LOW);
  while (!readLimit(limitPin)) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(homingStepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(homingStepDelayUs);
  }

  Serial.print(axisName);
  Serial.println(" limit hit, backing off...");

  // الابتعاد شوي عن الليمت سويتش (اتجاه عكسي)
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < backoffSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(homingStepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(homingStepDelayUs);
  }

  Serial.print(axisName);
  Serial.println(" homed.");
}

void homeAllAxes() {
  Serial.println("===== HOMING START =====");
  homeAxis(X_STEP_PIN, X_DIR_PIN, X_LIMIT_PIN, "X");
  homeAxis(Y_STEP_PIN, Y_DIR_PIN, Y_LIMIT_PIN, "Y");
  homeAxis(Z_STEP_PIN, Z_DIR_PIN, Z_LIMIT_PIN, "Z");
  Serial.println("===== HOMING DONE =====");
}

void stepOneAxis(int stepPin, int limitPin, int steps) {
  for (int i = 0; i < steps; i++) {
    if (readLimit(limitPin)) {
      Serial.println("!! Limit switch triggered - stopping this axis !!");
      break;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

void stepAllAxes(int steps) {
  for (int i = 0; i < steps; i++) {
    bool xOk = !readLimit(X_LIMIT_PIN);
    bool yOk = !readLimit(Y_LIMIT_PIN);
    bool zOk = !readLimit(Z_LIMIT_PIN);

    if (!xOk && !yOk && !zOk) {
      Serial.println("!! All axes hit limit switches - stopping !!");
      break;
    }

    if (xOk) digitalWrite(X_STEP_PIN, HIGH);
    if (yOk) digitalWrite(Y_STEP_PIN, HIGH);
    if (zOk) digitalWrite(Z_STEP_PIN, HIGH);

    delayMicroseconds(stepDelayUs);

    if (xOk) digitalWrite(X_STEP_PIN, LOW);
    if (yOk) digitalWrite(Y_STEP_PIN, LOW);
    if (zOk) digitalWrite(Z_STEP_PIN, LOW);

    delayMicroseconds(stepDelayUs);

    if (!xOk) Serial.println("X limit reached");
    if (!yOk) Serial.println("Y limit reached");
    if (!zOk) Serial.println("Z limit reached");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(Z_STEP_PIN, LOW);

  Serial.println();
  Serial.println("===== XYZ AXIS TEST WITH LIMIT SWITCHES =====");
  delay(2000);

  homeAllAxes();
}

void loop() {

  // X
  Serial.println("Moving X...");
  digitalWrite(X_DIR_PIN, HIGH);
  stepOneAxis(X_STEP_PIN, X_LIMIT_PIN, stepsToMove);
  Serial.println("X Done");
  delay(2000);

  // Y
  Serial.println("Moving Y...");
  digitalWrite(Y_DIR_PIN, HIGH);
  stepOneAxis(Y_STEP_PIN, Y_LIMIT_PIN, stepsToMove);
  Serial.println("Y Done");
  delay(2000);

  // Z
  Serial.println("Moving Z...");
  digitalWrite(Z_DIR_PIN, HIGH);
  stepOneAxis(Z_STEP_PIN, Z_LIMIT_PIN, stepsToMove);
  Serial.println("Z Done");
  delay(2000);

  // XYZ together
  Serial.println("Moving X + Y + Z together...");

  digitalWrite(X_DIR_PIN, HIGH);
  digitalWrite(Y_DIR_PIN, HIGH);
  digitalWrite(Z_DIR_PIN, HIGH);

  stepAllAxes(stepsToMove);

  Serial.println("All Axes Done");
  delay(2000);

  // Reverse all
  Serial.println("Reversing all axes...");

  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(Z_DIR_PIN, LOW);

  stepAllAxes(stepsToMove);

  Serial.println("Reverse Done");
  Serial.println("-------------------------");

  delay(3000);
}

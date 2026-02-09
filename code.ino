/******************** SMART LINE FOLLOWER ********************
 * FEATURES:
 * ✔ Auto calibration (5 sec)
 * ✔ Auto threshold & normalization
 * ✔ PID line following
 * ✔ 90° LEFT / RIGHT hard turns (pattern based)
 * ✔ Line lost = SAME LOCATION SPIN
 * ✔ S5 (A5) DISABLED (FAULTY SENSOR)
 ************************************************************/

// ---------- SENSOR PINS (LEFT → RIGHT) ----------
const int sensorPins[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
#define DEAD_SENSOR 2   // S5 = index 2 (A5)

// ---------- TB6612 MOTOR PINS ----------
#define PWMA 3
#define AIN1 5
#define AIN2 4
#define PWMB 9
#define BIN1 7
#define BIN2 8
#define STBY 6

// ---------- SENSOR DATA ----------
int sMin[8], sMax[8];
int raw[8];
float kSensor[8];   // 0.0 = white, 1.0 = black

// ---------- PID ----------
int weights[8] = {-7, -5, 0, -1, 1, 3, 5, 7}; // S5 weight = 0
float error = 0, lastError = 0;
float Kp = 25;
float Kd = 120;

// ---------- SPEED ----------
int baseSpeed = 120;
int sharpSpeed = 90;

// ---------- STATE ----------
int lastTurnDir = 0;   // -1 = left, +1 = right

// ============================================================
// ========================== SETUP ===========================
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // init calibration
  for (int i = 0; i < 8; i++) {
    sMin[i] = 1023;
    sMax[i] = 0;
  }

  autoCalibrate();
}

// ============================================================
// ===================== AUTO CALIBRATION =====================
// ============================================================
void autoCalibrate() {
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    for (int i = 0; i < 8; i++) {
      if (i == DEAD_SENSOR) continue;   // <<< SKIP S5
      int v = analogRead(sensorPins[i]);
      if (v < sMin[i]) sMin[i] = v;
      if (v > sMax[i]) sMax[i] = v;
    }
  }
}

// ============================================================
// ======================= READ SENSORS =======================
// ============================================================
void readSensors() {
  for (int i = 0; i < 8; i++) {
    if (i == DEAD_SENSOR) {
      kSensor[i] = 0;    // <<< FORCE IGNORE
      continue;
    }
    raw[i] = analogRead(sensorPins[i]);
    int mapped = map(raw[i], sMin[i], sMax[i], 0, 1000);
    mapped = constrain(mapped, 0, 1000);
    kSensor[i] = mapped / 1000.0;
  }
}

// ============================================================
// ======================= PID ERROR ==========================
// ============================================================
float calcError() {
  float sum = 0, wsum = 0;
  for (int i = 0; i < 8; i++) {
    if (i == DEAD_SENSOR) continue;
    sum  += kSensor[i];
    wsum += weights[i] * kSensor[i];
  }
  if (sum == 0) return lastError;
  return wsum / sum;
}

// ============================================================
// =================== 90° TURN DETECTION =====================
// ============================================================
int detect90Turn() {

  // LEFT 90°
  if (kSensor[7] < 0.2 && kSensor[6] < 0.2 &&
      (kSensor[0] > 0.4 || kSensor[1] > 0.4)) {
    return -1;
  }

  // RIGHT 90°
  if (kSensor[0] < 0.2 && kSensor[1] < 0.2 &&
      (kSensor[6] > 0.4 || kSensor[7] > 0.4)) {
    return 1;
  }

  return 0;
}

// ============================================================
// ===================== LINE LOST CHECK ======================
// ============================================================
bool isLineLost() {
  for (int i = 0; i < 8; i++) {
    if (i == DEAD_SENSOR) continue;
    if (kSensor[i] > 0.25) return false;
  }
  return true;
}

bool centerFound() {
  return (kSensor[3] > 0.4 || kSensor[4] > 0.4);
}

// ============================================================
// ======================= MOTOR ==============================
// ============================================================
void setMotor(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);

  digitalWrite(AIN1, l >= 0);
  digitalWrite(AIN2, l < 0);
  digitalWrite(BIN1, r >= 0);
  digitalWrite(BIN2, r < 0);

  analogWrite(PWMA, abs(l));
  analogWrite(PWMB, abs(r));
}

// ============================================================
// =========================== LOOP ===========================
// ============================================================
void loop() {

  readSensors();

  int turn90 = detect90Turn();

  if (turn90 == -1) {
    setMotor(-150, 150);
    lastTurnDir = -1;
    return;
  }

  if (turn90 == 1) {
    setMotor(150, -150);
    lastTurnDir = 1;
    return;
  }

  if (isLineLost()) {
    if (lastTurnDir <= 0)
      setMotor(-130, 130);
    else
      setMotor(130, -130);
    return;
  }

  float e = calcError();
  float d = e - lastError;
  lastError = e;

  float pid = (Kp * e) + (Kd * d);

  int base = baseSpeed;
  if (abs(e) > 0.6) base = sharpSpeed;

  setMotor(base + pid, base - pid);
}
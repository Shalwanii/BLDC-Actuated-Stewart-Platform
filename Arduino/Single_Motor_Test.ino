#include <Servo.h>

/* ===== Pins ===== */
const uint8_t ESC_PINS[6] = {6, 9, 11, 10, 3, 5};   // 1L,1R,2L,2R,3L,3R
const uint8_t POT_PINS[6] = {A4, A5, A1, A0, A2, A3};
const char*   NAMES[6]    = {"1L","1R","2L","2R","3L","3R"};

/* ===== ESC centered PWM ===== */
const int ESC_MID    = 1500;
const int MIN_OFFSET = 20;
const int MAX_OFFSET = 35;

/* ===== Angle↔Count calibration ===== */
const float ZERO_COUNT          = 520.0f;
const float DEG_PER_COUNT_LEFT  = -60.0f / (800.0f - 520.0f);  // -0.2142857
const float DEG_PER_COUNT_RIGHT = -60.0f / (240.0f - 520.0f);  // +0.2142857
const float CNT_PER_DEG_LEFT    = 1.0f / DEG_PER_COUNT_LEFT;   // -4.6666667
const float CNT_PER_DEG_RIGHT   = 1.0f / DEG_PER_COUNT_RIGHT;  // +4.6666667

enum Side { LEFT_SIDE, RIGHT_SIDE };
const Side SIDE[6] = {LEFT_SIDE, RIGHT_SIDE, LEFT_SIDE, RIGHT_SIDE, LEFT_SIDE, RIGHT_SIDE};

inline int angleToCounts(Side s, float deg) {
  return (int)lroundf(ZERO_COUNT + deg * ((s == LEFT_SIDE) ? CNT_PER_DEG_LEFT : CNT_PER_DEG_RIGHT));
}
inline float countsToAngle(Side s, int counts) {
  float dpc = (s == LEFT_SIDE) ? DEG_PER_COUNT_LEFT : DEG_PER_COUNT_RIGHT;
  return (counts - ZERO_COUNT) * dpc;
}

/* ===== Targets (angles) ===== */
const float degTargets[] = {-30.0f, 0.0f, +30.0f};
const int   N_TGT = sizeof(degTargets)/sizeof(degTargets[0]);

/* ===== Sync hold ===== */
const int ARRIVE_TOL = 20;             // counts
const unsigned long HOLD_MS = 2000;    // ms (sync hold)

/* ===== PD gains ===== */
float Kp_us_per_count     = 0.0025f;
float Kd_us_per_cnt_per_s = 0.75f;
const float DERIV_LP_ALPHA = 0.12f;

/* ===== Repeat cycles ===== */
const int N_CYCLES = 5;
int cyclesDone = 0;

/* ===== Safety limits ===== */
const float SAFE_MIN_DEG = -65.0f;   // SAFETY
const float SAFE_MAX_DEG =  70.0f;   // SAFETY
const unsigned long POT_TIMEOUT_MS = 200; // if pot stuck, force neutral

/* ===== State ===== */
Servo esc[6];
int   plusEffect[6];
float lastCounts[6];
float vel_filt[6];
unsigned long lastPotUpdate[6];   // SAFETY

int   idx = 0;
bool  arrived[6] = {false};
unsigned long holdStart = 0;
bool  holding = false;

unsigned long lastMicros = 0;
bool finished = false;

/* ---- Helpers ---- */
int readPotFiltered(uint8_t pin) {
  long s=0; for (int i=0;i<8;++i) s += analogRead(pin);
  return (int)(s>>3);
}a

void calibrateDirection(Servo& esc, uint8_t potPin, int &plusEff) {
  int r0 = readPotFiltered(potPin);
  esc.writeMicroseconds(ESC_MID + 30);
  delay(60);
  esc.writeMicroseconds(ESC_MID);
  delay(40);
  int r1 = readPotFiltered(potPin);
  plusEff = (r1 > r0) ? +1 : -1;
}

int pdToPulse(float u_us) {
  int sign = (u_us > 0) ? +1 : (u_us < 0 ? -1 : 0);
  int mag  = (int)fabs(u_us);
  if (mag < MIN_OFFSET) mag = MIN_OFFSET;
  if (mag > MAX_OFFSET) mag = MAX_OFFSET;
  return ESC_MID + sign * mag;
}

/* ===== Arduino ===== */
void setup() {
  Serial.begin(115200);

  for (int i=0;i<6;++i) {
    pinMode(POT_PINS[i], INPUT);
    esc[i].attach(ESC_PINS[i]);
    esc[i].writeMicroseconds(ESC_MID);
    lastPotUpdate[i] = millis();  // SAFETY
  }
  Serial.println(F("Arming 6 ESC channels at 1500us..."));
  delay(3000);

  for (int i=0;i<6;++i) calibrateDirection(esc[i], POT_PINS[i], plusEffect[i]);

  for (int i=0;i<6;++i) { lastCounts[i] = NAN; vel_filt[i] = 0.0f; }

  lastMicros = micros();

  Serial.println(F("All motors ACTIVE, sync sequence -30,0,+30 deg"));
  Serial.println(F("time_ms,"
                   "raw_1L,ang_1L,tgt_1L,pwm_1L,"
                   "raw_1R,ang_1R,tgt_1R,pwm_1R,"
                   "raw_2L,ang_2L,tgt_2L,pwm_2L,"
                   "raw_2R,ang_2R,tgt_2R,pwm_2R,"
                   "raw_3L,ang_3L,tgt_3L,pwm_3L,"
                   "raw_3R,ang_3R,tgt_3R,pwm_3R"));
}

void loop() {
  if (finished) return;

  unsigned long nowMicros = micros();
  float dt_s = (nowMicros - lastMicros) / 1e6f;
  if (dt_s <= 0) dt_s = 1e-3f;
  lastMicros = nowMicros;

  int raw[6]; float ang[6]; int pwm[6];
  float tgtDeg = degTargets[idx];

  bool allArrived = true;

  for (int i=0;i<6;++i) {
    raw[i] = readPotFiltered(POT_PINS[i]);
    ang[i] = countsToAngle(SIDE[i], raw[i]);

    // SAFETY: check pot change
    static int lastRaw[6] = {0};
    if (raw[i] != lastRaw[i]) {
      lastRaw[i] = raw[i];
      lastPotUpdate[i] = millis();
    }

    bool potStuck = (millis() - lastPotUpdate[i]) > POT_TIMEOUT_MS;

    // SAFETY: angle or pot failure
    if (ang[i] < SAFE_MIN_DEG || ang[i] > SAFE_MAX_DEG || potStuck) {
      esc[i].writeMicroseconds(ESC_MID);
      pwm[i] = ESC_MID;
      arrived[i] = true;   // force "arrived" to avoid deadlock
      continue;
    }

    int tgt = angleToCounts(SIDE[i], tgtDeg);
    int err = tgt - raw[i];
    int aerr = abs(err);

    if (isnan(lastCounts[i])) lastCounts[i] = raw[i];
    float vel = (raw[i] - lastCounts[i]) / dt_s;
    lastCounts[i] = raw[i];
    vel_filt[i] = (1.0f - DERIV_LP_ALPHA)*vel_filt[i] + DERIV_LP_ALPHA*vel;

    pwm[i] = ESC_MID;

    if (aerr <= ARRIVE_TOL) {
      arrived[i] = true;
      esc[i].writeMicroseconds(ESC_MID);
    } else {
      arrived[i] = false;
      float u_base = Kp_us_per_count * (float)err - Kd_us_per_cnt_per_s * vel_filt[i];
      int desiredSignCounts = (err > 0) ? +1 : -1;
      int sideSign = (plusEffect[i] == desiredSignCounts) ? +1 : -1;
      float u_signed = sideSign * fabs(u_base);
      pwm[i] = pdToPulse(u_signed);
      esc[i].writeMicroseconds(pwm[i]);
    }

    if (!arrived[i]) allArrived = false;
  }

  // Sync hold logic
  if (allArrived) {
    if (!holding) { holding = true; holdStart = millis(); }
    else if (millis() - holdStart >= HOLD_MS) {
      idx++;
      holding = false;
      if (idx >= N_TGT) {
        idx = 0;
        cyclesDone++;
        if (cyclesDone >= N_CYCLES) {
          for (int i=0;i<6;++i) esc[i].writeMicroseconds(ESC_MID);
          Serial.println(F("# Finished all cycles, motors neutral, logging stopped."));
          finished = true;
        }
      }
    }
  } else {
    holding = false;
  }

  if (!finished) {
    Serial.print(millis()); Serial.print(",");
    for (int i=0;i<6;++i) {
      Serial.print(raw[i]); Serial.print(",");
      Serial.print(ang[i],2); Serial.print(",");
      Serial.print(tgtDeg,1); Serial.print(",");
      Serial.print(pwm[i]);
      if (i<5) Serial.print(",");
    }
    Serial.println();
  }

}

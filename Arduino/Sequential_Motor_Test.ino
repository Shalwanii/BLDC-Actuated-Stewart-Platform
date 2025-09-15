#include <Servo.h>

/* ===== Pins ===== */
const uint8_t ESC_PINS[6] = {3, 5, 6, 9, 10, 11};   // 1L,1R,2L,2R,3L,3R
const uint8_t POT_PINS[6] = {A0, A1, A2, A3, A4, A5};
const char*   NAMES[6]    = {"1L","1R","2L","2R","3L","3R"};

/* ===== ESC centered PWM ===== */
const int ESC_MID    = 1500;
const int MIN_OFFSET = 25;
const int MAX_OFFSET = 50;

/* ===== Angle↔Count calibration ===== */
const float ZERO_COUNT          = 520.0f;
const float DEG_PER_COUNT_LEFT  = -60.0f / (800.0f - 520.0f);
const float DEG_PER_COUNT_RIGHT = -60.0f / (240.0f - 520.0f);
const float CNT_PER_DEG_LEFT    = 1.0f / DEG_PER_COUNT_LEFT;   // -4.6667
const float CNT_PER_DEG_RIGHT   = 1.0f / DEG_PER_COUNT_RIGHT;  // +4.6667

enum Side { LEFT_SIDE, RIGHT_SIDE };
const Side SIDE[6] = {LEFT_SIDE, RIGHT_SIDE, LEFT_SIDE, RIGHT_SIDE, LEFT_SIDE, RIGHT_SIDE};

inline int angleToCounts(Side s, float deg) {
  return (int)lroundf(ZERO_COUNT + deg * ((s == LEFT_SIDE) ? CNT_PER_DEG_LEFT : CNT_PER_DEG_RIGHT));
}
inline float countsToAngle(Side s, int counts) {
  const float dpc = (s == LEFT_SIDE) ? DEG_PER_COUNT_LEFT : DEG_PER_COUNT_RIGHT;
  return (counts - ZERO_COUNT) * dpc;
}

/* ===== PD control (counts domain) ===== */
float Kp_us_per_count     = 0.0025f;
float Kd_us_per_cnt_per_s = 0.50f;
const float DERIV_LP_ALPHA = 0.12f;

/* ===== Arrival / hold / plan ===== */
const int   ARRIVE_TOL            = 10;       // ±counts
const unsigned long HOLD_MS       = 300;     // neutral hold after arrival/timeout
const unsigned long MOVE_TIMEOUT_MS = 5000;  // full time allowed to try
const int   N_POSES               = 200;
const float MIN_ANGLE             = -30.0f;   // random target range (deg)
const float MAX_ANGLE             =  60.0f;

/* ===== Safety (angle bounds only) ===== */
const float SAFE_MIN_DEG = -70.0f;
const float SAFE_MAX_DEG =  90.0f;

/* ===== State ===== */
Servo esc[6];
int   plusEffect[6] = {-1,-1,-1,-1,-1,-1};  // calibrated at start
float lastCounts[6] = {NAN,NAN,NAN,NAN,NAN,NAN};
float vel_filt[6]   = {0,0,0,0,0,0};
unsigned long lastMicros = 0;

int   poseIdx = 0;
int   activeMotor = -1;
float targetDeg = 0.0f;
int   targetCounts = 0;

bool  moving = false;
bool  holding = false;
unsigned long poseStart = 0;
unsigned long holdStart = 0;
unsigned long snapshotMillis = 0; 
unsigned long reach_ms = 0;       
bool  finished = false;

/* Snapshot captured exactly when we enter HOLD (arrival or timeout) */
float snapDeg[6];
bool  snapReached = false;

/* ===== Helpers ===== */
int readPotFiltered(uint8_t pin) {
  long s=0; for (int i=0;i<8; ++i) s += analogRead(pin);
  return (int)(s>>3);
}

void calibrateDirection(Servo& e, uint8_t potPin, int &plusEff) {
  int r0 = readPotFiltered(potPin);
  e.writeMicroseconds(ESC_MID + 30);
  delay(60);
  e.writeMicroseconds(ESC_MID);
  delay(40);
  int r1 = readPotFiltered(potPin);
  plusEff = (r1 > r0) ? +1 : -1;   // +μs increased counts?
}

int pdToPulse(float u_us) {
  int sign = (u_us > 0) ? +1 : (u_us < 0 ? -1 : 0);
  int mag  = (int)fabs(u_us);
  if (mag < MIN_OFFSET) mag = MIN_OFFSET;
  if (mag > MAX_OFFSET) mag = MAX_OFFSET;
  return ESC_MID + sign * mag;
}

void neutralAll() { for (int k=0;k<6; ++k) esc[k].writeMicroseconds(ESC_MID); }

void beginNewPose() {
  activeMotor = random(0, 6);

  long r = random((long)MIN_ANGLE, (long)MAX_ANGLE + 1); // integer degrees
  if (r < (long)MIN_ANGLE) r = (long)MIN_ANGLE;
  if (r > (long)MAX_ANGLE) r = (long)MAX_ANGLE;
  targetDeg = (float)r;
  targetCounts = angleToCounts(SIDE[activeMotor], targetDeg);

  // Recalibrate direction at pose start
  calibrateDirection(esc[activeMotor], POT_PINS[activeMotor], plusEffect[activeMotor]);

  moving = true;
  holding = false;
  poseStart = millis();
  lastCounts[activeMotor] = NAN;
  vel_filt[activeMotor]   = 0.0f;
}

/* ===== Arduino ===== */
void setup() {
  Serial.begin(115200);
  randomSeed((unsigned long)micros());

  for (int i=0;i<6; ++i) {
    pinMode(POT_PINS[i], INPUT);
    esc[i].attach(ESC_PINS[i]);
    esc[i].writeMicroseconds(ESC_MID);
  }
  delay(3000); // arming

  for (int i=0;i<6; ++i) calibrateDirection(esc[i], POT_PINS[i], plusEffect[i]);
  lastMicros = micros();

  // CSV header 
  Serial.println(F("pose,active,target_deg,status,reach_ms,last_1L_deg,last_1R_deg,last_2L_deg,last_2R_deg,last_3L_deg,last_3R_deg"));

  beginNewPose();
}

void loop() {
  if (finished) return;

  // Timing
  unsigned long nowMicros = micros();
  float dt_s = (nowMicros - lastMicros) / 1e6f;
  if (dt_s <= 0) dt_s = 1e-3f;
  lastMicros = nowMicros;

  // Read all channels
  int raw[6]; float ang[6];
  for (int k=0;k<6; ++k) {
    raw[k] = readPotFiltered(POT_PINS[k]);
    ang[k] = countsToAngle(SIDE[k], raw[k]);
  }

  // Neutral non-active motors
  for (int k=0;k<6; ++k) if (k != activeMotor) esc[k].writeMicroseconds(ESC_MID);

  // Control active motor
  int i = activeMotor;
  if (!holding) {
    // derivative for D-term
    if (isnan(lastCounts[i])) lastCounts[i] = raw[i];
    float vel_cnt_s = (raw[i] - lastCounts[i]) / dt_s;
    lastCounts[i] = raw[i];
    vel_filt[i] = (1.0f - DERIV_LP_ALPHA)*vel_filt[i] + DERIV_LP_ALPHA*vel_cnt_s;

    bool timedOut = (millis() - poseStart) > MOVE_TIMEOUT_MS;

    int err  = targetCounts - raw[i];
    int aerr = abs(err);

    if (aerr <= ARRIVE_TOL || timedOut) {
      // ARRIVE or TIMEOUT → neutral, SNAPSHOT NOW, start hold
      esc[i].writeMicroseconds(ESC_MID);

      for (int k=0;k<6; ++k) snapDeg[k] = ang[k];
      int finalErrCounts = abs(targetCounts - raw[i]);
      snapReached = (!timedOut) && (finalErrCounts <= ARRIVE_TOL);

      snapshotMillis = millis();            // <-- stamp
      reach_ms = snapshotMillis - poseStart; // <-- how long this pose took
      holdStart = snapshotMillis;
      holding = true;
    } else {
      // keep trying until tolerance or full timeout
      float u_base = Kp_us_per_count * (float)err - Kd_us_per_cnt_per_s * vel_filt[i];
      int desiredSignCounts = (err > 0) ? +1 : -1;
      int sideSign          = (plusEffect[i] == desiredSignCounts) ? +1 : -1;
      float u_signed        = sideSign * fabs(u_base);
      int pwm               = pdToPulse(u_signed);
      esc[i].writeMicroseconds(pwm);
    }
  } else {
    // Holding: stay neutral
    esc[i].writeMicroseconds(ESC_MID);
  }

  // End-of-hold → log the SNAPSHOT (arrival/timeout), then advance
  if (holding && (millis() - holdStart >= HOLD_MS)) {
    Serial.print(poseIdx + 1); Serial.print(",");
    Serial.print(NAMES[activeMotor]); Serial.print(",");
    Serial.print(targetDeg, 1); Serial.print(",");
    Serial.print(snapReached ? "reached" : "not_reached"); Serial.print(",");
    Serial.print(reach_ms); Serial.print(","); // <-- new column
    Serial.print(snapDeg[0], 3); Serial.print(",");
    Serial.print(snapDeg[1], 3); Serial.print(",");
    Serial.print(snapDeg[2], 3); Serial.print(",");
    Serial.print(snapDeg[3], 3); Serial.print(",");
    Serial.print(snapDeg[4], 3); Serial.print(",");
    Serial.println(snapDeg[5], 3);

    // Next pose or finish
    poseIdx++;
    if (poseIdx >= N_POSES) {
      neutralAll();
      finished = true;
      return;
    } else {
      holding = false;
      beginNewPose();
    }
  }

}

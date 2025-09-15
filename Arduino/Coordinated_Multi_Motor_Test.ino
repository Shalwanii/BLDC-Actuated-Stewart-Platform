#include <Servo.h>

/* ===== Pins (UNO, your mapping) ===== */
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

inline int   angleToCounts(Side s, float deg) { return (int)lroundf(ZERO_COUNT + deg * ((s==LEFT_SIDE)?CNT_PER_DEG_LEFT:CNT_PER_DEG_RIGHT)); }
inline float countsToAngle(Side s, int counts){ return (counts - ZERO_COUNT) * ((s==LEFT_SIDE)?DEG_PER_COUNT_LEFT:DEG_PER_COUNT_RIGHT); }

/* ===== PD control (counts domain; NO CREEP) ===== */
float Kp_us_per_count     = 0.0025f;
float Kd_us_per_cnt_per_s = 0.50f;
const float DERIV_LP_ALPHA = 0.12f;

/* ===== Arrival / timing (NO CREEP) ===== */
const int   ARRIVE_TOL              = 20;        // ±counts to declare arrived
const unsigned long HOLD_MS         = 1000;      // neutral hold (logging)
const unsigned long MOVE_TIMEOUT_MS = 5000;     // full time allowed to reach a pose

/* ===== Safety (angle bounds only) ===== */
const float SAFE_MIN_DEG = -70.0f;
const float SAFE_MAX_DEG =  90.0f;

/* ===== POSES from Stewart IK (deg) =====
   Each row = {1L,1R,2L,2R,3L,3R}
   Replace with your IK outputs:
*/// Order per row: {1L, 1R, 2L, 2R, 3L, 3R} in degrees
const float POSES[][6] = {
  { -1.286f,   1.286f,  -7.929f, -20.571f, -25.286f, -11.571f },  // row 1
  { -5.571f, -21.000f, -13.071f,  -1.929f,  25.929f,  -0.857f },  // row 2
  {-10.500f, -20.786f, -14.571f, -12.857f,  -1.714f, -16.071f },  // row 3
  { 11.786f, -24.000f, -19.714f, -20.357f,   0.429f,   6.643f },  // row 4
  { -9.643f, -26.143f, -19.929f, -18.857f,  -8.143f,  -5.786f },  // row 5
  {-14.143f, -25.286f, -24.429f,  15.000f,  12.429f,   4.929f },  // row 6
  {  2.786f, -19.071f, -28.500f,  -6.000f,  -0.214f,   5.571f },  // row 7
  {-10.714f, -21.429f, -22.929f, -10.286f,  -5.571f,  -5.786f },  // row 8
  {-12.429f, -21.429f, -21.214f,  -1.286f,  -2.143f,  -5.357f },  // row 9
  { -2.571f, -25.500f, -33.214f,  -5.357f,   0.000f,   8.786f },  // row 10
  {-15.214f, -24.643f, -25.714f,   8.143f,   3.214f,  -0.429f },  // row 11
  {-31.500f,   7.071f,  30.000f,  17.143f,   2.786f, -31.071f },  // row 12
  {-34.286f,   1.286f,   4.714f,   4.929f,  -0.643f, -28.714f },  // row 13
  {  7.714f,   2.357f,  -2.143f, -10.500f,  -3.429f,  -8.786f },  // row 14
  { -3.214f, -13.714f, -15.857f, -20.786f, -14.571f,  -4.929f },  // row 15
  {  3.643f,  28.929f,   7.929f, -22.071f, -32.357f,  -2.786f },  // row 16
  {  3.857f,  18.214f, -13.286f, -28.286f, -33.857f,   3.214f },  // row 17
  {-12.643f,   1.714f, -16.714f, -21.214f, -29.143f,  -4.714f },  // row 18
  {-29.357f,   1.286f,  -3.643f,  27.000f,   8.786f,  -3.643f },  // row 19
  {-23.571f,  -9.857f, -18.214f,   9.000f,  -3.429f,  -8.786f },  // row 20
  {-24.214f,  30.000f,  14.571f,   5.571f, -19.714f, -15.000f },  // row 21
  {-28.929f,  32.357f,  25.714f,  25.286f,  -7.500f, -15.214f },  // row 22
  {-20.786f,  27.643f,  18.643f,  25.929f,   2.143f,   5.143f },  // row 23
  {  6.643f,   2.357f, -17.357f,   7.286f,  -0.214f,  25.714f },  // row 24
  {-12.857f,  -9.643f, -19.500f,   4.929f,   0.000f,   8.571f },  // row 25
  {-14.357f, -15.214f, -20.571f,  16.714f,  20.143f,  13.286f },  // row 26
  {-11.357f,  10.929f,  -9.857f,  12.429f,   3.857f,   8.786f },  // row 27
  { 30.214f, -12.000f, -13.286f, -19.714f,  -4.929f,  16.071f },  // row 28
  { -0.214f, -27.000f, -23.571f, -11.786f,   0.000f,  -0.857f },  // row 29
  {  1.929f, -24.429f, -22.071f, -20.357f,  -3.429f,   7.071f },  // row 30
  {  7.714f, -15.000f, -25.500f, -15.857f,  -3.643f,  16.286f },  // row 31
  { -0.857f,  -4.286f,   6.857f, -16.714f, -11.786f,  -3.214f },  // row 32
  { -6.000f, -10.500f,   0.429f, -13.929f, -10.286f,  -4.071f },  // row 33
  {-10.071f, -24.000f,  -7.286f,   1.071f,  18.857f,  -6.214f },  // row 34
  { -1.500f, -31.714f, -24.000f,  -3.857f,  17.357f,  13.286f },  // row 35
  {  0.643f, -32.357f, -31.714f,  -2.786f,  10.071f,  14.786f },  // row 36
  {  0.214f, -30.000f, -27.643f,  -7.714f,   4.286f,   1.286f },  // row 37
  {-21.429f,  -1.714f,  19.714f,   3.643f,   1.071f, -27.214f },  // row 38
  {-31.500f,  -6.000f,  12.643f,  17.143f,   9.429f, -28.071f },  // row 39
  {-28.929f,  -7.929f,   4.286f,   4.286f,   9.214f, -27.214f },  // row 40
  { 22.286f, -20.786f, -10.500f,  -9.000f,  14.571f,  12.643f },  // row 41
  {-12.000f, -26.571f, -15.214f,  -9.214f,   8.357f, -11.786f },  // row 42
  { -4.714f, -30.643f, -14.571f,  -2.143f,  25.929f,  -4.714f },  // row 43
  {-17.143f,  29.786f,  28.714f,   0.000f, -11.571f, -18.000f },  // row 44
  {-16.714f,  10.500f,   7.714f,  -6.643f, -15.429f,  -8.143f },  // row 45
  {-10.714f,  -3.000f, -15.643f, -20.786f, -28.071f,   1.071f },  // row 46
  {-13.929f,   6.429f,   7.071f, -17.786f, -29.357f,  -9.429f },  // row 47
  {-15.214f,  -0.214f,  -7.286f, -22.929f, -31.714f,  -7.714f },  // row 48
  { -9.429f,  -3.643f, -11.786f, -24.643f, -28.071f,  -4.714f },  // row 49
  {-12.857f,   9.429f,  26.143f, -12.214f, -20.357f, -16.071f },  // row 50
};


const int N_POSES = sizeof(POSES)/sizeof(POSES[0]);

/* ===== State ===== */
Servo esc[6];
int   plusEffect[6] = {-1,-1,-1,-1,-1,-1};    // +us -> +counts? (calibrated at startup)
float lastCounts[6] = {NAN,NAN,NAN,NAN,NAN,NAN};
float vel_filt[6]   = {0,0,0,0,0,0};

unsigned long lastMicros = 0;

int   poseIdx = 0;
int   tgtCounts[6];

bool  arrived[6];
unsigned long poseStart = 0;
bool  holding = false;
unsigned long holdStart = 0;
unsigned long snapshotMillis = 0;  // <-- new
unsigned long reach_ms = 0;        // <-- new
bool  finished = false;

/* Snapshot at hold entry */
float snapDeg[6];
const char* snapStatus[6];   // "reached" / "not_reached"

/* ===== Helpers ===== */
int readPotFiltered(uint8_t pin) { long s=0; for (int i=0;i<8;++i) s += analogRead(pin); return (int)(s>>3); }

void calibrateDirection(Servo& e, uint8_t potPin, int &plusEff) {
  int r0 = readPotFiltered(potPin);
  e.writeMicroseconds(ESC_MID + 30); delay(60);
  e.writeMicroseconds(ESC_MID);      delay(40);
  int r1 = readPotFiltered(potPin);
  plusEff = (r1 > r0) ? +1 : -1;     // +μs increased counts?
}

int pdToPulse(float u_us) {
  int sign = (u_us > 0) ? +1 : (u_us < 0 ? -1 : 0);
  int mag  = (int)fabs(u_us);
  if (mag < MIN_OFFSET) mag = MIN_OFFSET;
  if (mag > MAX_OFFSET) mag = MAX_OFFSET;
  return ESC_MID + sign * mag;
}

void neutralAll() { for (int i=0;i<6;++i) esc[i].writeMicroseconds(ESC_MID); }

void beginPose(int p) {
  for (int i=0;i<6;++i) {
    tgtCounts[i] = angleToCounts(SIDE[i], POSES[p][i]);
    arrived[i]   = false;
    lastCounts[i]= NAN;  // reset D-term state
    vel_filt[i]  = 0.0f;
  }
  holding = false;
  poseStart = millis();
}

/* ===== Arduino ===== */
void setup() {
  Serial.begin(115200);

  for (int i=0;i<6;++i) {
    pinMode(POT_PINS[i], INPUT);
    esc[i].attach(ESC_PINS[i]);
    esc[i].writeMicroseconds(ESC_MID);
  }
  delay(3000); // arming

  // Calibrate all channels once
  for (int i=0;i<6;++i) calibrateDirection(esc[i], POT_PINS[i], plusEffect[i]);

  lastMicros = micros();

  // CSV header: added reach_ms
  Serial.println(F("pose,"
                   "status_1L,status_1R,status_2L,status_2R,status_3L,status_3R,pose_status,reach_ms,"
                   "last_1L_deg,last_1R_deg,last_2L_deg,last_2R_deg,last_3L_deg,last_3R_deg"));

  beginPose(0);
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
  for (int i=0;i<6;++i) { raw[i] = readPotFiltered(POT_PINS[i]); ang[i] = countsToAngle(SIDE[i], raw[i]); }

  // Drive all six (NO CREEP)
  bool allArrived = true;
  bool timedOut   = (millis() - poseStart) > MOVE_TIMEOUT_MS;

  if (!holding) {
    for (int i=0;i<6;++i) {
      // Angle safety only
      if (ang[i] < SAFE_MIN_DEG || ang[i] > SAFE_MAX_DEG) {
        arrived[i] = true;  // stop trying this motor
        esc[i].writeMicroseconds(ESC_MID);
        continue;
      }

      int err = tgtCounts[i] - raw[i];
      int aerr = abs(err);

      // D-term in counts/s, filtered
      if (isnan(lastCounts[i])) lastCounts[i] = raw[i];
      float vel_cnt_s = (raw[i] - lastCounts[i]) / dt_s;
      lastCounts[i] = raw[i];
      vel_filt[i] = (1.0f - DERIV_LP_ALPHA)*vel_filt[i] + DERIV_LP_ALPHA*vel_cnt_s;

      if (aerr <= ARRIVE_TOL) {
        arrived[i] = true;
        esc[i].writeMicroseconds(ESC_MID);
      } else if (!timedOut) {
        arrived[i] = false;
        float u_base = Kp_us_per_count * (float)err - Kd_us_per_cnt_per_s * vel_filt[i];
        int desiredSignCounts = (err > 0) ? +1 : -1;
        int sideSign          = (plusEffect[i] == desiredSignCounts) ? +1 : -1;
        float u_signed        = sideSign * fabs(u_base);
        int pwm               = pdToPulse(u_signed);
        esc[i].writeMicroseconds(pwm);
      } else {
        // timed out: stop driving this channel
        arrived[i] = false; // will be not_reached
        esc[i].writeMicroseconds(ESC_MID);
      }

      if (!arrived[i]) allArrived = false;
    }

    // Enter HOLD when either all in tolerance OR timeout elapsed
    if (allArrived || timedOut) {
      // Snapshot once at hold entry; decide per-motor status here
      for (int i=0;i<6;++i) {
        snapDeg[i] = ang[i];
        int finalErrCounts = abs(tgtCounts[i] - raw[i]);
        bool thisReached   = (finalErrCounts <= ARRIVE_TOL) && !timedOut;
        snapStatus[i] = thisReached ? "reached" : "not_reached";
        esc[i].writeMicroseconds(ESC_MID); // neutral during hold
      }
      snapshotMillis = millis();                 // <-- stamp
      reach_ms = snapshotMillis - poseStart;     // <-- how long pose took
      holdStart = snapshotMillis;
      holding = true;
    }
  } else {
    // In HOLD: remain neutral (NO CREEP)
    for (int i=0;i<6;++i) esc[i].writeMicroseconds(ESC_MID);

    // End of hold → log & advance
    if (millis() - holdStart >= HOLD_MS) {
      bool poseOK = true;
      for (int i=0;i<6;++i) if (snapStatus[i][0] == 'n') poseOK = false;

      Serial.print(poseIdx + 1); Serial.print(",");
      Serial.print(snapStatus[0]); Serial.print(",");
      Serial.print(snapStatus[1]); Serial.print(",");
      Serial.print(snapStatus[2]); Serial.print(",");
      Serial.print(snapStatus[3]); Serial.print(",");
      Serial.print(snapStatus[4]); Serial.print(",");
      Serial.print(snapStatus[5]); Serial.print(",");
      Serial.print(poseOK ? "reached" : "not_reached"); Serial.print(",");
      Serial.print(reach_ms); Serial.print(",");  // <-- new column
      Serial.print(snapDeg[0], 3); Serial.print(",");
      Serial.print(snapDeg[1], 3); Serial.print(",");
      Serial.print(snapDeg[2], 3); Serial.print(",");
      Serial.print(snapDeg[3], 3); Serial.print(",");
      Serial.print(snapDeg[4], 3); Serial.print(",");
      Serial.println(snapDeg[5], 3);

      // Next pose
      poseIdx++;
      if (poseIdx >= N_POSES) {
        neutralAll();
        finished = true;
        return;
      } else {
        holding = false;
        beginPose(poseIdx);
      }
    }
  }

  delay(2);
}

/*
 * ================================================================
 * SUMO ROBOT — ESP32-S3  |  Full FSM v2.0
 * ================================================================
 * Pinout:
 *   I2C SDA=19 SCL=18
 *   ToF: FRONT(23/0x2A) FL(32/0x2B) FR(5/0x2C) L(25/0x2D) R(16/0x2E)
 *   TCRT: FL=14  FR=27  REAR=13
 *   PCA9685 0x40: CH0=RPWM_L CH1=LPWM_L CH2=RPWM_R CH3=LPWM_R
 *   MPU6050 0x68
 * ================================================================
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Bật/tắt debug Serial (TẮT khi thi đấu để giảm jitter) ──────────────
#define DEBUG_SERIAL  1   // 0 = tắt hoàn toàn Serial print

#if DEBUG_SERIAL
  #define DLOG(...)   Serial.printf(__VA_ARGS__)
#else
  #define DLOG(...)
#endif

// ================================================================
// I. PINOUT
// ================================================================
#define I2C_SDA        19
#define I2C_SCL        18
#define PIN_TCRT_FL    14
#define PIN_TCRT_FR    27
#define PIN_TCRT_REAR  13

#define PCA9685_ADDR   0x40
#define PWM_FREQ       1000
#define RPWM_LEFT      0
#define LPWM_LEFT      1
#define RPWM_RIGHT     2
#define LPWM_RIGHT     3
SemaphoreHandle_t i2c_mutex = NULL;
// ================================================================
// II. CONSTANTS & THRESHOLDS
// ================================================================
const uint16_t CONF_ENY    = 470;
const uint16_t WARN_DIST   = 250;
const uint16_t STRIKE_DIST = 100;

const uint32_t MIN_STT_TIME     = 20;
const uint32_t PUSH_MS          = 500;
const uint32_t HOLD_PUSH_MS     = 500;
const uint32_t ATK_LOCK_TIME    = 500;
const uint32_t SIDE_DANGER_TIME = 40;
const uint32_t FLK_STABLE_TIME  = 50;
const uint32_t IGNORE_ANTI_PUSH = 200;
const uint32_t EDGE_TIMEOUT     = 80;
const uint32_t MAX_RECOVER_TIME = 800;
const uint32_t LOST_TIMEOUT     = 1500;
const uint32_t FSM_WATCHDOG_MS  = 3000; 

const uint8_t  MAX_LOCK_RETRIES = 2;

// TCRT — chỉnh sau khi đo Serial thực tế
const uint16_t THRES_FL         = 300;
const uint16_t THRES_FR         = 300;
const uint16_t THRES_REAR       = 300;
const uint16_t THRES_HYST       = 200;  

// IMU
const float ACC_IMPACT_TH = 6.0f;
const float PITCH_LIFT_TH = 20.0f;
const float ROLL_LIFT_TH  = 20.0f;

// Motor speed
const float SPD_FULL    = 100.0f;
const float SPD_PUSH    =  85.0f;
const float SPD_FORWARD =  65.0f;
const float SPD_LOCK    =  50.0f;
const float SPD_TURN    =  55.0f;
const float SPD_SCAN    =  35.0f;
const float SPD_RECOVER =  60.0f;

// Gyro PID (ATK_LOCK)
const float PID_KP = 1.2f;
const float PID_KI = 0.5f;
const float PID_KD = 0.3f;

uint8_t  global_strike_fail = 0;
uint32_t last_strategy_change = 0;
// ===============================================================
// III. STATE SPACE
// ================================================================
enum RobotState {
  STATE_IDLE, STATE_INIT_DELAY,
  STATE_ATK_STRIKE, STATE_ATK_FLANK_FRONT, STATE_ATK_FLANK_SIDE,
  STATE_ATK_FLANK_REAR, STATE_ATK_FEINT, STATE_ATK_DELAY_RUSH, STATE_ATK_LOCK,
  STATE_DEF_ANTI_PUSH, STATE_DEF_SIDE_GUARD, STATE_DEF_REAR_GUARD,
  STATE_DEF_EDGE_AVOID, STATE_DEF_ANTI_LIFT, STATE_DEF_LAST_STAND,
  STATE_REC_RECOVER, STATE_SEARCH_ENEMY
};

const char* stateName(RobotState s) {
  switch(s) {
    case STATE_IDLE:            return "IDLE";
    case STATE_INIT_DELAY:      return "INIT_DELAY";
    case STATE_ATK_STRIKE:      return "ATK_STRIKE";
    case STATE_ATK_FLANK_FRONT: return "ATK_FLANK_FRONT";
    case STATE_ATK_FLANK_SIDE:  return "ATK_FLANK_SIDE";
    case STATE_ATK_FLANK_REAR:  return "ATK_FLANK_REAR";
    case STATE_ATK_FEINT:       return "ATK_FEINT";
    case STATE_ATK_DELAY_RUSH:  return "ATK_DELAY_RUSH";
    case STATE_ATK_LOCK:        return "ATK_LOCK";
    case STATE_DEF_ANTI_PUSH:   return "DEF_ANTI_PUSH";
    case STATE_DEF_SIDE_GUARD:  return "DEF_SIDE_GUARD";
    case STATE_DEF_REAR_GUARD:  return "DEF_REAR_GUARD";
    case STATE_DEF_EDGE_AVOID:  return "DEF_EDGE_AVOID";
    case STATE_DEF_ANTI_LIFT:   return "DEF_ANTI_LIFT";
    case STATE_DEF_LAST_STAND:  return "DEF_LAST_STAND";
    case STATE_REC_RECOVER:     return "REC_RECOVER";
    case STATE_SEARCH_ENEMY:    return "SEARCH_ENEMY";
    default:                    return "UNKNOWN";
  }
}

// ── State management ─────────────────────────────────────────────────────
volatile RobotState currentState = STATE_IDLE;
volatile RobotState prevState    = STATE_IDLE;
volatile uint32_t   stateEnterMs = 0;

uint32_t stateElapsedMs() { return millis() - stateEnterMs; }

void changeState(RobotState s) {
  if (s == currentState) return;
  if (stateElapsedMs() < MIN_STT_TIME) return;
  DLOG("[FSM] %s -> %s\n", stateName(currentState), stateName(s));
  prevState    = currentState;
  currentState = s;
  stateEnterMs = millis();
}

void forceState(RobotState s) {
  if (s == currentState) return;
  DLOG("[FORCE] %s -> %s\n", stateName(currentState), stateName(s));
  prevState    = currentState;
  currentState = s;
  stateEnterMs = millis();
}

// ================================================================
// IV. TOF — với median filter 3 mẫu
// ================================================================
struct ToFSensor {
  const char       *name;
  uint8_t           xshut, addr;
  Adafruit_VL53L0X  drv;
  bool              active;
  uint16_t          dist;
  uint16_t          buf[3];  
  uint8_t           buf_idx;
};

ToFSensor tof[] = {
  { "FRONT",       23, 0x2A, Adafruit_VL53L0X(), false, 8190, {8190,8190,8190}, 0 },
  { "FRONT_LEFT",  32, 0x2B, Adafruit_VL53L0X(), false, 8190, {8190,8190,8190}, 0 },
  { "FRONT_RIGHT",  5, 0x2C, Adafruit_VL53L0X(), false, 8190, {8190,8190,8190}, 0 },
  { "LEFT",        25, 0x2D, Adafruit_VL53L0X(), false, 8190, {8190,8190,8190}, 0 },
  { "RIGHT",       16, 0x2E, Adafruit_VL53L0X(), false, 8190, {8190,8190,8190}, 0 },
};
const int TOF_N = 5;

#define d0    tof[0].dist
#define d_m45 tof[1].dist
#define d_p45 tof[2].dist
#define d_m90 tof[3].dist
#define d_p90 tof[4].dist

// [NEW] Median 3 mẫu — loại bỏ spike đơn lẻ
uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void tof_init() {
  DLOG("[ToF] Khởi tạo...\n");
  for (int i = 0; i < TOF_N; i++) {
    pinMode(tof[i].xshut, OUTPUT);
    digitalWrite(tof[i].xshut, LOW);
  }
  delay(100);
  for (int i = 0; i < TOF_N; i++) {
    digitalWrite(tof[i].xshut, HIGH);
    delay(20);
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0) {
      DLOG("  ❌ %s\n", tof[i].name);
      tof[i].active = false;
      digitalWrite(tof[i].xshut, LOW);
      continue;
    }
    tof[i].drv.begin(0x29, false, &Wire);
    tof[i].drv.setAddress(tof[i].addr);
    tof[i].drv.setMeasurementTimingBudgetMicroSeconds(20000);
    tof[i].drv.startRangeContinuous();
    tof[i].active = true;
    DLOG("  ✅ %s -> 0x%02X\n", tof[i].name, tof[i].addr);
  }
  DLOG("\n");
}

void tof_read() {
  for (int i = 0; i < TOF_N; i++) {
    if (!tof[i].active) continue;

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(3)) != pdTRUE) continue;
    bool ready = tof[i].drv.isRangeComplete();
    uint16_t raw = ready ? tof[i].drv.readRangeResult() : tof[i].dist;
    xSemaphoreGive(i2c_mutex);

    if (!ready) continue;
    raw = (raw > 8000) ? 8190 : raw;
    tof[i].buf[tof[i].buf_idx] = raw;
    tof[i].buf_idx = (tof[i].buf_idx + 1) % 3;
    tof[i].dist = median3(tof[i].buf[0], tof[i].buf[1], tof[i].buf[2]);
  }
}

// ================================================================
// V. IMU — Complementary filter
// ================================================================
volatile float imu_pitch = 0, imu_roll = 0, imu_yaw = 0;
volatile float current_PWM_L = 0, current_PWM_R = 0;

Adafruit_MPU6050 mpu;
bool     mpu_ok     = false;
float    accel_x    = 0, accel_y  = 0, accel_z = 9.8f;
float    prev_ax    = 0, prev_ay  = 0;
uint32_t last_imu_t = 0;
static const float RAD2DEG = 180.0f / PI;

void imu_init() {
  if (!mpu.begin()) {
    DLOG("[MPU6050] ❌ Không tìm thấy!\n");
    mpu_ok = false; return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  last_imu_t = millis();
  mpu_ok = true;
  DLOG("[MPU6050] ✅ OK\n");
}

void imu_read() {
  if (!mpu_ok) return;
  sensors_event_t a, g, t;

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(3)) != pdTRUE) return;
  mpu.getEvent(&a, &g, &t);
  xSemaphoreGive(i2c_mutex);

  uint32_t now = millis();
  float dt = (now - last_imu_t) / 1000.0f;
  if (dt <= 0 || dt > 0.5f) { last_imu_t = now; return; }
  last_imu_t = now;

  accel_x = a.acceleration.x;
  accel_y = a.acceleration.y;
  accel_z = a.acceleration.z;

  // Gyro integrate mỗi 5ms — dùng RAD2DEG constant
  imu_pitch += g.gyro.x * RAD2DEG * dt;
  imu_roll  += g.gyro.y * RAD2DEG * dt;
  imu_yaw   += g.gyro.z * RAD2DEG * dt;

  // atan2f đắt → chỉ correct drift mỗi 50ms (mỗi 10 lần read)
  static uint32_t last_accel_correct = 0;
  if (now - last_accel_correct >= 50) {
    last_accel_correct = now;
    float ap = atan2f(-accel_x, accel_z) * RAD2DEG;
    float ar = atan2f( accel_y, accel_z) * RAD2DEG;
    imu_pitch = 0.98f * imu_pitch + 0.02f * ap;
    imu_roll  = 0.98f * imu_roll  + 0.02f * ar;
  }

  if (imu_yaw >  180.0f) imu_yaw -= 360.0f;
  if (imu_yaw < -180.0f) imu_yaw += 360.0f;
}

// ================================================================
// VI. MOTOR
// ================================================================
Adafruit_PWMServoDriver pca(PCA9685_ADDR);

inline float _cl(float v, float lo, float hi)  {
  return v < lo ? lo : (v > hi ? hi : v);
}

void motor_control_fast(float l, float r) {
  l = _cl(l, -100.0f, 100.0f);
  r = _cl(r, -100.0f, 100.0f);

  float r_inv = -r;  // đảo chiều motor phải

  uint16_t rpwm_l = (l    > 0) ? (uint16_t)( l    / 100.0f * 4095.0f) : 0;
  uint16_t lpwm_l = (l    < 0) ? (uint16_t)(-l    / 100.0f * 4095.0f) : 0;
  uint16_t rpwm_r = (r_inv > 0) ? (uint16_t)( r_inv / 100.0f * 4095.0f) : 0;
  uint16_t lpwm_r = (r_inv < 0) ? (uint16_t)(-r_inv / 100.0f * 4095.0f) : 0;

  uint8_t buf[16] = {
    0x00, 0x00, (uint8_t)(rpwm_l & 0xFF), (uint8_t)(rpwm_l >> 8),
    0x00, 0x00, (uint8_t)(lpwm_l & 0xFF), (uint8_t)(lpwm_l >> 8),
    0x00, 0x00, (uint8_t)(rpwm_r & 0xFF), (uint8_t)(rpwm_r >> 8),
    0x00, 0x00, (uint8_t)(lpwm_r & 0xFF), (uint8_t)(lpwm_r >> 8),
  };
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(4)) != pdTRUE) return;
  current_PWM_L = l;
  current_PWM_R = r;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06);
  Wire.write(buf, 16);
  Wire.endTransmission();
  xSemaphoreGive(i2c_mutex);
}

void motor_stop()     { motor_control_fast( 0,           0);           }
void motor_forward()  { motor_control_fast( SPD_FORWARD,  SPD_FORWARD); }
void motor_backward() { motor_control_fast(-SPD_RECOVER, -SPD_RECOVER); }
void motor_attack()   { motor_control_fast( SPD_PUSH,     SPD_PUSH);    }
void motor_full()     { motor_control_fast( SPD_FULL,     SPD_FULL);    }
void motor_turn_left(float s)  { motor_control_fast(-s,  s); }
void motor_turn_right(float s) { motor_control_fast( s, -s); }

// ================================================================
// VII. CONTINUOUS EVALUATION
// ================================================================
uint16_t tcrt_fl_raw = 0, tcrt_fr_raw = 0, tcrt_rear_raw = 0;

// [NEW] TCRT hysteresis state
bool tcrt_fl_state   = false;
bool tcrt_fr_state   = false;
bool tcrt_rear_state = false;

bool edgeDetect = false, edge_fl = false, edge_fr = false, edge_rear = false;
volatile bool closingFast        = false;
volatile bool sideDanger         = false;
volatile bool impactDetected     = false;
volatile bool beingLifted        = false;
volatile bool fallOut            = false;
volatile bool flkPossible        = false;
volatile bool isTargetLost       = true;
volatile float enemy_angle_est   = 0;
volatile float enemy_angle_last  = 0;
bool maneuverInitiatedByRobot = false;

uint16_t prev_d0_val  = 8190;
uint32_t prev_d0_time = 0;
uint32_t side_danger_start = 0;
bool     side_danger_raw   = false;
uint32_t flk_start    = 0;
bool     flk_candidate = false;
uint32_t last_strike_time = 0;
uint32_t last_edge_time   = 0;

uint32_t last_seen_time = 0;

float angle_histogram[50] = {0};
int   hist_idx = 0;
uint8_t lock_retries = 0;

// ── [NEW] Gyro PID cho ATK_LOCK ─────────────────────────────────────────
float pid_integral  = 0;
float pid_prev_err  = 0;
uint32_t pid_last_t = 0;

float gyro_pid(float target_angle) {
  uint32_t now = millis();
  float dt = (now - pid_last_t) / 1000.0f;
  pid_last_t = now;
  if (dt <= 0 || dt > 0.5f) return 0;

  float err = target_angle - enemy_angle_est;
  pid_integral += err * dt;
  pid_integral  = _cl(pid_integral, -30.0f, 30.0f); // Anti-windup
  float deriv   = (err - pid_prev_err) / dt;
  pid_prev_err  = err;

  return _cl(PID_KP * err + PID_KI * pid_integral + PID_KD * deriv,
             -SPD_FULL, SPD_FULL);
}

// Thêm vào khu vực khai báo flags
uint8_t  strike_count    = 0;
uint32_t strike_count_ts = 0;
const uint8_t RETRY_LIMIT = 7;

uint16_t d0_before_strike = 8190;


void reset_pid() {
  pid_integral = 0;
  pid_prev_err = 0;
  pid_last_t   = millis();
}

// ── Tính góc địch từ 5 ToF (weighted average + IMU fusion) ──────────────
void update_enemy_angle() {
  // static const → tính 1 lần, không tạo trên stack mỗi lần gọi
  static const float ANGLES[5] = { 0.0f, -45.0f, 45.0f, -90.0f, 90.0f };

  float sw = 0, sa = 0;
  bool  any = false;
  for (int i = 0; i < TOF_N; i++) {
    uint16_t d = tof[i].dist;
    if (d < CONF_ENY && d > 0) {
      float w = 1.0f / (float)d;
      sa += ANGLES[i] * w;
      sw += w;
      any = true;
    }
  }
  if (any) {
    float tof_angle  = sa / sw;
    enemy_angle_est = tof_angle;   // ToF đã đủ chính xác
    imu_yaw = 0;                   // Reset yaw mỗi khi thấy địch → chống drift
    enemy_angle_last = enemy_angle_est;
    last_seen_time   = millis();
    isTargetLost     = false;
  } else {
    if (millis() - last_seen_time > LOST_TIMEOUT) isTargetLost = true;
    enemy_angle_est = enemy_angle_last;
  }
}

void update_closingFast() {
  static uint16_t prev = 8190;
  static uint32_t pt   = 0;
  uint32_t now = millis();
  if (now - pt < 40) return;

  bool robot_is_pushing = (current_PWM_L > 70 && current_PWM_R > 70);

   if (!robot_is_pushing && d0 != 8190 && prev != 8190)
    closingFast = ((int)prev - (int)d0 > 30);
  else
    closingFast = false;
  prev = d0; pt = now;
}
void update_flags() {
  uint32_t now = millis();
  // ── closingFast ──────────────────────────────────────────────────────
  update_closingFast();

  // ── sideDanger debounce ──────────────────────────────────────────────
  side_danger_raw = (d_m45 < WARN_DIST || d_p45 < WARN_DIST ||
                     d_m90 < WARN_DIST || d_p90 < WARN_DIST);
  if (side_danger_raw) {
    if (!side_danger_start) side_danger_start = now;
    sideDanger = (now - side_danger_start >= SIDE_DANGER_TIME);
  } else {
    side_danger_start = 0; sideDanger = false;
  }

  // ── impactDetected ───────────────────────────────────────────────────
  float dax = fabsf(accel_x - prev_ax);
  float day = fabsf(accel_y - prev_ay);
  impactDetected = ((dax > ACC_IMPACT_TH || day > ACC_IMPACT_TH) &&
                    d0 < STRIKE_DIST + 20);
  prev_ax = accel_x; prev_ay = accel_y;

  // ── beingLifted ──────────────────────────────────────────────────────
  bool tilt  = (fabsf(imu_pitch) > PITCH_LIFT_TH ||
                fabsf(imu_roll)  > ROLL_LIFT_TH);
  bool light = (accel_z < 7.0f);
  beingLifted = (tilt && light && mpu_ok);

  // ── fallOut ──────────────────────────────────────────────────────────
  fallOut = (tilt && (now - last_edge_time < 500) && mpu_ok);

  // ── flkPossible ──────────────────────────────────────────────────────
  bool flk_now = (!isTargetLost) && (d0 > CONF_ENY) &&
                 (d_m45 < CONF_ENY || d_p45 < CONF_ENY ||
                  d_m90 < CONF_ENY || d_p90 < CONF_ENY);
  if (flk_now) {
    if (!flk_candidate) { flk_candidate = true; flk_start = now; }
    flkPossible = (now - flk_start >= FLK_STABLE_TIME);
  } else { flk_candidate = false; flkPossible = false; }

  update_enemy_angle();
}

// ================================================================
// VIII. TRACKING ENGINE
// ================================================================
void run_tracking() {
  float angle = enemy_angle_est;
  uint16_t best = min({d0, d_m45, d_p45, d_m90, d_p90});

  if (best < STRIKE_DIST)         { motor_full(); return; }
  if (fabsf(angle) > 60.0f) {
    if (angle > 0) motor_turn_right(SPD_TURN);
    else           motor_turn_left(SPD_TURN);
    return;
  }
  float steer = (angle / 90.0f) * SPD_TURN;
  motor_control_fast(_cl(SPD_FORWARD + steer, -SPD_FULL, SPD_FULL),
                _cl(SPD_FORWARD - steer, -SPD_FULL, SPD_FULL));
}

// ================================================================
// IX. FSM STATES
// ================================================================

void run_idle() {
  motor_stop();
  // Build angle histogram
  static uint32_t last_hist = 0;
  if (millis() - last_hist >= 20 && hist_idx < 50) {
    angle_histogram[hist_idx++] = enemy_angle_last;
    last_hist = millis();
  }
  if (stateElapsedMs() >= 5000) changeState(STATE_INIT_DELAY);
}

void run_init_delay() {
  motor_stop();
  if (stateElapsedMs() < 3000) return;
  float sum = 0;
  for (int i = 0; i < hist_idx; i++) sum += angle_histogram[i];
  if (hist_idx > 0) {
    enemy_angle_last = sum / hist_idx;
    enemy_angle_est  = enemy_angle_last;
  }
  changeState(isTargetLost ? STATE_SEARCH_ENEMY : STATE_ATK_LOCK);
}

void run_search() {
  uint32_t t = stateElapsedMs();  // reset tự động mỗi lần vào state

  if      (t < 1000)  motor_turn_right(SPD_SCAN);
  else if (t < 2000)  motor_turn_left(SPD_SCAN);
  else if (t < 2300)  motor_forward();
  else                forceState(STATE_SEARCH_ENEMY);  // restart chu kỳ

  if (d0 < CONF_ENY) {
    lock_retries = 0;
    changeState(STATE_ATK_LOCK);
  } else if (d_m90 < CONF_ENY || d_p90 < CONF_ENY ||
             d_m45 < CONF_ENY || d_p45 < CONF_ENY) {
    changeState(STATE_ATK_FLANK_SIDE);
  }
}

void run_atk_lock() {
  uint32_t now     = millis();
  uint32_t elapsed = now - stateEnterMs;

  // Debounce: chỉ đếm thất bại 1 lần mỗi lần vào state [FIX]
  static bool fail_counted = false;
  if (elapsed < MIN_STT_TIME) fail_counted = false;  // reset khi mới vào

  if (!fail_counted && elapsed >= PUSH_MS + HOLD_PUSH_MS && d0 >= STRIKE_DIST) {
    fail_counted = true;
    if (++global_strike_fail >= 3) {
      global_strike_fail = 0;
      if (now - last_strategy_change > 3000) {
        last_strategy_change = now;
        changeState(STATE_REC_RECOVER);
        return;
      }
    }
  }

  if (d0 < STRIKE_DIST) { global_strike_fail = 0; lock_retries = 0; }
  if (isTargetLost)     { lock_retries = 0; changeState(STATE_SEARCH_ENEMY); return; }

  float correction = gyro_pid(0.0f);
  motor_control_fast(_cl(SPD_LOCK - correction, -SPD_FULL, SPD_FULL),
                     _cl(SPD_LOCK + correction, -SPD_FULL, SPD_FULL));

  bool aligned     = (fabsf(enemy_angle_est) < 10.0f && d0 < WARN_DIST);
  bool timed_out   = (elapsed >= ATK_LOCK_TIME);
  bool enemy_static = (!closingFast && d0 < WARN_DIST &&
                       d0 > STRIKE_DIST && fabsf(enemy_angle_est) < 15.0f);

  if (enemy_static && !sideDanger) { changeState(STATE_ATK_DELAY_RUSH); return; }
  if (flkPossible && !closingFast) { changeState(STATE_ATK_FEINT);       return; }

  if (aligned || timed_out) {
    reset_pid();
    if (++lock_retries >= MAX_LOCK_RETRIES) {
      lock_retries = 0; changeState(STATE_SEARCH_ENEMY);
    } else {
      changeState(STATE_ATK_STRIKE);
    }
  }
}

void run_atk_strike() {
  uint32_t t = stateElapsedMs();

  static bool strike_initialized = false;

  // Reset nếu mới vào state (elapsed < 1 FSM tick = 30ms)
  if (t < 30) strike_initialized = false;

  if (!strike_initialized) {
    strike_initialized = true;
    d0_before_strike   = d0;

    if (millis() - strike_count_ts > 1000) {
      strike_count    = 0;
      strike_count_ts = millis();
    }
    strike_count++;
    if (strike_count >= RETRY_LIMIT) {
      strike_count       = 0;
      strike_initialized = false;  // reset cho lần vào state sau
      changeState(STATE_ATK_FLANK_SIDE);
      return;
    }
  }
  last_strike_time = millis();
  if (t < PUSH_MS)                { motor_attack(); return; }
  if (t < PUSH_MS + HOLD_PUSH_MS) { motor_full();   return; }

  // Phase 3 outcome
  int outcome = (int)d0_before_strike - (int)d0;
  strike_initialized = false;  // reset trước khi đổi state

  if (outcome > 50) {
    lock_retries = 0;
    motor_full();
    if (d0 < STRIKE_DIST)         return;
    if (isTargetLost)             { changeState(STATE_SEARCH_ENEMY); return; }
    changeState(STATE_ATK_LOCK);
  } else if (outcome < -30) {
    forceState(STATE_DEF_ANTI_PUSH);
  } else {
    changeState(STATE_ATK_FLANK_SIDE);
  }
}

void run_atk_delay_rush() {
  motor_control_fast(SPD_SCAN, SPD_SCAN);
  if (closingFast || stateElapsedMs() >= 400) changeState(STATE_ATK_STRIKE);
}

void run_atk_flank_front() {
  maneuverInitiatedByRobot = true;
  if (isTargetLost)           { changeState(STATE_REC_RECOVER);    return; }
  if (impactDetected) {
    motor_full();
    if (d0 > CONF_ENY)        { changeState(STATE_REC_RECOVER); }
    return;
  }
  if (fabsf(enemy_angle_est) > 25.0f) { changeState(STATE_ATK_FLANK_SIDE); return; }
  float steer = (enemy_angle_est / 25.0f) * 15.0f;
  motor_control_fast(_cl(SPD_FORWARD + steer, 0, SPD_FULL),
                _cl(SPD_FORWARD - steer, 0, SPD_FULL));
}

void run_atk_feint() {
  if (isTargetLost) { changeState(STATE_SEARCH_ENEMY); return; }
  uint32_t t = stateElapsedMs();
  if (t < 200)      motor_turn_right(SPD_SCAN);
  else if (t < 400) motor_turn_left(SPD_SCAN);
  else               changeState(STATE_ATK_STRIKE);
}

void run_atk_flank_side() {
  maneuverInitiatedByRobot = true;
  if (isTargetLost)  { changeState(STATE_SEARCH_ENEMY); return; }
  if (impactDetected) {
    motor_full();
    if (d0 > CONF_ENY && stateElapsedMs() > 300) changeState(STATE_REC_RECOVER);
    return;
  }
  if (d0 < CONF_ENY) { changeState(STATE_ATK_LOCK); return; }
  run_tracking();
}

void run_atk_flank_rear() {
  maneuverInitiatedByRobot = true;
  if (isTargetLost) { changeState(STATE_SEARCH_ENEMY); return; }
  if (d0 < CONF_ENY) { changeState(STATE_ATK_LOCK); return; }
  if (enemy_angle_est > 0) motor_control_fast(SPD_FORWARD, -SPD_SCAN);
  else                      motor_control_fast(-SPD_SCAN,  SPD_FORWARD);
}

void run_def_edge_avoid() {
  if      (edge_fl && edge_fr) motor_backward();
  else if (edge_fl)            motor_control_fast(-SPD_FORWARD, -SPD_SCAN);
  else if (edge_fr)            motor_control_fast(-SPD_SCAN, -SPD_FORWARD);
  else if (edge_rear)          motor_forward();

  static uint32_t edge_clear_t = 0;
  if (stateElapsedMs() < MIN_STT_TIME) edge_clear_t = 0;

  if (!edgeDetect) {
    if (!edge_clear_t) edge_clear_t = millis();
    if (millis() - edge_clear_t >= EDGE_TIMEOUT) {
      edge_clear_t = 0;
      changeState(STATE_REC_RECOVER);
    }
  } else {
    edge_clear_t = 0;
  }
}

void run_def_last_stand() {
  if (!mpu_ok) { changeState(STATE_REC_RECOVER); return; }
  if      (imu_pitch < -10.0f) motor_backward();
  else if (imu_pitch >  10.0f) motor_forward();
  else if (imu_roll  >  15.0f) motor_control_fast(-SPD_FORWARD, SPD_SCAN);
  else if (imu_roll  < -15.0f) motor_control_fast(SPD_SCAN, -SPD_FORWARD);
  else                          motor_stop();
  bool flat = (fabsf(imu_pitch) < 5.0f && fabsf(imu_roll) < 5.0f);
  if (flat && !fallOut) changeState(STATE_REC_RECOVER);
}

void run_def_anti_lift() {
  if ((millis() / 300) % 2 == 0) motor_control_fast(-SPD_FULL, -SPD_SCAN);
  else                             motor_control_fast(-SPD_SCAN, -SPD_FULL);
  if (!beingLifted) changeState(STATE_REC_RECOVER);
}

void run_def_anti_push() {
  uint32_t t = stateElapsedMs();
  if (t < 150)      motor_backward();
  else if (t < 450) {
    if ((t / 100) % 2 == 0) motor_turn_right(SPD_TURN);
    else                                motor_turn_left(SPD_TURN);
  } else changeState(STATE_REC_RECOVER);
}

void run_def_side_guard() {
  if (d_m90 < WARN_DIST || d_m45 < WARN_DIST)
    motor_control_fast(-SPD_SCAN, -SPD_FORWARD);
  else
    motor_control_fast(-SPD_FORWARD, -SPD_SCAN);
  if (stateElapsedMs() > 300) changeState(STATE_ATK_LOCK);
}

void run_def_rear_guard() {
  if (enemy_angle_last > 0) motor_control_fast(SPD_FORWARD, SPD_SCAN);
  else                        motor_control_fast(SPD_SCAN, SPD_FORWARD);
  if (stateElapsedMs() > 500)
    changeState(flkPossible ? STATE_ATK_FLANK_SIDE : STATE_REC_RECOVER);
}

void run_recover() {
  uint32_t t = stateElapsedMs();
  if (t < 300)      motor_backward();
  else if (t < 600) {
    if (enemy_angle_last >= 0) motor_turn_right(SPD_TURN);
    else                        motor_turn_left(SPD_TURN);
  } else {
    changeState(isTargetLost ? STATE_SEARCH_ENEMY : STATE_ATK_LOCK);
  }
  if (t >= MAX_RECOVER_TIME) changeState(STATE_SEARCH_ENEMY);
}

// ================================================================
// X. MAIN FSM RUNNER
// ================================================================
void run_fsm_normal() {
  // Emergency override — ưu tiên cao nhất, xử lý trước watchdog
  if (edgeDetect    && currentState != STATE_DEF_EDGE_AVOID) {
    forceState(STATE_DEF_EDGE_AVOID); // chỉ đổi state
    // motor sẽ được điều khiển trong run_def_edge_avoid() ngay vòng này
  } else if (fallOut   && currentState != STATE_DEF_LAST_STAND) {
    forceState(STATE_DEF_LAST_STAND);
  } else if (beingLifted && currentState != STATE_DEF_ANTI_LIFT) {
    forceState(STATE_DEF_ANTI_LIFT);
  }

  // ── [NEW] FSM Watchdog — tự thoát nếu treo > 3s ─────────────────────
  if (stateElapsedMs() > FSM_WATCHDOG_MS &&
    currentState != STATE_IDLE &&
    currentState != STATE_INIT_DELAY) {
    DLOG("[WDT] State %s treo > 3s, force SEARCH\n", stateName(currentState));
    forceState(STATE_SEARCH_ENEMY);
  }

  // ── INTERRUPT CONDITIONS ─────────────────────────────────────────────
  bool anti_push_ok = (millis() - last_strike_time > IGNORE_ANTI_PUSH);
  if (sideDanger && anti_push_ok && impactDetected &&
      !maneuverInitiatedByRobot && currentState == STATE_ATK_STRIKE) {
    forceState(STATE_DEF_ANTI_PUSH);
  }
  if (impactDetected && sideDanger &&
      (d_m90 < WARN_DIST || d_p90 < WARN_DIST) &&
      fabsf(enemy_angle_est) > 45.0f &&
      !maneuverInitiatedByRobot &&
      currentState != STATE_DEF_SIDE_GUARD &&
      currentState != STATE_DEF_EDGE_AVOID) {
    forceState(STATE_DEF_SIDE_GUARD);
  }

  // Reset maneuver flag
  if (currentState != STATE_ATK_FLANK_FRONT &&
      currentState != STATE_ATK_FLANK_SIDE  &&
      currentState != STATE_ATK_FLANK_REAR  &&
      currentState != STATE_ATK_FEINT)
    maneuverInitiatedByRobot = false;

  // ── NORMAL FSM ───────────────────────────────────────────────────────
  switch (currentState) {
    case STATE_IDLE:            run_idle();            break;
    case STATE_INIT_DELAY:      run_init_delay();      break;
    case STATE_SEARCH_ENEMY:    run_search();          break;
    case STATE_ATK_LOCK:        run_atk_lock();        break;
    case STATE_ATK_STRIKE:      run_atk_strike();      break;
    case STATE_ATK_FEINT:       run_atk_feint();       break;
    case STATE_ATK_DELAY_RUSH:  run_atk_delay_rush();  break;
    case STATE_ATK_FLANK_FRONT: run_atk_flank_front(); break;
    case STATE_ATK_FLANK_SIDE:  run_atk_flank_side();  break;
    case STATE_ATK_FLANK_REAR:  run_atk_flank_rear();  break;
    case STATE_DEF_ANTI_PUSH:   run_def_anti_push();   break;
    case STATE_DEF_SIDE_GUARD:  run_def_side_guard();  break;
    case STATE_DEF_REAR_GUARD:  run_def_rear_guard();  break;
    case STATE_DEF_EDGE_AVOID:  run_def_edge_avoid();  break;
    case STATE_DEF_ANTI_LIFT:   run_def_anti_lift();   break;
    case STATE_DEF_LAST_STAND:  run_def_last_stand();  break;
    case STATE_REC_RECOVER:     run_recover();         break;
    default:                    changeState(STATE_SEARCH_ENEMY); break;
  }
}

// ================================================================
// XI. DUAL CORE TASKS
// ================================================================
// Core 0: Sensor + Flags (tách riêng để không block FSM)
void task_sensors(void* param) {
  uint32_t t_tof   = 0;
  uint32_t t_imu   = 0;
  for (;;) {
    uint32_t now = millis();

    // IMU ưu tiên cao nhất — 5ms/lần (200Hz)
    if (now - t_imu >= 5) {
      t_imu = now;
      imu_read();
    }

    // ToF + flags cùng 1 chu kỳ — không tách rời
    if (now - t_tof >= 20) {
      t_tof = now;
      tof_read();
      update_flags(); // ← Chạy NGAY sau ToF, không chờ timer riêng
    }

    vTaskDelay(1);
  }
}
// Thêm flag khẩn cấp
volatile bool emergency_edge  = false;
volatile bool emergency_fall   = false;
volatile bool emergency_lift   = false;

void task_safety(void* param) {
  for (;;) {
    // Chỉ set flag — KHÔNG gọi run_*() hay motor_control_fast()
    emergency_edge = edgeDetect;
    emergency_fall = fallOut;
    emergency_lift = beingLifted;
    vTaskDelay(5);  // 5ms — đủ nhanh
  }
}
// Core 1: FSM + Motor (ưu tiên cao hơn)
void task_fsm(void* param) {
  TickType_t xLastWake = xTaskGetTickCount();
  uint32_t t_debug = 0;
  for (;;) {
    run_fsm_normal();  // gọi trực tiếp, không cần if-timer

    #if DEBUG_SERIAL
    uint32_t now = millis();
    if (now - t_debug >= 500) {
      t_debug = now;
      DLOG("[%s] d0:%4d ang:%.1f lost:%d fast:%d edge[%d%d%d]\n"
           "  pit:%.1f rol:%.1f yaw:%.1f TCRT[%4d %4d %4d]\n\n",
           stateName(currentState), d0, enemy_angle_est,
           isTargetLost, closingFast, edge_fl, edge_fr, edge_rear,
           imu_pitch, imu_roll, imu_yaw,
           tcrt_fl_raw, tcrt_fr_raw, tcrt_rear_raw);
    }
    #endif

    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(30));  // chính xác 30ms
  }
}

void task_edge_guard(void* param) {
  // Biến local — không dùng qua flag toàn cục để tránh delay
  bool fl = false, fr = false, rear = false;
  //int  raw_fl, raw_fr, raw_rear;

  for (;;) {
    // Đọc ADC nhanh — ~30µs tổng cả 3 sensor
    int raw_fl = analogRead(14);
    int raw_fr = analogRead(27);
    int raw_rear = analogRead(13);

    // Hysteresis — chống rung
    if (raw_fl   > THRES_FL)              fl   = true;
    else if (raw_fl   < THRES_FL   - THRES_HYST) fl   = false;
    if (raw_fr   > THRES_FR)              fr   = true;
    else if (raw_fr   < THRES_FR   - THRES_HYST) fr   = false;
    if (raw_rear > THRES_REAR)            rear = true;
    else if (raw_rear < THRES_REAR - THRES_HYST) rear = false;

    // Cập nhật global cho FSM dùng
    edge_fl    = fl;
    edge_fr    = fr;
    edge_rear  = rear;
    edgeDetect = (fl || fr || rear);

    // PHẢN ỨNG MOTOR TRỰC TIẾP — không qua FSM, không chờ task khác
    if (edgeDetect) {
      bool ignore_rear = (current_PWM_L < -30 && current_PWM_R < -30);
      last_edge_time = millis();

      // Tính hướng lùi ngay lập tức
      if (fl && fr)          motor_control_fast(-SPD_FULL, -SPD_FULL);
      else if (fl)           motor_control_fast(-SPD_SCAN, -SPD_FULL);
      else if (fr)           motor_control_fast(-SPD_FULL, -SPD_SCAN);
      else if (rear && !ignore_rear) motor_control_fast(SPD_FULL, SPD_FULL);

      // Force state — nhưng không block motor
      if (currentState != STATE_DEF_EDGE_AVOID)
        forceState(STATE_DEF_EDGE_AVOID);
    }

    vTaskDelay(2); // 2ms — nhanh nhất có thể mà không starve task khác
  }
}
// ================================================================
// XII. SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================");
  Serial.println("   SUMO ROBOT v2.0 — ESP32-S3");
  Serial.println("==============================\n");

  pinMode(PIN_TCRT_FL,   INPUT);
  pinMode(PIN_TCRT_FR,   INPUT);
  pinMode(PIN_TCRT_REAR, INPUT);
  analogSetAttenuation(ADC_11db);           
  Wire.begin(I2C_SDA, I2C_SCL);
  
    
    Serial.println("Đã cài đặt ngắt ToF thành công!");
  Wire.setClock(400000);

  // I2C Scan
  Serial.println("[I2C SCAN]");
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0)
      Serial.printf("  ✅ 0x%02X\n", a);
  }
  Serial.println();

  // PCA9685
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PWM_FREQ);
  motor_stop();
  Serial.println("[PCA9685] ✅ OK\n");
  i2c_mutex = xSemaphoreCreateMutex();
  // MPU6050
  imu_init();

  // ToF
  tof_init();
  

  stateEnterMs = millis();
  Serial.printf("[FSM] START -> %s\n\n", stateName(currentState));

  // Spawn tasks trên 2 core riêng biệt
  xTaskCreatePinnedToCore(task_safety,  "safety",  4096, NULL, 3, NULL, 1); // Priority 3
  xTaskCreatePinnedToCore(task_fsm,     "fsm",     8192, NULL, 2, NULL, 1); // Priority 2
  xTaskCreatePinnedToCore(task_sensors, "sensors", 8192, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(task_edge_guard, "edge", 4096, NULL, 4, NULL, 1); // Priority 4 — cao hơn tất cả // Core 1 — cùng core với FSM để tránh race condition motor
}

void loop() {
  // Không dùng — logic chạy trong 2 tasks riêng
  vTaskDelay(portMAX_DELAY);
}

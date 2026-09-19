#include "globals.h"

// ============================================================
//  globals.h'de extern olarak tanimlanan degerlerin gercek
//  tanimlari. Sayisal degerler ve yorumlar orijinal main.cpp
//  ile BIREBIR AYNI - davranista degisiklik yok, sadece tasima.
// ============================================================

const uint8_t WIFI_CH = 1;          // Kumanda v181 ile AYNI olmali
const bool    DEBUG_SERIAL = true;  // Debug seri ciktisi (5 Hz). Kapatmak icin false.

// ==================== DONGU HIZLARI ====================
const uint32_t PID_LOOP_MS = 10;
const uint32_t VEL_LOOP_MS = 11;
const uint32_t TELEM_MS    = 50;

// ==================== STEP MOTOR KONUMU ====================
volatile long motorPos1 = 0, motorPos2 = 0;

// ==================== ACI PID DEGISKENLERI ====================
double turning       = 500;
double baseSetpoint  = -3;
double setpoint      = -2;
double throttleInput = 0;
double input         = 0.0;
double output        = 0.0;

double Kp = 15.0;
double Ki = 40.0;
double Kd = 0.5;

double integral      = 0.0;
double filteredGyro  = 0.0;
unsigned long lastComputeTime = 0;

// --- v181 DUVAR 2: dinamik integral tavani ---
const double INT_AUTHORITY = 255.0;
double intLimitAngle = 6.375;   // = INT_AUTHORITY / Ki

double gyroX = 0, gyroY = 0, gyroZ = 0;
const double GYRO_SIGN = -1.0;

// ==================== TITRESIM KONTROL ====================
double filteredInput  = 0.0;
double filteredOutput = 0.0;
const double INPUT_FILTER   = 0.3;
const double OUTPUT_FILTER  = 0.4;
const double ANGLE_DEADZONE = 0.0;   // 0 = kapali

// ==================== HIZ PI (DIS DONGU - CASCADE) ====================
const double VEL_SCALE = 0.001;

// v181 DUVAR 1: doyma esigi = maxVelTilt / (Kp_vel*VEL_SCALE).
double Kp_vel = 2.30;
double Ki_vel = 3.0;

double vel_integral    = 0.0;
double velSetpointTilt = 0.0;
double filteredSpeed   = 0.0;
double realMotorHz     = 0.0;
long   lastStepCount1 = 0, lastStepCount2 = 0;
unsigned long lastVelTime = 0;

// v181 DUVAR 3: 0.15 -> 0.06 (BalancingWii ~0.035 s civari)
const double SPEED_TC = 0.06;

// v181 DUVAR 4: hedef hiz rampasi
double driveAccelHz    = 4000.0;
double targetHzRamped  = 0.0;
double lastVelError    = 0.0;

const bool   VEL_ACTIVE_WHEN_IDLE = false;
const double THROTTLE_DZ          = 0.05;
const double TILT_RELEASE_TC      = 0.25;

double targetHzMax = 3000.0;
double maxVelTilt  = 4.0;

// v183: feedforward lean.
double FF_LEAN_MAX = 6.0;
double ffLean      = 0.0;

const double VEL_TILT_SIGN = -1.0;
const double THROTTLE_DIR  = 1.0;

// Donus (steering) yonu. Robot tam sola basildiginda ters yone donerse
// bunu -1.0 yap. -> Test edildi: yon tersti, -1.0 yapildi.
const double STEER_SIGN = -1.0;

// Donus (steering) yetkisi: steer=+-1 iken her tekerlege eklenen/cikan Hz.
// -> 4000 denendi: buyuk fark dengeleme donguyle etkilesip hizin
//    kontrolsuz artmasina (~4x hedef) yol acti, GUVENLIK icin geri
//    500'e cekildi. Daire ozelligi bu limitle calisir, "sinirsiz
//    donus" bu robotta guvenli degil.
const double TURN_AUTHORITY_HZ = 500.0;

double angleOffset = 0.0;

// ==================== GUVENLIK ====================
const bool     CURRENT_CUTOFF_ENABLED = false;
const double   OUTPUT_DEADZONE        = 5.0;
const uint16_t CURRENT_FULL           = 1000;
const uint16_t CURRENT_IDLE           = 300;
bool motorCurrentOn = true;

const uint16_t CURRENT_MIN = 200;
const uint16_t CURRENT_MAX = 1500;
uint16_t MOTOR_CURRENT = CURRENT_FULL;
uint16_t appliedCurrent = 0;

const double   FALL_ANGLE   = 40.0;
const double   FALL_RECOVER = 10.0;
bool fallen = false;

const uint32_t CMD_TIMEOUT_MS = 500;
volatile uint32_t lastCmdMs = 0;
bool emergencyStop = false;

// ==================== TELEMETRI SAYACLARI ====================
volatile uint32_t telemSent = 0, telemFail = 0;

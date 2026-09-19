#pragma once
#include <Arduino.h>

// ============================================================
//  PAYLASILAN DURUM VE AYARLANABILIR PARAMETRELER
//  Birden fazla modul (control/motor/comms/sensors/main) bu
//  degiskenleri okuyup yaziyor -> tek yerden extern tanimlanir,
//  gercek degerler globals.cpp icinde.
// ============================================================

// ---------------- Genel ayar ----------------
extern const uint8_t WIFI_CH;       // Kumanda ile AYNI olmali
extern const bool     DEBUG_SERIAL; // Seri debug ciktisi ac/kapa

// ---------------- Dongu hizlari ----------------
extern const uint32_t PID_LOOP_MS;  // aci PID   -> 100 Hz
extern const uint32_t VEL_LOOP_MS;  // hiz PI    -> ~90 Hz
extern const uint32_t TELEM_MS;     // telemetri -> 20 Hz

// ---------------- Step motor / encoder-suz konum ----------------
extern volatile long motorPos1, motorPos2;   // ISR icinde artan adim sayaci

// ---------------- Aci PID (ic dongu) ----------------
extern double turning;        // notr = 500, +-STEER... ile degisir
extern double baseSetpoint;   // temel denge acisi
extern double setpoint;       // base + hiz PI egimi + FF
extern double throttleInput;  // -1..1
extern double input;          // filtreli aci
extern double output;         // PID cikisi (-255..255)

extern double Kp, Ki, Kd;

extern double integral;
extern double filteredGyro;
extern unsigned long lastComputeTime;

extern const double INT_AUTHORITY;  // integralin kullanabilecegi cikis payi (0-255)
extern double intLimitAngle;        // = INT_AUTHORITY / Ki (dinamik)

extern double gyroX, gyroY, gyroZ;
extern const double GYRO_SIGN;

extern double filteredInput;
extern double filteredOutput;
extern const double INPUT_FILTER;
extern const double OUTPUT_FILTER;
extern const double ANGLE_DEADZONE;

// ---------------- Hiz PI (dis dongu - cascade) ----------------
extern const double VEL_SCALE;   // Kp_vel/Ki_vel birimi: derece / 1000 Hz

extern double Kp_vel, Ki_vel;

extern double vel_integral;
extern double velSetpointTilt;   // ACI egimi (derece)
extern double filteredSpeed;     // olculen tekerlek hizi (Hz)
extern double realMotorHz;       // motora giden komut (Hz)
extern long   lastStepCount1, lastStepCount2;
extern unsigned long lastVelTime;

extern const double SPEED_TC;

extern double driveAccelHz;      // Hz/s, web'den ayarlanabilir
extern double targetHzRamped;
extern double lastVelError;      // telemetri icin

extern const bool   VEL_ACTIVE_WHEN_IDLE;
extern const double THROTTLE_DZ;
extern const double TILT_RELEASE_TC;

extern double targetHzMax;   // web: MaxHz
extern double maxVelTilt;    // web: maxLean  <- ivmenin gercek kolu

extern double FF_LEAN_MAX;   // web: ff_cruise alani
extern double ffLean;        // anlik hesaplanan FF egim (telemetri)

extern const double VEL_TILT_SIGN;
extern const double THROTTLE_DIR;

// Donus (steering) yonu/yetkisi
extern const double STEER_SIGN;
extern const double TURN_AUTHORITY_HZ;

extern double angleOffset;

// ---------------- Guvenlik ----------------
extern const bool     CURRENT_CUTOFF_ENABLED;
extern const double   OUTPUT_DEADZONE;
extern const uint16_t CURRENT_FULL;
extern const uint16_t CURRENT_IDLE;
extern bool motorCurrentOn;

extern const uint16_t CURRENT_MIN;
extern const uint16_t CURRENT_MAX;
extern uint16_t MOTOR_CURRENT;   // web'den degisir
extern uint16_t appliedCurrent;  // suan surucude yazili olan deger

extern const double FALL_ANGLE;
extern const double FALL_RECOVER;
extern bool fallen;

extern const uint32_t CMD_TIMEOUT_MS;
extern volatile uint32_t lastCmdMs;
extern bool emergencyStop;

// ---------------- Telemetri sayaclari (comms + control debug) ----------------
extern volatile uint32_t telemSent, telemFail;

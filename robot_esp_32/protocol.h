#pragma once
#include <Arduino.h>

// ============================================================
//  ESP-NOW PAKET FORMATLARI (DEGISMEDI - kumanda ile birebir uyumlu
//  kalmali, alan eklemek/cikarmak/sira degistirmek PROTOKOLU BOZAR)
// ============================================================

// Kumanda -> Robot
typedef struct struct_message {
  float targetAngle;
  float steering;
  float kp, kd, ki;
  bool  stop;
  float balanceAngle;
  float maxLean;
  float kp_vel, ki_vel;
  float ff_break, ff_cruise, drive_max_hz, drive_accel;
  float kp_drive, ki_drive;
} struct_message;

// Robot -> Kumanda (telemetri)
typedef struct {
  float angle, target_angle, error, rate, motorSpeed, throttle;
  float tgtMS, fMS, velIntegral;
  uint8_t frozen;
  float kpVel, kiVel, kpAngle, kdAngle, tiltRaw, ffDeg;
  float driveHz, driveErrorHz, driveIntegral;
  float kpStill, kdStill, kiAngle, ffBreak;
  float driveMaxHz, driveAccelHz, balAngle, maxLean;
  uint32_t cmdAge, ms;
  float angleScale;
} TelemetriPaketi;

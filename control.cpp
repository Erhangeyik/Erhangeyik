#include <Arduino.h>

#include "globals.h"
#include "control.h"

void updateIntLimit() {
  intLimitAngle = (Ki > 0.5) ? (INT_AUTHORITY / Ki) : 500.0;
}

// ==================== ACI PID (IC DONGU) ====================
void computePID() {
  unsigned long currentTime = millis();
  double dt = (currentTime - lastComputeTime) / 1000.0;
  lastComputeTime = currentTime;
  if (dt <= 0)   dt = 0.001;
  if (dt > 0.1)  dt = 0.1;

  // v183: FF egim = throttle ile orantili, dis dongu isaretiyle uyumlu.
  // Gaza basar basmaz robot egilir (integrali beklemeden).
  ffLean = VEL_TILT_SIGN * THROTTLE_DIR * throttleInput * FF_LEAN_MAX;
  setpoint = baseSetpoint + velSetpointTilt + ffLean;
  double error = setpoint - input;

  if (ANGLE_DEADZONE > 0.0 && fabs(error) < ANGLE_DEADZONE &&
      fabs(throttleInput) < THROTTLE_DZ && fabs(velSetpointTilt) < 0.2) {
    error = 0.0;
  }

  double rawGyroRate = GYRO_SIGN * gyroX;
  filteredGyro = filteredGyro * 0.7 + rawGyroRate * 0.3;

  // v181: sabit 3.0 yerine dinamik tavan. Ki*intLimitAngle = INT_AUTHORITY
  double candInt = integral + error * dt;
  if (candInt >  intLimitAngle) candInt =  intLimitAngle;
  if (candInt < -intLimitAngle) candInt = -intLimitAngle;
  double testOut = Kp * error + Ki * candInt - Kd * filteredGyro;
  bool sat = (testOut >  255.0 && error > 0) ||
             (testOut < -255.0 && error < 0);
  if (!sat) integral = candInt;

  double rawOutput = Kp * error + Ki * integral - Kd * filteredGyro;

  filteredOutput = filteredOutput * (1.0 - OUTPUT_FILTER) + rawOutput * OUTPUT_FILTER;
  output = filteredOutput;
  if (output >  255) output =  255;
  if (output < -255) output = -255;

  // Debug (5Hz)
  if (DEBUG_SERIAL) {
    static unsigned long lastDbg = 0;
    static double lastAngleDbg = 0;
    if (millis() - lastDbg >= 200) {
      double dAdt = (input - lastAngleDbg) / ((millis() - lastDbg) / 1000.0);
      lastDbg = millis();
      lastAngleDbg = input;
      Serial.print("Pitch:");  Serial.print(input, 1);
      Serial.print(" Set:");   Serial.print(setpoint, 2);
      Serial.print(" dA/dt:"); Serial.print(dAdt, 1);
      Serial.print(" fGyr:");  Serial.print(filteredGyro, 1);
      Serial.print(" Hz:");    Serial.print(realMotorHz, 0);
      Serial.print(" iT:");    Serial.print(Ki * integral, 0);
      Serial.print(" vTilt:"); Serial.print(velSetpointTilt, 2);
      Serial.print(" ffL:");   Serial.print(ffLean, 2);
      Serial.print(" fSpd:");  Serial.print(filteredSpeed, 0);
      Serial.print(" ramp:");  Serial.print(targetHzRamped, 0);
      Serial.print(" tgt:");   Serial.print(THROTTLE_DIR * throttleInput * targetHzMax, 0);
      Serial.print(" vInt:");  Serial.print(vel_integral, 0);
      Serial.print(" cmdAge:");Serial.print(millis() - lastCmdMs);
      Serial.print(" txF:");   Serial.print(telemFail);
      Serial.print("/");       Serial.println(telemSent);
    }
  }
}

// ==================== HIZ PI (DIS DONGU) ====================
// Girisi Hz, cikisi DERECE (velSetpointTilt).
void computeVelocityPI() {
  unsigned long now = millis();
  double dt = (now - lastVelTime) / 1000.0;
  lastVelTime = now;
  if (dt <= 0)  dt = 0.001;
  if (dt > 0.2) dt = 0.2;

  long pos1 = motorPos1, pos2 = motorPos2;
  double stepsDelta = 0.5 * ((double)(pos1 - lastStepCount1) + (double)(pos2 - lastStepCount2));
  lastStepCount1 = pos1;
  lastStepCount2 = pos2;
  double realHz = stepsDelta / dt;

  double alpha = dt / (SPEED_TC + dt);
  filteredSpeed = filteredSpeed + alpha * (realHz - filteredSpeed);

  // --- GAZ YOKSA HEDEF ACIYI SERBEST BIRAK ---
  if (fabs(throttleInput) < THROTTLE_DZ && !VEL_ACTIVE_WHEN_IDLE) {
    vel_integral   = 0.0;
    targetHzRamped = 0.0;
    lastVelError   = 0.0;
    double a = dt / (TILT_RELEASE_TC + dt);
    velSetpointTilt += a * (0.0 - velSetpointTilt);
    if (fabs(velSetpointTilt) < 0.02) velSetpointTilt = 0.0;
    return;
  }

  // --- v181 DUVAR 4: hedef hiz rampasi ---
  // Joystick basamak atlar; PI'ya basamak vermek + ters faz = overshoot.
  // Referansi sekillendir, kazancla dovusme.
  double targetHzRaw = THROTTLE_DIR * throttleInput * targetHzMax;
  double maxDelta    = driveAccelHz * dt;
  if      (targetHzRamped < targetHzRaw - maxDelta) targetHzRamped += maxDelta;
  else if (targetHzRamped > targetHzRaw + maxDelta) targetHzRamped -= maxDelta;
  else                                              targetHzRamped  = targetHzRaw;

  double velError = targetHzRamped - filteredSpeed;
  lastVelError = velError;

  double kp_eff = Kp_vel * VEL_SCALE;   // derece / Hz
  double ki_eff = Ki_vel * VEL_SCALE;   // derece / (Hz*s)

  double raw = kp_eff * velError + ki_eff * vel_integral;
  bool saturated = (raw >=  maxVelTilt && velError > 0) ||
                   (raw <= -maxVelTilt && velError < 0);
  if (!saturated) vel_integral += velError * dt;

  double intLimit = maxVelTilt / (ki_eff > 1e-9 ? ki_eff : 1e-9);
  if (vel_integral >  intLimit) vel_integral =  intLimit;
  if (vel_integral < -intLimit) vel_integral = -intLimit;

  raw = kp_eff * velError + ki_eff * vel_integral;
  if (raw >  maxVelTilt) raw =  maxVelTilt;
  if (raw < -maxVelTilt) raw = -maxVelTilt;

  velSetpointTilt = VEL_TILT_SIGN * raw;
}

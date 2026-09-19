// ============================================================
//  robot_esp32_full  -  v183
//
//  v182'ye gore DEGISEN: FEEDFORWARD LEAN (FF egim) geri getirildi.
//
//  ISTEK: gaza basinca robot dis dongunun yavas integralini beklemeden
//  DIREKT one egilsin (eskiden yapmistik). throttle ile orantili sabit
//  bir egim dogrudan setpoint'e eklenir:
//
//    setpoint = baseSetpoint + velSetpointTilt + ffLean
//    ffLean   = VEL_TILT_SIGN * THROTTLE_DIR * throttleInput * FF_LEAN_MAX
//
//  FF egim, hiz PI'nin urettigi velSetpointTilt'in USTUNE eklenir:
//   - FF  -> hizli tepki, gaza basar basmaz robot egilir
//   - PI  -> uzerine ince ayar / kalici hiz hatasi telafisi
//
//  Isaret: dis dongude ileri gitmek icin setpoint AZALIR (VEL_TILT_SIGN
//  = -1). FF de ayni yonde. Gaza basinca robot GERI gidiyorsa THROTTLE_DIR
//  = -1 yap (hem FF hem PI birlikte doner, tutarli kalir).
//
//  FF_LEAN_MAX web'den ayarlanir (ff_cruise alani, PROTOKOL DEGISMEDI).
//  Telemetride ffDeg = anlik ffLean.
//
//  NOT: FF egim gaz birakilinca ANINDA 0 olur (throttle 0 -> ffLean 0).
//  velSetpointTilt ise eskisi gibi TILT_RELEASE_TC ile yumusak iner.
//
//  ---- v182 mirasi: canli motor akimi (holding torque) kontrolu ----
//  ---- v181 mirasi: cascade doygunluk duvarlari, rampa ----
//  ---- v180 mirasi: WebSocket haberlesme ----
//
//  SORUN: "Kp_vel yetersiz, arttirinca hala yetersiz ama overshoot yapiyor"
//  Bu tek bir kazanc sorunu degil, UST USTE BINEN DORT DUVAR:
//
//  ---- DUVAR 1: dis dongu P terimi zaten doygunlukta ----
//    raw = (Kp_vel * 0.001) * velError, sonra +-maxVelTilt ile kirpiliyor.
//    Doyma esigi = maxVelTilt / (Kp_vel*0.001)
//    Kritik deger: Kp_vel = maxVelTilt/(0.001*targetHzMax).
//    Ustundeki her sey tam gazda olu agirlik; hedefe YAKIN bolgeyi
//    sertlestirip overshoot yapar.
//
//  ---- DUVAR 2: IC DONGU INTEGRAL TAVANI (asil "yetersiz" sebebi) ----
//    Sabit hizda seyirde govde denge acisinda oturur: aci hatasi ~0,
//    jiro ~0. Geriye TEK terim kalir: output = Ki * integral. Steppere
//    encoder yok; iç integral aslinda "hiz hafizasi" gorevi goruyor.
//    -> INT_LIMIT sabit degil: intLimitAngle = INT_AUTHORITY / Ki
//       (control.cpp / globals.cpp)
//
//  ---- DUVAR 3: 150 ms olcum gecikmesi ----
//    SPEED_TC 0.15 -> 0.06 (BalancingWii ~0.035 s civari referans alindi).
//
//  ---- DUVAR 4: basamak referans + ters faz (non-minimum phase) ----
//    Joystick basamak atlarsa dis dongu bant genisliginin matematiksel
//    tavani asilir. -> targetHz RAMPALANIYOR (driveAccelHz).
//
//  *** UYARI ***
//  Integral yetkisi buyudugu icin, baseSetpoint gercek denge acisindan
//  sapmissa BOSTAKI KAYMA HIZLANIR. Bosta kayma artmissa cozum kazanc
//  degil, baseSetpoint'i duzeltmektir.
//  Duruşta titreme/huzursuzluk olursa INT_AUTHORITY'yi 255 -> 180 yap
//  (globals.cpp).
//
//  TMC2209 + BNO055 (0x29) + web kumanda (ESP-NOW), Timer ISR
//
//  ---- DOSYA HARITASI ----
//    pins.h        : pin / donanim define'lari
//    protocol.h    : kumanda <-> robot ESP-NOW paket formatlari
//    globals.h/.cpp: modullerin ortak kullandigi durum + parametreler
//    sensors.h/.cpp: BNO055 (aci + gyro) okuma
//    motor.h/.cpp  : TMC2209 surucu + step timer + motor karistirma
//    control.h/.cpp: aci PID (ic dongu) + hiz PI (dis dongu)
//    comms.h/.cpp  : WiFi/ESP-NOW baslatma + komut/telemetri
//    main.cpp      : sadece setup()/loop()/app_main() - orkestrasyon
// ============================================================

#include <Arduino.h>

#include "globals.h"
#include "sensors.h"
#include "motor.h"
#include "control.h"
#include "comms.h"

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(300);

  initMotors();
  initSensors();

  updateIntLimit();
  Serial.print("intLimitAngle = "); Serial.print(intLimitAngle, 3);
  Serial.print("  (Ki*limit = ");   Serial.print(Ki * intLimitAngle, 0);
  Serial.println(" / 255)");
  Serial.print("Kp_vel doyma esigi = ");
  Serial.print(maxVelTilt / (Kp_vel * VEL_SCALE), 0);
  Serial.println(" Hz");

  initComms();

  lastComputeTime = millis();
  lastVelTime     = millis();
  lastCmdMs       = millis();
  lastStepCount1  = motorPos1;
  lastStepCount2  = motorPos2;
  Serial.println("HAZIR (v181 - cascade doygunluk duvarlari kaldirildi).");
}

// ==================== LOOP ====================
void loop() {
  applyPendingCommand();

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    throttleInput = 0.0;
    turning       = 500.0;
  }

  // --- Aci PID (100 Hz) ---
  static unsigned long lastSensorReadTime = 0;
  if (millis() - lastSensorReadTime >= PID_LOOP_MS) {
    lastSensorReadTime = millis();

    readSensors();

    if (!fallen && fabs(input) > FALL_ANGLE)   fallen = true;
    if ( fallen && fabs(input) < FALL_RECOVER) fallen = false;

    if (fallen || emergencyStop) {
      stopMotors();
      setMotorCurrent(false);
      integral = 0; vel_integral = 0; velSetpointTilt = 0;
      filteredOutput = 0; output = 0; realMotorHz = 0;
      targetHzRamped = 0;
      lastStepCount1 = motorPos1; lastStepCount2 = motorPos2;
    } else {
      computePID();
      updateMotorControl(output);
    }
  }

  // --- Hiz PI dis dongusu (~90 Hz) ---
  static unsigned long lastVelLoop = 0;
  if (millis() - lastVelLoop >= VEL_LOOP_MS) {
    lastVelLoop = millis();
    if (fallen || emergencyStop) {
      lastVelTime = millis();
      lastStepCount1 = motorPos1; lastStepCount2 = motorPos2;
    } else {
      computeVelocityPI();
    }
  }

  // --- Telemetri (20 Hz) ---
  static unsigned long lastTelemTime = 0;
  if (millis() - lastTelemTime >= TELEM_MS) {
    lastTelemTime = millis();
    sendTelemetry();
  }
}

// ==================== ESP-IDF GIRIS NOKTASI ====================
extern "C" void app_main() {
  initArduino();
  setup();
  for (;;) {
    loop();
  }
}

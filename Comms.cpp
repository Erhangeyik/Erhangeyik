#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "globals.h"
#include "protocol.h"
#include "comms.h"
#include "control.h"  // updateIntLimit (Ki degisince tavan guncellenir)

// Bu modul icinde kalan durum - baska hicbir dosya dogrudan erismiyor.
static TelemetriPaketi     telPkt;
static esp_now_peer_info_t peerInfo;
static uint8_t pcMAC[] = {0xF4, 0x2D, 0xC9, 0x79, 0x9B, 0x78};

static struct_message  cmdStaged;
static volatile bool   cmdPending = false;
static portMUX_TYPE    cmdMux = portMUX_INITIALIZER_UNLOCKED;

void initComms() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("Robot STA MAC (kumandada robotMAC bu olmali): ");
  Serial.println(WiFi.macAddress());
  Serial.print("Kanal: "); Serial.println(WIFI_CH);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init HATASI");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, pcMAC, 6);
  peerInfo.channel = WIFI_CH;
  peerInfo.ifidx   = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer eklenemedi");
    return;
  }
}

// ==================== ESP-NOW CALLBACK'LERI ====================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return;
  portENTER_CRITICAL(&cmdMux);
  memcpy(&cmdStaged, incomingData, sizeof(cmdStaged));
  cmdPending = true;
  portEXIT_CRITICAL(&cmdMux);
  lastCmdMs = millis();
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  telemSent++;
  if (status != ESP_NOW_SEND_SUCCESS) telemFail++;
}

void applyPendingCommand() {
  if (!cmdPending) return;
  struct_message d;
  portENTER_CRITICAL(&cmdMux);
  memcpy(&d, &cmdStaged, sizeof(d));
  cmdPending = false;
  portEXIT_CRITICAL(&cmdMux);

  if (d.stop) {
    emergencyStop = true;
    turning       = 500;
    throttleInput = 0;
    return;
  }

  if (emergencyStop) {
    emergencyStop = false;
    integral = 0; vel_integral = 0; velSetpointTilt = 0;
    filteredOutput = 0; targetHzRamped = 0;
    lastComputeTime = millis();
    lastVelTime     = millis();
    lastStepCount1  = motorPos1;
    lastStepCount2  = motorPos2;
  }

  turning       = 500 + STEER_SIGN * d.steering * TURN_AUTHORITY_HZ;
  throttleInput = constrain(d.targetAngle, -1.0f, 1.0f);

  if (d.kp > 0) {
    Kp = d.kp; Kd = d.kd; Ki = d.ki;
    updateIntLimit();          // v181: Ki degisince integral tavani da degisir
  }
  if (d.kp_vel > 0)   Kp_vel = d.kp_vel;
  if (d.ki_vel > 0)   Ki_vel = d.ki_vel;
  if (d.balanceAngle > -10.0 && d.balanceAngle < 10.0) baseSetpoint = d.balanceAngle;
  if (d.drive_max_hz > 0) targetHzMax = d.drive_max_hz;
  if (d.maxLean > 0)      maxVelTilt  = d.maxLean;
  if (d.drive_accel > 0)  driveAccelHz = d.drive_accel;   // v181

  // v182: canli motor akimi. kp_drive alani mA tasiyor.
  if (d.kp_drive > 0) {
    uint16_t yeni = (uint16_t)d.kp_drive;
    if (yeni < CURRENT_MIN) yeni = CURRENT_MIN;
    if (yeni > CURRENT_MAX) yeni = CURRENT_MAX;
    MOTOR_CURRENT = yeni;
    // Bir sonraki updateMotorControl -> setMotorCurrent(true) cagrisi
    // yeni degeri surucuye yazacak (appliedCurrent farkli oldugu icin).
  }

  // v183: FF lean. ff_cruise alani dereceyi tasir. 0 = FF kapali.
  // Ayri >=0 kontrolu: kullanici 0 yazarak FF'i kapatabilsin diye
  // -1 sentineli kullaniyoruz (kumanda bos birakinca -1 yollar).
  if (d.ff_cruise > -0.5) {
    FF_LEAN_MAX = d.ff_cruise;
    if (FF_LEAN_MAX < 0)  FF_LEAN_MAX = 0;
    if (FF_LEAN_MAX > 15) FF_LEAN_MAX = 15;   // guvenlik: asiri egim yok
  }
}

// ==================== TELEMETRI ====================
void sendTelemetry() {
  telPkt.angle        = input;
  telPkt.target_angle = setpoint;
  telPkt.error        = setpoint - input;
  telPkt.rate         = filteredGyro;
  telPkt.motorSpeed   = realMotorHz;
  telPkt.throttle     = throttleInput;
  telPkt.fMS          = filteredSpeed;
  telPkt.tgtMS        = THROTTLE_DIR * throttleInput * targetHzMax;  // ham istek
  telPkt.kpAngle = Kp; telPkt.kdAngle = Kd;
  telPkt.kpStill = Kp; telPkt.kdStill = Kd; telPkt.kiAngle = Ki;
  telPkt.kpVel = Kp_vel; telPkt.kiVel = Ki_vel;
  telPkt.tiltRaw     = velSetpointTilt;
  telPkt.velIntegral = vel_integral;
  telPkt.balAngle    = baseSetpoint;
  telPkt.driveMaxHz  = targetHzMax;
  telPkt.maxLean     = maxVelTilt;
  // --- v181 yeni teshis alanlari ---
  telPkt.driveHz       = targetHzRamped;      // rampalanmis hedef
  telPkt.driveErrorHz  = lastVelError;        // dis dongu hatasi
  telPkt.driveIntegral = Ki * integral;       // iTerm (255'e dayaniyor mu?)
  telPkt.driveAccelHz  = driveAccelHz;
  telPkt.ffBreak       = (float)appliedCurrent;  // v182: aktif motor akimi (mA)
  telPkt.ffDeg         = ffLean;                  // v183: anlik FF egim (derece)
  telPkt.angleScale  = 1.0;
  telPkt.frozen      = (fallen || emergencyStop) ? 1 : 0;
  telPkt.cmdAge      = millis() - lastCmdMs;
  telPkt.ms          = millis();
  esp_now_send(pcMAC, (uint8_t *)&telPkt, sizeof(telPkt));
}

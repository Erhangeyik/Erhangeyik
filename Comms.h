#pragma once
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
//  HABERLESME MODULU (WiFi + ESP-NOW + telemetri)
// ============================================================

// WiFi'yi ayarlar, ESP-NOW'i baslatir, kumandayi peer olarak ekler.
void initComms();

// ESP-NOW alici/verici callback'leri (donanim tarafindan cagirilir).
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);

// OnDataRecv'nin kritik bolgede sakladigi son komutu uygular
// (gains, throttle, steering, guvenlik komutlari vs).
void applyPendingCommand();

// Anlik durumu TelemetriPaketi'ne doldurup kumandaya gonderir.
void sendTelemetry();

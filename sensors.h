#pragma once

// ============================================================
//  SENSOR MODULU (BNO055: aci + gyro)
// ============================================================

// BNO055'i baslatir, kalibrasyon icin angleOffset'i hesaplar.
// Sensor bulunamazsa Serial'e hata basip sonsuz dongude kalir
// (orijinal davranis - guvenlik icin robot calismaya devam etmez).
void initSensors();

// BNO055'ten aci + gyro okur, globals.h'deki input/filteredInput
// ve gyroX/Y/Z'yi gunceller. PID_LOOP_MS periyoduyla cagrilir.
void readSensors();

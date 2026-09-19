#pragma once

// ============================================================
//  MOTOR MODULU (TMC2209 surucu + step timer + karistirma)
// ============================================================

// TMC2209 suruculeri + step timer donanimini baslatir.
void initMotors();

// Bir tekerlegin hedef step frekansini (Hz, isaret=yon) uygular.
void setMotorHz(int motor, double hz);

// Her iki motoru da durdurur (step timer'lari kapatir).
void stopMotors();

// Aci PID cikisini (pidOutput, -255..255) + donus (turning) degerini
// motor Hz komutlarina cevirip setMotorHz ile uygular.
void updateMotorControl(double pidOutput);

// Motor tutma akimini (holding current) acar/kapatir.
void setMotorCurrent(bool on);

// Genel amacli lineer eslesme yardimcisi.
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

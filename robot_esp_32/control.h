#pragma once

// ============================================================
//  KONTROL MODULU (Aci PID + Hiz PI kaskad kontrol)
// ============================================================

// Ki degistiginde integral tavanini (intLimitAngle) gunceller.
void updateIntLimit();

// Ic dongu: aci PID hesabi. globals.h'deki output/integral/setpoint
// gibi degiskenleri gunceller. PID_LOOP_MS periyoduyla cagrilir.
void computePID();

// Dis dongu: hiz PI hesabi. globals.h'deki velSetpointTilt'i gunceller.
// VEL_LOOP_MS periyoduyla cagrilir.
void computeVelocityPI();

#include <Arduino.h>
#include <TMCStepper.h>
#include <soc/gpio_struct.h>

#include "pins.h"
#include "globals.h"
#include "motor.h"

// TMC2209 suruculeri + step timer donanimi - sadece bu modul icinde
// kullaniliyor, disariya sadece motor.h'deki fonksiyonlar acik.
static HardwareSerial TMCSerial(2);
static TMC2209Stepper driver1(&TMCSerial, R_SENSE, 0);
static TMC2209Stepper driver2(&TMCSerial, R_SENSE, 0);

// ==================== TIMER ISR STEP ALTYAPISI ====================
static hw_timer_t *stepTimer1 = NULL;
static hw_timer_t *stepTimer2 = NULL;
static volatile int8_t motorDir1 = 0, motorDir2 = 0;

static volatile uint32_t *stepSetReg1, *stepClrReg1;
static volatile uint32_t *stepSetReg2, *stepClrReg2;
static uint32_t stepBit1, stepBit2;

static uint32_t lastPeriod1 = 0, lastPeriod2 = 0;
static bool     alarmOn1 = false, alarmOn2 = false;

static const int32_t MIN_STEP_HZ = 2;
static const int32_t MAX_STEP_HZ = 8500;

void IRAM_ATTR stepISR1() {
  *stepSetReg1 = stepBit1;
  __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;"
                       "nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
  *stepClrReg1 = stepBit1;
  motorPos1 += motorDir1;
}
void IRAM_ATTR stepISR2() {
  *stepSetReg2 = stepBit2;
  __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;"
                       "nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
  *stepClrReg2 = stepBit2;
  motorPos2 += motorDir2;
}

static void setupStepRegisters() {
  if (MOTOR1_STEP_PIN < 32) { stepSetReg1=&GPIO.out_w1ts; stepClrReg1=&GPIO.out_w1tc; stepBit1=(1U<<MOTOR1_STEP_PIN); }
  else { stepSetReg1=(volatile uint32_t*)&GPIO.out1_w1ts; stepClrReg1=(volatile uint32_t*)&GPIO.out1_w1tc; stepBit1=(1U<<(MOTOR1_STEP_PIN-32)); }
  if (MOTOR2_STEP_PIN < 32) { stepSetReg2=&GPIO.out_w1ts; stepClrReg2=&GPIO.out_w1tc; stepBit2=(1U<<MOTOR2_STEP_PIN); }
  else { stepSetReg2=(volatile uint32_t*)&GPIO.out1_w1ts; stepClrReg2=(volatile uint32_t*)&GPIO.out1_w1tc; stepBit2=(1U<<(MOTOR2_STEP_PIN-32)); }
}

void setMotorHz(int motor, double hz) {
  int32_t absHz = (int32_t)fabs(hz);

  if (absHz < MIN_STEP_HZ) {
    if (motor == 1) {
      if (alarmOn1) { timerStop(stepTimer1); alarmOn1 = false; }
      lastPeriod1 = 0; motorDir1 = 0;
    } else {
      if (alarmOn2) { timerStop(stepTimer2); alarmOn2 = false; }
      lastPeriod2 = 0; motorDir2 = 0;
    }
    return;
  }
  if (absHz > MAX_STEP_HZ) absHz = MAX_STEP_HZ;
  uint32_t period = 1000000UL / (uint32_t)absHz;

  if (motor == 1) {
    int8_t d = (hz > 0) ? 1 : -1;
    if (d != motorDir1) {
      digitalWrite(MOTOR1_DIR_PIN, (d > 0) ? HIGH : LOW);
      motorDir1 = d;
    }
    bool wasOff1 = !alarmOn1;
    if (period != lastPeriod1) {
      timerAlarm(stepTimer1, period, true, 0);
      lastPeriod1 = period;
    }
    if (wasOff1) {
      timerStart(stepTimer1);
      alarmOn1 = true;
    }
  } else {
    int8_t d = (hz > 0) ? 1 : -1;
    if (d != motorDir2) {
      digitalWrite(MOTOR2_DIR_PIN, (d > 0) ? LOW : HIGH);
      motorDir2 = d;
    }
    bool wasOff2 = !alarmOn2;
    if (period != lastPeriod2) {
      timerAlarm(stepTimer2, period, true, 0);
      lastPeriod2 = period;
    }
    if (wasOff2) {
      timerStart(stepTimer2);
      alarmOn2 = true;
    }
  }
}

void stopMotors() {
  if (alarmOn1) { timerStop(stepTimer1); alarmOn1 = false; }
  if (alarmOn2) { timerStop(stepTimer2); alarmOn2 = false; }
  lastPeriod1 = 0; lastPeriod2 = 0;
  motorDir1 = 0;  motorDir2 = 0;
}

static void initStepTimers() {
  pinMode(MOTOR1_STEP_PIN, OUTPUT); pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR1_STEP_PIN, LOW); digitalWrite(MOTOR2_STEP_PIN, LOW);
  setupStepRegisters();
  stepTimer1 = timerBegin(1000000); timerAttachInterrupt(stepTimer1, &stepISR1);
  stepTimer2 = timerBegin(1000000); timerAttachInterrupt(stepTimer2, &stepISR2);
}

void initMotors() {
  TMCSerial.begin(115200, SERIAL_8N1, -1, TMC_UART_PIN);
  delay(100);
  driver1.begin(); driver1.toff(3); driver1.rms_current(MOTOR_CURRENT);
  driver1.microsteps(16); driver1.en_spreadCycle(false); driver1.pwm_autoscale(true);
  driver2.begin(); driver2.toff(3); driver2.rms_current(MOTOR_CURRENT);
  driver2.microsteps(16); driver2.en_spreadCycle(false); driver2.pwm_autoscale(true);
  appliedCurrent = MOTOR_CURRENT;
  initStepTimers();
}

void updateMotorControl(double pidOutput) {
  if (CURRENT_CUTOFF_ENABLED) {
    bool needMotor = (fabs(pidOutput) >= OUTPUT_DEADZONE) ||
                     (fabs(turning - 500.0) > 10.0) ||
                     (fabs(throttleInput) > THROTTLE_DZ);
    setMotorCurrent(needMotor);
  } else {
    setMotorCurrent(true);
  }

  const double MaxPIDOutput      = 255.0;
  const double MaxStepsPerSecond = 5500.0;
  const double MinStepsPerSecond = -5500.0;

  if (pidOutput >  MaxPIDOutput) pidOutput =  MaxPIDOutput;
  if (pidOutput < -MaxPIDOutput) pidOutput = -MaxPIDOutput;

  double baseHz = mapDouble(pidOutput, -255, 255, MinStepsPerSecond, MaxStepsPerSecond);
  realMotorHz = baseHz;

  // turning-500: -500(tam sol)..+500(tam sag). base=0 iken (throttle=0)
  // m1 ve m2 ZIT isaretli olur -> tekerlekler zit yonde doner -> yerinde
  // donus. Yon ters gelirse STEER_SIGN'i degistir (globals.cpp'de).
  double m1 = baseHz + (turning - 500);
  double m2 = baseHz - (turning - 500);
  setMotorHz(1, m1);
  setMotorHz(2, m2);
}

void setMotorCurrent(bool on) {
  // v182: "on" durumunda sabit CURRENT_FULL yerine web'den ayarlanan
  // MOTOR_CURRENT kullanilir. Idle durumu (cutoff kapali oldugu icin
  // pratikte hic girilmiyor) yine CURRENT_IDLE.
  uint16_t hedef = on ? MOTOR_CURRENT : CURRENT_IDLE;
  motorCurrentOn = on;
  if (hedef != appliedCurrent) {          // gereksiz UART trafigi olmasin
    driver1.rms_current(hedef);
    driver2.rms_current(hedef);
    appliedCurrent = hedef;
  }
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

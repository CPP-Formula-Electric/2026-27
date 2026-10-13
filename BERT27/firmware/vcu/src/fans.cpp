/**
 * @file    fans.cpp
 * @brief   Sets the cooling fan PWM from motor/inverter temperature.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 */
#include "fans.h"

#include "pins.h"
#include "calibration.h"
#include "main.h"

uint8_t dutyPercentToFanPwmWrite(int dutyPercent);

static constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t FAN_PWM_WRITE_MAX = 255;

// Duty cycles are set automatically from motor/inverter temps unless a
// manual override is entered over serial (digits = override, 'a' = auto).
int fan1DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan2DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan1LastDutyPercent = -1;
int fan2LastDutyPercent = -1;
bool fanManualOverride = false;

// ---------------- Fans ----------------
void setupFans() {
  pinMode(FAN_1_PIN, OUTPUT);
  pinMode(FAN_2_PIN, OUTPUT);
  analogWriteFrequency(FAN_1_PIN, FAN_PWM_FREQUENCY_HZ);
  analogWriteFrequency(FAN_2_PIN, FAN_PWM_FREQUENCY_HZ);
  serviceFans();
}

void updateFanDutyFromTemps() {
  if (fanManualOverride) {
    return;
  }

  // Drive the fans from the hotter of the PM100 motor and inverter temps.
  bool haveTemp = false;
  float tempC = 0.0f;
  if (haveInverterTemp) {
    tempC = (float)inverterTempTenthsC / 10.0f;
    haveTemp = true;
  }
  if (havePm100MotorTemp) {
    float motorTempC = (float)pm100MotorTempTenthsC / 10.0f;
    if (!haveTemp || motorTempC > tempC) {
      tempC = motorTempC;
    }
    haveTemp = true;
  }

  int duty;
  if (!haveTemp || tempC <= FAN_TEMP_MIN_C) {
    duty = FAN_DUTY_MIN_PERCENT;
  } else if (tempC >= FAN_TEMP_MAX_C) {
    duty = FAN_DUTY_MAX_PERCENT;
  } else {
    float span = FAN_TEMP_MAX_C - FAN_TEMP_MIN_C;
    float fraction = (tempC - FAN_TEMP_MIN_C) / span;
    duty = FAN_DUTY_MIN_PERCENT +
           (int)(fraction * (float)(FAN_DUTY_MAX_PERCENT - FAN_DUTY_MIN_PERCENT) + 0.5f);
  }

  fan1DutyPercent = duty;
  fan2DutyPercent = duty;
}

void serviceFans() {
  int duty1 = constrain(fan1DutyPercent, 0, 100);
  int duty2 = constrain(fan2DutyPercent, 0, 100);

  if (duty1 != fan1LastDutyPercent) {
    fan1LastDutyPercent = duty1;
    analogWrite(FAN_1_PIN, dutyPercentToFanPwmWrite(duty1));
  }

  if (duty2 != fan2LastDutyPercent) {
    fan2LastDutyPercent = duty2;
    analogWrite(FAN_2_PIN, dutyPercentToFanPwmWrite(duty2));
  }
}

uint8_t dutyPercentToFanPwmWrite(int dutyPercent) {
  int duty = constrain(dutyPercent, 0, 100);
  return (uint8_t)((duty * FAN_PWM_WRITE_MAX + 50) / 100);
}

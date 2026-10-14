/**
 * @file    power_derate.cpp
 * @brief   Caps torque so battery power stays under the cell-voltage and temperature limit.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "power_derate.h"

#include "safety_limits.h"
#include "bms_link.h"
#include "inverter.h"

// Below this rpm the power cap exceeds MAX_TORQUE_NM anyway; the guard also
// avoids dividing by zero / near-zero rpm.
static const int      POWER_DERATE_MIN_RPM       = 50;
static const uint32_t POWER_DERATE_SAMPLE_MS     = 250;    // rolling-average sample period
static const uint32_t POWER_DERATE_WINDOW_MS     = 15000;  // rolling-average window
static const uint8_t  POWER_DERATE_WINDOW_SAMPLES =
    (uint8_t)(POWER_DERATE_WINDOW_MS / POWER_DERATE_SAMPLE_MS);  // 60 samples

// Fixed-cadence rolling average over POWER_DERATE_WINDOW_SAMPLES samples. Defined
// here (ahead of any function) so the .ino auto-generated prototypes can see it.
struct RollingWindow {
  float    buf[POWER_DERATE_WINDOW_SAMPLES];
  float    sum;
  uint8_t  count;
  uint8_t  head;
};

void rollingWindowPush(RollingWindow &w, float value);
bool rollingWindowAverage(const RollingWindow &w, float &out);
float cellDerateKw(float avgMv);

static RollingWindow minCellMvWindow = {};
static RollingWindow battMaxTempWindow = {};
bool powerDerateActive = false;
// Active power limit (kW) while powerDerateActive; the lowest cap across all
// tripped conditions. Updated each sample by servicePowerDerate().
float powerDerateKw = POWER_DERATE_KW;

// ---------------- Power derate ----------------
void rollingWindowPush(RollingWindow &w, float value) {
  if (w.count == POWER_DERATE_WINDOW_SAMPLES) {
    w.sum -= w.buf[w.head];           // evict oldest before overwriting
  } else {
    w.count++;
  }
  w.buf[w.head] = value;
  w.sum += value;
  w.head = (uint8_t)((w.head + 1) % POWER_DERATE_WINDOW_SAMPLES);
}

bool rollingWindowAverage(const RollingWindow &w, float &out) {
  if (w.count == 0) return false;
  out = w.sum / (float)w.count;
  return true;
}

// Linear low-voltage power limit (kW) for a rolling-average minimum cell of
// avgMv: POWER_DERATE_CELL_MAX_KW at CELL_FULL_MV, 0 kW at CELL_ZERO_MV, and
// linear in between. At or above CELL_FULL_MV there is no voltage derate, so
// this returns CELL_MAX_KW (callers treat >= full-voltage as "no cell trip").
float cellDerateKw(float avgMv) {
  if (avgMv >= (float)POWER_DERATE_CELL_FULL_MV) return POWER_DERATE_CELL_MAX_KW;
  if (avgMv <= (float)POWER_DERATE_CELL_ZERO_MV) return 0.0f;
  float frac = (avgMv - (float)POWER_DERATE_CELL_ZERO_MV) /
               (float)(POWER_DERATE_CELL_FULL_MV - POWER_DERATE_CELL_ZERO_MV);
  return POWER_DERATE_CELL_MAX_KW * frac;
}

// Torque that produces powerDerateKw at the given shaft speed, clamped to the
// normal max. Below POWER_DERATE_MIN_RPM the cap exceeds MAX_TORQUE_NM, so the
// limiter has no effect and we simply return the unrestricted max.
float powerDerateTorqueCapNm(int16_t rpm) {
  if (powerDerateKw <= 0.0f) return 0.0f;
  int absRpm = abs((int)rpm);
  if (absRpm < POWER_DERATE_MIN_RPM) return MAX_TORQUE_NM;
  float cap = TORQUE_PER_KW_PER_RPM * powerDerateKw / (float)absRpm;
  if (cap > MAX_TORQUE_NM) cap = MAX_TORQUE_NM;
  return cap;
}

// Samples the protection signals on a fixed cadence and updates the rolling
// averages, then sets powerDerateActive when any trip condition is met.
void servicePowerDerate(uint32_t nowMs) {
  static uint32_t lastSampleMs = 0;
  if (nowMs - lastSampleMs < POWER_DERATE_SAMPLE_MS) return;
  lastSampleMs = nowMs;

  if (haveCellVoltageStats) rollingWindowPush(minCellMvWindow, (float)minCellVoltageMv);
  if (haveCellTempStats)    rollingWindowPush(battMaxTempWindow, (float)maxCellTempC);

  bool derate = false;
  float capKw = POWER_DERATE_CELL_MAX_KW;  // lowest cap across active trips

  // Low cell voltage: linear derate on the rolling-average minimum cell.
  float avgCellMv;
  if (rollingWindowAverage(minCellMvWindow, avgCellMv) &&
      avgCellMv < (float)POWER_DERATE_CELL_FULL_MV) {
    derate = true;
    float kw = cellDerateKw(avgCellMv);
    if (kw < capKw) capKw = kw;
  }

  if (havePm100MotorTemp &&
      (float)pm100MotorTempTenthsC / 10.0f > POWER_DERATE_MOTOR_TEMP_C) {
    derate = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  float avgBattTemp;
  if (rollingWindowAverage(battMaxTempWindow, avgBattTemp) &&
      avgBattTemp > (float)POWER_DERATE_BATT_TEMP_C) {
    derate = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  powerDerateKw = derate ? capKw : POWER_DERATE_CELL_MAX_KW;

  if (derate != powerDerateActive) {
    powerDerateActive = derate;
    if (derate) {
      Serial.print("[Power] Derate active; capping to ");
      Serial.print(powerDerateKw, 1);
      Serial.println(" kW");
    } else {
      Serial.println("[Power] Derate cleared; full power restored");
    }
  }
}

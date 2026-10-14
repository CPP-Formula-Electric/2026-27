/**
 * @file    pedals.cpp
 * @brief   Reads the accelerator and brake pedals and runs the APPS plausibility and BSPC checks.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "pedals.h"

#include "pins.h"
#include "safety_limits.h"
#include "calibration.h"
#include "main.h"
#include "inverter_command.h"
#include "dashboard.h"

float scalePercentNormal(int rawValue, int rawMin, int rawMax);
float scalePercentInverted(int rawValue, int rawMin, int rawMax);
float adcCountsToVoltage(int rawValue);
float scaleBrakePressurePercent(float voltage);

static const int ADC_MAX_COUNTS      = (1 << ADC_RESOLUTION_BITS) - 1;
static const float ADC_REFERENCE_VOLTAGE = 3.3f;

static constexpr bool APPS_SERIAL_LOG_ENABLED = true;

static constexpr uint32_t APPS_SERIAL_LOG_PERIOD_MS = 100;

float apps1Pct = 0.0f;
float apps2Pct = 0.0f;
float pedalPct = 0.0f;
float desiredTorqueNm = 0.0f;

bool appsPlausible = false;
bool appsDisagreeing = false;
uint32_t appsDisagreeStartMs = 0;
// Names the kind of APPS problem (channel disagreement vs. out-of-range
// sensor) for the dash fault texts; only meaningful while not plausible.
const char *appsFaultText = "APPS Disagree";
// True while brake + accelerator >25% travel are applied together; commands
// zero torque only for as long as the condition holds (no latch).
bool bspcActive = false;
bool appsFilterInitialized = false;
int rawApps1Last = 0;
int rawApps2Last = 0;
int rawBse1Last = 0;
int rawBse2Last = 0;
float bse1Voltage = 0.0f;
float bse2Voltage = 0.0f;
float bse1Pct = 0.0f;
float bse2Pct = 0.0f;
float brakePressurePct = 0.0f;
bool braking = false;
bool hardBraking = false;

// ---------------- Pedal ----------------
void updatePedal() {
  int rawApps1 = analogRead(APPS_1_PIN);
  int rawApps2 = analogRead(APPS_2_PIN);
  rawApps1Last = rawApps1;
  rawApps2Last = rawApps2;

  float newApps1Pct = scalePercentInverted(rawApps1, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float newApps2Pct = scalePercentNormal(rawApps2, APPS_2_RAW_MIN, APPS_2_RAW_MAX);

  if (!appsFilterInitialized) {
    apps1Pct = newApps1Pct;
    apps2Pct = newApps2Pct;
    appsFilterInitialized = true;
  } else {
    apps1Pct = APPS_EMA_ALPHA * newApps1Pct + (1.0f - APPS_EMA_ALPHA) * apps1Pct;
    apps2Pct = APPS_EMA_ALPHA * newApps2Pct + (1.0f - APPS_EMA_ALPHA) * apps2Pct;
  }

  bool apps1InRange = rawApps1 >= APPS_1_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps1 <= APPS_1_RAW_MAX + APPS_RAW_FAULT_MARGIN;
  bool apps2InRange = rawApps2 >= APPS_2_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps2 <= APPS_2_RAW_MAX + APPS_RAW_FAULT_MARGIN;

  float percentError = fabsf(apps1Pct - apps2Pct);
  bool wasPlausible = appsPlausible;
  bool appsAbnormal = !apps1InRange || !apps2InRange ||
                      percentError > APPS_IMPLAUSIBLE_PCT;

  if (appsAbnormal) {
    if (!apps1InRange && !apps2InRange) {
      appsFaultText = "APPS1+2 Range";
    } else if (!apps1InRange) {
      appsFaultText = "APPS1 Range";
    } else if (!apps2InRange) {
      appsFaultText = "APPS2 Range";
    } else {
      appsFaultText = "APPS Disagree";
    }
  }

  if (!appsAbnormal) {
    appsDisagreeing = false;
    appsPlausible = true;
  } else {
    uint32_t now = millis();
    if (!appsDisagreeing) {
      appsDisagreeing = true;
      appsDisagreeStartMs = now;
    }
    if (now - appsDisagreeStartMs > APPS_IMPLAUSIBILITY_PERSIST_MS) {
      appsPlausible = false;
    }
  }

  float pedalTravelPct = (apps1Pct + apps2Pct) * 0.5f;

  pedalPct = pedalTravelPct;
  if (pedalPct <= PEDAL_ZERO_DEADBAND_PCT) {
    pedalPct = 0.0f;
  } else {
    // Rescale the remaining travel back up to 0-100% so output ramps
    // smoothly from zero instead of jumping past the deadband.
    pedalPct = (pedalPct - PEDAL_ZERO_DEADBAND_PCT) * 100.0f / (100.0f - PEDAL_ZERO_DEADBAND_PCT);
  }

  // BSPC: zero torque once the brake (light-braking threshold) and
  // accelerator >25% travel are applied together. Per EV.4.7/T.4.7 this latches:
  // once tripped, torque stays suppressed until the accelerator is released
  // below 5% travel, regardless of brake state. Releasing the brake alone does
  // not restore torque.
  bool bspcWasActive = bspcActive;
  if (bspcActive) {
    // Latched: only the accelerator returning below 5% clears the cut.
    if (pedalPct < BSPC_APPS_RESET_PCT) {
      bspcActive = false;
    }
  } else if (braking && pedalPct > BSPC_APPS_THRESHOLD_PCT) {
    bspcActive = true;
  }
  if (bspcActive != bspcWasActive) {
    Serial.println(bspcActive ? "[BSPC] Active; commanding zero torque"
                              : "[BSPC] Cleared");
  }

  if (appsPlausible) {
    desiredTorqueNm = bspcActive ? 0.0f : (pedalPct / 100.0f) * MAX_TORQUE_NM;
  } else {
    desiredTorqueNm = 0.0f;
    if (wasPlausible) {
      Serial.print("[APPS] Implausible (");
      Serial.print(appsFaultText);
      Serial.print("): raw1=");
      Serial.print(rawApps1);
      Serial.print(" raw2=");
      Serial.print(rawApps2);
      Serial.print(" deviation=");
      Serial.print(percentError, 1);
      Serial.print("% for >");
      Serial.print(APPS_IMPLAUSIBILITY_PERSIST_MS);
      Serial.println(" ms");
      showFaultBanner(appsFaultText);
    }
    // T.4.2.10 only requires zero torque on implausibility; the car stays in
    // drive and torque returns once the channels agree again.
  }
}

void logAppsTelemetry() {
  if (!APPS_SERIAL_LOG_ENABLED) {
    return;
  }

  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < APPS_SERIAL_LOG_PERIOD_MS) {
    return;
  }
  lastPrintMs = now;

  Serial.print("[APPS] raw1=");
  Serial.print(rawApps1Last);
  Serial.print(" raw2=");
  Serial.print(rawApps2Last);
  Serial.print(" apps1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% apps2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% pedal=");
  Serial.print(pedalPct, 1);
  Serial.print("% torque=");
  Serial.print(desiredTorqueNm, 1);
  Serial.print("Nm cmd=");
  Serial.print(commandedTorqueNm, 1);
  Serial.print("Nm bspc=");
  Serial.print(bspcActive ? "ACTIVE" : "clear");
  Serial.print(" plausible=");
  Serial.println(appsPlausible ? "YES" : "NO");
}

void updateBrakePressureSensors() {
  rawBse1Last = analogRead(BSE_1_PIN);
  rawBse2Last = analogRead(BSE_2_PIN);

  braking = (rawBse1Last > BSE_1_BRAKING_RAW_THRESHOLD) ||
            (rawBse2Last > BSE_2_BRAKING_RAW_THRESHOLD);
  hardBraking = (rawBse1Last > BSE_1_HARD_BRAKING_RAW_THRESHOLD) ||
                (rawBse2Last > BSE_2_HARD_BRAKING_RAW_THRESHOLD);

  bse1Voltage = adcCountsToVoltage(rawBse1Last);
  bse2Voltage = adcCountsToVoltage(rawBse2Last);

  bse1Pct = scaleBrakePressurePercent(bse1Voltage);
  bse2Pct = scaleBrakePressurePercent(bse2Voltage);
  brakePressurePct = max(bse1Pct, bse2Pct);
}

float scalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaled = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaled, 0.0f, 100.0f);
}

float scalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaled = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaled, 0.0f, 100.0f);
}

float adcCountsToVoltage(int rawValue) {
  return ((float)rawValue * ADC_REFERENCE_VOLTAGE) / (float)ADC_MAX_COUNTS;
}

float scaleBrakePressurePercent(float voltage) {
  float scaled = (voltage - BSE_SENSOR_MIN_VOLTAGE) * 100.0f /
                 (BSE_SENSOR_MAX_VOLTAGE - BSE_SENSOR_MIN_VOLTAGE);
  scaled = constrain(scaled, 0.0f, 100.0f);
  return roundf(scaled * 100.0f) / 100.0f;
}

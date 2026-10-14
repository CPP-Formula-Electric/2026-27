/**
 * @file    regen.cpp
 * @brief   Calculates regen braking torque within the pack voltage and charge-current limits.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "regen.h"

#include "safety_limits.h"
#include "calibration.h"
#include "vehicle_speed.h"
#include "bms_link.h"
#include "inverter.h"
#include "power_derate.h"
#include "main.h"

static constexpr uint32_t REGEN_VOLTAGE_WINDOW_MS = 10000;
// A 24-entry ring retains at least 10 seconds at the BMS's 500 ms cadence.
static constexpr uint8_t REGEN_VOLTAGE_WINDOW_CAPACITY = 24;
// The BMS monitors 96 channels, but one is a bypassed busbar channel; the pack
// has 95 active series cell banks.
static constexpr uint8_t REGEN_PACK_SERIES_BANKS = 95;

// Each entry contains a synchronized 0x310 maximum-cell voltage and 0x311 pack
// current. Pack current is positive while discharging and negative during
// charge/regen, hence Voc ~= Vterminal + Ipack*R.
struct RegenVoltageSampleWindow {
  float terminalV[REGEN_VOLTAGE_WINDOW_CAPACITY];
  float estimatedOpenCircuitV[REGEN_VOLTAGE_WINDOW_CAPACITY];
  uint32_t timestampMs[REGEN_VOLTAGE_WINDOW_CAPACITY];
  uint8_t count;
  uint8_t head;
};

bool regenVoltageWindowMaximums(uint32_t nowMs, float &maxTerminalV,
                                float &maxEstimatedOpenCircuitV);
float regenCellChargeCurrentLimitA(uint32_t nowMs);

static RegenVoltageSampleWindow regenVoltageWindow = {};
static uint32_t lastRegenVoltageSampleMs = 0;

float regenTorqueNm = 0.0f;

// Store one voltage/current pair for the regen charge-current model. This is
// called only when the two back-to-back BMS frames arrived close enough to be
// treated as one measurement.
bool pushRegenVoltageSample(uint32_t nowMs) {
  // Reject missing/corrupt BMS voltage statistics. The normal operating range
  // is narrower, but these bounds avoid hiding a genuine high-cell condition.
  if (!haveCellVoltageStats || !haveBmsCurrent ||
      minCellVoltageMv < 1000 || maxCellVoltageMv > 5000 ||
      minCellVoltageMv > avgCellVoltageMv ||
      avgCellVoltageMv > maxCellVoltageMv ||
      fabsf(packCurrent) > REGEN_PACK_CURRENT_SENSOR_MAX_A) {
    return false;
  }

  float terminalV = (float)maxCellVoltageMv / 1000.0f;
  // BMS convention: positive pack current is discharge, negative is charge.
  // Vterminal = Voc - Ipack*R, therefore Voc = Vterminal + Ipack*R.
  float estimatedOpenCircuitV = terminalV +
      packCurrent * REGEN_CELL_BANK_RESISTANCE_OHM;
  if (estimatedOpenCircuitV < 0.0f || estimatedOpenCircuitV > 5.0f) {
    return false;
  }

  uint8_t slot = regenVoltageWindow.head;
  regenVoltageWindow.terminalV[slot] = terminalV;
  regenVoltageWindow.estimatedOpenCircuitV[slot] = estimatedOpenCircuitV;
  regenVoltageWindow.timestampMs[slot] = nowMs;
  regenVoltageWindow.head =
      (uint8_t)((slot + 1u) % REGEN_VOLTAGE_WINDOW_CAPACITY);
  if (regenVoltageWindow.count < REGEN_VOLTAGE_WINDOW_CAPACITY) {
    regenVoltageWindow.count++;
  }
  lastRegenVoltageSampleMs = nowMs;
  return true;
}

// Return the worst terminal and current-corrected maximum-cell voltages still
// inside the 10-second window. Timestamp subtraction remains valid through the
// millis() rollover because all intervals are much shorter than 2^31 ms.
bool regenVoltageWindowMaximums(uint32_t nowMs, float &maxTerminalV,
                                float &maxEstimatedOpenCircuitV) {
  bool found = false;
  for (uint8_t i = 0; i < regenVoltageWindow.count; i++) {
    if ((uint32_t)(nowMs - regenVoltageWindow.timestampMs[i]) >
        REGEN_VOLTAGE_WINDOW_MS) {
      continue;
    }

    float terminalV = regenVoltageWindow.terminalV[i];
    float estimatedOpenCircuitV =
        regenVoltageWindow.estimatedOpenCircuitV[i];
    if (!found || terminalV > maxTerminalV) maxTerminalV = terminalV;
    if (!found || estimatedOpenCircuitV > maxEstimatedOpenCircuitV) {
      maxEstimatedOpenCircuitV = estimatedOpenCircuitV;
    }
    found = true;
  }
  return found;
}

// Absolute allowed charge-current magnitude from the most conservative
// current-corrected cell voltage observed in the last 10 seconds. A recent
// terminal-voltage hit at the 4.180 V stop holds regen at zero for the same
// window, providing hysteresis below the BMS's 4.200 V hard cutoff.
float regenCellChargeCurrentLimitA(uint32_t nowMs) {
  if (!regenBmsPairValid || !haveBmsCurrent || !haveCellVoltageStats ||
      (uint32_t)(nowMs - lastBmsCellVoltageRxMs) > REGEN_BMS_DATA_TIMEOUT_MS ||
      (uint32_t)(nowMs - lastBmsCurrentRxMs) > REGEN_BMS_DATA_TIMEOUT_MS ||
      regenVoltageWindow.count == 0 ||
      (uint32_t)(nowMs - lastRegenVoltageSampleMs) > REGEN_BMS_DATA_TIMEOUT_MS) {
    return 0.0f;
  }

  float maxTerminalV = 0.0f;
  float maxEstimatedOpenCircuitV = 0.0f;
  if (!regenVoltageWindowMaximums(nowMs, maxTerminalV,
                                  maxEstimatedOpenCircuitV)) {
    return 0.0f;
  }

  if (maxTerminalV >= REGEN_CELL_TERMINAL_STOP_V) return 0.0f;

  float voltageHeadroomV =
      REGEN_CELL_CURRENT_TARGET_V - maxEstimatedOpenCircuitV;
  if (voltageHeadroomV <= 0.0f) return 0.0f;
  return voltageHeadroomV / REGEN_CELL_BANK_RESISTANCE_OHM;
}

// Returns a positive regen-torque magnitude. The caller applies it as negative
// PM100 torque. Mechanical braking remains fully independent of this request.
float calculateRegenTorqueNm() {
  if (!REGEN_ENABLED || !braking || !haveBmsStatus || !appsPlausible ||
      bspcActive || pedalPct > BSPC_APPS_RESET_PCT ||
      speedKmh <= REGEN_CUTOFF_SPEED_KMH) {
    return 0.0f;
  }

  // No fresh, synchronized BMS voltage/current pair means no safe basis for a
  // charging-current limit, so regen fails off.
  float cellChargeCurrentLimitA = regenCellChargeCurrentLimitA(millis());
  if (cellChargeCurrentLimitA <= 0.0f) return 0.0f;

  // Triangular pressure curve: build regen through light braking, then taper
  // it away as the driver enters the heavy mechanical-braking region.
  float pressureFactor;
  if (brakePressurePct <= REGEN_LIGHT_BRAKE_END_PCT) {
    pressureFactor = brakePressurePct / REGEN_LIGHT_BRAKE_END_PCT;
  } else if (brakePressurePct < REGEN_HEAVY_BRAKE_END_PCT) {
    pressureFactor = (REGEN_HEAVY_BRAKE_END_PCT - brakePressurePct) /
                     (REGEN_HEAVY_BRAKE_END_PCT - REGEN_LIGHT_BRAKE_END_PCT);
  } else {
    pressureFactor = 0.0f;
  }

  // Fade smoothly from full regen at 15 km/h to zero at the 5 km/h cutoff.
  float speedFactor = constrain((speedKmh - REGEN_CUTOFF_SPEED_KMH) /
                                (REGEN_FULL_SPEED_KMH - REGEN_CUTOFF_SPEED_KMH),
                                0.0f, 1.0f);

  // Linearly reduce the allowed charging power from 20 kW at 90% SOC to zero
  // at 95% SOC. The resulting torque cap enforces P = torque*rpm/9549.
  float socFactor = constrain((REGEN_SOC_DISABLE_PCT - (float)stateOfChargePct) /
                              (REGEN_SOC_DISABLE_PCT - REGEN_SOC_DERATE_START_PCT),
                              0.0f, 1.0f);
  float torqueNm = REGEN_MAX_TORQUE_NM * pressureFactor * speedFactor;
  float absRpm = fabsf((float)motorRpm);
  if (absRpm > 1.0f) {
    // Use min-cell voltage times the 95 active series banks as a lower bound
    // on pack voltage. With unity conversion efficiency, this makes the
    // mechanical-power-to-current conversion conservative.
    float packVoltageLowerBoundV =
        ((float)minCellVoltageMv / 1000.0f) *
        (float)REGEN_PACK_SERIES_BANKS;
    if (packVoltageLowerBoundV <= 0.0f) return 0.0f;

    float regenPowerLimitKw = REGEN_MAX_POWER_KW * socFactor;
    float cellVoltagePowerLimitKw =
        packVoltageLowerBoundV * cellChargeCurrentLimitA / 1000.0f;
    if (cellVoltagePowerLimitKw < regenPowerLimitKw) {
      regenPowerLimitKw = cellVoltagePowerLimitKw;
    }

    float powerTorqueCapNm = TORQUE_PER_KW_PER_RPM *
                             regenPowerLimitKw / absRpm;
    if (torqueNm > powerTorqueCapNm) torqueNm = powerTorqueCapNm;
  }
  return constrain(torqueNm, 0.0f, REGEN_MAX_TORQUE_NM);
}

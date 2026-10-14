/**
 * @file    bms_link.cpp
 * @brief   BMS status and temperature frames, cell data requests, BMS state.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "bms_link.h"

#include "safety_limits.h"
#include "can_bytes.h"
#include "main.h"
#include "vehicle_can.h"
#include "faults.h"
#include "regen.h"
#include "rtd.h"
#include "data_log.h"
#include "can_trace.h"
#include "fault_text.h"

bool regenBmsPairValid = false;

static constexpr uint32_t CELL_LOG_REQUEST_PERIOD_MS = 1000;
// A 96-cell voltage dump takes 32 frames * 5 ms in the BMS firmware.
static constexpr uint32_t CELL_VOLTAGE_DUMP_WAIT_MS = 250;

float packVoltage = 0.0f;
float packCurrent = 0.0f;

uint8_t stateOfChargePct = 0;
uint8_t batteryTempC = 0;
uint16_t minCellVoltageMv = 0;
uint16_t maxCellVoltageMv = 0;
uint16_t avgCellVoltageMv = 0;
bool haveCellVoltageStats = false;
int16_t minCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t maxCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t avgCellTempC = CELL_TEMP_UNKNOWN_C;
bool haveCellTempStats = false;

bool bmsActive = false;
bool bmsFault = true;
bool bmsTimedOut = true;
bool haveBmsStatus = false;
uint8_t bmsFlags = 0;
uint8_t bms2026FaultCode = 0;
bool haveBms2026FaultCode = false;
uint32_t lastBmsRxMs = 0;
bool haveBmsCurrent = false;
uint32_t lastBmsCellVoltageRxMs = 0;
uint32_t lastBmsCurrentRxMs = 0;

uint32_t lastCellLogRequestMs = 0;
uint32_t cellVoltageRequestMs = 0;
bool cellTemperatureRequestPending = false;

// Request one complete 96-cell voltage dump and one temperature dump each
// second. The delay keeps the two BMS response sequences from overlapping.
void serviceCellDataRequests(uint32_t nowMs) {
  if (!sdLogActive || !haveBmsStatus || bmsTimedOut) return;

  if (cellTemperatureRequestPending) {
    if (nowMs - cellVoltageRequestMs < CELL_VOLTAGE_DUMP_WAIT_MS) return;

    CAN_message_t request{};
    request.id = BMS_CELL_REQUEST_2026_ID;
    request.len = 1;
    request.buf[0] = CELL_DATA_REQUEST_TEMPERATURES;
    sendCanFrame(request);
    cellTemperatureRequestPending = false;
    return;
  }

  if (nowMs - lastCellLogRequestMs < CELL_LOG_REQUEST_PERIOD_MS) return;

  CAN_message_t request{};
  request.id = BMS_CELL_REQUEST_2026_ID;
  request.len = 1;
  request.buf[0] = CELL_DATA_REQUEST_VOLTAGES;
  sendCanFrame(request);
  lastCellLogRequestMs = nowMs;
  cellVoltageRequestMs = nowMs;
  cellTemperatureRequestPending = true;
}

// Contactors open is a normal pre-RTD condition, not a fault: it should drop/
// block RTD (back to neutral) without lighting the fault indicator.
bool contactorsOpen() {
  return haveBmsStatus && !(bmsFlags & 0x02);
}

// ---------------- CAN receive handlers ----------------
void handleBms2026StatusFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool wasActive = bmsActive;
  bool previousFault = bmsFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  uint8_t flags = msg.buf[0];
  minCellVoltageMv = u16Be(msg.buf[1], msg.buf[2]);
  maxCellVoltageMv = u16Be(msg.buf[3], msg.buf[4]);
  avgCellVoltageMv = u16Be(msg.buf[5], msg.buf[6]);
  haveCellVoltageStats = true;

  uint32_t nowMs = millis();
  haveBmsStatus    = true;
  bmsTimedOut      = false;
  lastBmsRxMs      = nowMs;
  lastBmsCellVoltageRxMs = nowMs;
  // Remain fail-safe until the following 0x311 frame supplies the matching
  // current measurement and the pair passes validation.
  regenBmsPairValid = false;
  bmsFlags         = flags;
  bmsFault         = (flags & 0x01) != 0;
  bmsActive        = !bmsFault;
  stateOfChargePct = msg.buf[7];
  packVoltage      = (avgCellVoltageMv / 1000.0f) * 96.0f;  // 96 cells in series

  if (bmsActive && !wasActive) {
    Serial.println("[BMS] Active");
  }

  if (faultDetectionArmed && bmsFault != previousFault) {
    Serial.print("[BMS] 2026 flags=0x");
    Serial.print(flags, HEX);
    Serial.print(" fault=");
    Serial.println(bmsFault ? "YES" : "NO");
    if (bmsFault) {
      showFaultBanner((haveBms2026FaultCode && bms2026FaultCode != 0)
                          ? bmsTemperatureFaultCodeName(bms2026FaultCode)
                          : "BMS Fault");
    }
  }

  if (anyFaultActive() || contactorsOpen()) {
    dropReadyToDrive(rtdDropCauseText());
  }

  screenDirty = true;
}

void handleBms2026TempFrame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  // byte 1 = max cell temp encoded as (°C + 40); 0xFF means no valid reading
  uint8_t previousFaultCode = haveBms2026FaultCode ? bms2026FaultCode : 0;
  bms2026FaultCode = msg.buf[3];
  haveBms2026FaultCode = true;
  if (bmsImdFaultDetectionArmed() && bms2026FaultCode != 0 &&
      bms2026FaultCode != previousFaultCode) {
    showFaultBanner(bmsTemperatureFaultCodeName(bms2026FaultCode));
  }

  haveCellTempStats = (msg.buf[0] != 0xFF && msg.buf[1] != 0xFF && msg.buf[2] != 0xFF);
  if (haveCellTempStats) {
    minCellTempC = (int16_t)msg.buf[0] - 40;
    maxCellTempC = (int16_t)msg.buf[1] - 40;
    avgCellTempC = (int16_t)msg.buf[2] - 40;
  } else {
    minCellTempC = CELL_TEMP_UNKNOWN_C;
    maxCellTempC = CELL_TEMP_UNKNOWN_C;
    avgCellTempC = CELL_TEMP_UNKNOWN_C;
  }

  if (msg.buf[1] != 0xFF) {
    batteryTempC = (uint8_t)constrain((int)msg.buf[1] - 40, 0, 255);
  }

  // bytes 4-5 = pack current, big-endian, raw = A * 10 + 3000
  int16_t curRaw = (int16_t)(((uint16_t)msg.buf[4] << 8) | msg.buf[5]);
  packCurrent = ((float)curRaw - 3000.0f) / 10.0f;
  haveBmsCurrent = true;
  uint32_t nowMs = millis();
  lastBmsCurrentRxMs = nowMs;

  // The BMS sends 0x310 immediately before 0x311. Pair only adjacent/fresh
  // frames so a current sample is never used to correct an unrelated voltage.
  regenBmsPairValid = false;
  if (haveCellVoltageStats &&
      (uint32_t)(nowMs - lastBmsCellVoltageRxMs) <=
          REGEN_BMS_PAIR_MAX_SKEW_MS) {
    regenBmsPairValid = pushRegenVoltageSample(nowMs);
  }

  screenDirty = true;
}

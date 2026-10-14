/**
 * @file    energy_meter.cpp
 * @brief   Decodes the FSAE Energy Meter frames for the dashboard.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * The VCU only listens to the Energy Meter; it never sends to it.
 */
#include "energy_meter.h"

#include <ST7796_t3.h>

#include "can_bytes.h"
#include "main.h"

// 0x40D status byte 0 bit masks.
static constexpr uint8_t EM_STATUS_VIOLATION    = 1u << 0;
static constexpr uint8_t EM_STATUS_LOGGING      = 1u << 1;
static constexpr uint8_t EM_STATUS_FAULT_ACTIVE = 1u << 2;
static constexpr uint8_t EM_STATUS_FAULT_PREV   = 1u << 3;
// The fastest EM frame is 20 ms; flag a comms fault if nothing arrives for this long.
static constexpr uint32_t EM_TIMEOUT_MS = 1000;

// ---- Energy Meter decoded state ----
// Populated from 0x10D / 0x40D / 0x60D. "have" flags gate the dashboard so an
// absent meter shows "--"/"NO COMMS" instead of stale zeros.
float emCurrentA = 0.0f;
float emVoltageV = 0.0f;
bool haveEmMeasurement = false;
uint8_t emStatusByte = 0;
bool haveEmStatus = false;
uint8_t emNumSensors = 0;
uint8_t emMinTempRaw = 0;
uint8_t emMaxTempRaw = 0;
bool haveEmTempSummary = false;
bool haveEmTemps = false;
uint32_t emLastRxMs = 0;
bool emTimedOut = true;

// ---------------- Energy Meter ----------------
// 0x10D Measurement: Current[A] float (bytes 0-3), Voltage[V] float (bytes 4-7).
void handleEmMeasurementFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 8) {
    return;
  }
  emCurrentA = f32Le(&msg.buf[0]);
  emVoltageV = f32Le(&msg.buf[4]);
  haveEmMeasurement = true;
  screenDirty = true;
}

// 0x40D Status: byte 0 holds the status bits (bytes 1-4 are Energy[Whr], unused
// on the dashboard).
void handleEmStatusFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 1) {
    return;
  }
  emStatusByte = msg.buf[0];
  haveEmStatus = true;
  screenDirty = true;
}

// 0x60D Temperature: byte 0 bits 0-2 = multiplexor, bits 3-7 = sensor count.
// The mux-0 frame carries the count plus min/max in bytes 1/2 (degC = raw*0.5),
// which is all the dashboard needs.
void handleEmTemperatureFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 8) {
    return;
  }
  if ((msg.buf[0] & 0x07) == 0) {
    emNumSensors = msg.buf[0] >> 3;
    emMinTempRaw = msg.buf[1];
    emMaxTempRaw = msg.buf[2];
    haveEmTempSummary = true;
  }
  haveEmTemps = true;
  screenDirty = true;
}

// Flags the EM comms-lost condition once nothing has arrived for EM_TIMEOUT_MS.
void serviceEnergyMeterTimeout(uint32_t now) {
  if (!haveEmMeasurement && !haveEmStatus && !haveEmTemps) {
    return;  // never seen the meter yet; stays "NO COMMS" without a timeout edge
  }
  emTimedOut = (now - emLastRxMs > EM_TIMEOUT_MS);
}

// One-glance EM health string for the main dashboard, with a colour out-param.
const char *emStatusSummaryText(uint16_t &colorOut) {
  if (emTimedOut || (!haveEmStatus && !haveEmMeasurement)) {
    colorOut = ST7735_YELLOW;
    return "NO COMMS";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_FAULT_ACTIVE)) {
    colorOut = ST7735_RED;
    return "FAULT";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_VIOLATION)) {
    colorOut = ST7735_RED;
    return "VIOLATION";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_LOGGING)) {
    colorOut = ST7735_GREEN;
    return "LOGGING";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_FAULT_PREV)) {
    colorOut = ST7735_YELLOW;
    return "IDLE/PFLT";
  }
  colorOut = ST7735_YELLOW;
  return "IDLE";
}

/**
 * @file    inverter.cpp
 * @brief   PM100DX status frames, CAN broadcast enable and automatic fault clear.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "inverter.h"

#include "safety_limits.h"
#include "can_bytes.h"
#include "bms_link.h"
#include "vehicle_can.h"
#include "rtd.h"
#include "fault_text.h"
#include "dashboard.h"

void sendPm100BroadcastEnable();
void sendPm100FaultClear();

static constexpr uint32_t PM100_PARAM_COMMAND_ID  = 0x0C1;

// EEPROM parameter 148 = "CAN Active Messages Lo Word"; bit 0x0200 enables the
// 0x0A9 Internal Voltages broadcast that carries the GLV (12V) voltage.
static constexpr uint16_t PM100_PARAM_CAN_ACTIVE_MSGS = 148;
static constexpr uint32_t PM100_BROADCAST_ENABLE_DELAY_MS = 5000;
static constexpr uint32_t PM100_BROADCAST_ENABLE_RETRY_MS = 5000;
static constexpr uint8_t PM100_BROADCAST_ENABLE_MAX_ATTEMPTS = 3;
// Parameter 20 = "Fault Clear"; writing 0 tells the inverter to clear latched
// fault codes. Sent automatically when the inverter is faulted but the BMS
// reports contactors closed and the DC bus is back at pack voltage, so a
// transient fault (e.g. under-voltage from an HV cycle or pack sag) doesn't
// require power-cycling the inverter. 240 V = 96 series cells at the 2.5 V
// minimum, so any bus above that means the pack is genuinely back on the bus.
static constexpr uint16_t PM100_PARAM_FAULT_CLEAR = 20;

float inverterDcBusVoltage = 0.0f;
float inverterDcBusCurrent = 0.0f;
bool haveInverterVoltageInfo = false;
bool haveInverterCurrentInfo = false;
// GLV (12V system) voltage reported by the PM100 Internal Voltages frame.
float glvVoltage = 0.0f;
bool haveGlvVoltage = false;
// If the inverter is talking but 0x0A9 never arrives, its broadcast mask has
// Internal Voltages disabled; we write parameter 148 once to turn it back on.
bool havePm100Traffic = false;
uint32_t firstPm100FrameMs = 0;
uint8_t pm100BroadcastEnableAttempts = 0;
uint32_t pm100BroadcastEnableLastTxMs = 0;
float powerKw = 0.0f;
int16_t inverterTempTenthsC = 0;
int16_t pm100MotorTempTenthsC = 0;
bool haveInverterTemp = false;
bool havePm100MotorTemp = false;
uint32_t pm100PostFaults = 0;
uint32_t pm100RunFaults = 0;
bool havePm100FaultStatus = false;
bool pm100Fault = false;
// Automatic fault-clear bookkeeping; attempts reset when the fault clears or
// the contactors open, so each HV cycle gets a fresh set of retries.
uint8_t pm100FaultClearAttempts = 0;
uint32_t pm100FaultClearLastTxMs = 0;
int16_t motorRpm = 0;

void handlePm100Temperatures1Frame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  int16_t hottestTempRaw = moduleATempRaw;
  if (moduleBTempRaw > hottestTempRaw) hottestTempRaw = moduleBTempRaw;
  if (moduleCTempRaw > hottestTempRaw) hottestTempRaw = moduleCTempRaw;
  if (gateDriverTempRaw > hottestTempRaw) hottestTempRaw = gateDriverTempRaw;

  inverterTempTenthsC = hottestTempRaw;
  haveInverterTemp = true;
  screenDirty = true;
}

void handlePm100Temperatures3Frame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  pm100MotorTempTenthsC = s16Le(msg.buf[4], msg.buf[5]);
  havePm100MotorTemp = true;
  screenDirty = true;
}

void handlePm100RpmFrame(const CAN_message_t &msg) {
  if (msg.len < 4) {
    return;
  }

  motorRpm = s16Le(msg.buf[2], msg.buf[3]);
  screenDirty = true;
}

void handlePm100CurrentInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  inverterDcBusCurrent = (float)s16Le(msg.buf[6], msg.buf[7]) / 10.0f;
  haveInverterCurrentInfo = true;
  screenDirty = true;
}

void handlePm100VoltageInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 2) {
    return;
  }

  inverterDcBusVoltage = (float)s16Le(msg.buf[0], msg.buf[1]) / 10.0f;
  haveInverterVoltageInfo = true;
  screenDirty = true;
}

void handlePm100InternalVoltagesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  // Bytes 6-7 = 12V System voltage, "Low Voltage" format (volts * 100).
  glvVoltage = (float)s16Le(msg.buf[6], msg.buf[7]) / 100.0f;
  haveGlvVoltage = true;
  screenDirty = true;
}

void handlePm100ParamResponseFrame(const CAN_message_t &msg) {
  if (msg.len < 3) {
    return;
  }

  uint16_t address = u16Le(msg.buf[0], msg.buf[1]);
  if (address == PM100_PARAM_CAN_ACTIVE_MSGS) {
    Serial.print("[PM100] CAN Active Messages parameter write ");
    Serial.println(msg.buf[2] ? "succeeded" : "FAILED");
  } else if (address == PM100_PARAM_FAULT_CLEAR) {
    Serial.print("[PM100] Fault clear command ");
    Serial.println(msg.buf[2] ? "accepted" : "REJECTED");
  }
}

// The 0x0A9 Internal Voltages broadcast (GLV voltage source) can be disabled
// in the inverter's EEPROM broadcast mask. If the inverter is alive but 0x0A9
// stays silent, write parameter 148 to enable all broadcast messages.
// EEPROM parameters are only writable while the motor is not running.
void servicePm100BroadcastEnable() {
  if (haveGlvVoltage || !havePm100Traffic || readyToDriveLatched) {
    return;
  }

  if (pm100BroadcastEnableAttempts >= PM100_BROADCAST_ENABLE_MAX_ATTEMPTS) {
    return;
  }

  uint32_t now = millis();
  if (now - firstPm100FrameMs < PM100_BROADCAST_ENABLE_DELAY_MS) {
    return;
  }

  if (pm100BroadcastEnableAttempts > 0 &&
      now - pm100BroadcastEnableLastTxMs < PM100_BROADCAST_ENABLE_RETRY_MS) {
    return;
  }

  pm100BroadcastEnableAttempts++;
  pm100BroadcastEnableLastTxMs = now;

  Serial.print("[PM100] No 0x0A9 Internal Voltages frames; enabling broadcasts, attempt ");
  Serial.println(pm100BroadcastEnableAttempts);
  sendPm100BroadcastEnable();
}

void sendPm100BroadcastEnable() {
  CAN_message_t tx = {};
  tx.id = PM100_PARAM_COMMAND_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS & 0xFF);
  tx.buf[1] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS >> 8);
  tx.buf[2] = 1;     // write
  tx.buf[3] = 0;     // reserved
  tx.buf[4] = 0xFF;  // CAN Active Messages Lo Word: enable all broadcasts
  tx.buf[5] = 0xFF;
  tx.buf[6] = 0xFF;  // Hi Word: keep command/BMS/OBD2 mailboxes enabled
  tx.buf[7] = 0xFF;
  sendCanFrame(tx);
}

// Automatically clears latched PM100 faults over CAN once the conditions that
// caused them are gone: the BMS must report contactors closed and the inverter
// must see pack voltage on its DC bus again. Without this, an under-voltage
// fault latched during an HV cycle or pack sag blocks torque until someone
// power-cycles the inverter. Fault Clear is a command parameter (not an EEPROM
// write), so it is safe to send any time; a fault whose cause is still present
// simply latches again.
void servicePm100FaultClear() {
  if (!havePm100FaultStatus || !pm100Fault) {
    return;
  }

  if (!haveBmsStatus || contactorsOpen()) {
    pm100FaultClearAttempts = 0;
    return;
  }

  if (!haveInverterVoltageInfo ||
      inverterDcBusVoltage < PM100_FAULT_CLEAR_MIN_BUS_V) {
    return;
  }

  if (pm100FaultClearAttempts >= PM100_FAULT_CLEAR_MAX_ATTEMPTS) {
    return;
  }

  uint32_t now = millis();
  if (pm100FaultClearAttempts > 0 &&
      now - pm100FaultClearLastTxMs < PM100_FAULT_CLEAR_RETRY_MS) {
    return;
  }

  pm100FaultClearAttempts++;
  pm100FaultClearLastTxMs = now;

  Serial.print("[PM100] Fault latched with contactors closed and bus at ");
  Serial.print(inverterDcBusVoltage, 1);
  Serial.print(" V; sending fault clear, attempt ");
  Serial.println(pm100FaultClearAttempts);
  sendPm100FaultClear();
}

void sendPm100FaultClear() {
  CAN_message_t tx = {};
  tx.id = PM100_PARAM_COMMAND_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = (uint8_t)(PM100_PARAM_FAULT_CLEAR & 0xFF);
  tx.buf[1] = (uint8_t)(PM100_PARAM_FAULT_CLEAR >> 8);
  tx.buf[2] = 1;  // write
  tx.buf[3] = 0;  // reserved
  tx.buf[4] = 0;  // data 0 = clear all faults
  tx.buf[5] = 0;
  tx.buf[6] = 0;
  tx.buf[7] = 0;
  sendCanFrame(tx);
}

void handlePm100FaultCodesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool previousFault = havePm100FaultStatus && pm100Fault;
  pm100PostFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  pm100RunFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
  havePm100FaultStatus = true;
  pm100Fault = (pm100PostFaults != 0 || pm100RunFaults != 0);
  if (pm100Fault && !previousFault) {
    showFaultBanner(firstPm100FaultName(pm100PostFaults, pm100RunFaults));
  }
  if (!pm100Fault) {
    if (previousFault && pm100FaultClearAttempts > 0) {
      Serial.println("[PM100] Faults cleared");
    }
    pm100FaultClearAttempts = 0;
  }
  screenDirty = true;
}

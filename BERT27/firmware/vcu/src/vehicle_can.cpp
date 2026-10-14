/**
 * @file    vehicle_can.cpp
 * @brief   Sets up CAN3, sends frames and dispatches received frames to their module.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "vehicle_can.h"

#include "energy_meter.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "data_log.h"
#include "can_trace.h"

void dispatchReceivedCanFrame(const CAN_message_t &msg);
void logCanFrameToSerial(const char *direction, const CAN_message_t &msg);

// ---------------- CAN ----------------
// GPIO 23 is used by the RTD button, so this sketch uses the Teensy 4.1 CAN3
// peripheral on pins 30=CAN RX and 31=CAN TX. These pins connect to the TXD/RXD
// side of an external CAN transceiver module; the module's CANH/CANL side
// connects to the vehicle CAN bus.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> VehicleCan;

static constexpr uint32_t CAN_BAUD = 500000;
static constexpr bool CAN_LOG_SERIAL_ENABLED = true;

static constexpr uint32_t PM100_PARAM_RESPONSE_ID = 0x0C2;

// ---- Formula SAE competition Energy Meter (EM_User_Manual_2026) ----
// The Energy Meter shares this 500 kbit/s vehicle bus. It broadcasts three
// 11-bit standard frames; all signals are little-endian. The VCU only listens
// and decodes Voltage, the module-temperature summary, and the status bits for
// the dashboard.
static constexpr uint32_t EM_MEASUREMENT_ID = 0x10D;  // 20 ms:  Current[A], Voltage[V] floats
static constexpr uint32_t EM_STATUS_ID      = 0x40D;  // 100 ms: status bits + Energy[Whr] float
static constexpr uint32_t EM_TEMPERATURE_ID = 0x60D;  // 250 ms: muxed module temperatures

// ---------------- CAN transport ----------------
void setupVehicleCan() {
  VehicleCan.begin();
  VehicleCan.setBaudRate(CAN_BAUD);
  VehicleCan.setMaxMB(16);
  VehicleCan.enableFIFO();

  Serial.println("[CAN] CAN3 started at 500 kbit/s on pins 30=RX, 31=TX");
}

void sendCanFrame(CAN_message_t &msg) {
  noInterrupts();
  VehicleCan.write(msg);
  interrupts();
  logCanFrame("TX", msg);
}

void receiveCanFrames() {
  CAN_message_t rx;
  while (VehicleCan.read(rx)) {
    logCanFrame("RX", rx);
    dispatchReceivedCanFrame(rx);
  }
}

void dispatchReceivedCanFrame(const CAN_message_t &msg) {
  if (!havePm100Traffic && msg.id >= PM100_TEMPERATURE_1_FRAME_ID &&
      msg.id <= PM100_TORQUE_CAPABILITY_FRAME_ID) {
    havePm100Traffic = true;
    firstPm100FrameMs = millis();
  }

  switch (msg.id) {
    case IMD_INFO_GENERAL_ID:
    case IMD_RESPONSE_ID:
      handleImdFrame(msg);
      break;

    case PM100_TEMPERATURE_1_FRAME_ID:
      handlePm100Temperatures1Frame(msg);
      break;

    case PM100_TEMPERATURE_3_FRAME_ID:
      handlePm100Temperatures3Frame(msg);
      break;

    case PM100_RPM_FRAME_ID:
      handlePm100RpmFrame(msg);
      break;

    case PM100_CURRENT_INFO_FRAME_ID:
      handlePm100CurrentInfoFrame(msg);
      break;

    case PM100_VOLTAGE_INFO_FRAME_ID:
      handlePm100VoltageInfoFrame(msg);
      break;

    case PM100_INTERNAL_VOLTAGES_FRAME_ID:
      handlePm100InternalVoltagesFrame(msg);
      break;

    case PM100_FAULT_CODES_FRAME_ID:
      handlePm100FaultCodesFrame(msg);
      break;

    case PM100_PARAM_RESPONSE_ID:
      handlePm100ParamResponseFrame(msg);
      break;

    case BMS_STATUS_2026_ID:
      handleBms2026StatusFrame(msg);
      break;

    case BMS_TEMPERATURE_2026_ID:
      handleBms2026TempFrame(msg);
      break;

    case BMS_CELL_RESPONSE_2026_ID:
      writeCellDataResponseLog(msg);
      break;

    case EM_MEASUREMENT_ID:
      handleEmMeasurementFrame(msg);
      break;

    case EM_STATUS_ID:
      handleEmStatusFrame(msg);
      break;

    case EM_TEMPERATURE_ID:
      handleEmTemperatureFrame(msg);
      break;

    default:
      break;
  }
}

// ---------------- CAN logging ----------------
void logCanFrame(const char *direction, const CAN_message_t &msg) {
  updateCanTranslationState(direction, msg);

  // Future SD-card logging should mirror this raw record and the decoded record here.
  if (CAN_LOG_SERIAL_ENABLED) {
    logCanFrameToSerial(direction, msg);
    logCanTranslationToSerial(direction, msg);
  }
}

void logCanFrameToSerial(const char *direction, const CAN_message_t &msg) {
  Serial.print("[CAN ");
  Serial.print(direction);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(msg.flags.extended ? " ext" : " std");
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < msg.len; i++) {
    printCanHexByte(msg.buf[i]);
    if (i + 1 < msg.len) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

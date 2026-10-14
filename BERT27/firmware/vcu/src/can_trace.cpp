/**
 * @file    can_trace.cpp
 * @brief   Prints CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "can_trace.h"

#include "can_bytes.h"
#include "imd.h"
#include "bms_link.h"
#include "vehicle_can.h"
#include "inverter_command.h"
#include "can_trace_imd.h"
#include "fault_text.h"

static constexpr bool CAN_TRANSLATION_SERIAL_ENABLED = true;

static constexpr uint8_t CELL_DATA_REQUEST_UNKNOWN      = 0x00;

// Bender iso175 IMD/TSSI frames from VCU_TSSI_CODE_5_31_26.ino.
static constexpr uint32_t IMD_INFO_ISOLATION_DETAIL_ID = 0x38;
static constexpr uint32_t IMD_INFO_VOLTAGE_ID = 0x39;
static constexpr uint32_t IMD_INFO_IT_SYSTEM_ID = 0x3A;
static constexpr uint32_t IMD_FALLBACK_REQUEST_ID = 0x0FF;

uint8_t lastCellDataRequestType = CELL_DATA_REQUEST_UNKNOWN;

typedef void (*CanTranslationPrinter)(const CAN_message_t &msg);

struct CanFrameTranslation {
  uint32_t id;
  const char *name;
  CanTranslationPrinter print;
};

void printBmsStatus2026Translation(const CAN_message_t &msg);
void printBmsTemperature2026Translation(const CAN_message_t &msg);
void printCellDataRequest2026Translation(const CAN_message_t &msg);
void printCellDataResponse2026Translation(const CAN_message_t &msg);
void printPm100CommandTranslation(const CAN_message_t &msg);
void printPm100Temperatures1Translation(const CAN_message_t &msg);
void printPm100Temperatures3Translation(const CAN_message_t &msg);
void printPm100MotorPositionTranslation(const CAN_message_t &msg);
void printPm100CurrentInfoTranslation(const CAN_message_t &msg);
void printPm100VoltageInfoTranslation(const CAN_message_t &msg);
void printPm100InternalVoltagesTranslation(const CAN_message_t &msg);
void printPm100FaultCodesTranslation(const CAN_message_t &msg);
void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg);
void printBmsTemperatureFaultCode(uint8_t faultCode);
void printCellDataRequestName(uint8_t requestType);
void printCellVoltageValue(uint16_t millivolts);

static const CanFrameTranslation CAN_FRAME_TRANSLATIONS[] = {
  {BMS_STATUS_2026_ID,        "BMS Status",        printBmsStatus2026Translation},
  {BMS_TEMPERATURE_2026_ID,   "BMS Temperature",   printBmsTemperature2026Translation},
  {BMS_CELL_REQUEST_2026_ID,  "Cell Data Request", printCellDataRequest2026Translation},
  {BMS_CELL_RESPONSE_2026_ID, "Cell Data Response", printCellDataResponse2026Translation},
  {IMD_REQUEST_ID, "IMD Request", printImdRequestTranslation},
  {IMD_FALLBACK_REQUEST_ID, "IMD Fallback Request", printImdRequestTranslation},
  {IMD_RESPONSE_ID, "IMD Response", printImdResponseTranslation},
  {IMD_INFO_GENERAL_ID, "IMD Info General", printImdInfoGeneralTranslation},
  {IMD_INFO_ISOLATION_DETAIL_ID, "IMD Info Isolation Detail", printImdInfoIsolationDetailTranslation},
  {IMD_INFO_VOLTAGE_ID, "IMD Info Voltage", printImdInfoVoltageTranslation},
  {IMD_INFO_IT_SYSTEM_ID, "IMD Info IT-System", printImdInfoItSystemTranslation},
  {PM100_CMD_ID, "PM100 Command", printPm100CommandTranslation},
  {PM100_TEMPERATURE_1_FRAME_ID, "PM100 Temperatures #1", printPm100Temperatures1Translation},
  {PM100_TEMPERATURE_3_FRAME_ID, "PM100 Temperatures #3", printPm100Temperatures3Translation},
  {PM100_RPM_FRAME_ID, "PM100 Motor Position", printPm100MotorPositionTranslation},
  {PM100_CURRENT_INFO_FRAME_ID, "PM100 Current Information", printPm100CurrentInfoTranslation},
  {PM100_VOLTAGE_INFO_FRAME_ID, "PM100 Voltage Information", printPm100VoltageInfoTranslation},
  {PM100_INTERNAL_VOLTAGES_FRAME_ID, "PM100 Internal Voltages", printPm100InternalVoltagesTranslation},
  {PM100_FAULT_CODES_FRAME_ID, "PM100 Fault Codes", printPm100FaultCodesTranslation},
  {PM100_TORQUE_CAPABILITY_FRAME_ID, "PM100 Torque Capability", printPm100TorqueCapabilityTranslation}
};

static constexpr uint8_t CAN_FRAME_TRANSLATION_COUNT =
    sizeof(CAN_FRAME_TRANSLATIONS) / sizeof(CAN_FRAME_TRANSLATIONS[0]);

// ---------------- CAN decoded logging ----------------
void updateCanTranslationState(const char *direction, const CAN_message_t &msg) {
  (void)direction;

  if (msg.flags.extended || msg.id != BMS_CELL_REQUEST_2026_ID || msg.len < 1) {
    return;
  }

  lastCellDataRequestType = msg.buf[0];
}

void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg) {
  if (!CAN_TRANSLATION_SERIAL_ENABLED) {
    return;
  }

  for (uint8_t i = 0; i < CAN_FRAME_TRANSLATION_COUNT; i++) {
    const CanFrameTranslation &translation = CAN_FRAME_TRANSLATIONS[i];
    if (msg.id != translation.id) {
      continue;
    }

    Serial.print("[CAN ");
    Serial.print(direction);
    Serial.print(" DECODED] ");
    Serial.print(translation.name);
    Serial.print(": ");
    translation.print(msg);
    Serial.println();
    return;
  }
}

void printBmsStatus2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t flags = msg.buf[0];
  uint16_t minCellMv = u16Be(msg.buf[1], msg.buf[2]);
  uint16_t maxCellMv = u16Be(msg.buf[3], msg.buf[4]);
  uint16_t avgCellMv = u16Be(msg.buf[5], msg.buf[6]);

  Serial.print("flags=0x");
  printCanHexByte(flags);
  Serial.print(" fault=");
  Serial.print((flags & (1u << 0)) ? "YES" : "NO");
  Serial.print(" posContactorClosed=");
  Serial.print((flags & (1u << 1)) ? "YES" : "NO");
  Serial.print(" chargeMode=");
  Serial.print((flags & (1u << 2)) ? "YES" : "NO");
  Serial.print(" minCell=");
  Serial.print(minCellMv);
  Serial.print("mV maxCell=");
  Serial.print(maxCellMv);
  Serial.print("mV avgCell=");
  Serial.print(avgCellMv);
  Serial.print("mV soc=");
  if (msg.buf[7] == 0xFF) {
    Serial.print("unknown");
  } else {
    Serial.print(msg.buf[7]);
    Serial.print('%');
  }
}

void printBmsTemperature2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t faultCode = msg.buf[3];
  uint16_t packCurrentRaw = u16Be(msg.buf[4], msg.buf[5]);
  float packCurrentA = ((int32_t)packCurrentRaw - 3000) * 0.1f;

  Serial.print("minCellTemp=");
  Serial.print((int)msg.buf[0] - 40);
  Serial.print("C maxCellTemp=");
  Serial.print((int)msg.buf[1] - 40);
  Serial.print("C avgCellTemp=");
  Serial.print((int)msg.buf[2] - 40);
  Serial.print("C faultCode=");
  Serial.print(faultCode);
  Serial.print('(');
  printBmsTemperatureFaultCode(faultCode);
  Serial.print(") packCurrent=");
  Serial.print(packCurrentA, 1);
  Serial.print("A reserved=");
  printCanHexByte(msg.buf[6]);
  Serial.print(' ');
  printCanHexByte(msg.buf[7]);
}

void printCellDataRequest2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request byte");
    return;
  }

  Serial.print("request=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printCellDataRequestName(msg.buf[0]);
  Serial.print(')');
}

void printCellDataResponse2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected frame index");
    return;
  }

  uint8_t frameIndex = msg.buf[0];

  Serial.print("frameIndex=");
  Serial.print(frameIndex);

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    uint8_t availablePairs = (msg.len > 1) ? ((msg.len - 1) / 2) : 0;
    if (availablePairs > 3) {
      availablePairs = 3;
    }

    Serial.print(" mode=voltages cells=");
    for (uint8_t i = 0; i < availablePairs; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 3u + i + 1u);
      Serial.print('=');
      printCellVoltageValue(u16Be(msg.buf[1 + (i * 2)], msg.buf[2 + (i * 2)]));
    }
    if (availablePairs == 0) {
      Serial.print("none");
    }
    if (msg.len > 7) {
      Serial.print(" extra=");
      printCanHexByte(msg.buf[7]);
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    uint8_t availableTemps = (msg.len > 1) ? (msg.len - 1) : 0;
    if (availableTemps > 7) {
      availableTemps = 7;
    }

    Serial.print(" mode=temperatures cells=");
    for (uint8_t i = 0; i < availableTemps; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 7u + i + 1u);
      Serial.print('=');
      Serial.print((int)msg.buf[1 + i] - 40);
      Serial.print('C');
    }
    if (availableTemps == 0) {
      Serial.print("none");
    }
  } else {
    Serial.print(" mode=unknown lastRequest=0x");
    printCanHexByte(lastCellDataRequestType);
    Serial.print(" payload=");
    printRawPayloadFrom(msg, 1);
  }
}

void printPm100CommandTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t torqueRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t speedCommandRpm = s16Le(msg.buf[2], msg.buf[3]);
  uint8_t directionCommand = msg.buf[4];
  uint8_t inverterEnable = msg.buf[5];
  uint8_t inverterDischarge = msg.buf[6];
  uint8_t speedModeEnable = msg.buf[7];

  Serial.print("torqueCommand=");
  Serial.print((float)torqueRaw / 10.0f, 1);
  Serial.print("Nm speedCommand=");
  Serial.print(speedCommandRpm);
  Serial.print("rpm direction=0x");
  printCanHexByte(directionCommand);
  Serial.print('(');
  if (directionCommand == 0x00) {
    Serial.print("reverse/clockwise");
  } else if (directionCommand == 0x01) {
    Serial.print("forward/counterclockwise");
  } else {
    Serial.print("unknown");
  }
  Serial.print(") inverterEnable=");
  Serial.print(inverterEnable ? "ON" : "OFF");
  Serial.print(" inverterEnableRaw=0x");
  printCanHexByte(inverterEnable);
  Serial.print(" inverterDischarge=0x");
  printCanHexByte(inverterDischarge);
  Serial.print(" speedModeEnable=0x");
  printCanHexByte(speedModeEnable);
}

void printPm100Temperatures1Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("moduleA=");
  Serial.print((float)moduleATempRaw / 10.0f, 1);
  Serial.print("C moduleB=");
  Serial.print((float)moduleBTempRaw / 10.0f, 1);
  Serial.print("C moduleC=");
  Serial.print((float)moduleCTempRaw / 10.0f, 1);
  Serial.print("C gateDriver=");
  Serial.print((float)gateDriverTempRaw / 10.0f, 1);
  Serial.print("C");
}

void printPm100Temperatures3Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t coolantTempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t hotSpotTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t motorTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t torqueShudderRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("coolant=");
  Serial.print((float)coolantTempRaw / 10.0f, 1);
  Serial.print("C hotSpot=");
  Serial.print((float)hotSpotTempRaw / 10.0f, 1);
  Serial.print("C motor=");
  Serial.print((float)motorTempRaw / 10.0f, 1);
  Serial.print("C torqueShudder=");
  Serial.print((float)torqueShudderRaw / 10.0f, 1);
  Serial.print("Nm");
}

void printPm100MotorPositionTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t motorAngleRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t motorSpeedRpm = s16Le(msg.buf[2], msg.buf[3]);
  int16_t electricalFrequencyRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t deltaResolverRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("motorAngle=");
  Serial.print((float)motorAngleRaw / 10.0f, 1);
  Serial.print("deg motorSpeed=");
  Serial.print(motorSpeedRpm);
  Serial.print("rpm electricalFrequency=");
  Serial.print((float)electricalFrequencyRaw / 10.0f, 1);
  Serial.print("Hz deltaResolver=");
  Serial.print((float)deltaResolverRaw / 10.0f, 1);
  Serial.print("deg");
}

void printPm100CurrentInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t phaseACurrentRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t phaseBCurrentRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t phaseCCurrentRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t dcBusCurrentRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("phaseA=");
  Serial.print((float)phaseACurrentRaw / 10.0f, 1);
  Serial.print("A phaseB=");
  Serial.print((float)phaseBCurrentRaw / 10.0f, 1);
  Serial.print("A phaseC=");
  Serial.print((float)phaseCCurrentRaw / 10.0f, 1);
  Serial.print("A dcBus=");
  Serial.print((float)dcBusCurrentRaw / 10.0f, 1);
  Serial.print("A");
}

void printPm100VoltageInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t dcBusVoltageRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t outputVoltageRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t vabVdVoltageRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t vbcVqVoltageRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("dcBus=");
  Serial.print((float)dcBusVoltageRaw / 10.0f, 1);
  Serial.print("V output=");
  Serial.print((float)outputVoltageRaw / 10.0f, 1);
  Serial.print("V vabOrVd=");
  Serial.print((float)vabVdVoltageRaw / 10.0f, 1);
  Serial.print("V vbcOrVq=");
  Serial.print((float)vbcVqVoltageRaw / 10.0f, 1);
  Serial.print("V");
}

void printPm100InternalVoltagesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t ref1V5Raw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t ref2V5Raw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t ref5V0Raw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t glvRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("ref1V5=");
  Serial.print((float)ref1V5Raw / 100.0f, 2);
  Serial.print("V ref2V5=");
  Serial.print((float)ref2V5Raw / 100.0f, 2);
  Serial.print("V ref5V0=");
  Serial.print((float)ref5V0Raw / 100.0f, 2);
  Serial.print("V glv12V=");
  Serial.print((float)glvRaw / 100.0f, 2);
  Serial.print("V");
}

void printPm100FaultCodesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint32_t postFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  uint32_t runFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

  Serial.print("post=0x");
  Serial.print((unsigned long)postFaults, HEX);
  Serial.print(" run=0x");
  Serial.print((unsigned long)runFaults, HEX);
  Serial.print(" active=");

  bool printedFault = false;
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100PostFaultName(bit));
      printedFault = true;
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100RunFaultName(bit));
      printedFault = true;
    }
  }

  if (!printedFault) {
    Serial.print("none");
  }
}

void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg) {
  if (msg.len < 2) {
    Serial.print("short frame, expected at least 2 bytes");
    return;
  }

  int16_t positiveTorqueRaw = s16Le(msg.buf[0], msg.buf[1]);

  Serial.print("positiveTorqueCapability=");
  Serial.print((float)positiveTorqueRaw / 10.0f, 1);
  Serial.print("Nm");

  if (msg.len >= 4) {
    int16_t negativeTorqueRaw = s16Le(msg.buf[2], msg.buf[3]);
    Serial.print(" possibleNegativeTorqueCapability=");
    Serial.print((float)negativeTorqueRaw / 10.0f, 1);
    Serial.print("Nm");
  }

  if (msg.len > 4) {
    Serial.print(" reserved=");
    printRawPayloadFrom(msg, 4);
  }
}

void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  if (startIndex >= msg.len) {
    Serial.print("none");
    return;
  }

  for (uint8_t i = startIndex; i < msg.len; i++) {
    if (i > startIndex) {
      Serial.print(' ');
    }
    printCanHexByte(msg.buf[i]);
  }
}

void printBmsTemperatureFaultCode(uint8_t faultCode) {
  Serial.print(bmsTemperatureFaultCodeName(faultCode));
}

void printCellDataRequestName(uint8_t requestType) {
  switch (requestType) {
    case CELL_DATA_REQUEST_VOLTAGES: Serial.print("cell voltages"); break;
    case CELL_DATA_REQUEST_TEMPERATURES: Serial.print("cell temperatures"); break;
    default: Serial.print("unknown"); break;
  }
}

void printCellVoltageValue(uint16_t millivolts) {
  if (millivolts == 0xFFFF) {
    Serial.print("pad");
  } else {
    Serial.print(millivolts);
    Serial.print("mV");
  }
}

void printCanHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

/**
 * @file    can_trace_imd.cpp
 * @brief   Prints Bender iso175 IMD CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "can_trace_imd.h"

#include "can_bytes.h"
#include "imd.h"
#include "tssi.h"
#include "can_trace.h"

void printImdRequestIndexName(uint8_t requestIndex);
void printImdResponseErrorName(uint8_t errorCode);
void printImdInfoMessageSelectorName(uint8_t selector);
void printImdProfileName(uint8_t profile);
void printImdRIsoStatusName(uint8_t status);
void printImdDeviceActivityName(uint8_t activity);
void printImdLockStateName(uint8_t state);
void printImdEarthliftStateName(uint8_t state);
void printImdVoltageModeName(uint8_t mode);
void printImdBaudRateName(uint8_t baudRate);
void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printImdWordValue(uint16_t value);
void printImdResistanceValue(uint16_t kohms);
void printImdVoltageValue(uint16_t raw);
void printImdCapacityValue(uint16_t raw);
void printImdPercentValue(uint8_t value);
void printImdCounterValue(uint8_t value);

static constexpr int32_t IMD_VOLTAGE_OFFSET   = 32128;
static constexpr float IMD_VOLTAGE_SCALE      = 0.05f;

void printImdRequestTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request index");
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printImdRequestIndexName(msg.buf[0]);
  Serial.print(") data=");
  printRawPayloadFrom(msg, 1);
}

void printImdResponseTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected response index");
    return;
  }

  uint8_t index = msg.buf[0];

  if (index == 0xFF) {
    Serial.print("error");
    if (msg.len >= 2) {
      Serial.print(" code=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdResponseErrorName(msg.buf[1]);
      Serial.print(')');
    }
    if (msg.len >= 3) {
      Serial.print(" requestedIndex=0x");
      printCanHexByte(msg.buf[2]);
      Serial.print('(');
      printImdRequestIndexName(msg.buf[2]);
      Serial.print(')');
    }
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(index);
  Serial.print('(');
  printImdRequestIndexName(index);
  Serial.print(") ");

  if (msg.len < 2) {
    Serial.print("missing response data");
    return;
  }

  uint16_t word = (msg.len >= 3) ? u16Le(msg.buf[1], msg.buf[2]) : msg.buf[1];

  switch (index) {
    case 0x0A:
    case 0x0C:
    case 0x0E:
    case 0x1E:
    case 0x20:
    case 0x22:
      Serial.print("value=");
      printImdWordValue(word);
      break;

    case 0x10:
    case 0x12:
    case 0x14:
    case 0x16:
    case 0x18:
    case 0x1A:
    case 0x1C:
      Serial.print("ascii=");
      printImdAsciiPayloadFrom(msg, 1);
      break;

    case 0x2A:
    case 0x2E:
      Serial.print("percent=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x2C:
    case 0x36:
    case 0x54:
    case 0x5C:
      Serial.print("counter=");
      printImdCounterValue(msg.buf[1]);
      break;

    case 0x30:
      Serial.print("selfHoldingAlarm=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      switch (msg.buf[1]) {
        case 0xFC: Serial.print("automatic alarm reset"); break;
        case 0xFD: Serial.print("self-holding alarm"); break;
        case 0xFE: Serial.print("reserved"); break;
        case IMD_SNV_BYTE: Serial.print("SNV"); break;
        default: Serial.print("unknown"); break;
      }
      Serial.print(')');
      break;

    case 0x38:
    case 0x3A:
      Serial.print("profile=");
      Serial.print(msg.buf[1]);
      Serial.print('(');
      printImdProfileName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x3E:
      Serial.print("quality=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x40:
    case 0x42:
    case 0x46:
    case 0x4A:
    case 0x4C:
    case 0x4E:
    case 0x7E:
      Serial.print("resistance=");
      printImdResistanceValue(word);
      break;

    case 0x44:
      Serial.print("rIsoStatus=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdRIsoStatusName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x48:
    case 0x50:
      Serial.print("seconds=");
      printImdWordValue(word);
      break;

    case 0x52:
      Serial.print("capacity=");
      printImdCapacityValue(word);
      break;

    case 0x58:
      Serial.print("selfTestPeriod=");
      if (word == 0) {
        Serial.print("disabled");
      } else if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((uint32_t)word * 10u);
        Serial.print("s");
      }
      break;

    case 0x5E:
    case 0x60:
    case 0x62:
      Serial.print("voltage=");
      printImdVoltageValue(word);
      break;

    case 0x64:
      Serial.print("mode=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdVoltageModeName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x66:
    case 0x72:
      Serial.print("voltage=");
      printImdWordValue(word);
      Serial.print("V");
      break;

    case 0x68:
      Serial.print("deviceActivity=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdDeviceActivityName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6A:
    case 0x6B:
      Serial.print("lock=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdLockStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6C:
      Serial.print("warningsAndAlarms=0x");
      Serial.print(word, HEX);
      printTssiWarnBits(word);
      break;

    case 0x70:
    case 0x71:
      Serial.print("earthlift=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdEarthliftStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x74:
      Serial.print("maxVoltageDifference=");
      if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((float)word * 0.01f, 2);
        Serial.print("V");
      }
      break;

    case 0x76:
    case 0x77:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" canId=");
      if (msg.len >= 3) {
        Serial.print("0x");
        printCanHexByte(msg.buf[2]);
      } else {
        Serial.print("missing");
      }
      break;

    case 0x78:
    case 0x79:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" period=");
      if (msg.len >= 3) {
        if (msg.buf[2] == 0) {
          Serial.print("deactivated");
        } else if (msg.buf[2] == IMD_SNV_BYTE) {
          Serial.print("SNV");
        } else {
          Serial.print((uint16_t)msg.buf[2] * 100u);
          Serial.print("ms");
        }
      } else {
        Serial.print("missing");
      }
      break;

    case 0x7B:
      Serial.print("baudRate=");
      printImdBaudRateName(msg.buf[1]);
      break;

    default:
      Serial.print("payload=");
      printRawPayloadFrom(msg, 1);
      break;
  }
}

void printImdInfoGeneralTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint16_t rIsoCorrected = u16Le(msg.buf[0], msg.buf[1]);
  uint8_t rIsoStatus = msg.buf[2];
  uint8_t isolationCounter = msg.buf[3];
  uint8_t warningsLowByte = msg.buf[4];
  uint8_t reservedByte = msg.buf[5];
  uint8_t deviceActivity = msg.buf[6];

  Serial.print("rIsoCorrected=");
  printImdResistanceValue(rIsoCorrected);
  Serial.print(" rIsoStatus=0x");
  printCanHexByte(rIsoStatus);
  Serial.print('(');
  printImdRIsoStatusName(rIsoStatus);
  Serial.print(") isolationCounter=");
  printImdCounterValue(isolationCounter);
  Serial.print(" warningsLow=0x");
  printCanHexByte(warningsLowByte);
  printTssiWarnBits(warningsLowByte);
  Serial.print(" reserved5=0x");
  printCanHexByte(reservedByte);
  Serial.print(" deviceActivity=0x");
  printCanHexByte(deviceActivity);
  Serial.print('(');
  printImdDeviceActivityName(deviceActivity);
  Serial.print(") byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("rIsoNeg=");
  printImdResistanceValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" rIsoPos=");
  printImdResistanceValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" rIsoOriginal=");
  printImdResistanceValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" isolationCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" quality=");
  printImdPercentValue(msg.buf[7]);
}

void printImdInfoVoltageTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("hvSystem=");
  printImdVoltageValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" hvNegToEarth=");
  printImdVoltageValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" hvPosToEarth=");
  printImdVoltageValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" voltageCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoItSystemTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("capacity=");
  printImdCapacityValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" capacityCounter=");
  printImdCounterValue(msg.buf[2]);
  Serial.print(" unbalance=");
  printImdPercentValue(msg.buf[3]);
  Serial.print(" unbalanceCounter=");
  printImdCounterValue(msg.buf[4]);
  Serial.print(" byte5=0x");
  printCanHexByte(msg.buf[5]);
  Serial.print(" byte6=0x");
  printCanHexByte(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdRequestIndexName(uint8_t requestIndex) {
  switch (requestIndex) {
    case 0x0A: Serial.print("bootloader build number"); break;
    case 0x0C: Serial.print("bootloader D-number"); break;
    case 0x0E: Serial.print("bootloader version"); break;
    case 0x10: Serial.print("hardware AH history"); break;
    case 0x12: Serial.print("hardware AH number"); break;
    case 0x14: Serial.print("hardware AH number part B"); break;
    case 0x16: Serial.print("hardware item number"); break;
    case 0x18: Serial.print("hardware article number part B"); break;
    case 0x1A: Serial.print("serial number"); break;
    case 0x1C: Serial.print("serial number part B"); break;
    case 0x1E: Serial.print("software build number"); break;
    case 0x20: Serial.print("software D-number"); break;
    case 0x22: Serial.print("software version"); break;
    case 0x2A: Serial.print("unbalance measured value"); break;
    case 0x2C: Serial.print("unbalance measurement counter"); break;
    case 0x2E:
    case 0x2F: Serial.print("unbalance threshold"); break;
    case 0x30:
    case 0x31: Serial.print("self-holding alarm activation"); break;
    case 0x33: Serial.print("reset alarm"); break;
    case 0x36: Serial.print("isolation measurement counter"); break;
    case 0x38:
    case 0x39: Serial.print("isolation active profile"); break;
    case 0x3A:
    case 0x3B: Serial.print("isolation power-on profile"); break;
    case 0x3E: Serial.print("isolation quality"); break;
    case 0x40: Serial.print("R_iso_neg"); break;
    case 0x42: Serial.print("R_iso_pos"); break;
    case 0x44: Serial.print("R_iso_status"); break;
    case 0x46:
    case 0x47: Serial.print("isolation threshold error"); break;
    case 0x48:
    case 0x49: Serial.print("isolation measurement timeout threshold"); break;
    case 0x4A:
    case 0x4B: Serial.print("isolation threshold warning"); break;
    case 0x4C: Serial.print("R_iso_corrected"); break;
    case 0x4E: Serial.print("R_iso_original"); break;
    case 0x50: Serial.print("time since last isolation measurement"); break;
    case 0x52: Serial.print("capacity measured value"); break;
    case 0x54: Serial.print("capacity measurement counter"); break;
    case 0x57: Serial.print("trigger self test"); break;
    case 0x58:
    case 0x59: Serial.print("self test period"); break;
    case 0x5C: Serial.print("voltage measurement counter"); break;
    case 0x5E: Serial.print("HV system voltage"); break;
    case 0x60: Serial.print("HV negative to earth voltage"); break;
    case 0x62: Serial.print("HV positive to earth voltage"); break;
    case 0x64:
    case 0x65: Serial.print("voltage mode"); break;
    case 0x66:
    case 0x67: Serial.print("undervoltage threshold"); break;
    case 0x68: Serial.print("device activity"); break;
    case 0x6A:
    case 0x6B: Serial.print("parameter lock"); break;
    case IDX_WARN_ALARMS: Serial.print("warnings and alarms"); break;
    case 0x6F: Serial.print("factory reset"); break;
    case 0x70:
    case 0x71: Serial.print("earthlift status"); break;
    case 0x72:
    case 0x73: Serial.print("threshold first reference estimation"); break;
    case 0x74:
    case 0x75: Serial.print("pre-estimation max difference"); break;
    case 0x76:
    case 0x77: Serial.print("interface CAN-ID"); break;
    case 0x78:
    case 0x79: Serial.print("interface periodic cycle time"); break;
    case 0x7B: Serial.print("interface baudrate"); break;
    case 0x7E:
    case 0x7F: Serial.print("isolation initial value"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdResponseErrorName(uint8_t errorCode) {
  switch (errorCode) {
    case 0x23: Serial.print("unknown or invalid request"); break;
    case 0x24: Serial.print("set command failed, parameter locked"); break;
    case 0x25: Serial.print("data1 range overflow"); break;
    case 0x26: Serial.print("data2 range overflow"); break;
    case 0x27: Serial.print("CAN-ID already in use"); break;
    case 0x28: Serial.print("write failed"); break;
    case 0x29: Serial.print("read failed"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdInfoMessageSelectorName(uint8_t selector) {
  switch (selector) {
    case 0x00: Serial.print("Request"); break;
    case 0x01: Serial.print("Response"); break;
    case 0x02: Serial.print("IMD_Info_General"); break;
    case 0x03: Serial.print("IMD_Info_IsolationDetail"); break;
    case 0x04: Serial.print("IMD_Info_Voltage"); break;
    case 0x05: Serial.print("IMD_Info_IT-System"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdProfileName(uint8_t profile) {
  switch (profile) {
    case 1: Serial.print("standard fast startup"); break;
    case 2: Serial.print("standard"); break;
    case 3: Serial.print("high capacity fast startup"); break;
    case 4: Serial.print("high capacity"); break;
    case 5: Serial.print("disturbed"); break;
    case 6: Serial.print("service"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdRIsoStatusName(uint8_t status) {
  switch (status) {
    case 0xFC: Serial.print("startup estimated"); break;
    case 0xFD: Serial.print("startup first measurement"); break;
    case 0xFE: Serial.print("normal operation"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdDeviceActivityName(uint8_t activity) {
  switch (activity) {
    case 0: Serial.print("initialization"); break;
    case 1: Serial.print("normal operation"); break;
    case 2: Serial.print("self test"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdLockStateName(uint8_t state) {
  switch (state) {
    case 0xFC: Serial.print("parameter write enable"); break;
    case 0xFD: Serial.print("parameter write disable"); break;
    case 0xFE: Serial.print("reserved"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdEarthliftStateName(uint8_t state) {
  switch (state) {
    case 0:
    case 0xFC: Serial.print("earth connection closed"); break;
    case 1:
    case 0xFD: Serial.print("earth connection open"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdVoltageModeName(uint8_t mode) {
  switch (mode) {
    case 0xFD: Serial.print("AC"); break;
    case 0xFE: Serial.print("DC"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdBaudRateName(uint8_t baudRate) {
  switch (baudRate) {
    case 0x01: Serial.print("1 Mbit/s"); break;
    case 0x02: Serial.print("800 kbit/s"); break;
    case 0x03: Serial.print("666 kbit/s"); break;
    case 0x04: Serial.print("500 kbit/s"); break;
    case 0x05: Serial.print("250 kbit/s"); break;
    case 0x06: Serial.print("125 kbit/s"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  Serial.print('"');
  for (uint8_t i = startIndex; i < msg.len; i++) {
    uint8_t value = msg.buf[i];
    if (value == IMD_SNV_BYTE) {
      break;
    }
    if (value >= 32 && value <= 126) {
      Serial.write(value);
    } else {
      Serial.print('.');
    }
  }
  Serial.print('"');
}

void printImdWordValue(uint16_t value) {
  if (value == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
  }
}

void printImdResistanceValue(uint16_t kohms) {
  if (kohms == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(kohms);
    Serial.print("kOhm");
  }
}

void printImdVoltageValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    float volts = ((int32_t)raw - IMD_VOLTAGE_OFFSET) * IMD_VOLTAGE_SCALE;
    Serial.print(volts, 2);
    Serial.print("V");
  }
}

void printImdCapacityValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print((float)raw * 0.1f, 1);
    Serial.print("uF");
  }
}

void printImdPercentValue(uint8_t value) {
  if (value == IMD_SNV_BYTE) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
    Serial.print('%');
  }
}

void printImdCounterValue(uint8_t value) {
  Serial.print(value);
}

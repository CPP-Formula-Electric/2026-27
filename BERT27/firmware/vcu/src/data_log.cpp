/**
 * @file    data_log.cpp
 * @brief   Writes the measured, derived and cell log files to the SD card and lists or dumps them over serial.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "data_log.h"

#include "pins.h"
#include "can_bytes.h"
#include "vehicle_speed.h"
#include "fans.h"
#include "imu.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "regen.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "main.h"

static constexpr uint16_t BMS_CELL_COUNT = 96;

// ---------------- Data logging ----------------
// Three binary files are created for each session: MEASnnnn.BIN contains values
// received directly from sensors/CAN, DERInnnn.BIN contains calculated/control
// values, and CELLnnnn.BIN contains every requested cell voltage/temperature.
// Every telemetry field is an IEEE-754 32-bit float; headers use integer framing.
static constexpr uint32_t LOG_FAST_PERIOD_MS   = 10;
static constexpr uint32_t LOG_MEDIUM_PERIOD_MS = 100;
static constexpr uint32_t LOG_SLOW_PERIOD_MS   = 1000;
static constexpr uint32_t LOG_SD_FLUSH_PERIOD_MS = 1000;
static constexpr uint8_t LOG_RECORD_FAST   = 0x01;
static constexpr uint8_t LOG_RECORD_MEDIUM = 0x02;
static constexpr uint8_t LOG_RECORD_SLOW   = 0x03;
static constexpr char MEASURED_LOG_MAGIC[8] = {'V','C','U','M','E','A','S','1'};
static constexpr char DERIVED_LOG_MAGIC[8]  = {'V','C','U','D','E','R','I','1'};
static constexpr char CELL_LOG_MAGIC[8]     = {'V','C','U','C','E','L','L','1'};

struct __attribute__((packed)) LogFileHeader {
  char magic[8];
  uint32_t session;
  uint8_t fastSize;
  uint8_t mediumSize;
  uint8_t slowSize;
  uint8_t imuSize;          // 0 in logs from firmware without an IMU
};

struct __attribute__((packed)) MeasuredFastRecord {
  float type, timeSeconds;
  float apps1AdcCounts, apps2AdcCounts, bse1AdcCounts, bse2AdcCounts;
  float hvCurrentAdcCounts, motorRpm, busVoltageV, busCurrentA;
};

struct __attribute__((packed)) DerivedFastRecord {
  float type, timeSeconds;
  float brakePressurePct, commandedTorqueNm, regenTorqueNm;
  float speedKmh, cumulativeDistanceKm, statusFlags;
};

struct __attribute__((packed)) MeasuredMediumRecord {
  float type, timeSeconds;
  float glvVoltageV, inverterTempC, motorTempC, socPct, batteryTempC;
  float bmsFlags, bmsFaultCode, imdWarnAlarms, pm100PostFaults, pm100RunFaults;
};

struct __attribute__((packed)) DerivedMediumRecord {
  float type, timeSeconds;
  float apps1Pct, apps2Pct, pedalPct, desiredTorqueNm, powerKw, speedMph;
  float fanDutyPct, rtdState;
};

struct __attribute__((packed)) MeasuredSlowRecord {
  float type, timeSeconds;
  float minCellVoltageV, maxCellVoltageV, avgCellVoltageV;
  float minCellTempC, maxCellTempC, avgCellTempC;
  float imdRIsoKohm, imdRIsoStatus, validFlags;
};

struct __attribute__((packed)) CellLogFileHeader {
  char magic[8];
  uint32_t session;
  uint16_t recordSize;
  uint16_t cellCount;
};

// type: 1 = voltage (value in V), 2 = temperature (value in degC).
struct __attribute__((packed)) CellLogRecord {
  float type, timeSeconds, cellIndex, value;
};

uint32_t nextLogSessionNumber();
uint32_t maxLogSessionInDir(File dir);
uint16_t logStatusFlags();
void writeFastLogRecord(uint32_t nowMs);
void writeMediumLogRecord(uint32_t nowMs);
void writeSlowLogRecord(uint32_t nowMs);
void flushSdLogBuffer(File &file, bool measuredLog);
void appendCellLog(const void *data, size_t len);
void flushCellLogBuffer();
void flushAllSdLogBuffers();
void listLogDir(File dir);

int rawHvCurrentLast = 0;

// Each log is staged independently in RAM and written in 512-byte chunks.
File sdMeasuredLogFile;
File sdDerivedLogFile;
File sdCellLogFile;
bool sdCardStarted = false;
bool sdLogActive = false;
uint32_t logSessionNumber = 0;
uint8_t measuredLogBuffer[512];
uint8_t derivedLogBuffer[512];
uint8_t cellLogBuffer[512];
size_t measuredLogBufferLen = 0;
size_t derivedLogBufferLen = 0;
size_t cellLogBufferLen = 0;

// ---------------- Data logging ----------------
void setupDataLogging() {
  sdCardStarted = SD.begin(BUILTIN_SDCARD);
  if (!sdCardStarted) {
    Serial.println("[Log] No SD card found; data logging disabled");
    return;
  }

  logSessionNumber = nextLogSessionNumber();

  char measuredName[16];
  char derivedName[16];
  char cellName[16];
  snprintf(measuredName, sizeof(measuredName), "MEAS%04lu.BIN", (unsigned long)logSessionNumber);
  snprintf(derivedName, sizeof(derivedName), "DERI%04lu.BIN", (unsigned long)logSessionNumber);
  snprintf(cellName, sizeof(cellName), "CELL%04lu.BIN", (unsigned long)logSessionNumber);

  LogFileHeader measuredHeader;
  memcpy(measuredHeader.magic, MEASURED_LOG_MAGIC, sizeof(measuredHeader.magic));
  measuredHeader.session = logSessionNumber;
  measuredHeader.fastSize = sizeof(MeasuredFastRecord);
  measuredHeader.mediumSize = sizeof(MeasuredMediumRecord);
  measuredHeader.slowSize = sizeof(MeasuredSlowRecord);
  measuredHeader.imuSize = sizeof(MeasuredImuRecord);

  LogFileHeader derivedHeader;
  memcpy(derivedHeader.magic, DERIVED_LOG_MAGIC, sizeof(derivedHeader.magic));
  derivedHeader.session = logSessionNumber;
  derivedHeader.fastSize = sizeof(DerivedFastRecord);
  derivedHeader.mediumSize = sizeof(DerivedMediumRecord);
  derivedHeader.slowSize = 0;
  derivedHeader.imuSize = 0;

  CellLogFileHeader cellHeader;
  memcpy(cellHeader.magic, CELL_LOG_MAGIC, sizeof(cellHeader.magic));
  cellHeader.session = logSessionNumber;
  cellHeader.recordSize = sizeof(CellLogRecord);
  cellHeader.cellCount = BMS_CELL_COUNT;

  sdMeasuredLogFile = SD.open(measuredName, FILE_WRITE);
  sdDerivedLogFile = SD.open(derivedName, FILE_WRITE);
  sdCellLogFile = SD.open(cellName, FILE_WRITE);
  if (sdMeasuredLogFile && sdDerivedLogFile && sdCellLogFile) {
    sdMeasuredLogFile.write(&measuredHeader, sizeof(measuredHeader));
    sdDerivedLogFile.write(&derivedHeader, sizeof(derivedHeader));
    sdCellLogFile.write(&cellHeader, sizeof(cellHeader));
    sdMeasuredLogFile.flush();
    sdDerivedLogFile.flush();
    sdCellLogFile.flush();
    sdLogActive = true;
    Serial.print("[Log] Measured data: ");
    Serial.println(measuredName);
    Serial.print("[Log] Derived data:  ");
    Serial.println(derivedName);
    Serial.print("[Log] Per-cell data: ");
    Serial.println(cellName);
  } else {
    Serial.println("[Log] Failed to create all session logs; logging disabled");
    if (sdMeasuredLogFile) sdMeasuredLogFile.close();
    if (sdDerivedLogFile) sdDerivedLogFile.close();
    if (sdCellLogFile) sdCellLogFile.close();
  }
}

// One paired log per power cycle. Legacy LOG files are included so session
// numbers are never reused on a card containing logs from older firmware.
uint32_t nextLogSessionNumber() {
  uint32_t maxSession = 0;

  File root = SD.open("/");
  if (root) {
    maxSession = maxLogSessionInDir(root);
    root.close();
  }

  return maxSession + 1;
}

uint32_t maxLogSessionInDir(File dir) {
  uint32_t maxSession = 0;

  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    unsigned int session = 0;
    bool isSessionLog = sscanf(entry.name(), "MEAS%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "DERI%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "CELL%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "LOG%u.BIN", &session) == 1;
    if (isSessionLog && (uint32_t)session > maxSession) {
      maxSession = session;
    }
    entry.close();
  }

  return maxSession;
}

void serviceDataLogging() {
  if (!sdLogActive) {
    return;
  }

  static uint32_t lastFastMs = 0;
  static uint32_t lastMediumMs = 0;
  static uint32_t lastSlowMs = 0;
  static uint32_t lastSdFlushMs = 0;

  uint32_t now = millis();

  if (now - lastFastMs >= LOG_FAST_PERIOD_MS) {
    lastFastMs = now;
    writeFastLogRecord(now);
    writeImuLogRecord(now);
  }

  if (now - lastMediumMs >= LOG_MEDIUM_PERIOD_MS) {
    lastMediumMs = now;
    writeMediumLogRecord(now);
  }

  if (now - lastSlowMs >= LOG_SLOW_PERIOD_MS) {
    lastSlowMs = now;
    writeSlowLogRecord(now);
  }

  // Periodic flush bounds data loss at power-off to about one second.
  if (sdLogActive && now - lastSdFlushMs >= LOG_SD_FLUSH_PERIOD_MS) {
    lastSdFlushMs = now;
    flushAllSdLogBuffers();
    if (sdLogActive) {
      sdMeasuredLogFile.flush();
      sdDerivedLogFile.flush();
      sdCellLogFile.flush();
    }
  }
}

uint16_t logStatusFlags() {
  uint16_t flags = 0;
  if (braking)                              flags |= 1u << 0;
  if (hardBraking)                          flags |= 1u << 1;
  if (bspcActive)                           flags |= 1u << 2;
  if (appsPlausible)                        flags |= 1u << 3;
  if (readyToDriveLatched)                  flags |= 1u << 4;
  if (driveMode == 'D')                     flags |= 1u << 5;
  if (bmsFault)                             flags |= 1u << 6;
  if (imdFault)                             flags |= 1u << 7;
  if (pm100Fault)                           flags |= 1u << 8;
  if (bmsTimedOut)                          flags |= 1u << 9;
  if (imdCanTimedOut)                       flags |= 1u << 10;
  if (!contactorsOpen())                    flags |= 1u << 11;
  if (digitalRead(BSPD_STATUS_PIN) == HIGH) flags |= 1u << 12;
  if (digitalRead(IMD_STATUS_PIN) == HIGH)  flags |= 1u << 13;
  if (digitalRead(BMS_STATUS_PIN) == HIGH)  flags |= 1u << 14;
  return flags;
}

void writeFastLogRecord(uint32_t nowMs) {
  rawHvCurrentLast = analogRead(HV_CURRENT_SENSE_PIN);

  MeasuredFastRecord measured;
  measured.type = (float)LOG_RECORD_FAST;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.apps1AdcCounts = (float)rawApps1Last;
  measured.apps2AdcCounts = (float)rawApps2Last;
  measured.bse1AdcCounts = (float)rawBse1Last;
  measured.bse2AdcCounts = (float)rawBse2Last;
  measured.hvCurrentAdcCounts = (float)rawHvCurrentLast;
  measured.motorRpm = (float)motorRpm;
  measured.busVoltageV = inverterDcBusVoltage;
  measured.busCurrentA = inverterDcBusCurrent;
  appendSdLog(sdMeasuredLogFile, true, &measured, sizeof(measured));

  DerivedFastRecord derived;
  derived.type = (float)LOG_RECORD_FAST;
  derived.timeSeconds = (float)nowMs / 1000.0f;
  derived.brakePressurePct = brakePressurePct;
  derived.commandedTorqueNm = commandedTorqueNm;
  derived.regenTorqueNm = regenTorqueNm;
  derived.speedKmh = speedKmh;
  derived.cumulativeDistanceKm = cumulativeDistanceKm;
  derived.statusFlags = (float)logStatusFlags();
  appendSdLog(sdDerivedLogFile, false, &derived, sizeof(derived));
}

void writeMediumLogRecord(uint32_t nowMs) {
  MeasuredMediumRecord measured;
  measured.type = (float)LOG_RECORD_MEDIUM;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.glvVoltageV = haveGlvVoltage ? glvVoltage : NAN;
  measured.inverterTempC = haveInverterTemp ? (float)inverterTempTenthsC / 10.0f : NAN;
  measured.motorTempC = havePm100MotorTemp ? (float)pm100MotorTempTenthsC / 10.0f : NAN;
  measured.socPct = haveBmsStatus ? (float)stateOfChargePct : NAN;
  measured.batteryTempC = haveCellTempStats ? (float)batteryTempC : NAN;
  measured.bmsFlags = (float)bmsFlags;
  measured.bmsFaultCode = haveBms2026FaultCode ? (float)bms2026FaultCode : NAN;
  measured.imdWarnAlarms = (float)imdWarnAlarms;
  measured.pm100PostFaults = (float)pm100PostFaults;
  measured.pm100RunFaults = (float)pm100RunFaults;
  appendSdLog(sdMeasuredLogFile, true, &measured, sizeof(measured));

  DerivedMediumRecord derived;
  derived.type = (float)LOG_RECORD_MEDIUM;
  derived.timeSeconds = (float)nowMs / 1000.0f;
  derived.apps1Pct = apps1Pct;
  derived.apps2Pct = apps2Pct;
  derived.pedalPct = pedalPct;
  derived.desiredTorqueNm = desiredTorqueNm;
  derived.powerKw = powerKw;
  derived.speedMph = speedMph;
  derived.fanDutyPct = (float)constrain(fan1DutyPercent, 0, 100);
  derived.rtdState = (float)rtdState;
  appendSdLog(sdDerivedLogFile, false, &derived, sizeof(derived));
}

void writeSlowLogRecord(uint32_t nowMs) {
  MeasuredSlowRecord measured;
  measured.type = (float)LOG_RECORD_SLOW;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.minCellVoltageV = haveCellVoltageStats ? (float)minCellVoltageMv / 1000.0f : NAN;
  measured.maxCellVoltageV = haveCellVoltageStats ? (float)maxCellVoltageMv / 1000.0f : NAN;
  measured.avgCellVoltageV = haveCellVoltageStats ? (float)avgCellVoltageMv / 1000.0f : NAN;
  measured.minCellTempC = haveCellTempStats ? (float)minCellTempC : NAN;
  measured.maxCellTempC = haveCellTempStats ? (float)maxCellTempC : NAN;
  measured.avgCellTempC = haveCellTempStats ? (float)avgCellTempC : NAN;
  measured.imdRIsoKohm = (imdRIsoKohm != IMD_SNV_WORD) ? (float)imdRIsoKohm : NAN;
  measured.imdRIsoStatus = (float)imdRIsoStatus;
  measured.validFlags = (float)((haveCellVoltageStats ? 0x01 : 0) |
                                (haveCellTempStats ? 0x02 : 0) |
                                (haveGlvVoltage ? 0x04 : 0));
  appendSdLog(sdMeasuredLogFile, true, &measured, sizeof(measured));
}

void appendSdLog(File &file, bool measuredLog, const void *data, size_t len) {
  if (!sdLogActive) {
    return;
  }

  uint8_t *buffer = measuredLog ? measuredLogBuffer : derivedLogBuffer;
  size_t &bufferLen = measuredLog ? measuredLogBufferLen : derivedLogBufferLen;
  if (bufferLen + len > 512) {
    flushSdLogBuffer(file, measuredLog);
    if (!sdLogActive) {
      return;
    }
  }

  memcpy(buffer + bufferLen, data, len);
  bufferLen += len;
}

void flushSdLogBuffer(File &file, bool measuredLog) {
  uint8_t *buffer = measuredLog ? measuredLogBuffer : derivedLogBuffer;
  size_t &bufferLen = measuredLog ? measuredLogBufferLen : derivedLogBufferLen;
  if (!sdLogActive || bufferLen == 0) {
    bufferLen = 0;
    return;
  }

  size_t written = file.write(buffer, bufferLen);
  if (written != bufferLen) {
    Serial.println("[Log] SD write failed; SD logging stopped");
    sdMeasuredLogFile.close();
    sdDerivedLogFile.close();
    sdCellLogFile.close();
    sdLogActive = false;
    screenDirty = true;
  }
  bufferLen = 0;
}

void appendCellLog(const void *data, size_t len) {
  if (!sdLogActive) return;
  if (cellLogBufferLen + len > sizeof(cellLogBuffer)) {
    flushCellLogBuffer();
    if (!sdLogActive) return;
  }
  memcpy(cellLogBuffer + cellLogBufferLen, data, len);
  cellLogBufferLen += len;
}

void flushCellLogBuffer() {
  if (!sdLogActive || cellLogBufferLen == 0) {
    cellLogBufferLen = 0;
    return;
  }
  if (sdCellLogFile.write(cellLogBuffer, cellLogBufferLen) != cellLogBufferLen) {
    Serial.println("[Log] SD write failed; SD logging stopped");
    sdMeasuredLogFile.close();
    sdDerivedLogFile.close();
    sdCellLogFile.close();
    sdLogActive = false;
    screenDirty = true;
  }
  cellLogBufferLen = 0;
}

void flushAllSdLogBuffers() {
  flushSdLogBuffer(sdMeasuredLogFile, true);
  flushSdLogBuffer(sdDerivedLogFile, false);
  flushCellLogBuffer();
}

void listLogFiles() {
  if (!sdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  Serial.println("[Log] SD card:");
  File root = SD.open("/");
  if (root) {
    listLogDir(root);
    root.close();
  }
}

void listLogDir(File dir) {
  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    if (!entry.isDirectory()) {
      Serial.print("  ");
      Serial.print(entry.name());
      Serial.print("  ");
      Serial.print((unsigned long)entry.size());
      Serial.println(" bytes");
    }
    entry.close();
  }
}

// Hex-dumps all files in a session over serial so logs can be exported
// without pulling the SD card.
void dumpLogFile(uint32_t session) {
  if (!sdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  // Make sure everything buffered for the current session is on the card.
  flushAllSdLogBuffers();
  if (sdLogActive) {
    sdMeasuredLogFile.flush();
    sdDerivedLogFile.flush();
    sdCellLogFile.flush();
  }

  const char *prefixes[3] = {"MEAS", "DERI", "CELL"};
  for (uint8_t fileIndex = 0; fileIndex < 3; fileIndex++) {
    char name[16];
    snprintf(name, sizeof(name), "%s%04lu.BIN", prefixes[fileIndex],
             (unsigned long)session);
    File f = SD.open(name, FILE_READ);
    if (!f) {
      Serial.print("[Log] ");
      Serial.print(name);
      Serial.println(" not found");
      continue;
    }

    Serial.print("[Log] DUMP BEGIN ");
    Serial.print(name);
    Serial.print(" ");
    Serial.println((unsigned long)f.size());

    uint8_t buf[32];
    int n;
    while ((n = f.read(buf, sizeof(buf))) > 0) {
      for (int i = 0; i < n; i++) printCanHexByte(buf[i]);
      Serial.println();
    }
    f.close();
    Serial.println("[Log] DUMP END");
  }
}

void writeCellDataResponseLog(const CAN_message_t &msg) {
  if (!sdLogActive || msg.flags.extended || msg.len < 2) return;

  const uint16_t frameIndex = msg.buf[0];
  const float timeSeconds = (float)millis() / 1000.0f;

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    for (uint8_t i = 0; i < 3 && (1 + i * 2 + 1) < msg.len; i++) {
      const uint16_t zeroBasedCell = frameIndex * 3u + i;
      if (zeroBasedCell >= BMS_CELL_COUNT) break;
      const uint16_t millivolts = u16Be(msg.buf[1 + i * 2], msg.buf[2 + i * 2]);
      CellLogRecord record = {
        (float)CELL_DATA_REQUEST_VOLTAGES, timeSeconds, (float)(zeroBasedCell + 1u),
        millivolts == 0xFFFF ? NAN : (float)millivolts / 1000.0f
      };
      appendCellLog(&record, sizeof(record));
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    for (uint8_t i = 0; i < 7 && (1 + i) < msg.len; i++) {
      const uint16_t zeroBasedCell = frameIndex * 7u + i;
      if (zeroBasedCell >= BMS_CELL_COUNT) break;
      const uint8_t rawTemperature = msg.buf[1 + i];
      CellLogRecord record = {
        (float)CELL_DATA_REQUEST_TEMPERATURES, timeSeconds, (float)(zeroBasedCell + 1u),
        rawTemperature == 0xFF ? NAN : (float)((int)rawTemperature - 40)
      };
      appendCellLog(&record, sizeof(record));
    }
  }
}

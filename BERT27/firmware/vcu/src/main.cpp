#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pins.h"
#include "safety_limits.h"
#include "calibration.h"
#include "can_bytes.h"
#include "vehicle_speed.h"
#include "buzzer.h"
#include "fans.h"
#include "imu.h"
#include "energy_meter.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "vehicle_can.h"
#include "faults.h"
#include "power_derate.h"
#include "regen.h"
#include "pedals.h"
#include "inverter_command.h"
#include "main.h"

/*
  BERT26 VCU integrated sketch

  Functions:
    - Drives the ST7796/XPT2046 touchscreen dashboard.
    - Plays a buzzer chirp when BMS status becomes active.
    - Plays a different buzzer pattern when the RTD button on GPIO 23 is pressed.
    - Reads the dual analog accelerator pedal sensor.
    - Sends Cascadia PM100DX torque commands at 100 Hz over CAN.
    - Logs float telemetry to separate measured (MEASnnnn.BIN), derived
      (DERInnnn.BIN), and per-cell (CELLnnnn.BIN) files on the built-in SD card.
*/

ST7796_t3 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

static constexpr bool CAN_TRANSLATION_SERIAL_ENABLED = true;

static constexpr uint32_t SCREEN_PERIOD_MS = 100;
static constexpr uint32_t DASHBOARD_FULL_REDRAW_MS = 120000;  // periodic anti-glitch repaint
static constexpr uint32_t FAULT_DISPLAY_CYCLE_MS = 1500;
static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;
static constexpr uint8_t DASHBOARD_MAX_FAULT_TEXTS = 48;
static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

static constexpr uint8_t CELL_DATA_REQUEST_UNKNOWN      = 0x00;
static constexpr uint16_t BMS_CELL_COUNT = 96;

static constexpr float EM_TEMP_SCALE_C = 0.5f;        // degC = raw * 0.5

// Bender iso175 IMD/TSSI frames from VCU_TSSI_CODE_5_31_26.ino.
static constexpr uint32_t IMD_INFO_ISOLATION_DETAIL_ID = 0x38;
static constexpr uint32_t IMD_INFO_VOLTAGE_ID = 0x39;
static constexpr uint32_t IMD_INFO_IT_SYSTEM_ID = 0x3A;
static constexpr uint32_t IMD_FALLBACK_REQUEST_ID = 0x0FF;
static constexpr uint16_t IMD_SNV_WORD        = 0xFFFF;
static constexpr int32_t IMD_VOLTAGE_OFFSET   = 32128;
static constexpr float IMD_VOLTAGE_SCALE      = 0.05f;

static constexpr uint32_t IMD_GET_REQ_PERIOD_MS    = 500;
 

enum RtdState {
  RTD_STATE_FAULT_BLOCKED,
  RTD_STATE_READY_FOR_BUTTON,
  RTD_STATE_READY_TO_DRIVE
};

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

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

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

uint8_t lastCellDataRequestType = CELL_DATA_REQUEST_UNKNOWN;

uint32_t tssiBlinkTimerMs = 0;
bool tssiRedOn = false;
// Once the TSSI has blinked red for any fault it stays latched in the red-blink
// state until the board is power-cycled (this flag clears on reset).
bool tssiFaultLatched = false;
uint32_t lastImdGetReqMs = 0;

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool readyToDriveLatched = false;

bool readyButtonLast = false;

// Fault banner: a red strip over the drive-button row naming the most recent
// fault, shown for FAULT_BANNER_DURATION_MS after each fault event.
char faultBannerText[48] = "";
uint32_t faultBannerUntilMs = 0;
bool faultBannerVisible = false;
bool faultBannerDirty = false;

int backlightBrightness = 255;
bool screenDirty = true;
bool dashboardNeedsFullRedraw = true;

// EV.10.2: ready-to-drive sound, 1-3 seconds. Solid continuous tone for the
// first 1.25 s (satisfies the >=1 s continuous requirement), then a short
// playful flourish. Step on+off times sum to 2000 ms total.
static const ToneStep RTD_PATTERN[] = {
  {2093, 1250,  0},  // C7 - continuous main sound (1.25 s)
  {2637,  120, 40},  // E7
  {3136,  120, 40},  // G7
  {2637,  120, 40},  // E7
  {3136,  270,  0}   // G7 - final flourish
};

typedef void (*CanTranslationPrinter)(const CAN_message_t &msg);

struct CanFrameTranslation {
  uint32_t id;
  const char *name;
  CanTranslationPrinter print;
};

struct DashboardFaultList {
  const char *texts[DASHBOARD_MAX_FAULT_TEXTS];
  uint8_t count;
};

// ---------------- Function declarations ----------------
void setupDisplay();
void resetDisplay();
void handleSerialConsoleInput();
void handleSerialConsoleCommand(const char *cmd);

void setupDataLogging();
void serviceDataLogging();
void writeFastLogRecord(uint32_t nowMs);
void writeMediumLogRecord(uint32_t nowMs);
void writeSlowLogRecord(uint32_t nowMs);
uint16_t logStatusFlags();
void appendSdLog(File &file, bool measuredLog, const void *data, size_t len);
void flushSdLogBuffer(File &file, bool measuredLog);
void appendCellLog(const void *data, size_t len);
void flushCellLogBuffer();
void flushAllSdLogBuffers();
uint32_t nextLogSessionNumber();
uint32_t maxLogSessionInDir(File dir);
void listLogFiles();
void listLogDir(File dir);
void dumpLogFile(uint32_t session);
void handleTouch();
char checkIfTouchedRaw(int x, int y);
void drawDashboard();
void drawDriveButtons(bool force);
void drawDashboardLabel(int x, int y, const char *label);
void drawTileFrame(int x, int y, const char *label);
void drawDerateBanner(bool force);
uint16_t motorTempColor(float tempC, bool valid);
uint16_t batteryTempColor(int tempC, bool valid);
uint16_t cellSpreadColor(int spreadMv, bool valid);
void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force);
void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid);
void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid);
void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid);
const char *dashboardFaultText();
void collectDashboardFaults(DashboardFaultList &faults);
void appendDashboardFault(DashboardFaultList &faults, const char *text);
void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits);
void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults);
const char *firstTssiFaultName(uint16_t bits);
const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults);
const char *bmsTemperatureFaultCodeName(uint8_t faultCode);
const char *pm100PostFaultName(uint8_t bit);
const char *pm100RunFaultName(uint8_t bit);
void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor);
void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor);
void drawShutdownStatusIndicators(bool force);
void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force);

void writeCellDataResponseLog(const CAN_message_t &msg);
void updateCanTranslationState(const char *direction, const CAN_message_t &msg);
void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg);
void printBmsStatus2026Translation(const CAN_message_t &msg);
void printBmsTemperature2026Translation(const CAN_message_t &msg);
void printCellDataRequest2026Translation(const CAN_message_t &msg);
void printCellDataResponse2026Translation(const CAN_message_t &msg);
void printImdRequestTranslation(const CAN_message_t &msg);
void printImdResponseTranslation(const CAN_message_t &msg);
void printImdInfoGeneralTranslation(const CAN_message_t &msg);
void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg);
void printImdInfoVoltageTranslation(const CAN_message_t &msg);
void printImdInfoItSystemTranslation(const CAN_message_t &msg);
void printPm100CommandTranslation(const CAN_message_t &msg);
void printPm100Temperatures1Translation(const CAN_message_t &msg);
void printPm100Temperatures3Translation(const CAN_message_t &msg);
void printPm100MotorPositionTranslation(const CAN_message_t &msg);
void printPm100CurrentInfoTranslation(const CAN_message_t &msg);
void printPm100VoltageInfoTranslation(const CAN_message_t &msg);
void printPm100InternalVoltagesTranslation(const CAN_message_t &msg);
void printPm100FaultCodesTranslation(const CAN_message_t &msg);
void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg);
void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printBmsTemperatureFaultCode(uint8_t faultCode);
void printCellDataRequestName(uint8_t requestType);
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
void printCellVoltageValue(uint16_t millivolts);
void printCanHexByte(uint8_t value);
bool tssiFaultFromWarnBits(uint16_t warnBits);
void setupTssi();
void serviceTssi();
void updateTssiLed(bool faultActive, uint32_t now);
void setTssiGreenOn();
void setTssiRedOn();
void setTssiRedOff();
void setTssiAllOff();
void printTssiWarnBits(uint16_t bits);

void serviceReadyToDrive();
bool readReadyButtonPressed();
bool rtdInputsReady();
bool readyForRtdButton();
void refreshRtdStateFromInputs();
void logRtdBlockedReason();
void handleRtdButtonPressed();
void setRtdState(RtdState newState);
void dropReadyToDrive(const char *reason);
const char *rtdDropCauseText();
void showFaultBanner(const char *text);
void serviceFaultBanner();
void drawFaultBanner(bool force);
const char *rtdStateName(RtdState state);
void setDriveMode(char newDriveMode);

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

// ---------------- Setup ----------------
void setup() {
  startupFaultDelayStartMs = millis();

  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  setupFans();

  pinMode(READY_BUTTON_PIN, INPUT);
  readyButtonLast = readReadyButtonPressed();
  refreshRtdStateFromInputs();

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(APPS_1_PIN, INPUT);
  pinMode(APPS_2_PIN, INPUT);
  pinMode(BSE_1_PIN, INPUT);
  pinMode(BSE_2_PIN, INPUT);
  pinMode(HV_CURRENT_SENSE_PIN, INPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);

  // Pulldowns so a disconnected status line reads LOW and shows red.
  pinMode(BSPD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(IMD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(BMS_STATUS_PIN, INPUT_PULLDOWN);

  setupTssi();
  setupVehicleCan();
  setupPm100CommandTimer();

  while (!Serial && millis() < 1500) {}

  Serial.println("BERT26 VCU starting");
  Serial.print("[Startup] BMS/IMD fault detection delayed for ");
  Serial.print(STARTUP_BMS_IMD_FAULT_DELAY_MS);
  Serial.println(" ms");

  setupImu();

  setupDataLogging();

  setupDisplay();

  drawDashboard();
}

// ---------------- Main loop ----------------
void loop() {
  handleSerialConsoleInput();
  updateFanDutyFromTemps();
  serviceFans();
  receiveCanFrames();
  updateBrakePressureSensors();
  updatePedal();
  digitalWrite(BRAKE_LIGHT_PIN, braking ? HIGH : LOW);
  logAppsTelemetry();
  serviceReadyToDrive();
  serviceBuzzer();
  serviceTssi();
  serviceFaultBanner();
  handleTouch();

  uint32_t now = millis();
  if (bmsImdFaultDetectionArmed() && haveBmsStatus && (now - lastBmsRxMs > BMS_TIMEOUT_MS)) {
    if (!bmsTimedOut) {
      Serial.println("[BMS] Status timeout"); //,disabling torque
      showFaultBanner("BMS Timeout");
      screenDirty = true;
    }
    bmsTimedOut = true;
    bmsActive = false;
    bmsFault = true;
    dropReadyToDrive("BMS Timeout");
  }

  serviceEnergyMeterTimeout(now);

  if (haveInverterVoltageInfo && haveInverterCurrentInfo) {
    powerKw = (inverterDcBusVoltage * inverterDcBusCurrent) / 1000.0f;
  } else {
    powerKw = 0.0f;
  }
  speedMph = fabsf((float)motorRpm) * MPH_PER_RPM;
  speedKmh = speedMph * KMH_PER_MPH;
  updateCumulativeDistance(now);
  servicePowerDerate(now);
  updatePm100CommandState();
  servicePm100TxLogging();
  servicePm100BroadcastEnable();
  servicePm100FaultClear();
  serviceCellDataRequests(now);
  serviceDataLogging();

  // Rewrite the entire screen every 2 minutes to wipe any accumulated SPI
  // glitches that the per-cell incremental redraw would otherwise leave behind.
  static uint32_t lastFullRedrawMs = 0;
  if (now - lastFullRedrawMs >= DASHBOARD_FULL_REDRAW_MS) {
    lastFullRedrawMs = now;
    dashboardNeedsFullRedraw = true;
    screenDirty = true;
  }

  static uint32_t lastDrawMs = 0;
  if (screenDirty || now - lastDrawMs >= SCREEN_PERIOD_MS) {
    lastDrawMs = now;
    screenDirty = false;
    drawDashboard();
  }
}

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

// ---------------- Display / touch ----------------
void setupDisplay() {
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCK(TFT_SCK);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);

  pinMode(TFT_LED, OUTPUT);
  analogWriteFrequency(TFT_LED, 20000);
  analogWrite(TFT_LED, backlightBrightness);

  resetDisplay();

  ts.begin();
  ts.setRotation(1);

  tft.init(320, 480);
  tft.setRotation(135);
  tft.fillScreen(ST7735_BLACK);
  dashboardNeedsFullRedraw = true;
  screenDirty = true;
}

void resetDisplay() {
  digitalWrite(TFT_RST, HIGH);
  delay(20);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
}

// Line-based serial console: 0-100 = manual fan duty, 'a' = auto fans,
// 'l' = list log files, 'd<n>' = hex-dump log session n for export.
void handleSerialConsoleInput() {
  static char input[12];
  static uint8_t index = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (index > 0) {
        input[index] = '\0';
        index = 0;
        handleSerialConsoleCommand(input);
      }
    } else if (index < sizeof(input) - 1) {
      input[index++] = c;
    } else {
      index = 0;
    }
  }
}

void handleSerialConsoleCommand(const char *cmd) {
  if (strcmp(cmd, "a") == 0 || strcmp(cmd, "A") == 0) {
    fanManualOverride = false;
    Serial.println("[Fans] Automatic temperature control resumed");
    return;
  }

  if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    listLogFiles();
    return;
  }

  if ((cmd[0] == 'd' || cmd[0] == 'D') && cmd[1] >= '0' && cmd[1] <= '9') {
    dumpLogFile((uint32_t)atoi(cmd + 1));
    return;
  }

  bool allDigits = cmd[0] != '\0';
  for (const char *p = cmd; *p; p++) {
    if (*p < '0' || *p > '9') {
      allDigits = false;
      break;
    }
  }

  if (allDigits) {
    int value = atoi(cmd);
    // Manual override: enter 0-100 to force both fan PWM duties.
    if (value >= 0 && value <= 100) {
      fanManualOverride = true;
      fan1DutyPercent = value;
      fan2DutyPercent = value;
      Serial.print("[Fans] Manual override duty set to ");
      Serial.print(value);
      Serial.println("% (send 'a' for auto)");
      return;
    }
  }

  Serial.println("[Console] Commands: 0-100 fan duty, 'a' auto fans, 'l' list logs, 'd<n>' dump log n");
}

void handleTouch() {
  // RTD owns driveMode: D while ready to drive, N in every other RTD state.
  return;

  if (!ts.touched()) {
    return;
  }

  TS_Point p = ts.getPoint();
  char newMode = checkIfTouchedRaw(p.x, p.y);
  if (newMode && newMode != driveMode) {
    setDriveMode(newMode);

    Serial.print("[Touch] Drive mode ");
    Serial.println(driveMode);
  }
}

char checkIfTouchedRaw(int x, int y) {
  const int yMin = 2650;
  const int yMax = 3500;

  if (x > 950 && x < 1550 && y > yMin && y < yMax) return 'D';
  if (x > 1900 && x < 2500 && y > yMin && y < yMax) return 'N';
  if (x > 2900 && x < 3500 && y > yMin && y < yMax) return 'R';

  return 0;
}

void drawDashboard() {
  // Primary-tile value caches (Zone B).
  static char lastSpeed[12] = "";   static uint16_t lastSpeedColor = ST7735_WHITE;
  static char lastDistance[20] = ""; static uint16_t lastDistanceColor = ST7735_WHITE;
  static char lastBatT[12] = "";    static uint16_t lastBatTColor = ST7735_WHITE;
  static char lastMtrT[12] = "";    static uint16_t lastMtrTColor = ST7735_WHITE;
  static char lastSpread[12] = "";  static uint16_t lastSpreadColor = ST7735_WHITE;
  static char lastPackV[12] = "";   static uint16_t lastPackVColor = ST7735_WHITE;
  static char lastMode[12] = "";    static uint16_t lastModeColor = ST7735_WHITE;

  // Diagnostics-strip caches (Zone C).
  static char lastPedal[24] = "";   static uint16_t lastPedalColor = ST7735_WHITE;
  static char lastTorque[16] = "";  static uint16_t lastTorqueColor = ST7735_WHITE;
  static char lastPower[16] = "";   static uint16_t lastPowerColor = ST7735_WHITE;
  static char lastSoc[24] = "";     static uint16_t lastSocColor = ST7735_WHITE;
  static char lastRpm[16] = "";     static uint16_t lastRpmColor = ST7735_WHITE;
  static char lastBms[24] = "";     static uint16_t lastBmsColor = ST7735_WHITE;
  static char lastInv[24] = "";     static uint16_t lastInvColor = ST7735_WHITE;
  static char lastCtr[16] = "";     static uint16_t lastCtrColor = ST7735_WHITE;
  static char lastVmn[16] = "";     static uint16_t lastVmnColor = ST7735_WHITE;
  static char lastVmx[16] = "";     static uint16_t lastVmxColor = ST7735_WHITE;
  static char lastTmn[16] = "";     static uint16_t lastTmnColor = ST7735_WHITE;
  static char lastTmx[16] = "";     static uint16_t lastTmxColor = ST7735_WHITE;
  static char lastGlv[16] = "";     static uint16_t lastGlvColor = ST7735_WHITE;
  static char lastPack[16] = "";    static uint16_t lastPackColor = ST7735_WHITE;
  static char lastInvt[16] = "";    static uint16_t lastInvtColor = ST7735_WHITE;
  static char lastFan[16] = "";     static uint16_t lastFanColor = ST7735_WHITE;
  static char lastEmV[24] = "";     static uint16_t lastEmVColor = ST7735_WHITE;
  static char lastEmT[32] = "";     static uint16_t lastEmTColor = ST7735_WHITE;
  static char lastEmF[24] = "";     static uint16_t lastEmFColor = ST7735_WHITE;
  static char lastFault[24] = "";   static uint16_t lastFaultColor = ST7735_WHITE;
  static char lastSd[24] = "";      static uint16_t lastSdColor = ST7735_WHITE;

  bool force = dashboardNeedsFullRedraw;

  if (force) {
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(2);
    tft.setCursor(6, 6);
    tft.print("BERT26");

    // Six primary tiles: 3 columns x 2 rows.
    drawTileFrame(2, 32, "SPEED");
    drawTileFrame(162, 32, "BAT T");
    drawTileFrame(322, 32, "MTR T");
    drawTileFrame(2, 136, "SPREAD");
    drawTileFrame(162, 136, "PACK V");
    drawTileFrame(322, 136, "MODE");

    dashboardNeedsFullRedraw = false;
  }

  char value[32];

  // ---- Header: SD/session status and power-derate banner ----
  uint16_t sdColor;
  if (!sdCardStarted) {
    snprintf(value, sizeof(value), "SD:NO LOG:----");
    sdColor = ST7735_RED;
  } else {
    snprintf(value, sizeof(value), "SD:%s LOG:%04lu", sdLogActive ? "OK" : "ERR",
             (unsigned long)logSessionNumber);
    sdColor = sdLogActive ? ST7735_GREEN : ST7735_YELLOW;
  }
  drawCachedDashboardText(100, 5, 196, 18, 1, value, sdColor,
                          lastSd, sizeof(lastSd), lastSdColor, force);
  drawDerateBanner(force);

  // ---- Zone B: large primary tiles (value cell = tileX+6, tileY+34) ----
  snprintf(value, sizeof(value), "%.0f", speedMph);
  drawCachedDashboardText(8, 66, 144, 48, 5, value, ST7735_WHITE,
                          lastSpeed, sizeof(lastSpeed), lastSpeedColor, force);

  snprintf(value, sizeof(value), "RUN:%.2fkm", cumulativeDistanceKm);
  drawCachedDashboardText(8, 118, 144, 10, 1, value, ST7735_WHITE,
                          lastDistance, sizeof(lastDistance), lastDistanceColor, force);

  bool battTempValid = haveCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C;
  if (battTempValid) {
    snprintf(value, sizeof(value), "%dC", (int)maxCellTempC);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  drawCachedDashboardText(168, 66, 144, 48, 4, value,
                          batteryTempColor((int)maxCellTempC, battTempValid),
                          lastBatT, sizeof(lastBatT), lastBatTColor, force);

  {
    float motorC = (float)pm100MotorTempTenthsC / 10.0f;
    if (havePm100MotorTemp) {
      snprintf(value, sizeof(value), "%.0fC", motorC);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    drawCachedDashboardText(328, 66, 144, 48, 4, value,
                            motorTempColor(motorC, havePm100MotorTemp),
                            lastMtrT, sizeof(lastMtrT), lastMtrTColor, force);
  }

  {
    int spread = (int)maxCellVoltageMv - (int)minCellVoltageMv;
    if (haveCellVoltageStats) {
      snprintf(value, sizeof(value), "%dmV", spread);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    drawCachedDashboardText(8, 170, 144, 48, 4, value,
                            cellSpreadColor(spread, haveCellVoltageStats),
                            lastSpread, sizeof(lastSpread), lastSpreadColor, force);
  }

  if (haveInverterVoltageInfo) {
    snprintf(value, sizeof(value), "%.0fV", inverterDcBusVoltage);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  drawCachedDashboardText(168, 170, 144, 48, 4, value,
                          haveInverterVoltageInfo ? ST7735_WHITE : ST7735_YELLOW,
                          lastPackV, sizeof(lastPackV), lastPackVColor, force);

  {
    bool inDrive = (driveMode == 'D');
    const char *modeText = inDrive ? "DRIVE" : "NEUTRAL";
    drawCachedDashboardText(328, 172, 144, 28, 3, modeText,
                            inDrive ? ST7735_GREEN : ST7735_WHITE,
                            lastMode, sizeof(lastMode), lastModeColor, force);
  }

  // ---- Zone C: diagnostics strip (size-1 grid) ----
  const int dc0 = 4, dc1 = 124, dc2 = 244, dc3 = 364;
  const int dw = 116, dh = 9;
  const int r0 = 242, r1 = 254, r2 = 266, r3 = 278, r4 = 290, r5 = 303;

  bool faultDetectionArmed = bmsImdFaultDetectionArmed();
  bool bms2026DisplayFault = haveBms2026FaultCode && bms2026FaultCode != 0;

  // r0: pedal/brake inputs, signed torque request, power, SOC/regen setting.
  // These pedal-derived diagnostics remain live while TS/RTD is off.
  snprintf(value, sizeof(value), "PED:%.0f%% BRK:%.0f%%", pedalPct, brakePressurePct);
  drawCachedDashboardText(dc0, r0, dw, dh, 1, value, appsPlausible ? ST7735_GREEN : ST7735_RED,
                          lastPedal, sizeof(lastPedal), lastPedalColor, force);

  snprintf(value, sizeof(value), "CMD:%+.1fNm", displayedCommandTorqueNm);
  drawCachedDashboardText(dc1, r0, dw, dh, 1, value, ST7735_WHITE,
                          lastTorque, sizeof(lastTorque), lastTorqueColor, force);

  bool haveInverterPower = haveInverterVoltageInfo && haveInverterCurrentInfo;
  if (haveInverterPower) {
    snprintf(value, sizeof(value), "PWR:%dkW", (int)roundf(powerKw));
  } else {
    snprintf(value, sizeof(value), "PWR:--");
  }
  drawCachedDashboardText(dc2, r0, dw, dh, 1, value, haveInverterPower ? ST7735_WHITE : ST7735_YELLOW,
                          lastPower, sizeof(lastPower), lastPowerColor, force);

  snprintf(value, sizeof(value), "SOC:%u%% REGEN:%s", (unsigned int)stateOfChargePct,
           REGEN_ENABLED ? "ON" : "OFF");
  drawCachedDashboardText(dc3, r0, dw, dh, 1, value,
                          REGEN_ENABLED ? ST7735_GREEN : ST7735_YELLOW,
                          lastSoc, sizeof(lastSoc), lastSocColor, force);

  // r1: RPM, BMS status, inverter status, contactor
  snprintf(value, sizeof(value), "RPM:%.0f", (float)motorRpm);
  drawCachedDashboardText(dc0, r1, dw, dh, 1, value, ST7735_WHITE,
                          lastRpm, sizeof(lastRpm), lastRpmColor, force);

  const char *bmsText = "WAIT";
  uint16_t bmsColor = ST7735_YELLOW;
  if (!faultDetectionArmed) {
    bmsText = "STARTUP"; bmsColor = ST7735_YELLOW;
  } else if (bmsTimedOut) {
    bmsText = "TIMEOUT"; bmsColor = ST7735_RED;
  } else if (bmsFault || bms2026DisplayFault) {
    bmsText = "FAULT"; bmsColor = ST7735_RED;
  } else if (bmsActive) {
    bmsText = "ACTIVE"; bmsColor = ST7735_GREEN;
  }
  snprintf(value, sizeof(value), "BMS:%s", bmsText);
  drawCachedDashboardText(dc1, r1, dw, dh, 1, value, bmsColor,
                          lastBms, sizeof(lastBms), lastBmsColor, force);

  {
    const char *invText;
    uint16_t invColor;
    if (!havePm100FaultStatus) {
      invText = "--"; invColor = ST7735_YELLOW;
    } else if (pm100Fault) {
      invText = firstPm100FaultName(pm100PostFaults, pm100RunFaults); invColor = ST7735_RED;
    } else {
      invText = "OK"; invColor = ST7735_GREEN;
    }
    snprintf(value, sizeof(value), "INV:%s", invText);
    drawCachedDashboardText(dc2, r1, dw, dh, 1, value, invColor,
                            lastInv, sizeof(lastInv), lastInvColor, force);
  }

  {
    bool ctrClosed = haveBmsStatus && (bmsFlags & 0x02) != 0;
    const char *ctrText = !haveBmsStatus ? "--" : (ctrClosed ? "CLOSED" : "OPEN");
    uint16_t ctrColor = !haveBmsStatus ? ST7735_YELLOW : (ctrClosed ? ST7735_GREEN : ST7735_RED);
    snprintf(value, sizeof(value), "CTR:%s", ctrText);
    drawCachedDashboardText(dc3, r1, dw, dh, 1, value, ctrColor,
                            lastCtr, sizeof(lastCtr), lastCtrColor, force);
  }

  // r2: cell voltage min/max, cell temp min/max
  if (haveCellVoltageStats) snprintf(value, sizeof(value), "Vmn:%.3f", minCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmn:--");
  drawCachedDashboardText(dc0, r2, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastVmn, sizeof(lastVmn), lastVmnColor, force);

  if (haveCellVoltageStats) snprintf(value, sizeof(value), "Vmx:%.3f", maxCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmx:--");
  drawCachedDashboardText(dc1, r2, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastVmx, sizeof(lastVmx), lastVmxColor, force);

  if (haveCellTempStats && minCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmn:%dC", (int)minCellTempC);
  else snprintf(value, sizeof(value), "Tmn:--");
  drawCachedDashboardText(dc2, r2, dw, dh, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastTmn, sizeof(lastTmn), lastTmnColor, force);

  if (haveCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmx:%dC", (int)maxCellTempC);
  else snprintf(value, sizeof(value), "Tmx:--");
  drawCachedDashboardText(dc3, r2, dw, dh, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastTmx, sizeof(lastTmx), lastTmxColor, force);

  // r3: GLV, BMS pack voltage, inverter temp, fan
  {
    uint16_t glvColor;
    if (!haveGlvVoltage) {
      snprintf(value, sizeof(value), "GLV:--"); glvColor = ST7735_YELLOW;
    } else {
      snprintf(value, sizeof(value), "GLV:%.1fV", glvVoltage);
      glvColor = (glvVoltage < GLV_LOW_VOLTAGE_WARN_V) ? ST7735_RED : ST7735_WHITE;
    }
    drawCachedDashboardText(dc0, r3, dw, dh, 1, value, glvColor,
                            lastGlv, sizeof(lastGlv), lastGlvColor, force);
  }

  if (haveCellVoltageStats) snprintf(value, sizeof(value), "PACK:%.0fV", packVoltage);
  else snprintf(value, sizeof(value), "PACK:--");
  drawCachedDashboardText(dc1, r3, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastPack, sizeof(lastPack), lastPackColor, force);

  if (haveInverterTemp) snprintf(value, sizeof(value), "INVT:%.1fC", inverterTempTenthsC / 10.0f);
  else snprintf(value, sizeof(value), "INVT:--");
  drawCachedDashboardText(dc2, r3, dw, dh, 1, value, haveInverterTemp ? ST7735_WHITE : ST7735_YELLOW,
                          lastInvt, sizeof(lastInvt), lastInvtColor, force);

  {
    int fan1Duty = constrain(fan1DutyPercent, 0, 100);
    int fan2Duty = constrain(fan2DutyPercent, 0, 100);
    snprintf(value, sizeof(value), "FAN:%d/%d", fan1Duty, fan2Duty);
    drawCachedDashboardText(dc3, r3, dw, dh, 1, value, ST7735_WHITE,
                            lastFan, sizeof(lastFan), lastFanColor, force);
  }

  // r4: energy meter V / T / status (shutdown chips occupy the far right)
  {
    bool emComms = (haveEmMeasurement || haveEmStatus || haveEmTemps) && !emTimedOut;
    uint16_t liveColor = emComms ? ST7735_WHITE : ST7735_YELLOW;

    if (haveEmMeasurement) snprintf(value, sizeof(value), "EMV:%.1fV", emVoltageV);
    else snprintf(value, sizeof(value), "EMV:--");
    drawCachedDashboardText(dc0, r4, dw, dh, 1, value, liveColor,
                            lastEmV, sizeof(lastEmV), lastEmVColor, force);

    if (haveEmTempSummary) {
      snprintf(value, sizeof(value), "EMT:%u %d-%dC", emNumSensors,
               (int)((float)emMinTempRaw * EM_TEMP_SCALE_C),
               (int)((float)emMaxTempRaw * EM_TEMP_SCALE_C));
    } else {
      snprintf(value, sizeof(value), "EMT:--");
    }
    drawCachedDashboardText(dc1, r4, dw, dh, 1, value, liveColor,
                            lastEmT, sizeof(lastEmT), lastEmTColor, force);

    uint16_t emColor;
    const char *emText = emStatusSummaryText(emColor);
    snprintf(value, sizeof(value), "EM:%s", emText);
    drawCachedDashboardText(dc2, r4, 112, dh, 1, value, emColor,
                            lastEmF, sizeof(lastEmF), lastEmFColor, force);
  }

  // r5: wide fault text (left of the shutdown chips)
  {
    const char *faultText = dashboardFaultText();
    bool faultActive = pm100Fault ||
                       (faultDetectionArmed &&
                        (anyFaultActive() || bmsTimedOut || imdCanTimedOut || bms2026DisplayFault));
    uint16_t faultColor = faultActive ? ST7735_RED : (faultDetectionArmed ? ST7735_GREEN : ST7735_YELLOW);
    snprintf(value, sizeof(value), "FAULT:%s", faultText);
    drawCachedDashboardText(dc0, r5, 350, dh, 1, value, faultColor,
                            lastFault, sizeof(lastFault), lastFaultColor, force);
  }

  drawShutdownStatusIndicators(force);

  if (faultBannerVisible) {
    drawFaultBanner(force);
  }
}

void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.3fV", (float)millivolts / 1000.0f);
}

void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid) {
  if (!valid || tempC == CELL_TEMP_UNKNOWN_C) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%dC", (int)tempC);
}

void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.1fC", (float)tempTenthsC / 10.0f);
}

const char *dashboardFaultText() {
  DashboardFaultList faults;
  collectDashboardFaults(faults);

  if (faults.count == 0) {
    if (!bmsImdFaultDetectionArmed()) {
      return "Startup Wait";
    }
    return "No Fault";
  }

  if (faults.count == 1) {
    return faults.texts[0];
  }

  uint8_t faultIndex = (millis() / FAULT_DISPLAY_CYCLE_MS) % faults.count;
  return faults.texts[faultIndex];
}

void collectDashboardFaults(DashboardFaultList &faults) {
  faults.count = 0;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && bmsTimedOut) {
    appendDashboardFault(faults, "BMS Timeout");
  }

  if (faultDetectionArmed && imdCanTimedOut) {
    appendDashboardFault(faults, "IMD Timeout");
  }

  if (faultDetectionArmed) {
    if (haveBms2026FaultCode && bms2026FaultCode != 0) {
      appendDashboardFault(faults, bmsTemperatureFaultCodeName(bms2026FaultCode));
    } else if (bmsFault && haveBmsStatus && !haveBms2026FaultCode) {
      appendDashboardFault(faults, "BMS Fault");
    }
  }

  if (faultDetectionArmed && imdFault && (imdWarnAlarms != 0 || !imdCanTimedOut)) {
    uint8_t beforeCount = faults.count;
    appendTssiFaultNames(faults, imdWarnAlarms);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "IMD Fault");
    }
  }

  if (havePm100FaultStatus && pm100Fault) {
    uint8_t beforeCount = faults.count;
    appendPm100FaultNames(faults, pm100PostFaults, pm100RunFaults);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "Inv Fault");
    }
  }

  if (!appsPlausible) {
    appendDashboardFault(faults, appsFaultText);
  }

  if (bspcActive) {
    appendDashboardFault(faults, "BSPC Zero Trq");
  }
}

void appendDashboardFault(DashboardFaultList &faults, const char *text) {
  if (faults.count >= DASHBOARD_MAX_FAULT_TEXTS || text == nullptr || text[0] == '\0') {
    return;
  }

  faults.texts[faults.count++] = text;
}

void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits) {
  if (bits & (1u << 0)) appendDashboardFault(faults, "DeviceError");
  if (bits & (1u << 1)) appendDashboardFault(faults, "HV+ConnFail");
  if (bits & (1u << 2)) appendDashboardFault(faults, "HV-ConnFail");
  if (bits & (1u << 3)) appendDashboardFault(faults, "EarthConnFail");
  if (bits & (1u << 4)) appendDashboardFault(faults, "IsoAlarm");
  if (bits & (1u << 7)) appendDashboardFault(faults, "Unbalance");
  if (bits & (1u << 8)) appendDashboardFault(faults, "Undervolt");
  if (bits & (1u << 9)) appendDashboardFault(faults, "UnsafeStart");
  if (bits & (1u << 10)) appendDashboardFault(faults, "EarthliftOpen");
  if (bits & (1u << 5)) appendDashboardFault(faults, "IsoWarning");
  if (bits & (1u << 6)) appendDashboardFault(faults, "IsoOutdated");
}

void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100PostFaultName(bit));
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100RunFaultName(bit));
    }
  }
}

const char *firstTssiFaultName(uint16_t bits) {
  if (bits & (1u << 0)) return "DeviceError";
  if (bits & (1u << 1)) return "HV+ConnFail";
  if (bits & (1u << 2)) return "HV-ConnFail";
  if (bits & (1u << 3)) return "EarthConnFail";
  if (bits & (1u << 4)) return "IsoAlarm";
  if (bits & (1u << 7)) return "Unbalance";
  if (bits & (1u << 8)) return "Undervolt";
  if (bits & (1u << 9)) return "UnsafeStart";
  if (bits & (1u << 10)) return "EarthliftOpen";
  if (bits & (1u << 5)) return "IsoWarning";
  if (bits & (1u << 6)) return "IsoOutdated";
  return "Fault";
}

const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) return pm100PostFaultName(bit);
  }
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) return pm100RunFaultName(bit);
  }
  return "No Fault";
}

const char *bmsTemperatureFaultCodeName(uint8_t faultCode) {
  switch (faultCode) {
    case 0:  return "No Fault";
    case 1:  return "Cell UV";
    case 2:  return "Cell OV";
    case 3:  return "Pack UV";
    case 4:  return "Pack OV";
    case 5:  return "Over Temp";
    case 6:  return "Cell Spread";
    case 7:  return "Prchg Timeout";
    case 8:  return "Prchg Fast";
    case 9:  return "Pack V Sense";
    case 10: return "Chgr Comms";
    case 11: return "BQ Init Fail";
    case 12: return "BQ Comms Fail";
    default: return "BMS Fault";
  }
}

const char *pm100PostFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "POST GateDesat";
    case 1: return "POST OverCur";
    case 2: return "POST AccelShrt";
    case 3: return "POST AccelOpen";
    case 4: return "POST CurSensLo";
    case 5: return "POST CurSensHi";
    case 6: return "POST ModTempLo";
    case 7: return "POST ModTempHi";
    case 8: return "POST CtrlTmpLo";
    case 9: return "POST CtrlTmpHi";
    case 10: return "POST GateTmpLo";
    case 11: return "POST GateTmpHi";
    case 12: return "POST 5V Low";
    case 13: return "POST 5V High";
    case 14: return "POST 12V Low";
    case 15: return "POST 12V High";
    case 16: return "POST 2.5V Lo";
    case 17: return "POST 2.5V Hi";
    case 18: return "POST 1.5V Lo";
    case 19: return "POST 1.5V Hi";
    case 20: return "POST DCBusHi";
    case 21: return "POST DCBusLo";
    case 22: return "POST PrechTmo";
    case 23: return "POST PrechVolt";
    case 24: return "POST EEPROMSum";
    case 25: return "POST EEPROMRg";
    case 26: return "POST EEPROMUpd";
    case 27: return "POST HWBusOV";
    case 28: return "POST GateInit";
    case 29: return "POST Reserved";
    case 30: return "POST BrakeShrt";
    case 31: return "POST BrakeOpen";
    default: return "POST Fault";
  }
}

const char *pm100RunFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "Mtr Overspeed";
    case 1: return "Inv Overcur";
    case 2: return "Inv Overvolt";
    case 3: return "Inv Overtemp";
    case 4: return "Accel Short";
    case 5: return "Accel Open";
    case 6: return "Dir Cmd Fault";
    case 7: return "Inv Resp Tmo";
    case 8: return "Gate/Desat";
    case 9: return "HW Overcur";
    case 10: return "Undervolt";
    case 11: return "CAN Cmd Lost";
    case 12: return "Mtr Overtemp";
    case 13: return "Run Reserved45";
    case 14: return "Run Reserved46";
    case 15: return "Run Reserved47";
    case 16: return "Brake Short";
    case 17: return "Brake Open";
    case 18: return "ModA Overtemp";
    case 19: return "ModB Overtemp";
    case 20: return "ModC Overtemp";
    case 21: return "PCB Overtemp";
    case 22: return "Gate1 Overtemp";
    case 23: return "Gate2 Overtemp";
    case 24: return "Gate3 Overtemp";
    case 25: return "Cur Sensor";
    case 26: return "Gate Overvolt";
    case 27: return "HWBusOV Gen3";
    case 28: return "HWBusOV Gen5";
    case 29: return "Run Reserved61";
    case 30: return "Resolver Disc";
    case 31: return "Run Reserved63";
    default: return "Inv Fault";
  }
}

void drawDashboardLabel(int x, int y, const char *label) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
}

// One 156x100 primary tile: rounded border + size-2 title in the top-left.
// Drawn only on a full redraw; the value cell inside is repainted incrementally.
void drawTileFrame(int x, int y, const char *label) {
  tft.drawRoundRect(x, y, 156, 100, 6, ST7735_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x + 6, y + 6);
  tft.print(label);
}

// Red "DERATE <n>kW" banner in the header while the power limiter is active;
// cleared to black when not. Custom drawn (filled box, not the text cache).
// Redraws on the active/inactive transition and whenever the shown (rounded)
// kW changes, so the linear voltage derate is reflected live.
void drawDerateBanner(bool force) {
  static bool lastDerate = false;
  static int  lastShownKw = -1;
  static bool initialized = false;
  int shownKw = (int)roundf(powerDerateKw);
  if (!force && initialized && powerDerateActive == lastDerate &&
      (!powerDerateActive || shownKw == lastShownKw)) {
    return;
  }
  initialized = true;
  lastDerate = powerDerateActive;
  lastShownKw = shownKw;

  const int x = 300, y = 2, w = 176, h = 24;
  if (powerDerateActive) {
    char banner[20];
    snprintf(banner, sizeof(banner), "DERATE %dkW", shownKw);
    tft.fillRect(x, y, w, h, ST7735_RED);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(x + 8, y + 5);
    tft.print(banner);
    tft.setTextColor(ST7735_WHITE);
  } else {
    tft.fillRect(x, y, w, h, ST7735_BLACK);
  }
}

// Color bands (user-specified). Invalid/no-data is shown yellow.
uint16_t motorTempColor(float tempC, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (tempC >= 95.0f) return ST7735_RED;
  if (tempC >= 65.0f) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t batteryTempColor(int tempC, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (tempC > 45) return ST7735_RED;
  if (tempC >= 30) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t cellSpreadColor(int spreadMv, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (spreadMv > 200) return ST7735_RED;
  if (spreadMv >= 100) return ST7735_YELLOW;
  return ST7735_GREEN;
}

void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force) {
  if (!force && strcmp(value, lastValue) == 0 && valueColor == lastColor) {
    return;
  }

  tft.fillRect(x, y, w, h, ST7735_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(valueColor);
  tft.setCursor(x, y);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);

  strncpy(lastValue, value, lastValueSize - 1);
  lastValue[lastValueSize - 1] = '\0';
  lastColor = valueColor;
}

void drawDriveButtons(bool force) {
  static char lastDriveMode = 0;

  if (!force && driveMode == lastDriveMode) {
    return;
  }

  int y = (int)(tft.height() * 0.72f);
  int buttonW = 72;
  int buttonH = 54;
  int startX = 42;
  int gap = 24;

  // Clear only the button band; the cell-stats column at x>=335 shares this row.
  tft.fillRect(0, y - 8, 320, buttonH + 16, ST7735_BLACK);

  const char modes[2] = {'D', 'N'};
  for (int i = 0; i < 2; i++) {
    int x = startX + i * (buttonW + gap);
    bool selected = (driveMode == modes[i]);
    uint16_t color = selected ? ST7735_RED : ST7735_WHITE;

    tft.drawRoundRect(x, y, buttonW, buttonH, 6, color);
    tft.setTextSize(4);
    tft.setTextColor(color);
    tft.setCursor(x + 24, y + 12);
    tft.print(modes[i]);
  }

  tft.setTextColor(ST7735_WHITE);
  lastDriveMode = driveMode;
}

void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
  tft.setTextColor(valueColor);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);
}

void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor) {
  char text[16];
  snprintf(text, sizeof(text), "%d%s", value, units);
  drawStatusText(x, y, label, text, valueColor);
}

// Shutdown-circuit status lights: GPIO HIGH = green, LOW = red.
void drawShutdownStatusIndicators(bool force) {
  static int lastBspd = -1;
  static int lastImd = -1;
  static int lastBms = -1;

  drawShutdownStatusIndicator(362, 291, "BSPD", digitalRead(BSPD_STATUS_PIN) == HIGH,
                              lastBspd, force);
  drawShutdownStatusIndicator(402, 291, "IMD", digitalRead(IMD_STATUS_PIN) == HIGH,
                              lastImd, force);
  drawShutdownStatusIndicator(442, 291, "BMS", digitalRead(BMS_STATUS_PIN) == HIGH,
                              lastBms, force);
}

void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force) {
  if (!force && (int)high == lastState) {
    return;
  }
  lastState = (int)high;

  const int w = 36;
  const int h = 16;
  uint16_t color = high ? ST7735_GREEN : ST7735_RED;
  tft.fillRoundRect(x, y, w, h, 4, color);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  int textW = (int)strlen(label) * 6;
  tft.setCursor(x + (w - textW) / 2, y + 4);
  tft.print(label);
  tft.setTextColor(ST7735_WHITE);
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

bool tssiFaultFromWarnBits(uint16_t warnBits) {
  return (warnBits & IMD_FAULT_BITS_MASK) != 0;
}

void setupTssi() {
  pinMode(TSSI_LED_RED, OUTPUT);
  pinMode(TSSI_LED_GREEN, OUTPUT);
  setTssiAllOff();

  imdFault = true;
  imdCanTimedOut = true;
  lastImdRxMs = millis();
  tssiBlinkTimerMs = millis();
  lastImdGetReqMs = 0;
}

void serviceTssi() {
  uint32_t now = millis();
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && now - lastImdRxMs > IMD_HEARTBEAT_TIMEOUT_MS) {
    if (!imdCanTimedOut) {
      Serial.println("[TSSI] IMD heartbeat lost, forcing FAULT");
      showFaultBanner("IMD Timeout");
    }
    imdCanTimedOut = true;
    imdFault = true;
  }

  if ((anyFaultActive() || contactorsOpen()) && readyToDriveLatched) {
    dropReadyToDrive(rtdDropCauseText());
  }

  if (now - lastImdGetReqMs >= IMD_GET_REQ_PERIOD_MS) {
    lastImdGetReqMs = now;
    requestImdWarnings();
  }

  // Latch the red blink the first time a fault is seen; once latched the TSSI
  // stays red until power cycle, even if the underlying fault clears.
  if (faultDetectionArmed && anyFaultActive()) {
    tssiFaultLatched = true;
  }

  if (tssiFaultLatched) {
    updateTssiLed(true, now);
  } else if (!faultDetectionArmed) {
    setTssiGreenOn();
  } else {
    updateTssiLed(anyFaultActive(), now);
  }
}

void updateTssiLed(bool faultActive, uint32_t now) {
  if (faultActive) {
    uint32_t elapsed = now - tssiBlinkTimerMs;
    if (elapsed >= TSSI_BLINK_PERIOD_MS) {
      tssiBlinkTimerMs += TSSI_BLINK_PERIOD_MS;
      elapsed = now - tssiBlinkTimerMs;
    }

    if (elapsed < TSSI_BLINK_ON_MS) {
      if (!tssiRedOn) {
        setTssiRedOn();
      }
    } else if (tssiRedOn) {
      setTssiRedOff();
    }
  } else {
    setTssiGreenOn();
  }
}

void setTssiGreenOn() {
  digitalWrite(TSSI_LED_RED, LOW);
  digitalWrite(TSSI_LED_GREEN, HIGH);
  tssiRedOn = false;
}

void setTssiRedOn() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, HIGH);
  tssiRedOn = true;
}

void setTssiRedOff() {
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void setTssiAllOff() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void printTssiWarnBits(uint16_t bits) {
  if (bits & (1u << 0)) Serial.print(" [DeviceError]");
  if (bits & (1u << 1)) Serial.print(" [HV+ConnFail]");
  if (bits & (1u << 2)) Serial.print(" [HV-ConnFail]");
  if (bits & (1u << 3)) Serial.print(" [EarthConnFail]");
  if (bits & (1u << 4)) Serial.print(" [IsoAlarm]");
  if (bits & (1u << 5)) Serial.print(" [IsoWarning]");
  if (bits & (1u << 6)) Serial.print(" [IsoOutdated]");
  if (bits & (1u << 7)) Serial.print(" [UnbalanceAlarm]");
  if (bits & (1u << 8)) Serial.print(" [Undervoltage]");
  if (bits & (1u << 9)) Serial.print(" [UnsafeToStart]");
  if (bits & (1u << 10)) Serial.print(" [EarthliftOpen]");
}

// ---------------- Ready to drive state machine ----------------
void serviceReadyToDrive() {
  refreshRtdStateFromInputs();

  bool pressed = readReadyButtonPressed();

  if (pressed && !readyButtonLast) {
    handleRtdButtonPressed();
    screenDirty = true;
  }

  readyButtonLast = pressed;
}

bool readReadyButtonPressed() {
  return digitalRead(READY_BUTTON_PIN) == HIGH;
}

bool rtdInputsReady() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see updatePedal/updatePm100CommandState).
  return !anyFaultActive() && !contactorsOpen();
}

bool readyForRtdButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}

void refreshRtdStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdInputsReady()) {
      dropReadyToDrive(rtdDropCauseText());
    }
    return;
  }

  bool inputsReady = rtdInputsReady();
  if (!inputsReady) {
    logRtdBlockedReason();
  }

  setRtdState(inputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}

// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void logRtdBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t now = millis();
  if (now - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = now;

  if (!bmsImdFaultDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (imdFault)            Serial.print(" imdFault");
  if (bmsFault)            Serial.print(" bmsFault");
  if (contactorsOpen())    Serial.print(" contactorsOpen");
  if (imdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (bmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  printCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}

void handleRtdButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (REQUIRE_BRAKE_FOR_RTD && !braking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      setRtdState(RTD_STATE_READY_TO_DRIVE);
      playTonePattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      dropReadyToDrive("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      setRtdState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(anyFaultActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}

void setRtdState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      setDriveMode(newDriveMode);
      screenDirty = true;
    }
    return;
  }

  rtdState = newState;
  readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  setDriveMode(newDriveMode);
  screenDirty = true;
}

void dropReadyToDrive(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || readyToDriveLatched;

  setRtdState(rtdInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      showFaultBanner(reason);
    }
  }
}

// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdDropCauseText() {
  if (bmsTimedOut && haveBmsStatus) return "BMS Timeout";
  if (imdCanTimedOut) return "IMD Timeout";
  if (bmsFault) {
    return (haveBms2026FaultCode && bms2026FaultCode != 0)
               ? bmsTemperatureFaultCodeName(bms2026FaultCode)
               : "BMS Fault";
  }
  if (imdFault) return firstTssiFaultName(imdWarnAlarms);
  if (contactorsOpen()) return "Contactors Open";
  return "Fault";
}

void showFaultBanner(const char *text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  if (!faultBannerVisible || strcmp(text, faultBannerText) != 0) {
    strncpy(faultBannerText, text, sizeof(faultBannerText) - 1);
    faultBannerText[sizeof(faultBannerText) - 1] = '\0';
    faultBannerDirty = true;
  }
  faultBannerVisible = true;
  faultBannerUntilMs = millis() + FAULT_BANNER_DURATION_MS;
  screenDirty = true;
}

void serviceFaultBanner() {
  if (!faultBannerVisible) {
    return;
  }

  if ((int32_t)(millis() - faultBannerUntilMs) >= 0) {
    faultBannerVisible = false;
    faultBannerText[0] = '\0';
    dashboardNeedsFullRedraw = true;
    screenDirty = true;
  }
}

// Draws the fault banner over the drive-button row; the row is repainted via
// a full redraw when the banner expires.
void drawFaultBanner(bool force) {
  if (!force && !faultBannerDirty) {
    return;
  }
  faultBannerDirty = false;

  int y = (int)(tft.height() * 0.72f) - 8;
  int h = 54 + 16;

  tft.fillRect(0, y, tft.width(), h, ST7735_RED);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(12, y + 8);
  tft.print("FAULT");
  tft.setTextSize(3);
  tft.setCursor(12, y + 32);
  tft.print(faultBannerText);
  tft.setTextColor(ST7735_WHITE);
}

const char *rtdStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}

void setDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  screenDirty = true;
}


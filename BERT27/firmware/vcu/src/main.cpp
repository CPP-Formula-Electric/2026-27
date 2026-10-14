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
#include "rtd.h"
#include "tssi.h"
#include "data_log.h"
#include "can_trace.h"
#include "can_trace_imd.h"
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

static constexpr uint32_t SCREEN_PERIOD_MS = 100;
static constexpr uint32_t DASHBOARD_FULL_REDRAW_MS = 120000;  // periodic anti-glitch repaint
static constexpr uint32_t FAULT_DISPLAY_CYCLE_MS = 1500;
static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;
static constexpr uint8_t DASHBOARD_MAX_FAULT_TEXTS = 48;

static constexpr float EM_TEMP_SCALE_C = 0.5f;        // degC = raw * 0.5

// Fault banner: a red strip over the drive-button row naming the most recent
// fault, shown for FAULT_BANNER_DURATION_MS after each fault event.
char faultBannerText[48] = "";
uint32_t faultBannerUntilMs = 0;
bool faultBannerVisible = false;
bool faultBannerDirty = false;

int backlightBrightness = 255;
bool screenDirty = true;
bool dashboardNeedsFullRedraw = true;

struct DashboardFaultList {
  const char *texts[DASHBOARD_MAX_FAULT_TEXTS];
  uint8_t count;
};

// ---------------- Function declarations ----------------
void setupDisplay();
void resetDisplay();
void handleSerialConsoleInput();
void handleSerialConsoleCommand(const char *cmd);

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

void showFaultBanner(const char *text);
void serviceFaultBanner();
void drawFaultBanner(bool force);

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


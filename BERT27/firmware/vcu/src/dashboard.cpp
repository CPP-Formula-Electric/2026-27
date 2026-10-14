/**
 * @file    dashboard.cpp
 * @brief   Draws the ST7796 touchscreen dashboard and handles touch input.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "dashboard.h"

#include <SPI.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>

#include "pins.h"
#include "calibration.h"
#include "vehicle_speed.h"
#include "fans.h"
#include "energy_meter.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "faults.h"
#include "power_derate.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "data_log.h"
#include "fault_text.h"

void resetDisplay();
char checkIfTouchedRaw(int x, int y);
void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid);
void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid);
void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid);
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
void drawDriveButtons(bool force);
void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor);
void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor);
void drawShutdownStatusIndicators(bool force);
void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force);
void drawFaultBanner(bool force);

ST7796_t3 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;

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

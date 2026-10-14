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
#include "vehicle_speed.h"
#include "buzzer.h"
#include "fans.h"
#include "imu.h"
#include "energy_meter.h"
#include "bms_link.h"
#include "inverter.h"
#include "vehicle_can.h"
#include "faults.h"
#include "power_derate.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "tssi.h"
#include "data_log.h"
#include "serial_console.h"
#include "dashboard.h"

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


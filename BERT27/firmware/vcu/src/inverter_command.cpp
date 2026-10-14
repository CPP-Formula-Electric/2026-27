/**
 * @file    inverter_command.cpp
 * @brief   Sends the PM100DX torque command every 10 ms from a timer interrupt.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "inverter_command.h"

#include <IntervalTimer.h>

#include "bms_link.h"
#include "inverter.h"
#include "vehicle_can.h"
#include "faults.h"
#include "power_derate.h"
#include "regen.h"
#include "pedals.h"
#include "main.h"

void pm100CommandTimerIsr();
float calculateDisplayedCommandTorqueNm();
void queuePm100TxLogFromIsr(const CAN_message_t &msg);
bool dequeuePm100TxLog(CAN_message_t &msg);
uint8_t pm100DirectionByteForMode(char mode);

static constexpr uint16_t TORQUE_SCALE       = 10;      // 0.1 Nm per LSB
static constexpr uint32_t CMD_PERIOD_MS      = 10;      // 100 Hz
static constexpr uint32_t PM100_CMD_PERIOD_US = CMD_PERIOD_MS * 1000u;

// Torque actually loaded into the PM100 command frame this cycle; this is
// kept separate from the diagnostic display request because the bus must stay
// at zero whenever TS/RTD is off.
float commandedTorqueNm = 0.0f;
// Signed torque request shown on the dashboard. It continues to follow the
// pedals (positive drive, negative regen) while TS is off without bypassing
// the safety gate on commandedTorqueNm/pm100CommandTorqueRaw.
float displayedCommandTorqueNm = 0.0f;

IntervalTimer pm100CommandTimer;
volatile int16_t pm100CommandTorqueRaw = 0;
volatile uint8_t pm100CommandDirection = 0x00;
volatile uint8_t pm100CommandEnable = 0x00;

static constexpr uint8_t PM100_TX_LOG_QUEUE_SIZE = 192;
CAN_message_t pm100TxLogQueue[PM100_TX_LOG_QUEUE_SIZE];
volatile uint8_t pm100TxLogHead = 0;
volatile uint8_t pm100TxLogTail = 0;
volatile uint32_t pm100TxLogDropped = 0;

// ---------------- PM100 CAN command output ----------------
void setupPm100CommandTimer() {
  updatePm100CommandState();
  pm100CommandTimerIsr();
  pm100CommandTimer.begin(pm100CommandTimerIsr, PM100_CMD_PERIOD_US);
  Serial.println("[PM100] 100 Hz command timer started");
}

void pm100CommandTimerIsr() {
  CAN_message_t tx = {};
  tx.id = PM100_CMD_ID;
  tx.len = 8;
  tx.flags.extended = 0;

  int16_t torqueRaw = pm100CommandTorqueRaw;
  tx.buf[0] = (uint8_t)(torqueRaw & 0xFF);
  tx.buf[1] = (uint8_t)((torqueRaw >> 8) & 0xFF);
  tx.buf[4] = pm100CommandDirection;
  tx.buf[5] = pm100CommandEnable;

  VehicleCan.write(tx);
  queuePm100TxLogFromIsr(tx);
}

void updatePm100CommandState() {
  int16_t torqueRaw = 0;
  uint8_t direction = 0x00;
  uint8_t inverterEnable = 0x00;

  // Calculate the diagnostic request before the TS/RTD safety gate. This lets
  // pedal, brake, and regen commissioning be observed with HV disabled while
  // the actual PM100 command below remains zero and disabled.
  displayedCommandTorqueNm = calculateDisplayedCommandTorqueNm();

  bool driveSelected = readyToDriveLatched && (driveMode == 'D');
  // APPS implausibility and an active BSPC keep the inverter enabled but command
  // zero torque: updatePedal already forces desiredTorqueNm to 0 for both.
  bool torqueAllowed = driveSelected && !anyFaultActive() && !contactorsOpen();

  if (torqueAllowed) {
    torqueRaw = (int16_t)roundf(displayedCommandTorqueNm * (float)TORQUE_SCALE);
    direction = pm100DirectionByteForMode(driveMode);
    inverterEnable = 0x01;
  }

  noInterrupts();
  pm100CommandTorqueRaw = torqueRaw;
  pm100CommandDirection = direction;
  pm100CommandEnable = inverterEnable;
  interrupts();

  commandedTorqueNm = (float)torqueRaw / (float)TORQUE_SCALE;
}

// Builds the signed, quantized torque request independently of TS/RTD state.
// Safety inputs still affect the request through updatePedal and
// calculateRegenTorqueNm; updatePm100CommandState separately decides whether
// this value is permitted onto the CAN bus.
float calculateDisplayedCommandTorqueNm() {
  float torqueNm = desiredTorqueNm;
  regenTorqueNm = calculateRegenTorqueNm();
  if (regenTorqueNm > 0.0f) {
    // Negative torque in the selected forward direction commands regeneration.
    torqueNm = -regenTorqueNm;
  }

  if (powerDerateActive && torqueNm > 0.0f) {
    float cap = powerDerateTorqueCapNm(motorRpm);
    if (torqueNm > cap) torqueNm = cap;
  }

  return roundf(torqueNm * (float)TORQUE_SCALE) / (float)TORQUE_SCALE;
}

void queuePm100TxLogFromIsr(const CAN_message_t &msg) {
  uint8_t nextHead = (uint8_t)((pm100TxLogHead + 1u) % PM100_TX_LOG_QUEUE_SIZE);
  if (nextHead == pm100TxLogTail) {
    pm100TxLogDropped++;
    return;
  }

  pm100TxLogQueue[pm100TxLogHead] = msg;
  pm100TxLogHead = nextHead;
}

bool dequeuePm100TxLog(CAN_message_t &msg) {
  bool haveMessage = false;

  noInterrupts();
  if (pm100TxLogTail != pm100TxLogHead) {
    msg = pm100TxLogQueue[pm100TxLogTail];
    pm100TxLogTail = (uint8_t)((pm100TxLogTail + 1u) % PM100_TX_LOG_QUEUE_SIZE);
    haveMessage = true;
  }
  interrupts();

  return haveMessage;
}

void servicePm100TxLogging() {
  CAN_message_t tx;
  while (dequeuePm100TxLog(tx)) {
    logCanFrame("TX", tx);
  }

  static uint32_t lastDroppedCount = 0;
  uint32_t droppedCount;

  noInterrupts();
  droppedCount = pm100TxLogDropped;
  interrupts();

  if (droppedCount != lastDroppedCount) {
    Serial.print("[CAN TX LOG] PM100 command log dropped=");
    Serial.println(droppedCount);
    lastDroppedCount = droppedCount;
  }
}

uint8_t pm100DirectionByteForMode(char mode) {
  (void)mode;
  return 0x00;
}

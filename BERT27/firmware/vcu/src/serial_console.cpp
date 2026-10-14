/**
 * @file    serial_console.cpp
 * @brief   Reads commands typed on the USB serial console: manual fan duty, log listing and log dumps.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "serial_console.h"

#include "fans.h"
#include "data_log.h"

void handleSerialConsoleCommand(const char *cmd);

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

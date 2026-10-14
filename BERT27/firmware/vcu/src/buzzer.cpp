/**
 * @file    buzzer.cpp
 * @brief   Plays tone patterns on the buzzer, such as the ready-to-drive sound.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.10.7
 *
 * serviceBuzzer() runs every loop() and steps through the active pattern
 * without blocking.
 */
#include "buzzer.h"

#include "pins.h"

void stopBuzzer();

static const ToneStep TS_READY_CHIME_PATTERN[] = {
  {1568, 200, 0},
  {1047, 100, 0},
  {1319, 300, 1500}
};

const ToneStep *activeTonePattern = nullptr;
uint8_t activeToneLength = 0;
uint8_t activeToneIndex = 0;
bool activeToneInGap = false;
bool activeToneRepeats = false;
uint32_t activeToneDeadlineMs = 0;

// ---------------- Buzzer ----------------
void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats) {
  if (pattern == nullptr || length == 0) {
    return;
  }

  activeTonePattern = pattern;
  activeToneLength = length;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = repeats;
  activeToneDeadlineMs = millis() + activeTonePattern[0].onMs;
  tone(BUZZER_PIN, activeTonePattern[0].frequencyHz);
}

void serviceBuzzer() {
  // 3-tone RTD-ready chime disabled for testing.
  // bool shouldChimeForRtd = readyForRtdButton();
  //
  // if (activeTonePattern != RTD_PATTERN) {
  //   if (shouldChimeForRtd) {
  //     if (activeTonePattern != TS_READY_CHIME_PATTERN) {
  //       playTonePattern(TS_READY_CHIME_PATTERN,
  //                       sizeof(TS_READY_CHIME_PATTERN) / sizeof(TS_READY_CHIME_PATTERN[0]),
  //                       true);
  //     }
  //   } else if (activeTonePattern == TS_READY_CHIME_PATTERN) {
  //     stopBuzzer();
  //     return;
  //   }
  // }

  if (activeTonePattern == nullptr) {
    return;
  }

  uint32_t now = millis();
  if ((int32_t)(now - activeToneDeadlineMs) < 0) {
    return;
  }

  const ToneStep &step = activeTonePattern[activeToneIndex];

  if (!activeToneInGap && step.offMs > 0) {
    noTone(BUZZER_PIN);
    activeToneInGap = true;
    activeToneDeadlineMs = now + step.offMs;
    return;
  }

  activeToneIndex++;
  activeToneInGap = false;

  if (activeToneIndex >= activeToneLength) {
    if (activeToneRepeats) {
      activeToneIndex = 0;
      activeToneInGap = false;
      tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
      activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
    } else {
      stopBuzzer();
    }
    return;
  }

  tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
  activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
}

void stopBuzzer() {
  noTone(BUZZER_PIN);
  activeTonePattern = nullptr;
  activeToneLength = 0;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = false;
  activeToneDeadlineMs = 0;
}

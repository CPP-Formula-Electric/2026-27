# Code Guide

One major problem from last year's codebase is that everything is just dumped into big files, making it hard to read, debug, and make changes.

### [IMPORTANT] Split code into smaller files 

For example `apps.h` + `apps.cpp` for the pedal sensors, `tssi.h` + `tssi.cpp` for the TSSI, and so on.

## Start every file with a header

```cpp
/**
 * @file    apps.cpp
 * @brief   Reads the pedal sensors and checks if they agree.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: Creator
 * Worked on by: Your Name
 * Rules:   FSAE 2027 V1 T.4.2.4, T.4.2.5
 */
```

Don't put dates or version numbers in the header. 

## Naming

| Thing | How to write it | Example |
|---|---|---|
| functions | startWithModuleName, then a verb | `appsUpdate()`, `bmsReadCells()` |
| variables | camelCase **+ unit** | `packVoltageV`, `cellTempC`, `elapsedMs` |
| true/false values | read as a question | `isPrechargeDone`, `hasImdFault` |
| constants | ALL_CAPS **+ unit** | `CELL_OVERTEMP_LIMIT_C` |
| pins | `PIN_` + what it does | `PIN_TSSI_RED` |
| types and enums | CapitalizedWords | `AppsState`, `VcuState::ReadyToDrive` |

**Units go at the end of the name.** Common ones: `V`, `Nm`, `Rpm`, `Ms`, `Hz`. (**TO DO FINISH THIS LIST**)

Avoid short, vague names like `val`, `temp`, `rx`, `data2`.

For naming files and where to put them, check [FILE_STRUCTURE_GUIDE.md](FILE_STRUCTURE_GUIDE.md).

## No bare numbers

Bad:
```cpp
if (cellTemp > 60) { ... }
```
Good:
```cpp
constexpr float CELL_OVERTEMP_LIMIT_C = 60.0f;  // cite the source here: cell datasheet page or test report
if (cellTempC > CELL_OVERTEMP_LIMIT_C) { ... }
```

In the future for example you would put:

Put rule and safety limits in `limits.h`, values you tune at the track in `calibration.h`, and pins in `pins.h`.

## Comments

- Say **why**. The code already shows what it does.
  ```cpp
  // T.4.2.5: power must stop once the pedal sensors disagree for 100 ms.
  torqueCmdNm = 0;
  ```
- Put `// T.x.y.z:` above code that follows a rule.
- Put `// SAFETY:` above code that has a high risk of hurting someone or damaging the car.
- Delete old code. Avoid commenting old code out unless it is absolutely necessary. 


## Formatting

**TODO: create .clang-format and add info here**

## Building the code

We should use **PlatformIO** instead of the Arduino IDE, as it locks library versions and is intergrated within in VS code.

- Build: `pio run`
- Flash: `pio run -t upload`

For the settings file, check `../templates/platformio.ini`.



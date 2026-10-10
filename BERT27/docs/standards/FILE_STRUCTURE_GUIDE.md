# File Structure Guide

1. **Don't put versions in file names.** Write `main.cpp`, not `VCU_Fall26_v2.ino`. Use git tags to mark versions.
2. **Don't keep old copies.** If you need an older copy, use the repo.

## Naming files

File names use lowercase_with_underscores: `sd_log.cpp`, `dash_ui.h`.

For naming functions, variables and constants, check [CODE_GUIDE.md](CODE_GUIDE.md).

## POTENTIAL Folder layout - **WIP still need to figure out how to structure documentation and log files**

```
./
├── README.md                  car-level overview, board list, how to build, links to docs
├── CONTRIBUTING.md            how to make a change: points to docs/standards/ and the quick cards
├── .clang-format              code format (templates/.clang-format)
├── .gitignore                 (templates/.gitignore)
├── .github/
│   ├── PULL_REQUEST_TEMPLATE.md
│   └── workflows/build.yml    builds every board and checks doc links on every PR
│
├── firmware/                  code that runs on the car, one folder per board (other boards follow same folder format as vcu)
│   ├── vcu/
│   │   ├── platformio.ini
│   │   ├── README.md          board doc (templates/BOARD_README_TEMPLATE.md)
│   │   ├── CHANGELOG.md       board changelog (templates/CHANGELOG_TEMPLATE.md)
│   │   ├── src/               main.cpp + one .cpp/.h pair per module
│   │   ├── include/           generated fw_version_gen.h lands here (git-ignored)
│   │   └── test/              host unit tests (pio test -e native)
│   ├── bms/
│   ├── dash/                  only if the dashboard moves off the VCU
│   └── telemetry/             ESP32-S3 WiFi node
│
├── lib/                       code shared by more than one board
│   ├── bert_common/           fw_version.h, scheduler, fault codes, unit helpers
│   └── bert_can/              CAN IDs and pack/unpack code generated from the DBC
│
├── can/
│   ├── bert27.dbc             Where the CAN frames on the car should be saved
│   ├── CHANGELOG.md           CAN database changes 
│   └── CAN_MESSAGES.md        generated from bert27.dbc
│
├── bench/                     
│   └── apps_calibration/
│
├── tools/
│   ├── scripts/               version_from_git.py, gen_can.py
│   ├── decoders/              log decoders, one per log format version
│   │   ├── requirements.txt   pinned Python packages
│   │   └── samples/           small sample logs used by decoder tests
│   └── analysis/              plotting and run-analysis scripts
│
├── docs/                      
│   ├── README.md              index: which folder answers which question
│   ├── reference/             facts to look up: log-format.md, calibration-values.md
|   |   └── templates/         
│   ├── explanation/           Save rough drafts for justification/reasoning behind how you wrote your code.
│   │   ├── general/           docs that cover the whole car (EX: architecture.md, safety-functions.md)
│   │   ├── vcu/
│   │   ├── bms/
│   │   ├── dash/
│   │   └── telemetry/
│   ├── official/              explanation docs that have been checked and approved (moved here from explanation/)
│   │   ├── general/
│   │   ├── vcu/
│   │   ├── bms/
│   │   ├── dash/
│   │   └── telemetry/
│   └── standards/             standards for documentation (i.e. versioning standards, code structuring, basic rules, etc.)
│
└── releases/
    └── FLASH_LOG.md           every flash on the car, plus car configuration table.
```


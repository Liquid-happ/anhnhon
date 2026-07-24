# BMS Bootloader Restructuring Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Restructure the existing BMS_Vilas126_Bootloader project into a layered architecture directory (`BMS_Vilas126_FW`) matching the spec sheet, separating bootloader & application logic and documenting the system.

**Architecture:** Layered architecture consisting of:
- Hardware generated layer (`Core/`, `Drivers/`, `MDK-ARM/`)
- Application coordination layer (`App/`)
- Service logic layer (`Svc/`)
- Communication protocol layer (`Proto/`)
- Driver wrapper layer (`Drv/`)
- Common utilities layer (`Common/`)
- Configuration layer (`Cfg/`)
- Technical documentation layer (`Doc/`)

**Tech Stack:** C, STM32 HAL, STM32F103VET6 microcontroller.

---

### Task 1: Project Scaffolding
**Files:**
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/App/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Svc/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Proto/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Drv/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Common/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Cfg/`
- Create: `BMS_Vilas126_Bootloader/BMS_Vilas126_FW/Doc/`

**Step 1: Create directories and copy baseline folders**
Create the refactored project directory structure and copy CubeMX configurations (`BMS_Vilas126.ioc`, `.mxproject`), the MDK-ARM Keil project folder (`MDK-ARM/`), and CubeMX generated driver libraries (`Drivers/`, `Core/`) from `BMS_Vilas126_Bootloader` to the target `BMS_Vilas126_FW/` directory.

Run:
```bash
mkdir -p BMS_Vilas126_FW/App BMS_Vilas126_FW/Svc BMS_Vilas126_FW/Proto BMS_Vilas126_FW/Drv BMS_Vilas126_FW/Common BMS_Vilas126_FW/Cfg BMS_Vilas126_FW/Doc
cp -r Core Drivers MDK-ARM BMS_Vilas126.ioc .mxproject BMS_Vilas126_FW/
```

**Step 2: Commit base scaffolding**
```bash
git add BMS_Vilas126_FW/
git commit -m "refactor: scaffold directory structure and copy CubeMX baseline"
```

---

### Task 2: Refactor Common Utilities Layer
**Files:**
- Create: `BMS_Vilas126_FW/Common/rb.h`, `BMS_Vilas126_FW/Common/rb.c`
- Create: `BMS_Vilas126_FW/Common/sha256.h`, `BMS_Vilas126_FW/Common/sha256.c`
- Create: `BMS_Vilas126_FW/Common/tick.h`, `BMS_Vilas126_FW/Common/tick.c`
- Create: `BMS_Vilas126_FW/Common/types.h`
- Create: `BMS_Vilas126_FW/Common/err.h`
- Create: `BMS_Vilas126_FW/Common/ctx.h`, `BMS_Vilas126_FW/Common/ctx.c`
- Create: `BMS_Vilas126_FW/Common/filter.h`, `BMS_Vilas126_FW/Common/filter.c`
- Create: `BMS_Vilas126_FW/Common/kalman.h`, `BMS_Vilas126_FW/Common/kalman.c`
- Create: `BMS_Vilas126_FW/Common/ntc.h`, `BMS_Vilas126_FW/Common/ntc.c`

**Step 1: Move ring buffer logic**
Move RingBuffer source from `Core/rd_ota/RingBuffer.c/.h` to `Common/rb.c/.h`, renaming functions to follow a clean `Rb_` naming scheme if needed, or keeping compatibility aliases.

**Step 2: Move SHA256 logic**
Move SHA256 implementation from `Core/rd_ota/rd_sha256.c/.h` to `Common/sha256.c/.h`.

**Step 3: Implement tick wrapper and placeholder filters**
Implement `Common/tick.c/.h` to wrap `HAL_GetTick()`. Port `kalmanfilter.c` to `Common/kalman.c/.h`, and `ntc_table.c` to `Common/ntc.c/.h`. Define unified error return codes in `Common/err.h` and data structures in `Common/types.h` / `Common/ctx.h`.

**Step 4: Commit common layer**
```bash
git add BMS_Vilas126_FW/Common/
git commit -m "refactor: implement Common utilities layer"
```

---

### Task 3: Refactor Driver Layer
**Files:**
- Create: `BMS_Vilas126_FW/Drv/bq76952.h`, `BMS_Vilas126_FW/Drv/bq76952.c`
- Create: `BMS_Vilas126_FW/Drv/bq_reg.h` (from BQ769x2Header.h)
- Create: `BMS_Vilas126_FW/Drv/w25qxx.h`, `BMS_Vilas126_FW/Drv/w25qxx.c`
- Create: `BMS_Vilas126_FW/Drv/ds1307.h`, `BMS_Vilas126_FW/Drv/ds1307.c`
- Create: `BMS_Vilas126_FW/Drv/flash_drv.h`, `BMS_Vilas126_FW/Drv/flash_drv.c` (Internal Flash)
- Create: `BMS_Vilas126_FW/Drv/can_drv.h`, `BMS_Vilas126_FW/Drv/can_drv.c`
- Create: `BMS_Vilas126_FW/Drv/rs485_drv.h`, `BMS_Vilas126_FW/Drv/rs485_drv.c`
- Create: `BMS_Vilas126_FW/Drv/uart_drv.h`, `BMS_Vilas126_FW/Drv/uart_drv.c`
- Create: `BMS_Vilas126_FW/Drv/led_drv.h`, `BMS_Vilas126_FW/Drv/led_drv.c`

**Step 1: Extract low-level flash driver**
Extract `Flash_Erase_App`, `Flash_WriteBuffer`, `Flash_WritePage`, and check routines from `rd_flash.c` to `BMS_Vilas126_FW/Drv/flash_drv.c/.h`.

**Step 2: Move device drivers**
Move `driver_w25qxx.c` -> `Drv/w25qxx.c`, `DS1307.c` -> `Drv/ds1307.c`, `led_flash.c` / `led_indication.c` -> `Drv/led_drv.c`. Extract low-level I2C/UART/CAN calls into `uart_drv.c`, `can_drv.c`, and `rs485_drv.c`. Convert `BQ769x2Header.h` into `bq_reg.h`.

**Step 3: Port BQ76952 driver**
Port BQ76952 low-level register reading/writing and subcommands from `main.c` to `Drv/bq76952.c/.h`.

**Step 4: Commit driver layer**
```bash
git add BMS_Vilas126_FW/Drv/
git commit -m "refactor: implement Driver layer wrappers and peripheral files"
```

---

### Task 4: Refactor Protocol Layer
**Files:**
- Create: `BMS_Vilas126_FW/Proto/xmodem.h`, `BMS_Vilas126_FW/Proto/xmodem.c`
- Create: `BMS_Vilas126_FW/Proto/ota_proto.h`, `BMS_Vilas126_FW/Proto/ota_proto.c`
- Create: `BMS_Vilas126_FW/Proto/pylon_can.h`, `BMS_Vilas126_FW/Proto/pylon_can.c`
- Create: `BMS_Vilas126_FW/Proto/pylon_485.h`, `BMS_Vilas126_FW/Proto/pylon_485.c`
- Create: `BMS_Vilas126_FW/Proto/dwin.h`, `BMS_Vilas126_FW/Proto/dwin.c`

**Step 1: Refactor Xmodem protocol**
Move `Xmodem.c/.h` to `Proto/xmodem.c/.h`. Make sure it interacts with UART through the standard UART driver wrapper (`Drv/uart_drv.h`) instead of calling raw HAL functions.

**Step 2: Implement OTA command protocol parser**
Create `Proto/ota_proto.c/.h`. Refactor ESP32 packet checksumming and opcode dispatch (`Checksum_tts`, `opcode_dispatch`, `RD_CheckData`, `RD_process`, `rd_send_data_esp`) from `rd_control.c` here.

**Step 3: Port communication protocols**
Move Pylon CAN protocol packet mapping from `pylon_can.c` to `Proto/pylon_can.c/.h`. Move parallel pack RS485 communication packets from `bq_bms_485.c` to `Proto/pylon_485.c/.h`. Move DWIN screen serialization to `Proto/dwin.c/.h`.

**Step 4: Commit protocol layer**
```bash
git add BMS_Vilas126_FW/Proto/
git commit -m "refactor: implement Protocol layer with Xmodem, OTA ESP32, CAN, and RS485"
```

---

### Task 5: Refactor Service Layer
**Files:**
- Create: `BMS_Vilas126_FW/Svc/meas.h`, `BMS_Vilas126_FW/Svc/meas.c`
- Create: `BMS_Vilas126_FW/Svc/prot.h`, `BMS_Vilas126_FW/Svc/prot.c`
- Create: `BMS_Vilas126_FW/Svc/fault.h`, `BMS_Vilas126_FW/Svc/fault.c`
- Create: `BMS_Vilas126_FW/Svc/ctrl.h`, `BMS_Vilas126_FW/Svc/ctrl.c`
- Create: `BMS_Vilas126_FW/Svc/soc.h`, `BMS_Vilas126_FW/Svc/soc.c`
- Create: `BMS_Vilas126_FW/Svc/soh.h`, `BMS_Vilas126_FW/Svc/soh.c`
- Create: `BMS_Vilas126_FW/Svc/bal.h`, `BMS_Vilas126_FW/Svc/bal.c`
- Create: `BMS_Vilas126_FW/Svc/store.h`, `BMS_Vilas126_FW/Svc/store.c`
- Create: `BMS_Vilas126_FW/Svc/pack.h`, `BMS_Vilas126_FW/Svc/pack.c`
- Create: `BMS_Vilas126_FW/Svc/inv.h`, `BMS_Vilas126_FW/Svc/inv.c`
- Create: `BMS_Vilas126_FW/Svc/hmi.h`, `BMS_Vilas126_FW/Svc/hmi.c`
- Create: `BMS_Vilas126_FW/Svc/ind.h`, `BMS_Vilas126_FW/Svc/ind.c`
- Create: `BMS_Vilas126_FW/Svc/ota_svc.h`, `BMS_Vilas126_FW/Svc/ota_svc.c`

**Step 1: Move business logic from main.c and helper files**
Separate the massive functions from `main.c` and `bms_state.c` into services:
- Measurement acquisition (`Meas_Update`) -> `Svc/meas.c`
- Safety bit matching (`Prot_Update`) -> `Svc/prot.c`
- Combined alarm management (`Fault_Update`) -> `Svc/fault.c`
- FET control allow states (`Ctrl_Update`) -> `Svc/ctrl.c`
- Cell balancing scheduler (`Bal_Update`) -> `Svc/bal.c`
- SOC / SOH calculations -> `Svc/soc.c` and `Svc/soh.c`
- External flash memory snapshots -> `Svc/store.c`
- UART HMI and parallel RS485 communication coordinators -> `Svc/hmi.c` and `Svc/pack.c`
- CAN inverter supervisor -> `Svc/inv.c`
- LED & buzzer output coordinators -> `Svc/ind.c`
- OTA ESP32 command callbacks (`handle_start_ota`, `handle_get_infor`, `handle_ping_stm32`) -> `Svc/ota_svc.c`

**Step 2: Commit service layer**
```bash
git add BMS_Vilas126_FW/Svc/
git commit -m "refactor: implement Service layer mapping all BMS functions"
```

---

### Task 6: Refactor Configuration Layer
**Files:**
- Create: `BMS_Vilas126_FW/Cfg/bms_cfg.h`
- Create: `BMS_Vilas126_FW/Cfg/prot_cfg.h`
- Create: `BMS_Vilas126_FW/Cfg/comm_cfg.h`
- Create: `BMS_Vilas126_FW/Cfg/store_cfg.h`
- Create: `BMS_Vilas126_FW/Cfg/hw_cfg.h`
- Create: `BMS_Vilas126_FW/Cfg/feat_cfg.h`

**Step 1: Extract all parameters and feature flags**
- Move cell thresholds, nominal capacity, and pack specifications to `Cfg/bms_cfg.h`.
- Move software security alerts and limits to `Cfg/prot_cfg.h`.
- Move peripheral addresses and baudrates to `Cfg/comm_cfg.h`.
- Move SPI flash addresses, sector allocations, and magic words to `Cfg/store_cfg.h`.
- Move GPIO mapping macros to `Cfg/hw_cfg.h`.
- Define all active software/hardware bypass options (`FEAT_BQ_INIT_APPLY`, `FEAT_CAN_TX_APPLY`, etc.) in `Cfg/feat_cfg.h`.

**Step 2: Commit configuration layer**
```bash
git add BMS_Vilas126_FW/Cfg/
git commit -m "refactor: implement Configuration layer for parameters and feature flags"
```

---

### Task 7: Refactor Application and Core Initializations
**Files:**
- Create: `BMS_Vilas126_FW/App/app.h`, `BMS_Vilas126_FW/App/app.c`
- Create: `BMS_Vilas126_FW/App/sched.h`, `BMS_Vilas126_FW/App/sched.c`
- Create: `BMS_Vilas126_FW/App/state.h`, `BMS_Vilas126_FW/App/state.c`
- Create: `BMS_Vilas126_FW/App/boot_mgr.h`, `BMS_Vilas126_FW/App/boot_mgr.c`
- Modify: `BMS_Vilas126_FW/Core/Src/main.c`
- Modify: `BMS_Vilas126_FW/Core/Src/stm32f1xx_it.c`

**Step 1: Implement Boot Manager and Application Lifecycles**
Create `App/boot_mgr.c/.h` to hold high-level boot loader behaviors (`rd_flash_init`, `rd_copy_app_b_to_a`, `rd_run_whiletrue_boot`, `Bootloader_JumpToApp`). Create `App/state.c/.h` for the unified state machine logic. Create `App/sched.c/.h` to handle periodic execution cycles.
In `App/app.c`:
- `App_Init()`: Initialize system context, setup drivers, check boot integrity.
- `App_Run()`: Run bootloader checking loops under `IS_BOOTLOADER == 1` or scheduler ticks under `IS_BOOTLOADER == 0`.

**Step 2: Clean up main.c and stm32f1xx_it.c**
Remove all raw BMS functions, macros, and variables from `Core/Src/main.c`. Include `app.h` and call `App_Init()` in `main()` after CubeMX hardware setup, then call `App_Run()` inside the infinite `while(1)` loop. Update `Core/Src/stm32f1xx_it.c` to route UART1 interrupts directly to the ring buffer via Common layer definitions.

**Step 3: Commit application layer and Core cleanups**
```bash
git add BMS_Vilas126_FW/App/ BMS_Vilas126_FW/Core/
git commit -m "refactor: integrate Application layer and clean Core main.c"
```

---

### Task 8: Documentation Layer
**Files:**
- Create: `BMS_Vilas126_FW/Doc/arch.md`
- Create: `BMS_Vilas126_FW/Doc/map.md`
- Create: `BMS_Vilas126_FW/Doc/state.md`
- Create: `BMS_Vilas126_FW/Doc/fault.md`
- Create: `BMS_Vilas126_FW/Doc/rule.md`

**Step 1: Draft comprehensive documentation**
Create detailed technical descriptions:
- `Doc/arch.md`: System layers, data flows, scheduler structure, and dependency diagram.
- `Doc/map.md`: Map old functions (e.g. `Update_SOC_SOH_FromBQ`, `rd_copy_app_b_to_a`) to new file positions.
- `Doc/state.md`: Overall state machine flowchart and bootloader upgrade sequence.
- `Doc/fault.md`: Table of fault flags and alarm mappings.
- `Doc/rule.md`: Coding standards, directory organization rules, and how to build in bootloader vs. application configurations.

**Step 2: Commit documentation layer**
```bash
git add BMS_Vilas126_FW/Doc/
git commit -m "docs: write detailed technical architecture and mapping documentation"
```

---

### Task 9: Verification and Validation
**Files:**
- Modify: `BMS_Vilas126_FW/MDK-ARM/BMS_Vilas126.uvprojx` (Keil MDK Project)

**Step 1: Configure Keil Project**
Add new refactored file folders to the Keil project tree. Set include paths to target `BMS_Vilas126_FW/App`, `BMS_Vilas126_FW/Svc`, `BMS_Vilas126_FW/Proto`, `BMS_Vilas126_FW/Drv`, `BMS_Vilas126_FW/Common`, and `BMS_Vilas126_FW/Cfg`. Ensure it compiles without error for both `IS_BOOTLOADER = 1` and `IS_BOOTLOADER = 0` configurations.

**Step 2: Commit finalized project**
```bash
git add BMS_Vilas126_FW/MDK-ARM/BMS_Vilas126.uvprojx
git commit -m "refactor: update Keil MDK project config to include restructured layers"
```

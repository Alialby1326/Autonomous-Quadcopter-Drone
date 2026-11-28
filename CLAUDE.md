# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for an STM32F401CCU6 "Blackpill" flight controller (MPU9250 IMU over SPI, telemetry
over UART) plus host-side Python tooling in `rpi_scripts/` for recording and plotting that
telemetry.

## Commands

### Firmware

```sh
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build
```

Needs `gcc-arm-none-eabi cmake ninja-build`. Prints flash/RAM usage and emits `.hex`/`.bin`
after each build.

```sh
cmake --build build --target flash                  # build, write, verify, reset via ST-Link
cmake -B build -DFLASH_PORT=usb1 && cmake --build build --target flash   # USB DFU instead
```

Uses STM32CubeProgrammer's CLI, found automatically (standalone install, else the newest copy
bundled with CubeIDE under `/mnt/c/ST`); override with `-DSTM32_PROGRAMMER_CLI=<path>`. Under
WSL it runs the Windows `.exe`, so the ST-Link needs no usbipd passthrough. DFU needs BOOT0
held while tapping NRST. STM32CubeIDE still owns debugging; its `Debug/` makefile is generated
and gitignored.

Confirm float printf actually linked (see "printf floats" below):

```sh
arm-none-eabi-nm build/drone_code.elf | grep _printf_float
```

### Host tools

```sh
cd rpi_scripts
python3 -m venv .venv && ./.venv/bin/pip install -r requirements.txt

./.venv/bin/python -m pytest                      # all tests
./.venv/bin/python -m pytest tests/test_wire_contract.py::test_flag_definitions_match  # one test

./.venv/bin/python record_log.py --port /dev/ttyUSB0 --output flight.log
./.venv/bin/python record_log.py --port /dev/ttyUSB0 --monitor          # print lines, save nothing
./.venv/bin/python record_log.py --port /dev/ttyUSB0 --monitor 250      # ≤1 sample line per 250 ms
./.venv/bin/python visualize_attitude.py --port /dev/ttyUSB0          # live attitude
./.venv/bin/python visualize_attitude.py --log flight.log --loop      # replay a capture
```

Tests are pure host code — no toolchain, no board. `pytest.ini` sets `pythonpath = .`, so
pytest must run from `rpi_scripts/`.

**Serial under WSL.** WSL2 can't see Windows USB devices, so `/dev/ttyUSB0` doesn't exist
until the CP210x USB-UART adapter is passed through with usbipd-win (installed; the adapter
is already bound/"Shared", which is the one-time admin step):

```sh
usbipd.exe list                                        # find the CP210x (10c4:ea60) bus ID
usbipd.exe attach --wsl --busid 1-4 --auto-attach      # keep running; reattaches on replug
```

The bus ID belongs to the physical USB socket, so recheck `usbipd.exe list` if the adapter moves.
Attach only the UART adapter, **never the ST-Link**: the flash target runs the Windows
programmer, which needs Windows to own the ST-Link. On the Raspberry Pi none of this applies.

## Architecture

The attitude estimator and PID run **on the STM32**, as one deterministic loop:
IMU → attitude → PID → PWM. The Raspberry Pi sits outside that loop (setpoints, logging,
CV) so the aircraft stays controllable if the Pi hangs or is absent.

**Keep `docs/DESIGN.md` in step.** Its Mermaid diagram is the system map. Whenever a change
adds, removes or moves a peripheral, pin, DMA stream, module or data path, update the diagram
in the same change, and mark what's implemented as solid and what's planned as dashed.

Neither the attitude estimator nor the PID exists yet. The PID comes first; filter work
waits until there is a controller to feed and hardware data to check it against. The host
tools record the raw stream and compare host-side attitude estimators on it
(`droneviz/attitude.py`, registered in `ESTIMATORS`); `tests/test_attitude.py` pins every
estimator's sign convention. No simulator.

### The seam between the two halves

One CSV line per sample on USART1 at **460800 baud**:

```
I,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<flags>\n
```

m/s², deg/s, µT; `t_us` from the free-running TIM5 counter. This format and the `TLM_FLAG_*`
bits are defined **twice** — `Core/Src/telemetry.c` + `Core/Inc/imu.h` in C, and
`droneviz/telemetry.py` in Python. `tests/test_wire_contract.py` reads the actual firmware
source and fails if they drift, including whether the C frame buffer still fits a worst-case
line. Change one side and run that test.

`sources.py` turns serial or a recorded log into one `Iterator[ImuSample]`, so the
visualizer treats live and replayed data identically.

### Frames

Body frame is the MPU9250's: **X forward, Y left, Z up**; level and at rest the accelerometer
reads ≈ (0, 0, +9.81). Positive roll about +X carries +Y toward +Z, so a right bank puts
gravity on **positive** body Y — this is counterintuitive and has already caused one wrong
test.

Over SPI the AK8963 magnetometer can't be addressed directly: it sits on the MPU9250's
internal I2C bus. `imu.c` uses the MPU's I2C master (slave 4 for one-shot setup, slave 0
polling ST1..ST2 at 100 Hz into `EXT_SENS_DATA`), so `IMU_ReadAll` gets accel, temp, gyro and
mag in one 22-byte burst. A sample can be flagged fresh on more than one loop iteration if the
loop runs faster than 100 Hz.

The AK8963 die is bonded rotated relative to the accel/gyro, so `imu.c` remaps it
(`mag X → accel Y`, `mag Y → accel X`, `mag Z → −accel Z`). Everything downstream sees one
frame.

## Things that will bite you

**printf floats.** newlib-nano drops float conversion from `printf` unless `-u _printf_float`
is forced at link time — every `%f` silently emits nothing. It is set in both `CMakeLists.txt`
and `.cproject` (Linker → Miscellaneous → Other flags, in *both* the Debug and Release tool
blocks) and the two must stay in step. Eclipse ignores an unrecognized option ID silently, so
verify in the CubeIDE GUI after touching `.cproject`.

**No CubeMX.** The `.ioc` is gone, and every peripheral is set up by hand in the module that
uses it, with HAL calls: `timebase.c` (TIM5), `telemetry.c` (USART1, PA9/PA10, DMA2 Stream 7,
both IRQs, `TLM_BAUD`), `motors.c` (TIM3, PA6/PA7/PB0/PB1), `imu.c` (SPI2, PB12–PB15). New
hardware follows the same pattern: the module enables its own clocks, pins and interrupts,
and claims its pins in the hardware map below. The CubeMX-generated files that remain
(`main.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_hal_conf.h`) are ordinary
source now, so edit them anywhere. A HAL module used for the first time must be enabled by hand
in `stm32f4xx_hal_conf.h`.

**Timing.** Schedule off `Timebase_Micros()` (1 MHz, 32-bit) rather than `HAL_GetTick()`,
whose 1 ms resolution is the same order as the loop period. The counter wraps every ~71.6 min,
so always compare with signed differences: `(int32_t)(a - b)`.

**Telemetry never blocks.** `Telemetry_Write` is a memcpy into a ring buffer drained by DMA,
and is all-or-nothing (a truncated frame corrupts the host's parse). Call `Telemetry_Service()`
regularly. Check `Telemetry_Dropped()` after raising the loop rate.

**Motor outputs drive LEDs for now.** `motors.c` puts standard ESC PWM (1000–2000 µs pulse,
400 Hz, 1 µs resolution) on TIM3, the same signal the ESCs will get. LEDs on those pins sit
at 40% duty at zero throttle, not off; that is the ESC idle pulse, not a bug. There is no
arming or failsafe yet: outputs start at `MOTOR_PULSE_MIN_US` as soon as `Motors_Init()`
runs. Add that before connecting ESCs with propellers on.

## Hardware map

SPI2 mode 3, SCK PB13 / MISO PB14 / MOSI PB15 / CS PB12 (MPU9250; ≤1 MHz for register
writes, 5.25 MHz for sensor reads) · USART1 PA9/PA10 (telemetry TX via DMA2 Stream 7) · TIM3
PA6/PA7/PB0/PB1 motor PWM · TIM5 microsecond timebase. I2C1 (PB6/PB7, planned for the
BMP280), ADC1, TIM2, TIM4, DMA2 Stream 0 and PC13 (onboard LED) are unused. Clock: 25 MHz HSE → 84 MHz.

## Status

Everything above is verified on a clean build and by the host tests; **the current firmware has
not yet been run on hardware.** The magnetometer has never produced non-zero data on the
board, and the printf-float and mag-axis fixes are unconfirmed there. Treat sensor-facing
claims as untested until a `record_log.py` capture says otherwise.

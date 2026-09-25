# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for an STM32F401CCU6 "Blackpill" flight controller (MPU9250 IMU over I2C, telemetry
over UART) plus host-side Python tooling in `rpi_scripts/` for recording and plotting that
telemetry.

## Commands

### Firmware

```sh
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build
```

Needs `gcc-arm-none-eabi cmake ninja-build`. Prints flash/RAM usage and emits `.hex`/`.bin`
after each build. STM32CubeIDE still owns debugging and flashing; its `Debug/` makefile is
generated and gitignored.

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
./.venv/bin/python visualize_imu.py --port /dev/ttyUSB0          # live raw plots
./.venv/bin/python visualize_imu.py --log flight.log --loop      # replay a capture
```

Tests are pure host code — no toolchain, no board. `pytest.ini` sets `pythonpath = .`, so
pytest must run from `rpi_scripts/`.

## Architecture

The attitude estimator and PID run **on the STM32**, as one deterministic loop:
IMU → attitude → PID → PWM. The Raspberry Pi sits outside that loop (setpoints, logging,
CV) so the aircraft stays controllable if the Pi hangs or is absent.

Neither the attitude estimator nor the PID exists yet. The PID comes first; filter work
waits until there is a controller to feed and hardware data to check it against. The host
tools only record and plot the raw stream — no filtering, no simulator.

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
visualizer treats live and replayed data identically. `iter_log` also reads the older
three-line `ACCELEROMETER:/GYROSCOPE:/MAGNETOMETER:` format in `sim_imu.log`.

### Frames

Body frame is the MPU9250's: **X forward, Y left, Z up**; level and at rest the accelerometer
reads ≈ (0, 0, +9.81). Positive roll about +X carries +Y toward +Z, so a right bank puts
gravity on **positive** body Y — this is counterintuitive and has already caused one wrong
test.

The AK8963 die is bonded rotated relative to the accel/gyro, so `imu.c` remaps it
(`mag X → accel Y`, `mag Y → accel X`, `mag Z → −accel Z`). Everything downstream sees one
frame.

## Things that will bite you

**printf floats.** newlib-nano drops float conversion from `printf` unless `-u _printf_float`
is forced at link time — every `%f` silently emits nothing. It is set in both `CMakeLists.txt`
and `.cproject` (Linker → Miscellaneous → Other flags, in *both* the Debug and Release tool
blocks) and the two must stay in step. Eclipse ignores an unrecognized option ID silently, so
verify in the CubeIDE GUI after touching `.cproject`.

**CubeMX regeneration.** `drone_code.ioc` regenerates `Core/Src/{adc,dma,gpio,i2c,tim,usart}.c`,
`main.c` (outside `USER CODE` blocks), and `stm32f4xx_it.c`. To survive that:

- `timebase.c`/`telemetry.c`/`imu.c` are hand-written and unmanaged by CubeMX.
- **TIM5 must stay unused in CubeMX** — `Timebase_Init()` configures it, and a generated
  `MX_TIM5_Init` would collide.
- **USART1 must have no DMA request and no NVIC entry in CubeMX** — `telemetry.c` sets up
  DMA2 Stream 7 itself and defines `DMA2_Stream7_IRQHandler`/`USART1_IRQHandler`, overriding
  the startup file's weak symbols. Enabling them in CubeMX produces a duplicate-symbol link
  error, which is the intended loud failure.
- All `main.c` edits go inside `USER CODE` blocks.
- Baud lives in generated `Core/Src/usart.c` and reverts to 115200 on regeneration.

**Timing.** Schedule off `Timebase_Micros()` (1 MHz, 32-bit) rather than `HAL_GetTick()`,
whose 1 ms resolution is the same order as the loop period. The counter wraps every ~71.6 min,
so always compare with signed differences: `(int32_t)(a - b)`.

**Telemetry never blocks.** `Telemetry_Write` is a memcpy into a ring buffer drained by DMA,
and is all-or-nothing (a truncated frame corrupts the host's parse). Call `Telemetry_Service()`
regularly. Check `Telemetry_Dropped()` after raising the loop rate.

**Motor PWM is not flight-ready.** TIM2 (3 ch @ 21 kHz) and TIM4 (1 ch @ 42 kHz) drive LEDs.
ESCs need four channels at a common frequency; TIM3 (PA6/PA7/PB0/PB1) is deliberately kept
free for that.

## Hardware map

I2C1 PB6/PB7 @ 400 kHz (MPU9250) · USART1 PA9/PA10 (telemetry TX via DMA2 Stream 7) ·
ADC1 PA1/PA2/PA3 via DMA2 Stream 0 (pot + joystick) · TIM5 microsecond timebase ·
TIM2/TIM4 PWM. Clock: 8 MHz HSE → 84 MHz.

## Status

Everything above is verified on a clean build and by the host tests; **the current firmware has
not yet been run on hardware.** The magnetometer has never produced non-zero data on the
board, and the printf-float and mag-axis fixes are unconfirmed there. Treat sensor-facing
claims as untested until a `record_log.py` capture says otherwise.

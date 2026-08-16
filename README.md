# Autonomous-Quadcopter-Drone

### (in progress)
### Currently learning throughout EE curriculum and trying to apply knowledge to the project along the way.
The goal is to build an autonomous quadcopter drone fusing multiple sensors and implementing computer vision for autonomy in the future.
Custom Quadcopter Drone Project:
- STM32F401CC "Blackpill" as the microcontroller
- Raspberry Pi to handle commands from Bluetooth (controller tbd), high-level navigation, and computer vision in the future.
- Custom PCB to be made to integrate sensors and distribute power
  #### Sensors: IMU: MPU9250, Barometer: BMP280, GPS: GT-U7

## Architecture

The attitude estimator and the PID loop run **on the STM32**, as one deterministic
loop: read IMU → estimate attitude → PID → motor PWM. The Raspberry Pi sits
outside that loop and sends setpoints, logs telemetry, and will eventually run
navigation and computer vision.

The inner control loop deliberately does not depend on the Pi. Linux is not a
real-time system and a UART round-trip through it would add latency and jitter
to the one loop that cannot tolerate either — and the drone stays controllable
if the Pi hangs, reboots, or is simply not plugged in.

The Python attitude filter in `rpi_scripts/` is a **development and validation
tool**, not the flight path: it is where the estimator gets tested against known
ground truth before being ported to C.

```
STM32 (100 Hz, deterministic)          Raspberry Pi / laptop (best-effort)
  IMU -> attitude -> PID -> PWM  --->    logging, visualization, tuning
         ^                               nav / CV (future)
         |  setpoints
```

## Building the firmware

STM32CubeIDE still works as before, and remains the debugger and flasher. There
is also a command-line build:

```sh
sudo apt install gcc-arm-none-eabi cmake ninja-build
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build
```

The linker needs `-u _printf_float`; newlib-nano omits float support from
`printf` otherwise and every `%f` in the telemetry silently prints nothing. It
is set in both `CMakeLists.txt` and `.cproject`, and the two must stay in step.

## Telemetry

The firmware emits one line per sample on USART1 at **460800 baud**:

```
I,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<flags>
```

Units are m/s², deg/s and µT; `t_us` is microseconds from the free-running TIM5
counter. `flags` reports health — I2C failure, stale or saturated magnetometer,
loop overrun, uncalibrated gyro — so a frozen sensor is visible in the stream
instead of looking like steady level flight.

Writes go into a ring buffer drained by DMA, so the control loop never blocks on
the UART and every sample is transmitted rather than every tenth.

## Host tools (`rpi_scripts/`)

```sh
cd rpi_scripts
python3 -m venv .venv && ./.venv/bin/pip install -r requirements.txt
```

```sh
# synthetic data with known ground truth -- no hardware needed
./.venv/bin/python publish_imu.py --source sim
./.venv/bin/python visualize_attitude.py

# replay a recording, or watch the real board
./.venv/bin/python publish_imu.py --source log --log flight.log --loop
./.venv/bin/python publish_imu.py --source serial --port /dev/ttyUSB0
./.venv/bin/python record_log.py --port /dev/ttyUSB0 --output flight.log

./.venv/bin/python -m pytest tests/
```

`--source sim` generates a scripted maneuver, derives the accelerometer,
gyroscope and magnetometer readings that maneuver implies, and adds realistic
noise, bias and quantization. Because the true attitude is known at every step,
the visualizer draws the estimate against a ground-truth ghost and plots the
error — which is the difference between "the filter runs" and "the filter is
right".

- `droneviz/telemetry.py` — wire format, `ImuSample`, ZeroMQ transport
- `droneviz/sources.py` — serial / log / simulator, one sample stream
- `droneviz/simulate.py` — trajectories and the sensor model
- `droneviz/attitude.py` — quaternion complementary filter, written to port to C

## Frames

Sensor frame is the MPU9250's own: X forward, Y left, Z up. Level and at rest,
the accelerometer reads ≈ (0, 0, +9.81). The AK8963 magnetometer's die is bonded
in rotated relative to the accel/gyro, so the driver remaps it
(`mag X → accel Y`, `mag Y → accel X`, `mag Z → −accel Z`) — everything
downstream sees one consistent frame.

World frame for the host filter is NWU: X magnetic north, Y west, Z up.

### Phase 1: Control testing
The STM32 Blackpill is being used to run control logic for the IMU. Once this is tested, the PID controller should be written, and the Raspberry Pi must be ready to handle commands and communicate them to the STM32 via UART.

- [x] configure timers, clock, and test PWM
- [x] test PWM on 4 channels
- [x] Read analog values from the potentiometer and use them to adjust all LEDs
- [x] Use multi-channel ADC with DMA to read joystick and potentiometer values
- [x] Interface IMU (MPU9250) sensor through i2c
- [x] AK8963 factory sensitivity (ASA) calibration and axis remapping
- [x] Gyro bias calibration at boot
- [x] Non-blocking DMA telemetry, one line per sample, microsecond timestamps
- [x] Python attitude filter validated against a ground-truth simulator
- [x] Python script to visualize drone orientation
- [ ] Hard-iron / soft-iron magnetometer calibration (redo after the PCB exists)
- [ ] Port the complementary filter from `droneviz/attitude.py` to C
- [ ] Write PID controller on STM32
- [ ] Four synchronized ESC channels on TIM3 (PA6/PA7/PB0/PB1), plus arming and failsafe

Goal for phase 1:
- [ ] STM32 holds a commanded attitude on its own, with the Pi supplying setpoints and recording telemetry.

### Phase 2: Sensor Fusion Design
Sensors and microcontrollers should go onto a PCB for further testing, incorporate GPS and Barometer into the control logic.
- [ ] finalize battery, esc, motor choices
- [ ] design PCB
- [ ] MORE TBD

7/1/2025 Current setup:
- 4 LEDS to test PWM (for motors), potentiometer would adjust all brightness levels
- USB to UART converter for STM32 to send serial data to read on the laptop
- MPU9250 communicating via i2c

Note: motor PWM currently runs on TIM2 (3 channels @ 21 kHz) and TIM4 (1 channel
@ 42 kHz) — fine for driving LEDs, but ESCs need four channels at a common
frequency. TIM3 can supply all four synchronized; that move is queued above.

![image](https://github.com/user-attachments/assets/7bd714c2-9c71-4ed1-be03-569fdb4183a3)

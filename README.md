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

```
STM32                                    Raspberry Pi / laptop 
  IMU -> attitude -> PID -> PWM  --->    logging, visualization, tuning
         ^                               nav / CV (future)
         |  setpoints
```

The full component diagram — peripherals, the telemetry link, host tooling, and
what is still planned — is in [docs/DESIGN.md](docs/DESIGN.md).

## Building the firmware

command-line build:

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
pip install -r requirements.txt
```

```sh
# record the board, then watch it live or replay the capture
python record_log.py --port /dev/ttyUSB0 --output flight.log
python visualize_attitude.py --port /dev/ttyUSB0
python visualize_attitude.py --log flight.log --loop

python -m pytest tests/
```

The visualizer runs every attitude estimator in `droneviz/attitude.py` on the
same samples and draws them side by side: a 3D quadcopter, roll and pitch over
time, and |a|/g to show when the accelerometer is seeing more than gravity.
It is a host-side bench for comparing estimator math, not the flight estimator.

- `droneviz/telemetry.py` — wire format and `ImuSample`
- `droneviz/sources.py` — serial / log, one sample stream
- `droneviz/attitude.py` — attitude estimators under comparison
- `tests/test_attitude.py` — pins every estimator's roll/pitch sign convention
- `tests/test_wire_contract.py` — fails if the C and Python copies of the
  wire format drift apart

## Frames

Sensor frame is the MPU9250's own: X forward, Y left, Z up. Level and at rest,
the accelerometer reads ≈ (0, 0, +9.81). The AK8963 magnetometer's die is bonded
in rotated relative to the accel/gyro, so the driver remaps it
(`mag X → accel Y`, `mag Y → accel X`, `mag Z → −accel Z`) — everything
downstream sees one consistent frame.

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
- [x] Python scripts to record and plot the raw IMU stream
- [ ] Hard-iron / soft-iron magnetometer calibration (redo after the PCB exists)
- [ ] Confirm the IMU stream on hardware with `record_log.py` (magnetometer still reads zero)
- [ ] Write PID controller on STM32
- [ ] Attitude estimation on the STM32
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

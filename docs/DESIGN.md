# System design

Solid boxes and arrows are implemented; dashed ones are planned or reserved.
See the [README](../README.md#architecture) for the reasoning behind the split
between the STM32 and the Raspberry Pi.

```mermaid
flowchart TB

%% Layout (TB): top row = Planned on the Pi | Operator
%%              middle  = Firmware | Host       (test_wire_contract.py sits between them)
%%              bottom  = Board Hardware | USART1 link
%% Mermaid's default (dagre) layout stacks two subgraphs vertically whenever an
%% edge joins them directly, which is why test_wire_contract.py is drawn outside
%% the Host box: its "reads source" edge into the firmware would otherwise stack
%% Host on top of Firmware.
%% "~~~" links are invisible and exist only to place things. Several of them make
%% dagre draw an arrow upward (e.g. sensors below the scheduler, USART1 below the
%% host). That depends on declaration order: Operator must be the first node, and
%% each node's layout links must come before its real edges. Re-render after edits.

node_operator(("Operator"))

%% ───────────────────────── Planned on the Pi ─────────────────────────
subgraph group_planned["Planned on the Pi"]
  direction TB
  node_nav_cv["Navigation / CV"]
  node_setpoints["Setpoint uplink"]
end

%% ───────────────────────── STM32 firmware ─────────────────────────
subgraph group_fw["STM32F401 Firmware · 84 MHz · the only flight-critical path"]
  direction TB

  subgraph group_loop["100 Hz control loop · main.c"]
    direction TB
    node_flight_loop["Scheduler<br/>signed-diff timing, overrun flag"]
    node_est_c["Attitude estimator"]
    node_pid["PID + motor mixer"]
  end

  node_timebase["µs Timebase · TIM5 1 MHz<br/>[timebase.c]"]
  node_imu_driver["IMU driver<br/>gyro cal · mag axis remap<br/>[imu.c]"]
  node_spi["SPI2 · mode 3<br/>≤1 MHz regs, 5.25 MHz data reads<br/>PB13–15, CS PB12 [imu.c]"]
  node_motors["Motors · TIM3 · 4 ch ESC PWM<br/>1000–2000 µs @ 400 Hz<br/>PA6 / PA7 / PB0 / PB1 [motors.c]"]
  node_telemetry["Telemetry ring buffer [telemetry.c]"]
  node_uart["USART1 TX · DMA2 Stream 7<br/>PA9 [telemetry.c + usart.c]"]
end

%% ───────────────────────── Host tooling ─────────────────────────
subgraph group_host["Host / Raspberry Pi · rpi_scripts/ (outside the flight loop)"]
  direction TB

  node_codec["Wire format<br/>parse_line · format_line · ImuSample<br/>[telemetry.py]"]
  node_recorder["Log recorder<br/>[record_log.py]"]
  node_logfile[("flight.log<br/>wire format, verbatim")]

  subgraph group_sources["Sample sources → Iterator[ImuSample] · droneviz/sources.py"]
    direction TB
    node_src_serial["iter_serial"]
    node_src_log["iter_log"]
  end

  node_visualizer["Attitude comparison<br/>3D model · roll/pitch · |a|/g<br/>[visualize_attitude.py + attitude.py]"]
end

%% ───────────────────────── Board hardware ─────────────────────────
subgraph group_hw["Board Hardware"]
  direction TB
  node_mpu[("MPU9250<br/>accel + gyro<br/>+ AK8963 mag, polled by<br/>its internal I²C master")]
  node_leds["LEDs<br/>(stand-in for motors)"]
  node_escs["ESCs + Motors<br/>(not yet wired)"]
end

node_test_wire["test_wire_contract.py<br/>C ↔ Python drift check"]

%% ───────────────────────── The seam ─────────────────────────
node_link{{"USART1 → USB-UART · 460800 baud<br/>I,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,flags<br/>m/s² · deg/s · µT"}}

%% ─── Layout-only (invisible) links ───
node_recorder ~~~ node_src_serial
node_visualizer ~~~ node_link
node_flight_loop ~~~ node_timebase
node_flight_loop ~~~ node_imu_driver
node_imu_driver ~~~ node_spi
node_setpoints ~~~ node_test_wire

%% ─── Operator and planned control ───
node_operator -.->|"commands"| node_setpoints
node_operator -->|"runs"| node_recorder
node_operator -->|"runs"| node_visualizer
node_nav_cv -.-> node_setpoints
node_setpoints -.->|"USART1 RX (no uplink yet)"| node_flight_loop

%% ─── Contract check ───
node_test_wire -.->|"compares"| node_codec
node_test_wire -.->|"reads source"| node_telemetry

%% ─── Firmware data path ───
node_spi <-->|"one 22-byte burst:<br/>accel · temp · gyro · EXT_SENS mag"| node_mpu
node_spi --> node_imu_driver
node_imu_driver -->|"IMU_ReadAll → MPU9250_Data"| node_flight_loop
node_timebase -->|"Timebase_Micros() → t_us"| node_flight_loop
node_flight_loop -->|"Telemetry_SendImu (memcpy)"| node_telemetry
node_telemetry -->|"Telemetry_Service drains"| node_uart
node_uart --> node_link
node_flight_loop -->|"Motors_Init at boot<br/>(idle 1000 µs)"| node_motors
node_motors --> node_leds

%% ─── Planned closed loop ───
node_flight_loop -.-> node_est_c
node_est_c -.->|"roll / pitch / yaw"| node_pid
node_pid -.->|"Motors_SetThrottle"| node_motors
node_motors -.->|"signal"| node_escs

%% ─── Host data path ───
node_link -->|"serial"| node_recorder
node_link -->|"serial"| node_src_serial
node_recorder --> node_logfile
node_logfile --> node_src_log
node_src_serial & node_src_log --> node_visualizer
node_codec -.->|"parse_line"| node_recorder
node_codec -.->|"parse_line"| node_src_serial

%% ─── Styling ───
classDef toneNeutral fill:#f8fafc,stroke:#334155,stroke-width:1.5px,color:#0f172a
classDef toneBlue fill:#dbeafe,stroke:#2563eb,stroke-width:1.5px,color:#172554
classDef toneAmber fill:#fef3c7,stroke:#d97706,stroke-width:1.5px,color:#78350f
classDef toneMint fill:#dcfce7,stroke:#16a34a,stroke-width:1.5px,color:#14532d
classDef toneRose fill:#ffe4e6,stroke:#e11d48,stroke-width:1.5px,color:#881337
classDef toneIndigo fill:#e0e7ff,stroke:#4f46e5,stroke-width:1.5px,color:#312e81
classDef toneTeal fill:#ccfbf1,stroke:#0f766e,stroke-width:1.5px,color:#134e4a
classDef planned fill:#f8fafc,stroke:#64748b,stroke-width:1.5px,stroke-dasharray:5 4,color:#475569

class node_flight_loop,node_timebase,node_imu_driver,node_spi,node_motors,node_telemetry,node_uart toneBlue
class node_recorder,node_logfile,node_src_serial,node_src_log,node_codec,node_visualizer toneAmber
class node_test_wire toneTeal
class node_link toneRose
class node_mpu,node_leds,node_operator toneIndigo
class node_est_c,node_pid,node_escs,node_nav_cv,node_setpoints planned
```

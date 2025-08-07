# Autonomous-Quadcopter-Drone
#### 🛠️ Sponsorship: Proudly Sponsored by [PCBWay!](https://www.pcbway.com) Thanks to their generosity, they are supporting the PCB prototyping for this project. 
### (in progress)
The goal is to build an autonomous quadcopter drone fusing multiple sensors and implementing computer vision for autonomy in the future. 
Custom Quadcopter Drone Project:
- STM32F411 "Blackpill" as the microcontroller
- Raspberry Pi to handle commands from Bluetooth (controller tbd) and handle control logic, as well as computer vision in the future.
- Custom PCB to be made to integrate sensors and distribute power - (In the design stage, prototyping support from [PCBWay](https://www.pcbway.com))
  #### Sensors: IMU: MPU9250, Barometer: BMP280, GPS: GT-U7
 
### Phase 1: Control testing
The STM32 Blackpill is being used to run control logic for the IMU. Once this is tested, the PID controller should be written, and the Raspberry Pi must be ready to handle commands and communicate them to the STM32 via UART.

- [x] configure timers, clock, and test PWM
- [x] test PWM on 4 channels
- [x] Read analog values from the potentiometer and use them to adjust all LEDs
- [x] Use multi-channel ADC with DMA to read joystick and potentiometer values
- [x] Interface IMU (MPU9250) sensor through i2c
- [x] Implement early magnetometer calibration, will need recalibration after PCB
- [ ] Write Python scripts for control logic (Extended Kalman Filter), so the Raspberry Pi can run the control logic. Think about doing it in c++
- [ ] Write Python script to visualize drone orientation
- [ ] Write PID controller on STM32

Goal for phase 1:
- [ ] Have STM32 send sensor data to Raspberry Pi to run filter logic, RPI sends filtered data back to STM32 to run through PID.

### Phase 2: Sensor Fusion Design
Sensors and microcontrollers should go onto a PCB for further testing, incorporate GPS and Barometer into the control logic.
- [ ] finalize battery, esc, motor choices
- [ ] design PCB
- [ ] MORE TBD

7/1/2025 Current setup: 
- 4 LEDS to test PWM (for motors), potentiometer would adjust all brightness levels
- USB to UART converter for STM32 to send serial data to read from PUTTY on laptop
- MPU9250 communicating via i2c

![image](https://github.com/user-attachments/assets/7bd714c2-9c71-4ed1-be03-569fdb4183a3)



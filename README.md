# Autonomous-Quadcopter-Drone
The goal is to build an autonomous quadcopter drone fusing multiple sensors and implementing computer vision in the future. 
Custom Quadcopter Drone Project:
- STM32F411 "Blackpill" as the microcontroller
- Raspberry Pi to handle commands from Bluetooth (controller tbd), as well as for automation and computer vision in the future.
- Custom PCB to be made to integrate sensors and distribute power
  #### Sensors: IMU: MPU9250, Barometer: BMP280, GPS: GT-U7
 
### Phase 1: Control testing
The STM32 Blackpill is being used to run control logic for the IMU. Once this is tested, the PID controller should be written, and the Raspberry Pi must be ready to handle commands and communicate them to the STM32 via UART.

- [x] configure timers, clock, and test PWM
- [x] test PWM on 4 channels
- [x] Read analog values from the potentiometer and use them to adjust all LEDs
- [x] Use multi-channel ADC with DMA to read joystick and potentiometer values
- [x] Interface IMU (MPU9250) sensor through i2c
- [ ] Implement magnetometer calibration to avoid hard and soft iron distortion.
- [ ] Write Madgwick filter code in STM32 for IMU sensor fusion.
- [ ] Write a Python script for visualization, send filtered data and Euler angles from STM32 to the pygame visual script




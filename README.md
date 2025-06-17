# Autonomous-Quadcopter-Drone
A quadcopter drone that uses an STM32 blackpill microcontroller, a Raspberry Pi Zero 2 W, along with sensor fusion on a custom PCB. Plans to implement computer vision for automation after manual control.

### Phase 1: Testing motor control logic
The STM32 Blackpill is being used to run control logic and output to the motors accordingly. Simulation is currently using LEDs as motors. The commands should be tested, and the control should be figured out before moving onto real manual control using a controller + Raspberry Pi Zero 2 W via Bluetooth. 
- [x] configure clock & test PWM
- [x] test PWM on 4 channels
- [x] Read analog values from the potentiometer and use them to adjust all LEDs
- [x] joystick control
- [ ] Interface IMU (MPU9250) sensor and implement Kalman filter
- [ ] Write a Python script for visualization, send filtered data and Euler angles from STM32 to the pygame visual script

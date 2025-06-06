# Autonomous-Quadcopter-Drone
A quadcopter drone that uses an stm32 blackpill microcontroller, a Raspberry pi zero 2 W along with sensor fusion on a custom PCB. Plans to implement computer vision for automation after manual control.

### Phase 1: Testing motor control logic
The STM32 Blackpill is being used to run control logic and output to the motors accordingly. Simulation is currently using LEDs as motors. The commands should be tested, and the control should be figured out before moving onto real manual control using a controller + Raspberry Pi Zero 2 W via Bluetooth. 
- [x] configure clock & test PWM
- [x] test PWM on 4 channels
- [ ] read analog values from potentiometer and use to adjust all LEDs
- [ ] figure out method for control logic
- [ ] joystick control 

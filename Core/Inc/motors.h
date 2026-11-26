/*
 *  motors.h
 *
 *  Four synchronized motor outputs on TIM3, in standard ESC PWM: a 1000-2000 us
 *  high pulse repeated at 400 Hz. 1000 us is zero throttle, 2000 us is full.
 *  All four channels share one counter, so their pulses start on the same tick.
 *
 *      MOTOR_1  TIM3_CH1  PA6
 *      MOTOR_2  TIM3_CH2  PA7
 *      MOTOR_3  TIM3_CH3  PB0
 *      MOTOR_4  TIM3_CH4  PB1
 *
 *  Which physical arm each one drives is decided by the mixer, which does not
 *  exist yet.
 *
 *  Until the ESCs are wired, LEDs on these pins show the same signal: duty
 *  equals pulse / 2500 us, so zero throttle is a 40% glow and full throttle
 *  80%. The output never goes fully dark while running, and that is correct:
 *  an ESC needs the 1000 us pulse to arm and to stay stopped.
 *
 *  Hand-written: this module configures TIM3, its pins and its clock itself.
 *  TIM3 and PA6/PA7/PB0/PB1 must stay unused in CubeMX.
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"

typedef enum {
	MOTOR_1 = 0,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_COUNT
} Motor;

#define MOTOR_PULSE_MIN_US  1000u
#define MOTOR_PULSE_MAX_US  2000u

/* Configure TIM3 and start all four outputs at MOTOR_PULSE_MIN_US. Call once,
 * after SystemClock_Config. Fails, leaving the pins silent, if the timer clock
 * cannot be divided down to exactly 1 MHz -- wrong pulse widths are worse
 * than none. */
HAL_StatusTypeDef Motors_Init(void);

/* Throttle 0.0-1.0, clamped. Takes effect at the start of the next period,
 * never mid-pulse. */
void Motors_SetThrottle(Motor motor, float throttle);

/* All four to zero throttle (MOTOR_PULSE_MIN_US). */
void Motors_StopAll(void);

#endif /* INC_MOTORS_H_ */

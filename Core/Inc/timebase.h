/*
 * timebase.h
 *
 *  Microsecond timebase built on TIM5.
 *
 *  HAL_GetTick() has 1 ms resolution, which is the same order as the control
 *  loop period -- so scheduling against it quantizes every interval to a
 *  multiple of the loop rate and the actual dt jitters by up to 100%. Attitude
 *  integration is directly sensitive to that. TIM5 is 32-bit on the F401, so a
 *  1 MHz free-running counter gives microsecond resolution and wraps only
 *  every ~71.6 minutes.
 *
 *  This file is hand-written rather than CubeMX-generated, so regenerating the
 *  project from drone_code.ioc will not disturb it. TIM5 must be left unused
 *  in CubeMX or the generated MX_TIM5_Init would collide with this.
 */

#ifndef INC_TIMEBASE_H_
#define INC_TIMEBASE_H_

#include "stm32f4xx_hal.h"

/* Configure and start TIM5 as a free-running 1 MHz counter. Call once, after
 * SystemClock_Config. */
HAL_StatusTypeDef Timebase_Init(void);

/* Microseconds since Timebase_Init. Wraps every 2^32 us; compare timestamps
 * with signed differences -- (int32_t)(a - b) -- and the wrap takes care of
 * itself. */
uint32_t Timebase_Micros(void);

/* Busy-wait. Only for short device delays; it burns the CPU. */
void Timebase_DelayUs(uint32_t us);

#endif /* INC_TIMEBASE_H_ */

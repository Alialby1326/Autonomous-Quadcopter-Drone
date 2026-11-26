/*
 * motors.c
 *
 *  See motors.h for the signal and pin map.
 */

#include "motors.h"

/* 1 MHz tick, so compare values are pulse widths in microseconds. */
#define MOTOR_TICK_HZ     1000000u
#define MOTOR_PERIOD_US   2500u      /* 400 Hz */

static TIM_HandleTypeDef htim3;
static uint8_t motors_ready;

static const uint32_t motor_channel[MOTOR_COUNT] = {
	TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
};

HAL_StatusTypeDef Motors_Init(void)
{
	GPIO_InitTypeDef gpio = {0};
	TIM_OC_InitTypeDef oc = {0};
	uint32_t timer_clock;

	motors_ready = 0u;

	/* TIM3 is on APB1, like TIM5 in timebase.c: the timer clock is twice
	 * PCLK1 whenever the APB1 prescaler is not 1 (84 MHz here). */
	timer_clock = HAL_RCC_GetPCLK1Freq();
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
		timer_clock *= 2U;
	}
	if (timer_clock % MOTOR_TICK_HZ != 0u) {
		return HAL_ERROR;
	}

	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	gpio.Alternate = GPIO_AF2_TIM3;
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &gpio);
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &gpio);

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = (timer_clock / MOTOR_TICK_HZ) - 1u;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = MOTOR_PERIOD_US - 1u;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	/* Preload, so a period change only lands at an update event. HAL also
	 * enables compare preload per channel, so throttle writes never cut a
	 * pulse short or stretch it. */
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		return HAL_ERROR;
	}

	oc.OCMode = TIM_OCMODE_PWM1;
	oc.Pulse = MOTOR_PULSE_MIN_US;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCFastMode = TIM_OCFAST_DISABLE;
	for (uint32_t i = 0u; i < MOTOR_COUNT; i++) {
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, motor_channel[i]) != HAL_OK) {
			return HAL_ERROR;
		}
	}
	for (uint32_t i = 0u; i < MOTOR_COUNT; i++) {
		if (HAL_TIM_PWM_Start(&htim3, motor_channel[i]) != HAL_OK) {
			return HAL_ERROR;
		}
	}

	motors_ready = 1u;
	return HAL_OK;
}

void Motors_SetThrottle(Motor motor, float throttle)
{
	uint32_t pulse;

	if (!motors_ready || motor >= MOTOR_COUNT) {
		return;
	}
	/* Written so NaN lands on zero throttle, not full. */
	if (!(throttle > 0.0f)) {
		throttle = 0.0f;
	} else if (throttle > 1.0f) {
		throttle = 1.0f;
	}

	pulse = MOTOR_PULSE_MIN_US
	      + (uint32_t)(throttle * (float)(MOTOR_PULSE_MAX_US - MOTOR_PULSE_MIN_US) + 0.5f);
	__HAL_TIM_SET_COMPARE(&htim3, motor_channel[motor], pulse);
}

void Motors_StopAll(void)
{
	for (uint32_t i = 0u; i < MOTOR_COUNT; i++) {
		Motors_SetThrottle((Motor)i, 0.0f);
	}
}

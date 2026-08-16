/*
 * timebase.c
 *
 *  See timebase.h for why this exists and why it is not CubeMX-generated.
 */

#include "timebase.h"

static TIM_HandleTypeDef htim5;

HAL_StatusTypeDef Timebase_Init(void)
{
	TIM_ClockConfigTypeDef clock_source = {0};
	TIM_MasterConfigTypeDef master_config = {0};
	uint32_t timer_clock;

	__HAL_RCC_TIM5_CLK_ENABLE();

	/*
	 * TIM5 hangs off APB1. When the APB1 prescaler is not 1 the timer clock is
	 * twice PCLK1, which is the case here (84 MHz core, PCLK1 = 42 MHz, so the
	 * timers see 84 MHz). Deriving it rather than hard-coding 84 means this
	 * still produces microseconds if the clock tree is ever retuned.
	 */
	timer_clock = HAL_RCC_GetPCLK1Freq();
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
		timer_clock *= 2U;
	}

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = (timer_clock / 1000000U) - 1U;  /* -> 1 MHz */
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0xFFFFFFFFU;                       /* TIM5 is 32-bit */
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		return HAL_ERROR;
	}

	clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &clock_source) != HAL_OK) {
		return HAL_ERROR;
	}

	master_config.MasterOutputTrigger = TIM_TRGO_RESET;
	master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &master_config) != HAL_OK) {
		return HAL_ERROR;
	}

	/* No interrupt: the counter is read directly, so there is nothing to
	 * service and nothing to add jitter. */
	return HAL_TIM_Base_Start(&htim5);
}

uint32_t Timebase_Micros(void)
{
	return __HAL_TIM_GET_COUNTER(&htim5);
}

void Timebase_DelayUs(uint32_t us)
{
	uint32_t start = Timebase_Micros();
	while ((uint32_t)(Timebase_Micros() - start) < us) {
		/* spin */
	}
}

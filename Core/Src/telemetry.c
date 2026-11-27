/*
 * telemetry.c
 *
 *  See telemetry.h for the wire format and the rationale.
 */

#include "telemetry.h"

#include <stdio.h>
#include <string.h>

#define TLM_MASK (TLM_BUFFER_SIZE - 1u)

#if (TLM_BUFFER_SIZE & TLM_MASK) != 0
#error "TLM_BUFFER_SIZE must be a power of two"
#endif

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef *tlm_uart;    /* NULL until Telemetry_Init succeeds */
static DMA_HandleTypeDef hdma_usart1_tx;

static uint8_t tlm_buffer[TLM_BUFFER_SIZE];
static volatile uint16_t tlm_head;      /* next write position; main loop only */
static volatile uint16_t tlm_tail;      /* next byte to send; advanced in the ISR */
static volatile uint16_t tlm_in_flight; /* bytes in the active DMA transfer */
static volatile uint32_t tlm_dropped;

/*
 * Start a transfer if one is not already running. Called both from the main
 * loop and from the DMA completion interrupt, so the test-and-claim of
 * tlm_in_flight is done with interrupts masked -- otherwise both paths could
 * see an idle link and start overlapping transfers from the same buffer.
 */
static void tlm_kick(void)
{
	uint32_t primask;
	uint16_t start;
	uint16_t chunk;

	if (tlm_uart == NULL) {
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	if (tlm_in_flight != 0u || tlm_head == tlm_tail) {
		__set_PRIMASK(primask);
		return;
	}

	/* DMA needs one contiguous run, so a wrapped buffer takes two transfers:
	 * this one stops at the end of the array and the completion interrupt
	 * picks up the remainder from the start. */
	start = tlm_tail;
	chunk = (tlm_head > tlm_tail) ? (uint16_t)(tlm_head - tlm_tail)
	                              : (uint16_t)(TLM_BUFFER_SIZE - tlm_tail);
	tlm_in_flight = chunk;

	__set_PRIMASK(primask);

	if (HAL_UART_Transmit_DMA(tlm_uart, &tlm_buffer[start], chunk) != HAL_OK) {
		tlm_in_flight = 0u;
	}
}

HAL_StatusTypeDef Telemetry_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	tlm_uart = NULL;
	tlm_head = 0u;
	tlm_tail = 0u;
	tlm_in_flight = 0u;
	tlm_dropped = 0u;

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* PA9 = TX, PA10 = RX. RX is configured but unused until the setpoint
	 * uplink exists. */
	gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &gpio);

	/* USART1 is on APB2 (84 MHz); HAL derives the divider from that. */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = TLM_BAUD;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		return HAL_ERROR;
	}

	// USART1_TX is DMA2 Stream 7, Channel 4 on the STM32F401. 
	hdma_usart1_tx.Instance = DMA2_Stream7;
	hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
		return HAL_ERROR;
	}
	__HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

	/* Priority 5: below anything flight-critical that gets an interrupt
	 * later. Telemetry is diagnostic and can afford to wait. */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	tlm_uart = &huart1;
	return HAL_OK;
}

uint8_t Telemetry_Write(const uint8_t *data, uint16_t len)
{
	uint16_t used;
	uint16_t space;
	uint16_t head;

	if (tlm_uart == NULL || data == NULL || len == 0u) {
		return 0u;
	}

	used = (uint16_t)((tlm_head - tlm_tail) & TLM_MASK);
	space = (uint16_t)(TLM_BUFFER_SIZE - 1u - used);

	/* All or nothing: a partially written frame would corrupt the line the
	 * host is trying to parse, which is worse than a missing one. */
	if (len > space) {
		tlm_dropped++;
		return 0u;
	}

	head = tlm_head;
	for (uint16_t i = 0u; i < len; i++) {
		tlm_buffer[head] = data[i];
		head = (uint16_t)((head + 1u) & TLM_MASK);
	}
	tlm_head = head;

	tlm_kick();
	return 1u;
}

uint8_t Telemetry_Print(const char *text)
{
	if (text == NULL) {
		return 0u;
	}
	return Telemetry_Write((const uint8_t *)text, (uint16_t)strlen(text));
}

uint8_t Telemetry_SendImu(uint32_t t_us, const MPU9250_Data *imu, uint8_t extra_flags)
{
	char line[160];
	int n;

	if (imu == NULL) {
		return 0u;
	}

	n = snprintf(line, sizeof(line),
	             "I,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%u\n",
	             (unsigned long)t_us,
	             (double)imu->ax, (double)imu->ay, (double)imu->az,
	             (double)imu->gx, (double)imu->gy, (double)imu->gz,
	             (double)imu->mx, (double)imu->my, (double)imu->mz,
	             (unsigned int)(imu->flags | extra_flags));

	if (n <= 0 || n >= (int)sizeof(line)) {
		return 0u;
	}

	return Telemetry_Write((const uint8_t *)line, (uint16_t)n);
}

void Telemetry_Service(void)
{
	tlm_kick();
}

uint32_t Telemetry_Dropped(void)
{
	return tlm_dropped;
}

/*
 * HAL calls this when a DMA transfer finishes. Advancing the tail here rather
 * than in the main loop is what lets transmission continue while the loop is
 * busy reading sensors.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart != tlm_uart) {
		return;
	}
	tlm_tail = (uint16_t)((tlm_tail + tlm_in_flight) & TLM_MASK);
	tlm_in_flight = 0u;
	tlm_kick();
}

/*
 * Without this a single UART error would leave tlm_in_flight set forever and
 * telemetry would stop for good. Drop the in-flight bytes and carry on: the
 * host discards unparseable lines anyway.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart != tlm_uart) {
		return;
	}
	tlm_tail = (uint16_t)((tlm_tail + tlm_in_flight) & TLM_MASK);
	tlm_in_flight = 0u;
	tlm_dropped++;
	tlm_kick();
}

/*
 * Interrupt handlers.
 *
 * These override the weak defaults in the startup file, so they live with
 * the module that owns USART1 rather than in stm32f4xx_it.c.
 */
void DMA2_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

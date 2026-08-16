/*
 * telemetry.h
 *
 *  Non-blocking UART telemetry output.
 *
 *  The previous approach called HAL_UART_Transmit directly from the control
 *  loop. At 115200 baud a four-line human-readable frame is roughly 17 ms of
 *  blocking transmission inside a 10 ms loop -- so the loop could only afford
 *  to print every tenth sample, and still overran when it did. Ninety percent
 *  of the data was thrown away to hide a timing problem.
 *
 *  Here the loop copies a frame into a ring buffer and returns immediately;
 *  DMA drains the buffer in the background. One line per sample, and the
 *  control loop never waits on the wire.
 *
 *  Wire format, one line per sample:
 *
 *      I,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<flags>\n
 *
 *  Units are m/s^2, deg/s and uT. Parsed by rpi_scripts/droneviz/telemetry.py,
 *  which must be kept in step with any change here.
 *
 *  Hand-written, not CubeMX-generated, so regenerating from drone_code.ioc
 *  leaves it alone. It configures DMA2 Stream 7 (the USART1_TX stream) itself,
 *  so USART1 must have no DMA request configured in CubeMX.
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "stm32f4xx_hal.h"
#include "imu.h"

/* Must be a power of two. 1 KiB holds roughly a dozen frames, which is far
 * more slack than a healthy link needs and enough to ride out a stall. */
#define TLM_BUFFER_SIZE 1024

/* Set up the DMA stream and interrupts for the given UART. */
HAL_StatusTypeDef Telemetry_Init(UART_HandleTypeDef *huart);

/* Queue bytes for transmission. Returns 0 and counts a drop if the buffer is
 * full -- telemetry is diagnostic, and stalling the control loop to deliver it
 * would be a poor trade. */
uint8_t Telemetry_Write(const uint8_t *data, uint16_t len);

/* Queue a NUL-terminated string. */
uint8_t Telemetry_Print(const char *text);

/* Format and queue one IMU sample. `extra_flags` is OR-ed with the driver's
 * own flags, which is how the loop reports its own overruns. */
uint8_t Telemetry_SendImu(uint32_t t_us, const MPU9250_Data *imu, uint8_t extra_flags);

/* Start the next DMA transfer if one is pending and the link is idle. Call
 * from the main loop; it is cheap and returns at once when there is nothing to
 * do. */
void Telemetry_Service(void);

/* Frames discarded because the buffer was full. Non-zero means the link cannot
 * keep up with the loop rate: raise the baud or lower the rate. */
uint32_t Telemetry_Dropped(void);

#endif /* INC_TELEMETRY_H_ */

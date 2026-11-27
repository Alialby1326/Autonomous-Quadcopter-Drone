/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "imu.h"
#include "motors.h"
#include "telemetry.h"
#include "timebase.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Control loop rate. The MPU9250 is configured for a 1 kHz internal sample
 * rate, so this can be raised well beyond 100 Hz once there is a PID loop
 * worth running faster -- check Telemetry_Dropped() afterwards to confirm the
 * link is still keeping up. */
#define LOOP_RATE_HZ    100u
#define LOOP_PERIOD_US  (1000000u / LOOP_RATE_HZ)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

MPU9250_Data imu;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/

static void report_imu_status(IMU_Status status)
{
	switch (status) {
	case IMU_OK:
		Telemetry_Print("# imu ok\n");
		break;
	case IMU_ERR_WHOAMI:
		Telemetry_Print("# imu error: WHO_AM_I mismatch; not an MPU9250, or check SPI wiring\n");
		break;
	case IMU_ERR_MAG_WHOAMI:
		Telemetry_Print("# imu error: AK8963 not answering the MPU9250's I2C master; mag disabled\n");
		break;
	case IMU_ERR_BUS:
	default:
		Telemetry_Print("# imu error: SPI transfer failed\n");
		break;
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  if (Timebase_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (Telemetry_Init() != HAL_OK)
  {
    Error_Handler();
  }

  Telemetry_Print("# drone_code boot\n");
  Telemetry_Print("# I,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,flags  (m/s^2, deg/s, uT)\n");

  /* LEDs stand in for the ESCs on these pins until the motors are wired.
   * A failure leaves the outputs silent rather than halting, so the IMU
   * stream still runs and says why. */
  if (Motors_Init() != HAL_OK)
  {
    Telemetry_Print("# motors error: TIM3 init failed; outputs off\n");
  }

  IMU_Status imu_status = IMU_Init(&imu);
  report_imu_status(imu_status);

  /* A missing magnetometer leaves accel and gyro working, so still calibrate. */
  if (imu_status == IMU_OK || imu_status == IMU_ERR_MAG_WHOAMI)
  {
    Telemetry_Print("# calibrating gyro, hold still\n");
    Telemetry_Service();
    if (IMU_CalibrateGyro(&imu) == IMU_OK)
    {
      Telemetry_Print("# gyro calibrated\n");
    }
    else
    {
      Telemetry_Print("# gyro calibration failed; bias left at zero\n");
    }
  }

  /* Infinite loop */

  /* Scheduling runs off the 1 MHz TIM5 counter rather than HAL_GetTick(),
   * whose 1 ms resolution is the same order as the loop period itself.
   * Timestamps are taken from the same counter and shipped with each sample,
   * so the host can see the real interval instead of assuming a nominal one.
   *
   * All the comparisons below are signed differences, which stay correct
   * across the counter's 32-bit wrap (about every 71 minutes). */
  uint32_t next_us = Timebase_Micros() + LOOP_PERIOD_US;

  while (1)
  {
      uint32_t now_us = Timebase_Micros();

      if ((int32_t)(now_us - next_us) < 0)
      {
          /* Time left over: keep the telemetry link fed and wait. */
          Telemetry_Service();
          continue;
      }

      uint8_t loop_flags = 0u;

      /* More than a full period late means the previous iteration overran.
       * Report it and resynchronize rather than trying to catch up, which
       * would run several iterations back to back with a meaningless dt. */
      if ((int32_t)(now_us - next_us) > (int32_t)LOOP_PERIOD_US)
      {
          loop_flags |= TLM_FLAG_LOOP_OVERRUN;
          next_us = now_us + LOOP_PERIOD_US;
      }
      else
      {
          next_us += LOOP_PERIOD_US;
      }

      IMU_ReadAll(&imu);

      /* Every sample goes out, not every tenth: the write is a memcpy into a
       * ring buffer and DMA does the rest, so this no longer costs the loop
       * anything measurable. */
      Telemetry_SendImu(now_us, &imu, loop_flags);
      Telemetry_Service();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/*
 * accelerometer.cpp
 *
 *  Created on: Jan 28, 2024
 *      Author: mihai
 */

#include <stdbool.h>
#include <stdio.h>

#include "accelerometer.h"
#include "bno055.c"
#include "bno055.h"
#include "bno_config.h"
#include "main.h"
#include "relay.h"
#include "stm32g4xx_hal.h"
#include "main.c"
/*
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_gpio_ex.h"
*/

bno055_t bno;
error_bno err;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


  bno = (bno055_t){
          .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU,
      };

  //Initialize the sensor
  bno055_init(&bno);
  bno055_vec3_t accelerometer;


//Error Handling Code
/*      if ((err = bno055_init(&bno)) != BNO_OK) {
          printf("[x] Error initializing BNO\n");
          Error_Handler();
      }

      err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_RPS,
                                BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_RAD);
      if (err != BNO_OK) {
    	  printf("[BNO] Failed to set units. Err: %d\r\n", err);
      } else {
    	  printf("[BNO] Unit selection success\r\n");
      }*/

void accelerometer_init(bno055_t* imu) {
	bno055_vec3_t accelerometer;
}

int read_accelerometer_x(bno055_t bno, bno055_vec3_t acc) {
	bno.acc(&bno ,&accelerometer);
	return acc.x;
}




static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = ERROR_LED;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ERROR_LED_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = STATUS_LED;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C1;
    gpio.Pin = I2C1_SCL | I2C1_SDA;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C1_PORT, &gpio);

    // gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    // gpio.Mode = GPIO_MODE_AF_OD;
    // gpio.Alternate = GPIO_AF4_I2C2;
    // gpio.Pin =  I2C2_SCL;
    // gpio.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(I2C2_PORT, &gpio);

    // gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    // gpio.Mode = GPIO_MODE_AF_OD;
    // gpio.Alternate = GPIO_AF9_I2C2;
    // gpio.Pin =  I2C2_SDA;
    // gpio.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(I2C2_PORT, &gpio);
}


void SystemClock_Config(void) {
    /* SysTick_IRQn interrupt configuration */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    //__disable_irq();
    while (1) {
        HAL_Delay(200);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.hpp"
#include "omwof_helper.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug_print.h"
#include "Adafruit_Crickit.hpp"
#include "omwof_ss_neopix.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static const uint32_t I2C_DELAY = 1000; // Time (ms) to wait for I2C
uint8_t crick_i2c_address = 0x49;       // Use 8-bit address

Adafruit_Crickit *crick1;
seesaw_NeoPixel *neopix1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void init_I2C1(void);
uint8_t I2C_bus_scan(I2C_HandleTypeDef *);

/* USER CODE BEGIN PFP */

void BlinkLED(uint32_t blink_delay, uint8_t num_blinks);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

void init_I2C1(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed=400000;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  
}
uint8_t I2C_bus_scan(I2C_HandleTypeDef *h_i2c)
{
  HAL_StatusTypeDef ret;
  uint8_t i;
  for (i = 1; i < 128; i++)
  {
    ret = HAL_I2C_IsDeviceReady(h_i2c, (uint16_t)(i << 1), 3, 5);
    if (ret == HAL_OK) /* No ACK Received At That Address */
    {
      h_i2c->Devaddress = (uint16_t)(i);
      DBG_PRINTF_TRACE("I2C reponse: %d", i);
      return i;
    }
  }
  DBG_PRINTF_TRACE("I2C NO reponse: %d", i);
  return 0;
}
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  uint8_t wheel_pos = 0;

  MX_USB_DEVICE_Init();
  HAL_Delay(3000);
  DBG_PRINTF_DEBUG("USB init");
  neopix1 = new seesaw_NeoPixel();

  // HAL_I2C_MspInit(&hi2c1);
  init_I2C1();


  uint8_t i2cscanres = I2C_bus_scan(&hi2c1);
  neopix1->set_I2C(&hi2c1);
  neopix1->i2c_address_local = i2cscanres;

  neopix1->sendtestbyte(i2cscanres);

  DBG_PRINTF_TRACE("update address %d", i2cscanres);
  neopix1->updateType(NEO_GRB + NEO_KHZ800);

  neopix1->updateLength(1);
  //neopix1->clear();
  neopix1->setPin((uint8_t)27 << 1);

  BlinkLED(100, 3);
  // crick1->begin(CRICK_I2C_ADDR,-1,true);
  neopix1->begin(i2cscanres, -1, true);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // If error writing to card, blink 3 times

    BlinkLED(200, 3);
    DBG_PRINTF_DEBUG("loop");

    HAL_Delay(100);

    if ((hi2c1.State != HAL_I2C_STATE_BUSY))
    {
      wheel_pos < 0xff ? wheel_pos++ : 0;
      neopix1->setPixelColor((neopix1->numPixels()-1), neopix1->Wheel(wheel_pos));
      neopix1->show();
      neopix1->digitalWrite(CRICKIT_SIGNAL3,HIGH);

      DBG_PRINTF_DEBUG("pixel output");
    }
    else
    {
      DBG_PRINTF_DEBUG("i2c not ready");
      if (HAL_I2C_GetError(&hi2c1))
      {
        DBG_PRINTF_DEBUG("nack i2c");
      }
    }

    // Wait before sampling again
    HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 -- REQUIRED FOR CDC USB */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __I2C1_CLK_ENABLE();

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

}

/* USER CODE BEGIN 4 */

// Blink onboard LED
void BlinkLED(uint32_t blink_delay, uint8_t num_blinks)
{
  for (int i = 0; i < num_blinks; i++)
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(blink_delay);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(blink_delay);
  }
}

int debug_print_callback(char *debugMessage, unsigned int length)
{
  CDC_Transmit_FS((uint8_t *)debugMessage, length);
  return true;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c1)
{
  DBG_PRINTF_DEBUG("receive  callback");
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c1)
{
  DBG_PRINTF_DEBUG("transmit callback");
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

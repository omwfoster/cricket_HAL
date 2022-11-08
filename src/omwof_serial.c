#include "omwof_serial.h"

UART_HandleTypeDef serial;



void MX_UART4_Init()
{
 

  serial.Instance = UART4;
  serial.Init.BaudRate = 9600;
  serial.Init.WordLength = UART_WORDLENGTH_8B;
  serial.Init.StopBits = UART_STOPBITS_1;
  serial.Init.Parity = UART_PARITY_NONE;
  serial.Init.Mode = UART_MODE_TX_RX;
  serial.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  serial.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&serial) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
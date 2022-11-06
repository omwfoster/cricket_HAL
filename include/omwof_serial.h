/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OMWOF_SERIAL
#define __OMWOF_SERIAL

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
UART_HandleTypeDef huart4;
/// @brief 
void MX_UART4_Init();


#ifdef __cplusplus
}
#endif


#endif





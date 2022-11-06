/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H



#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "Adafruit_Crickit.hpp"



void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

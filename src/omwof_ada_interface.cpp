#include "omwof_ada_interface.hpp"
#include "Adafruit_Crickit.hpp"
#include "stm32f4xx.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef *hi2c3;
DMA_HandleTypeDef *hdma_i2c3_rx;
DMA_HandleTypeDef *hdma_i2c3_tx;

Adafruit_Crickit * hcrick1;

void cricket_main_cpp()
{
    hcrick1 = new Adafruit_Crickit(hi2c3);
    hcrick1->begin(SEESAW_ADDRESS);

    while (true)
    {


    }

    
};

extern "C" void cricket_main_c()
{
    cricket_main_cpp();
}
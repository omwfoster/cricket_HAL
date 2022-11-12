#include "omwof_ada_interface.hpp"
#include "Adafruit_Crickit.hpp"
#include "omwof_packet.hpp"
#include "seesaw_neopixel.h"
#include "seesaw_servo.h"
#include "stm32f4xx.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"


I2C_HandleTypeDef *hi2c;
DMA_HandleTypeDef *hdma_i2c3_rx;
DMA_HandleTypeDef *hdma_i2c3_tx;

Adafruit_Crickit *hcrick1;

om_packet p_put; // packet for transmission
om_packet p_get; // packet for retreival;

void put_pac(om_packet *put_p);
void get_pac(om_packet *get_p);

void cricket_main_cpp()
{
    hcrick1 = new Adafruit_Crickit(hi2c);
    hcrick1->begin(SEESAW_ADDRESS);
    p_put.processed = true;
    p_get.processed = true;

    while (true)
    {

        if (p_get.processed == false)
        {
        }

        if (p_put.processed == true)
        {
        }
    }
};

void full_test()
{

#define NEOPIX_PIN 20

#define USE_NEOPIX


    hcrick1 = new Adafruit_Crickit(hi2c);
    hcrick1->begin(SEESAW_ADDRESS);
    p_put.processed = true;
    p_get.processed = true;
    seesaw_NeoPixel strip = seesaw_NeoPixel(24, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);
    Adafruit_Crickit crickit();
    seesaw_Servo s1(hcrick1);
    seesaw_Servo s2(hcrick1);
    seesaw_Servo s3(hcrick1);
    seesaw_Servo s4(hcrick1);

#define NUM_SERVOS 4
    seesaw_Servo *servos[NUM_SERVOS] = {&s1, &s2, &s3, &s4};

#define COLOR_MAX 150
#define RED strip.Color(COLOR_MAX, 0, 0)
#define YELLOW strip.Color(COLOR_MAX, 150, 0)
#define GREEN strip.Color(0, COLOR_MAX, 0)
#define CYAN strip.Color(0, COLOR_MAX, 255)
#define BLUE strip.Color(0, 0, COLOR_MAX)
#define PURPLE strip.Color(180, 0, COLOR_MAX)

#define CRICKIT_NUM_ADC 8
    static const uint8_t crickit_adc[CRICKIT_NUM_ADC] = {CRICKIT_SIGNAL1, CRICKIT_SIGNAL2, CRICKIT_SIGNAL3, CRICKIT_SIGNAL4,
                                                         CRICKIT_SIGNAL5, CRICKIT_SIGNAL6, CRICKIT_SIGNAL7, CRICKIT_SIGNAL8};

#define CRICKIT_NUM_TOUCH 4
    static const uint8_t crickit_drive[CRICKIT_NUM_TOUCH] = {CRICKIT_DRIVE1, CRICKIT_DRIVE2, CRICKIT_DRIVE3, CRICKIT_DRIVE4};

#define CAPTOUCH_THRESH 500

uint8_t A0 = 0;    //TODO:  definitions for analogue pins 

   
   
   
    s1.attach(CRICKIT_SERVO1);
    s2.attach(CRICKIT_SERVO2);
    s3.attach(CRICKIT_SERVO3);
    s4.attach(CRICKIT_SERVO4);

#ifdef USE_NEOPIX
    for (uint16_t i = 0; i < strip.numPixels(); i++)
        strip.setPixelColor(i, RED);
    strip.show();
#endif

    for (int i = 0; i < CRICKIT_NUM_ADC; i++)
    {
     //   Serial.print(hcrick1->analogRead(crickit_adc[i]));
     //   Serial.print("\t");
    }
    //Serial.println("");

    // TODO: fix drive3 and drive4
    for (int i = 0; i < 4; i++)
    {
        uint16_t val = hcrick1->touchRead(i);

        if (val > CAPTOUCH_THRESH)
        {
            hcrick1->analogWrite(crickit_drive[i], (1UL << 16) - 1);
       //     Serial.print("CT");
       //     Serial.print(i + 1);
       //     Serial.print(" touched! value: ");
       //     Serial.println(val);
        }
        else
            hcrick1->analogWrite(crickit_drive[i], 0);
    }

#ifdef USE_NEOPIX
    for (uint16_t i = 0; i < strip.numPixels(); i++)
        strip.setPixelColor(i, GREEN);
    strip.show();
#endif

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        seesaw_Servo *s = servos[i];
        s->write(1000);
    }
   // delay(500);

#ifdef USE_NEOPIX
    for (uint16_t i = 0; i < strip.numPixels(); i++)
        strip.setPixelColor(i, BLUE);
    strip.show();
#endif

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        seesaw_Servo *s = servos[i];
        s->write(2000);
    }
   // delay(500);
}

void put_pac(om_packet *put_p)
{
}
void get_pac(om_packet *get_p)
{
}

extern "C" void cricket_main_c()
{
    cricket_main_cpp();
}
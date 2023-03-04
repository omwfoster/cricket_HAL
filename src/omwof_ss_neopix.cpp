/*-------------------------------------------------------------------------
  Stm32f4 library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for various color orders.  Handles most output pins
  (possible exception with upper PORT registers on the Arduino Mega).
  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!
  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.
  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.
  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "omwof_ss_neopix.hpp"
#include "omwof_ss.hpp"
#include "debug_print.h"

// Constructor when length, pin and type are known at compile-time:
seesaw_NeoPixel::seesaw_NeoPixel(uint16_t n, uint8_t p, neoPixelType t,
                                 I2C_HandleTypeDef *x)
    : Adafruit_seesaw() {}

// via Michael Vogt/neophob: empty constructor is used when strand length
// isn't known at compile-time; situations where program config might be
// read from internal flash memory or an SD card, or arrive via serial
// command.  If using this constructor, MUST follow up with updateType(),
// updateLength(), etc. to establish the strand type, length and pin number!
seesaw_NeoPixel::seesaw_NeoPixel(I2C_HandleTypeDef *x)
    : Adafruit_seesaw(),
#ifdef NEO_KHZ400
      is800KHz(true),
#endif
      begun(false), numLEDs(0), numBytes(0), pin(-1), brightness(0)
{
}
seesaw_NeoPixel::seesaw_NeoPixel(I2C_HandleTypeDef *x, uint16_t numled, uint8_t pin)
    : Adafruit_seesaw(x), is800KHz(true),
      begun(false), numBytes(0), pin(-1), brightness(0)
{

  this->pin = pin;
}

seesaw_NeoPixel::~seesaw_NeoPixel()
{
  if (ptr_pixels)
    free(ptr_pixels);
}

bool seesaw_NeoPixel::begin(uint16_t numLEDs, int8_t flow)
{
  if (this->begun)
  {
    return true;
  }

  if (!Adafruit_seesaw::begin(this->i2c_address_local, flow, numLEDs))
    return false;
  this->begun = true;
  return true;
}

bool seesaw_NeoPixel::begin()
{
  if (this->begun)
  {
    return true;
  }
  if (!Adafruit_seesaw::begin(this->i2c_address_local << 1, (uint8_t)(-1)), numLEDs)
  {
    this->begun = false;
    return false;
  }

  this->begun = true;
  return true;
}

void seesaw_NeoPixel::updateLength(uint16_t n)
{
  DBG_PRINTF_TRACE("dealloc pixels");
  if (!(ptr_pixels == NULL))
    // free(ptr_pixels); // Free existing data (if any)
    if (!(this->ptr_output_buffer == NULL))
      // free(this->ptr_output_buffer); // Free existing data (if any)

      // Allocate new data -- note: ALL PIXELS ARE CLEARED
      this->numBytes = n * sizeof(colour_RGB);
  ptr_pixels = (colour_RGB *)malloc(this->numBytes);

  memset(ptr_pixels, 0, this->numBytes);
  this->numLEDs = n;
  DBG_PRINTF_TRACE("malloc pixels");

  uint8_t buf[] = {(uint8_t)(numBytes >> 8), (uint8_t)(numBytes & 0xFF)};
  this->write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, buf, 2);
}

void seesaw_NeoPixel::updateType(neoPixelType t)
{

  is800KHz = (t < 256); // 400 KHz flag is 1<<8

  this->write8(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, is800KHz);
}

void seesaw_NeoPixel::show(void)
{

  if (!ptr_pixels)
    DBG_PRINTF_DEBUG("neopixel::show no pixels", 2);
  return;

  // Data latch = 300+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  // while (!canShow())
  //  ;

  //TODO:POSSIBLE CULPRIT FOR HANGING
  this->output_stream();
  this->write((uint8_t)SEESAW_NEOPIXEL_BASE, (uint8_t)SEESAW_NEOPIXEL_SHOW, this->ptr_output_buffer, numBytes);
  DBG_PRINTF_DEBUG("neopixel::write");
}

// Set the output pin number
void seesaw_NeoPixel::setPin(uint8_t p)
{
  DBG_PRINTF_DEBUG("neopixel::setpin pre");
  this->write8(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, p);
}

// Set pixel color from separate R,G,B components:
void seesaw_NeoPixel::setPixelColor(uint16_t n, uint8_t r, uint8_t g,
                                    uint8_t b)
{

  if (n < numLEDs)
  {
    if (brightness)
    { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    colour_RGB *p;

    p = &this->ptr_pixels[n];
    p->r = r;
    p->g = g;
    p->b = b;
  }
}

// Set pixel color from 'packed' 32-bit RGB color:
void seesaw_NeoPixel::setPixelColor(uint16_t n, colour_RGB c)
{
  if (n < numLEDs)
  {
    if (brightness)
    { // See notes in setBrightness()
      c.r = (c.r * brightness) >> 8;
      c.g = (c.g * brightness) >> 8;
      c.b = (c.b * brightness) >> 8;
    }
    colour_RGB *p;

    p = &this->ptr_pixels[n];
    p->r = c.r;
    p->g = c.g;
    p->b = c.b;
  }
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
colour_RGB seesaw_NeoPixel::Color(uint8_t r, uint8_t g, uint8_t b)
{
  return colour_RGB{r, g, b};
}

// Convert separate R,G,B,W into packed 32-bit WRGB color.
// Packed format is always WRGB, regardless of LED strand color order.
colour_RGB seesaw_NeoPixel::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  return colour_RGB{
      r,
      g,
      b,
  };
}

// Query color from previously-set pixel (returns packed 32-bit RGB value)
colour_RGB seesaw_NeoPixel::getPixelColor(uint16_t n)
{
  if (n >= numLEDs)
  {
    return colour_RGB{0, 0, 0}; // Out of bounds, return no color.
  }
  else
  {
    return colour_RGB{0, 0, 0, 0};
  }
}

// Returns pointer to pixels[] array.  Pixel data is stored in device-
// native format and is not translated here.  Application will need to be
// aware of specific pixel data format and handle colors appropriately.
colour_RGB *seesaw_NeoPixel::getPixels(void) const { return NULL; } // colour_RGB{0, 0, 0}; }

uint16_t seesaw_NeoPixel::numPixels(void) const { return numLEDs; }

void seesaw_NeoPixel::clear()
{

  DBG_PRINTF_TRACE("clear buffers");
  uint16_t sz_pix_arr = (numLEDs * sizeof(ptr_pixels));
  uint16_t sz_out_buf = 2 * sz_pix_arr;
  // Clear local pixel buffer

  memset(ptr_output_buffer, 0, sz_out_buf);

  this->write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, ptr_output_buffer, sz_out_buf);
}

void seesaw_NeoPixel::setBrightness(uint8_t b) { brightness = b; }

void seesaw_NeoPixel::output_stream()
{

  for (int i = 0; i < this->numLEDs; i++)
  {
    DBG_PRINTF_DEBUG("OUTPUT_STREAM");
    *ptr_output_buffer = (ptr_pixels->r);
    ptr_output_buffer++;
    *ptr_output_buffer = (ptr_pixels->g);
    ptr_output_buffer++;
    *ptr_output_buffer = (ptr_pixels->b);
    ptr_output_buffer++;
    ptr_pixels++;
  }
}

colour_RGB seesaw_NeoPixel::Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return this->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return this->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return this->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

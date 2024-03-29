/*--------------------------------------------------------------------
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
  --------------------------------------------------------------------*/

#ifndef CC8A05C2_F379_44BB_9A3C_1536186EDAB4
#define CC8A05C2_F379_44BB_9A3C_1536186EDAB4

#ifndef SEESAW_NEOPIXEL_H
#define SEESAW_NEOPIXEL_H

#include "omwof_ss.hpp"
#include <arm_math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include "debug_print.h"

__STATIC_INLINE uint32_t micros(void){
	return  DWT->CYCCNT / (SystemCoreClock / 1000000U);
}


// The order of primary colors in the NeoPixel data stream can vary
// among device types, manufacturers and even different revisions of
// the same item.  The third parameter to the seesaw_NeoPixel
// constructor encodes the per-pixel byte offsets of the red, green
// and blue primaries (plus white, if present) in the data stream --
// the following #defines provide an easier-to-use named version for
// each permutation.  e.g. NEO_GRB indicates a NeoPixel-compatible
// device expecting three bytes per pixel, with the first byte
// containing the green value, second containing red and third
// containing blue.  The in-memory representation of a chain of
// NeoPixels is the same as the data-stream order; no re-ordering of
// bytes is required when issuing data to the chain.

// Bits 5,4 of this value are the offset (0-3) from the first byte of
// a pixel to the location of the red color byte.  Bits 3,2 are the
// green offset and 1,0 are the blue offset.  If it is an RGBW-type
// device (supporting a white primary in addition to R,G,B), bits 7,6
// are the offset to the white byte...otherwise, bits 7,6 are set to
// the same value as 5,4 (red) to indicate an RGB (not RGBW) device.
// i.e. binary representation:
// 0bWWRRGGBB for RGBW devices
// 0bRRRRGGBB for RGB

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0))

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#define NEO_KHZ400 0x0100 // 400 KHz datastream

typedef uint16_t neoPixelType;
#define  NUM_PIXELS 1


typedef struct colour{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t padding;
} __attribute__((packed, aligned(4))) colour_RGB;







/** Adafruit_NeoPixel-compatible 'wrapper' for LED control over seesaw
 */
class seesaw_NeoPixel : public Adafruit_seesaw {

public:
  seesaw_NeoPixel(uint16_t n, uint8_t p = 6,
                  neoPixelType t = NEO_GRB + NEO_KHZ800, I2C_HandleTypeDef * x = NULL);
  seesaw_NeoPixel(I2C_HandleTypeDef * x = NULL);
  seesaw_NeoPixel(I2C_HandleTypeDef *x,uint16_t numled,uint8_t pin);
  ~seesaw_NeoPixel();

  bool begin(uint16_t numLEDs=1 ,int8_t flow = -1);
  bool begin();
  void show(void), setPin(uint8_t p),
      setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b),
      setPixelColor(uint16_t n, colour_RGB c), setBrightness(uint8_t), clear(),
      updateLength(uint16_t n), updateType(neoPixelType t);
  colour_RGB *getPixels(void) const, getBrightness(void) const;
  int8_t getPin(void) { return pin; };
  uint16_t numPixels(void) const;
  static colour_RGB Color(uint8_t r, uint8_t g, uint8_t b),
      Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
  colour_RGB getPixelColor(uint16_t n);
  colour_RGB Wheel(byte);
  inline bool canShow(void) { return (micros() - endTime) >= 300L; }

protected:
  void output_stream(void);
  bool is800KHz, // ...true if 800 KHz pixels
      begun;        // true if begin() previously called
  uint16_t numLEDs, // Number of RGB LEDs in strip
      numBytes;     // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
  int8_t pin;
  uint8_t brightness,      // Holds LED color values (3 or 4 bytes each)
          rOffset,      // Index of red byte within each 3- or 4-byte pixel
          gOffset,      // Index of green byte
          bOffset,      // Index of blue byte
          wOffset;      // Index of white byte (same as rOffset if no white)
  uint32_t endTime; // Latch timing reference
  

  colour_RGB * ptr_pixels = NULL;
  uint8_t * ptr_output_buffer = NULL;
  uint16_t type;
  
  
};

#endif // seesaw_NeoPixel_H


#endif /* CC8A05C2_F379_44BB_9A3C_1536186EDAB4 */

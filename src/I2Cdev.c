// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
// 03/28/2017 by Kamnev Yuriy <kamnev.u1969@gmail.com>
//
// Changelog:
//     2017-03-28 - ported to STM32 using Keil MDK Pack

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg
Copyright (c) 2017 Kamnev Yuriy

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"
#include <string.h>
#include "debug_print.h"
#include "stdbool.h"

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t output_buffer[256];
uint8_t read_buffer[256];

// i2c entry point

uint8_t i2c_transmit(uint8_t dev_addr, uint8_t *data, uint8_t len, bool pending)
{
	return HAL_I2C_Master_Transmit(&hi2c1, ((uint16_t)((dev_addr << 1)|0x1) ), data, len, 50); // TODO :this is where it hangs
}

uint8_t i2c_receive(uint8_t dev_addr, uint8_t *data, uint8_t len, bool pending)
{
	return HAL_I2C_Master_Receive(&hi2c1, ((uint16_t)((dev_addr << 1)|0x0) ), data, len, 50);

}

#define i2c_transmit_ack(dev_addr, data, len) i2c_transmit(dev_addr, data, len, true)
#define i2c_transmit_nack(dev_addr, data, len) i2c_transmit(dev_addr, data, len, false)

#define i2c_receive_ack(dev_addr, data, len) i2c_receive(dev_addr, data, len, true)
#define i2c_receive_nack(dev_addr, data, len) i2c_receive(dev_addr, data, len, false)

/** Read several byte from an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param len 		How many bytes to read
 * @param data 		Buffer to save data into
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBytes(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t len, uint8_t *data)
{
	int8_t err = 0;
	uint8_t reg_data[2] = {reg_high, reg_low};

	err = i2c_transmit_ack(dev_addr, reg_data, 2);

	

	HAL_Delay(50);

	err = i2c_receive_nack(dev_addr, data, len);

	HAL_Delay(50);

	return err;
}

/** Read a single byte from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register reg_addr to read from
 * @param data 		Buffer to save data into
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readByte(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low)
{
	return I2Cdev_readBytes(dev_addr, reg_high, reg_low, 1, read_buffer);
}

/** Read a several 16-bit words from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register reg_addr to read from
 * @param len		Number of words to read
 * @param data 		Buffer to save data into
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev_readWords(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t len, uint16_t *data)
{
	int8_t err;
	uint16_t bytes_num = len * 2;

	uint8_t reg_info[2] = {reg_high, reg_low};
	err = i2c_transmit_ack(dev_addr, reg_info, 2);

	if (err < 0)
	{
		return err;
	}

	uint8_t words_in_bytes[bytes_num];
	err = i2c_receive_nack(dev_addr, words_in_bytes, bytes_num);

	if (err < 0)
	{
		return err;
	}

	uint8_t words_cnt = 0;
	for (uint16_t i = 0; i < bytes_num; i += 2)
	{
		data[words_cnt++] = (words_in_bytes[i] << 8) | words_in_bytes[i + 1];
	}

	return 0;
}

/** Read a single word from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param data 		Container for single word
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readWord(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint16_t *data)
{
	return I2Cdev_readWords(dev_addr, reg_high, reg_low, 1, data);
}

/** Read a single bit from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param bitn 		Bit position to read (0-15)
 * @param data 		Container for single bit value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBit(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t bitn, uint8_t *data)
{
	int8_t err;

	err = I2Cdev_readByte(dev_addr, reg_high, reg_low);
	*data = (*data >> bitn) & 0x01;

	return err;
}

/** Read several bits from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param start_bit First bit position to read (0-7)
 * @param len		Number of bits to read (<= 8)
 * @param data 		Container for right-aligned value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBits(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t start_bit,
					   uint8_t len, uint8_t *data)
{
	int8_t err;

	uint8_t b;
	if ((err = I2Cdev_readByte(dev_addr, reg_high, reg_low)) == 0)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		read_buffer[0] &= mask;
		read_buffer[0] >>= (start_bit - len + 1);
		*data = read_buffer[0];
	}

	return err;
}

/** Read a single bit from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param bit_n 	Bit position to read (0-15)
 * @param data 		Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev_readBitW(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t bit_n, uint16_t *data)
{
	int8_t err;

	err = I2Cdev_readWord(dev_addr, reg_high, reg_low, data);
	*data = (*data >> bit_n) & 0x01;

	return err;
}

/** Read several bits from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param start_bit First bit position to read (0-15)
 * @param len		Number of bits to read (<= 16)
 * @param data 		Container for right-aligned value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBitsW(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t start_bit,
						uint8_t len, uint16_t *data)
{
	int8_t err;
	uint16_t w;

	if ((err = I2Cdev_readWord(dev_addr, reg_high, reg_low, &w)) == 0)
	{
		uint16_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		w &= mask;
		w >>= (start_bit - len + 1);
		*data = w;
	}

	return err;
}

/** Write multiple bytes to an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	First register address to write to
 * @param len 		Number of bytes to write
 * @param data 		Buffer to copy new data from
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBytes(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t len)
{
	

	output_buffer[0] = reg_high;
	output_buffer[1] = reg_low;

	for (uint8_t i = 0; i < len; i++)
	{
		output_buffer[i + 2] = *data;
		data++;
	}

	DBG_PRINTF_TRACE("pre -  transmit");

	return	i2c_transmit_nack(dev_addr, output_buffer, len + 2);

}

/** Write single byte to an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register address to write to
 * @param data 		New byte value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeByte(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low)
{

	return I2Cdev_writeBytes(dev_addr, reg_high, reg_low, 1);
}

/** Write single 16-bit word to an 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register address to write to
 * @param data 		New byte value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeWord(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint16_t data)
{

	output_buffer[0] = reg_high;
	output_buffer[1] = reg_low;
	output_buffer[2] = (data >> 8) & 0xFF;
	output_buffer[3] = data & 0xFF;

	return i2c_transmit_nack(dev_addr, output_buffer, 4);
}

/** Write multiple words to a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	First register address to write to
 * @param len 		Number of words to write
 * @param data 		Buffer to copy new data from
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeWords(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t len, uint16_t *data)
{
	uint16_t bytes_num = len * 2 + 2;
	uint8_t bytes[bytes_num];

	bytes[0] = reg_high;
	bytes[1] = reg_low;

	uint16_t bytes_pos = 2;
	for (uint8_t i = 0; i < len; i++)
	{
		bytes[bytes_pos] = (data[i] >> 8) & 0xFF;
		bytes[bytes_pos + 1] = data[i] & 0xFF;

		bytes_pos += 2;
	}

	return i2c_transmit_nack(dev_addr, bytes, bytes_num);
}

/** write a single bit in an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param bit_n 	Bit position to write (0-7)
 * @param data 		New bit value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBit(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t bit_n, uint8_t data)
{
	uint8_t b;
	int8_t err;

	err = I2Cdev_readByte(dev_addr, reg_high, reg_low);
	if (err < 0)
	{
		return err;
	}

	write_buffer[0] = (data != 0) ? (write_buffer[0] | (1 << bit_n)) : (write_buffer[0] &= ~(1 << bit_n));

	return I2Cdev_writeByte(dev_addr, reg_high, reg_low);
}

/** write a single bit in a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param bit_n 	Bit position to write (0-15)
 * @param data 		New bit value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBitW(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t bit_n, uint16_t data)
{
	uint16_t w;
	I2Cdev_readWord(dev_addr, reg_high, reg_low, &w);

	w = (data != 0) ? (w | (1 << bit_n)) : (w &= ~(1 << bit_n));

	return I2Cdev_writeWord(dev_addr, reg_high, reg_low, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param start_bit First bit position to write (0-7)
 * @param len 		Number of bits to write (not more than 8)
 * @param data 		Right-aligned value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBits(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t start_bit,
						uint8_t len, uint8_t data)
{
	uint8_t b;
	int8_t err;

	if ((err = I2Cdev_readByte(dev_addr, reg_high, reg_low, &b)) == 0)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask;					// zero all non-important bits in data
		b &= ~(mask);					// zero all important bits in existing byte
		b |= data;						// combine data with existing byte

		return I2Cdev_writeByte(dev_addr, reg_high, reg_low, b);
	}
	else
	{
		return err;
	}
}

/** Write multiple bits in a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param start_bit First bit position to write (0-15)
 * @param len 		Number of bits to write (not more than 16)
 * @param data 		Right-aligned value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBitsW(uint8_t dev_addr, uint8_t reg_high, uint8_t reg_low, uint8_t start_bit,
						 uint8_t len, uint16_t data)
{
	uint16_t w;
	int8_t err;

	if ((err = I2Cdev_readWord(dev_addr, reg_high, reg_low, &w)) != 0)
	{
		uint16_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask;					// zero all non-important bits in data
		w &= ~(mask);					// zero all important bits in existing word
		w |= data;						// combine data with existing word
		return I2Cdev_writeWord(dev_addr, reg_high, reg_low, w);
	}
	else
	{
		return err;
	}
}
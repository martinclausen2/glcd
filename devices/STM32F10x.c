/**
 * \file STM32F10x.c
 * \brief Device implementation for ST STM32F10x ARM Cortex-M3 MCUs
 *        Requires the use of ST's Standard Peripheral Library
 * \author Andy Gock, Martin Clausen
 */

/*
	Copyright (c) 2012, Andy Gock

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the name of Andy Gock nor the
		  names of its contributors may be used to endorse or promote products
		  derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL ANDY GOCK BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#if defined(GLCD_DEVICE_STM32F10X)
#include "STM32F10x.h"  //our own code
#include "ST7565R.h"
#include "stm32f1xx.h"  //Cube32F generated

extern SPI_HandleTypeDef hspi1;

void glcd_init_device(void)
{
    /* STM32CubeMX generated code must be called elsewhere*/

#if defined(GLCD_CONTROLLER_ST7565R)
	/* Initialization for ST7565R controller */

	/* Make sure chip is de-selected by default */
	GLCD_DESELECT();
	glcd_reset();
	glcd_ST7565R_init();
#else
	#error "Controller not supported by STM32F10x"
#endif

}

void glcd_spi_write(uint8_t c)
{

	GLCD_SELECT();
	//todo: add block write with DMA, use SPI_TIMEOUT_VALUE
	HAL_SPI_Transmit(&hspi1, &c, 1, 10);
	GLCD_SELECT();
}

void glcd_reset(void)
{
	GLCD_SELECT();
	GLCD_RESET_LOW();
	delay_ms(GLCD_RESET_TIME);
	GLCD_RESET_HIGH();
	GLCD_DESELECT();
}

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

#endif

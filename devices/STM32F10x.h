/**
 * \file STM32F10x.h
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
#ifndef STM32F10X_H_
#define STM32F10X_H_

#if defined(GLCD_DEVICE_STM32F10X)

#include <stdint.h>
#include "glcd.h"
#include "stm32f1xx_hal.h"

/** SPI is configured by STM32CubeMX*/

/** Set GPIO pins used to control display*/

#define CONTROLLER_SPI_A0_PIN_Pin GPIO_PIN_6
#define CONTROLLER_SPI_RST_PIN_Pin GPIO_PIN_7
#define CONTROLLER_SPI_SS_PIN_Pin GPIO_PIN_8

#define GLCD_SELECT()       HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_SS_PIN_Pin, GPIO_PIN_RESET)
#define GLCD_DESELECT()     HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_SS_PIN_Pin, GPIO_PIN_SET)
#define GLCD_A0_LOW()       HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_A0_PIN_Pin, GPIO_PIN_RESET)
#define GLCD_A0_HIGH()      HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_A0_PIN_Pin, GPIO_PIN_SET)
#define GLCD_RESET_LOW()    HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_RST_PIN_Pin, GPIO_PIN_RESET)
#define GLCD_RESET_HIGH()   HAL_GPIO_WritePin(GPIOC, CONTROLLER_SPI_RST_PIN_Pin, GPIO_PIN_SET)

#else
	#error "Device not supported by STM32F10X"
#endif

void delay_ms(uint32_t ms);     //will use the HAL_Delay

#endif /* STM32F10X_H_ */

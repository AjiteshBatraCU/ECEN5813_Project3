/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    ECEN5813_Project3.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
//#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"
#include "adc_dma_peripheral.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

//#define ADC16_Rslt_Reg (DEMO_ADC16_BASE->R[DEMO_ADC16_CHANNEL_GROUP])
/*
 * @brief   Application entry point.
 */

/*******************************************************************************

 / * Variables
 / ******************************************************************************/
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

bool destReg_Flag = false;
uint32_t destAddr1[BUFF_LENGTH] = { 0 };
uint32_t destAddr2[BUFF_LENGTH] = { 0 };
uint16_t largest = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/

void DMA0_IRQHandler(void) {
	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[0].DCR &= !(DMA_DCR_EINT(1));	//interrupt

	PrintDMABuffer();
	max_value();
	log_calculation();
	destReg_Flag = !destReg_Flag;
	DMA0_Init();
}

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	LED_Init();

	ADC0_Init();

	DMA0_Init();
	printf("Application data:\n");

	while (1) {
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.*/
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP,
				&adc16ChannelConfigStruct);
#if ADC_PRINT == 1
		while (0U == (kADC16_ChannelConversionDoneFlag
						& ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE,
								DEMO_ADC16_CHANNEL_GROUP))) {
		}
		GPIOD->PTOR = (1<<1);
		PRINTF("ADC Value: %d\r\n",
				ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
						DEMO_ADC16_CHANNEL_GROUP));
#endif
	}
}


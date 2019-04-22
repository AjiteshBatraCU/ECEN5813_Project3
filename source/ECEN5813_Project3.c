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
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /*PTE20, ADC0_SE0 */
/*
 * @brief   Application entry point.
 */

#define BUFF_LENGTH 256
#define DMA_CHANNEL 0
#define DMA_SOURCE 63
/*******************************************************************************

 * Variables
 ******************************************************************************/
dma_handle_t g_DMA_Handle;
volatile bool g_Transfer_Done = false;
/*******************************************************************************
 * Code
 ******************************************************************************/

/* User callback function for DMA transfer. */
void DMA_Callback(dma_handle_t *handle, void *param)
{
    g_Transfer_Done = true;
}

int main(void) {

	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	PRINTF("\r\nADC16 polling Example.\r\n");
	uint32_t srcAddr;
	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	ADC16_GetDefaultConfig(&adc16ConfigStruct);

	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */

	adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;
	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;


	    uint32_t destAddr[BUFF_LENGTH] = {0};
	    uint32_t i = 0;
	    dma_transfer_config_t transferConfig;

	    PRINTF("DMA memory to memory transfer example begin.\r\n\r\n");
	    PRINTF("Destination Buffer:\r\n");
	    for (i = 0; i < BUFF_LENGTH; i++)
	    {
	        PRINTF("%d\t", destAddr[i]);
	    }
	    transferConfig.enableSrcIncrement = false;
	    /* Configure DMAMUX */
	    DMAMUX_Init(DMAMUX0);
	    DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL, DMA_SOURCE);
	    DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL);
	    /* Configure DMA one shot transfer */
	    DMA_Init(DMA0);
	    DMA_CreateHandle(&g_DMA_Handle, DMA0, DMA_CHANNEL);
	    DMA_SetCallback(&g_DMA_Handle, DMA_Callback, NULL);
	    DMA_PrepareTransfer(&transferConfig, srcAddr, 4U, destAddr, 4U, 4U, kDMA_PeripheralToMemory);
	    DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);
	    DMA_StartTransfer(&g_DMA_Handle);
	    /* Wait for DMA transfer finish */
	    while (g_Transfer_Done != true)
	    {
	    }
	    /* Print destination buffer */
	    PRINTF("\r\n\r\nDMA memory to memory transfer example finish.\r\n\r\n");
	    PRINTF("Destination Buffer:\r\n");
	    for (i = 0; i < BUFF_LENGTH; i++)
	    {
	        PRINTF("%d\t", destAddr[i]);
	    }
//	    while (1)
//	    {
//	    }
	while (1) {
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.
		 */
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP,
				&adc16ChannelConfigStruct);
		while (0U
				== (kADC16_ChannelConversionDoneFlag
						& ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE,
								DEMO_ADC16_CHANNEL_GROUP))) {
		}
		PRINTF("ADC Value: %d\r\n",
				ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
						DEMO_ADC16_CHANNEL_GROUP));
	}
}


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
#define ADC16_Rslt_Reg (DEMO_ADC16_BASE->R[DEMO_ADC16_CHANNEL_GROUP])
/*
 * @brief   Application entry point.
 */

#define BUFF_LENGTH 64
#define DMA_CHANNEL 0
//#define DMA_SOURCE ADC16_Rslt_Reg
//kDmaRequestMux0ADC0
#define DECAY_COEFFICIENT 0.9


void PrintDMABuffer(void);
/*******************************************************************************

/ * Variables
/ ******************************************************************************/
dma_handle_t g_DMA_Handle;
bool destReg_Flag = false;
uint32_t destAddr1[BUFF_LENGTH] = {0};
uint32_t destAddr2[BUFF_LENGTH] = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/
void max_value()
{
	uint16_t largest = 0;
	uint16_t current_largest;
	uint16_t decay_value = 0;
	printf("\n");
	for(int i=0; i<4; i++){
	for (int j = i*16; j < (i+1)*16; j++)
	        {
			current_largest = destAddr1[i*16];
			if (current_largest < destAddr1[j])
				current_largest = destAddr1[j];
		}

	printf("%d\t", current_largest);
	if (largest<current_largest)
		printf("nxt: %d\t", current_largest);
			//largest = current_largest;
		else
			printf("nxt: %d\t", DECAY_COEFFICIENT*(largest));
			//decay_value = DECAY_COEFFICIENT*(largest);

}


	//printf("\nMaxest value: %d", largest);
}
void DMA0_IRQHandler(void)
{
	//ADC0->SC1[1] |= ADC_SC1_DIFF(0);
	DMA0->DMA[0].DCR &= !(DMA_DSR_BCR_DONE(1));
	DMA0->DMA[0].DCR &= !(DMA_DCR_EINT(1));	//interrupt
	DMA0->DMA[0].SAR |= (uint32_t)&ADC0->R[0];
	if(destReg_Flag == false)
		DMA0->DMA[0].DAR |= (uint32_t)destAddr2;  //DMA_DAR_DAR(destAddr1); //destination addr
	else if(destReg_Flag ==  true)
		DMA0->DMA[0].DAR |= (uint32_t)destAddr1;
		DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_BCR(BUFF_LENGTH*4); //transfer size
		DMA0->DMA[0].DCR |= DMA_DCR_CS(1);	//run until BCR 0
	PrintDMABuffer();
	max_value();
	DMA0->DMA[0].DCR |= (DMA_DCR_EINT(1));	//interrupt
	DMA0->DMA[0].DCR |= DMA_DCR_ERQ(1);
	destReg_Flag = !destReg_Flag;
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

	/* LED init */
		SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
		GPIOD->PSOR |= (1<<1);
		GPIOD->PDDR |= (1<<1);
		PORTD->PCR[1] |= PORT_PCR_MUX(1);

//	dma_transfer_config_t transferConfig;
//
//	transferConfig.enableSrcIncrement = false;
	/* Configure DMAMUX */
	/*DMAMUX_Init(DMAMUX0);
	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL, 40);
	//DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL);
	DMAMUX0->CHCFG[DMA_CHANNEL] |= DMAMUX_CHCFG_ENBL_MASK;*/

	/* Configure DMA one shot transfer */
	//DMA_Init(DMA0);
	//DMA_CreateHandle(&g_DMA_Handle, DMA0, DMA_CHANNEL);
	//DMA_SetCallback(&g_DMA_Handle, DMA_Callback, NULL);

//	transferConfig.destAddr = destAddr1[0];
//	transferConfig.destSize = kDMA_Transfersize32bits;
//	transferConfig.srcAddr = 0x4003B000u;
//	transferConfig.srcSize = kDMA_Transfersize32bits;
//	transferConfig.transferSize = 256;
//	transferConfig.enableSrcIncrement = false;
//	transferConfig.enableDestIncrement = true;
	//DMA_PrepareTransfer(&transferConfig, ADC16_Rslt_Reg, 4U, destAddr1, 4U, 256, kDMA_PeripheralToMemory);
	//DMA_ResetChannel(DMA0, DMA_CHANNEL);
	//DMA_SetTransferConfig(DMA0, DMA_CHANNEL, &transferConfig);
	//DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);


//	SIM->SCGC6 |= SIM_SCGC6_ADC0(1);
//	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(1);
//	ADC0->CFG1 |= ADC_CFG1_ADICLK(0);
//	ADC0->CFG1 |= ADC_CFG1_MODE(3);
//	ADC0->CFG1 |= ADC_CFG1_ADLSMP(0);
//	ADC0->CFG1 |= ADC_CFG1_ADIV(0);
//
//
//	ADC0->SC2 |= ADC_SC2_REFSEL(0);
	//ADC0->SC2 |= ADC_SC2_ADTRG(0);


	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	adc16ConfigStruct.clockSource = 0U;
	adc16ConfigStruct.enableAsynchronousClock = false;
	adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
	adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;

	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */


	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

	SIM->SCGC6 |= SIM_SCGC6_DMAMUX(1); //enable clk
	DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40);
	DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK;
	SIM->SCGC7 |= SIM_SCGC7_DMA(1);	//enable clk

	//DMA0->DMA[0].SAR |= DMA_SAR_SAR(0x4003B000u); //src addr
	DMA0->DMA[0].SAR |= (uint32_t)&ADC0->R[0];
	DMA0->DMA[0].DAR |= (uint32_t)destAddr1;  //DMA_DAR_DAR(destAddr1); //destination addr
	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_BCR(BUFF_LENGTH*4); //transfer size
	DMA0->DMA[0].DCR |= DMA_DCR_CS(1);	//run until BCR 0
	DMA0->DMA[0].DCR |= DMA_DCR_DINC(1);	//dest inc yes
	DMA0->DMA[0].DCR |= DMA_DCR_SINC(0);	//src incr no
	DMA0->DMA[0].DCR |= DMA_DCR_DSIZE(0); //size 32bits
	DMA0->DMA[0].DCR |= DMA_DCR_SSIZE(0);


	PRINTF("Destination Buffer @ Start:\r\n");
//	for (uint32_t i = 0; i < BUFF_LENGTH; i++)
//	{
//		PRINTF("%d. %d\t", i, destAddr1[i]);
//	}

	//uint32_t srcAddr;
	//srcAddr = ADC_R_D();
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



	DMA0->DMA[0].DCR |= DMA_DCR_ERQ(1);	//peripheral req yes
	ADC0->SC2 |= ADC_SC2_DMAEN(1);
	DMA0->DMA[0].DCR |= DMA_DCR_EINT(1);	//interrupt
	NVIC_EnableIRQ(DMA0_IRQn);
	DMA0->DMA[0].DCR |= DMA_DCR_START_MASK;	//DMA_DCR_START(1);


	while (1) {
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.*/

		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//		while (0U == (kADC16_ChannelConversionDoneFlag
//						& ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE,
//								DEMO_ADC16_CHANNEL_GROUP))) {
//		}
//		ADC0->SC2 |= ADC_SC2_ADTRG(0);
//		ADC0->SC1[0] &= ADC_SC1_ADCH(0);
//		while (0U == (ADC_SC1_COCO_MASK & ADC0->SC1[0])) {
//				}
//		GPIOD->PTOR = (1<<1);
//		PRINTF("ADC Value: %d\r\n",
//				//ADC0->R[2]);
//				ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
//						DEMO_ADC16_CHANNEL_GROUP));
//		ADC0->CFG1 |= ADC_CFG1_ADIV(0);
	}
}

void PrintDMABuffer(void)
{
    /* Print destination buffer */
    PRINTF("\r\n\r\nDMA memory to memory transfer finish.\r\n\r\n");
    PRINTF("Destination Buffer:\r\n");
    if(destReg_Flag == false)
    {
    for (int i = 0; i < BUFF_LENGTH; i++)
    {
        PRINTF("%d. %d\t",i, destAddr1[i]);
    }
    }
    if(destReg_Flag == true)
        {
        for (int i = 0; i < BUFF_LENGTH; i++)
        {
            PRINTF("%d. %d\t",i, destAddr2[i]);
        }
        }
}


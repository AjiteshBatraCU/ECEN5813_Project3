/*
 * adc_dma_peripheral.c
 *
 *  Created on: Apr 29, 2019
 *      Author: Ajitesh
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
#include "adc_dma_peripheral.h"

extern adc16_config_t adc16ConfigStruct;
extern adc16_channel_config_t adc16ChannelConfigStruct;

extern bool destReg_Flag;
extern uint32_t destAddr1[BUFF_LENGTH];
extern uint32_t destAddr2[BUFF_LENGTH];
extern uint16_t largest;

void PrintDMABuffer(void) {
	/* Print destination buffer */
	PRINTF("\r\n\r\nDMA peripheral to memory transfer finish.\r\n\r\n");
	PRINTF("Destination Buffer:\r\n");
	if (destReg_Flag == false) {
		for (int i = 0; i < BUFF_LENGTH; i++) {
			PRINTF("%d. %d\t", i, destAddr1[i]);
		}
	}
	if (destReg_Flag == true) {
		for (int i = 0; i < BUFF_LENGTH; i++) {
			PRINTF("%d. %d\t", i, destAddr2[i]);
		}
	}
}

void DMA0_Init() {
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX(1); //enable clk
	DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40);
	DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK;
	SIM->SCGC7 |= SIM_SCGC7_DMA(1);	//enable clk

	//DMA0->DMA[0].SAR |= DMA_SAR_SAR(0x4003B000u); //source address
	DMA0->DMA[0].SAR |= (uint32_t) &ADC0->R[0];
	if (destReg_Flag == false)
		DMA0->DMA[0].DAR = (uint32_t) destAddr1; //DMA_DAR_DAR(destAddr1); //destination address
	else if (destReg_Flag == true)
		DMA0->DMA[0].DAR = (uint32_t) destAddr2;
	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_BCR(BUFF_LENGTH*4); //transfer size
	DMA0->DMA[0].DCR |= DMA_DCR_CS(1);	//run until BCR 0
	DMA0->DMA[0].DCR |= DMA_DCR_DINC(1);	//dest inc yes
	DMA0->DMA[0].DCR |= DMA_DCR_SINC(0);	//src incr no
	DMA0->DMA[0].DCR |= DMA_DCR_DSIZE(0); //size 32bits
	DMA0->DMA[0].DCR |= DMA_DCR_SSIZE(0);

#if DMA_PRINT == 1
	PRINTF("Destination Buffer @ Start:\r\n");
	for (uint32_t i = 0; i < BUFF_LENGTH; i++)
	{
		PRINTF("%d. %d\t", i, destAddr1[i]);
	}
#endif
	DMA0->DMA[0].DCR |= DMA_DCR_ERQ(1);	//enable peripheral request
	ADC0->SC2 |= ADC_SC2_DMAEN(1);		//enable DMA request for ADC
	DMA0->DMA[0].DCR |= DMA_DCR_EINT(1);	//interrupt
	NVIC_EnableIRQ(DMA0_IRQn);
}

void LED_Init() {
	/* LED init */
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	GPIOD->PSOR |= (1 << 1);
	GPIOD->PDDR |= (1 << 1);
	PORTD->PCR[1] |= PORT_PCR_MUX(1);
}

void ADC0_Init() {
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
	adc16ConfigStruct.clockSource = 0U;
	adc16ConfigStruct.enableAsynchronousClock = false;
	adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
	adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;

	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
}

void max_value() {
	/*Find maximum value*/
	uint16_t current_largest;
	for (int i = 0; i < BUFF_LENGTH; i++) {
		if (destReg_Flag == false) {
			current_largest = destAddr1[0];
			if (current_largest < destAddr1[i])
				current_largest = destAddr1[i];
		} else {
			current_largest = destAddr2[0];
			if (current_largest < destAddr2[i])
				current_largest = destAddr2[i];
		}
	}

	/*Print new maximum value or decay value*/
	if (largest < current_largest) {
		largest = current_largest;
		printf("%d\t", current_largest);
	} else {
		largest = DECAY_COEFFICIENT * (largest);
		printf("%d\t", largest);
	}
}

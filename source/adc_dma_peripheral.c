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
#if DMA_PRINT == 1
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
#endif
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

#if DMA_INIT_PRINT == 1
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
#if LOG_CALC == 0
		printf("%d\t", largest);
#endif
	} else {
		largest = DECAY_COEFFICIENT * (largest);
#if LOG_CALC == 0
		printf("%d\t", largest);
#endif
	}
	log_calculation();
}

void log_calculation() {
	struct log_lookup {
		int adc_value;
		int log_dBFS;
	};

	const struct log_lookup log_values[] = { { 992, -36 }, { 1985, -30 }, {
			2978, -26 }, { 3971, -24 }, { 4964, -22 }, { 5957, -20 }, { 6950,
			-19 }, { 7943, -18 }, { 8936, -17 }, { 9929, -16 }, { 10922, -15 },
			{ 11915, -14 }, { 12908, -14 }, { 13901, -13 }, { 14894, -12 }, {
					15887, -12 }, { 16880, -11 }, { 17873, -11 },
			{ 18866, -10 }, { 19859, -10 }, { 20852, -9 }, { 21845, -9 }, {
					22838, -9 }, { 23831, -8 }, { 24824, -8 }, { 25817, -8 }, {
					26810, -7 }, { 27803, -7 }, { 28796, -7 }, { 29789, -6 }, {
					30782, -6 }, { 31775, -6 }, { 32768, -6 }, { 33760, -5 }, {
					34753, -5 }, { 35746, -5 }, { 36739, -5 }, { 37732, -4 }, {
					38725, -4 }, { 39718, -4 }, { 40711, -4 }, { 41704, -3 }, {
					42697, -3 }, { 43690, -3 }, { 44683, -3 }, { 45676, -3 }, {
					46669, -2 }, { 47662, -2 }, { 48655, -2 }, { 49648, -2 }, {
					50641, -2 }, { 51634, -2 }, { 52627, -1 }, { 53620, -1 }, {
					54613, -1 }, { 55606, -1 }, { 56599, -1 }, { 57592, -1 }, {
					58585, 0 }, { 59578, 0 }, { 60571, 0 }, { 61564, 0 }, {
					62557, 0 }, { 63550, 0 }, { 64543, 0 }, { 65536, 0 } };
#if LOG_CALC == 1
	if (largest == 0)
	printf("%d : undefined", largest);
	else
	for (int i = 0; i < 66; i++)
	{
		if (largest < log_values[i].adc_value)
		{
			printf("%largest : %ddBFS\n", largest, log_values[i].log_dBFS);
			break;
		}
	}
#endif
}


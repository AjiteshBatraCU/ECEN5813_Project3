/*
 * adc_dma_peripheral.h
 *
 *  Created on: Apr 29, 2019
 *      Author: Ajitesh
 */

#ifndef ADC_DMA_PERIPHERAL_H_
#define ADC_DMA_PERIPHERAL_H_

#define ADC_PRINT 0
#define DMA_INIT_PRINT 0
#define DMA_PRINT 1
#define LOG_CALC 1

#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /*PTE20, ADC0_SE0 */

#define BUFF_LENGTH 1024

#define DECAY_COEFFICIENT 0.9

void ADC0_Init();
void DMA0_Init();
void LED_Init();
void PrintDMABuffer(void);
void max_value();
void log_calculation();

#endif /* ADC_DMA_PERIPHERAL_H_ */

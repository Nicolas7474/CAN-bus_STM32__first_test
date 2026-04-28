/* Bare-metal Test with ADC1
 * two 10K Pots ch. 9 & 19 with DMA and chip temp. sensor, Vbat, VRefInt ch. 17 & 18 Triggered by TIM2 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "myFirst_adc.h"


void ADC1_Base_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     		  // Enable ADC1 Clock
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;  // Set Prescaler to /8 (Common for all ADC1/2/3)
    ADC1->CR2 |= ADC_CR2_ADON;    // Enable ADC1 Power
    ADC1->CR1 |= ADC_CR1_SCAN;    // Enable Scan Mode (Required if you ever use more than 1 channel)
}

void ADC1_InternalSensors_Init(void) {

    ADC->CCR |= ADC_CCR_TSVREFE; // Enable the internal temperature/Vref connections
    ADC1->JSQR = (1 << ADC_JSQR_JL_Pos) | (18 << 10) | (17 << 15);  // Configure Injected Sequence: 2 conversions (JSQ3 = Channel 18 (Temp), JSQ4 = Channel 17 (Vref))
    ADC1->SMPR1 |= (7 << 24) | (7 << 21); // Set long sampling time for internal sensors (very high Z) (Ch 18 and 17 -> 480 cycles)
    ADC1->CR2 |= (1 << 20) | (3 << 16); //Setup hardware trigger (TIM2) or leave for Software Trigger (JEXTEN rising edge, JEXTSEL = TIM2_TRGO)
}

void ADC1_Potentiometers_DMA_Init(uint32_t srcAdd, uint32_t destAdd, uint8_t size) {
    // 1. Enable GPIO Clocks and DMA2 Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;

    // 2. Configure Pins to Analog (PB1, PC2)
    GPIOB->MODER |= (3 << GPIO_MODER_MODER1_Pos);
    GPIOC->MODER |= (3 << GPIO_MODER_MODER2_Pos);

    // 3. Configure Regular Sequence: 2 conversions (Ch 9 then Ch 12)
    ADC1->SQR1 |= (1 << ADC_SQR1_L_Pos);
    ADC1->SQR3 &= ~(0x3FFFFFFF); // Clear whole 1st 30bits in register
    ADC1->SQR3 = (9 << 0) | (12 << 5);
    ADC1->SMPR1 |= (7 << 6);  // Ch 12 -> 480 cycles
    ADC1->SMPR2 |= (7 << 27); // Ch 9 -> 480 cycles

    // 4. Configure DMA2 Stream 0
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream0->PAR  = srcAdd;
    DMA2_Stream0->M0AR = destAdd;
    DMA2_Stream0->NDTR = size; // Two pots
    // CIRC, MINC, PSIZE=16bit, MSIZE=16bit, Channel 0
    DMA2_Stream0->CR = DMA_SxCR_CIRC | DMA_SxCR_MINC | (1 << 13) | (1 << 11) | DMA_SxCR_EN;

    // 5. Link ADC to DMA and Start
	ADC1->SR = 0; // clear the status register
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion of regular channels
}


void ADC_IRQHandler()
{
	// Check if Overrun occurred on ADC1
	if (ADC1->SR & ADC_SR_OVR) {
		 GPIOG->ODR^=GPIO_ODR_OD6;

		// Disable and Re-Enable the ADC to prevent Data Corruption
		ADC1->CR2 &= ~(1<<0);   // ADON =0 Disable ADC1
		ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1

		/* To start the DMA again, we need to update the NDTR Counter and also the Interrupt Flags must be cleared
	    NDTR can only be Updated while the DMA is Disabled
	    */
		DMA2_Stream0->CR &= ~(1<<0); // Disable the DMA2
		DMA2->LIFCR = 0xffffffff; 	// Clear the Interrupt pending flags. This is important before restarting the DMA
		DMA2->HIFCR = 0xffffffff;
		DMA2_Stream0->NDTR = 2; // Set the data size in NDTR Register
		DMA2_Stream0->CR |= 1<<0; // Enable the DMA2
		ADC1->CR2 |= (1<<30);  // start the ADC again
		ADC1->SR &= ~ADC_SR_OVR; 	// Clear the flag by writing 0 to it
	}
}



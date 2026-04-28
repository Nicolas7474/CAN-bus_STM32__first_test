#include "stm32f469xx.h"
#include "stm32f4xx.h"
#include "timers.h"


unsigned int countWakeUp = 0;  // extern
int flagmsTicks = 0;  // extern
volatile uint32_t msTicks = 0; // Volatile ensures the compiler doesn't optimize out reads of this value

// Records the starting tick count and waits until the required nb of ms has passed. It’s non-blocking, the CPU can still handle interrupts while waiting
void NBdelay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {}
}


//void RTC_WKUP_IRQHandler()
//{
//    EXTI->PR = (1U<<22);// This bit is set when the selected edge event arrives on the external interrupt line. This bit is cleared by programming it to ‘1’.
//    if((RTC->ISR & RTC_ISR_WUTF)!=0) // (1U<<10 WUTF: this flag is set by hardware when the wakeup auto-reload counter reaches 0.
//    {									// 1: Wakeup timer configuration update allowed
//    	GPIOK->ODR ^= GPIO_ODR_OD3; //toggle PK3 (bleu)
//    	RTC->ISR &= ~RTC_ISR_WUTF; // (0U<<10); this flag is cleared by software by writing 0
//    	countWakeUp++;
//    }
//}

#include <stm32f4xx.h>
#include "stm32f4xx_tim.h"
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_syscfg.h"
#include "misc.h"

struct dmp_clock_s {

    volatile uint32_t timestamp;
    unsigned short ms_per_interrupt;
    unsigned char enabled;
    unsigned short ticks_per_interrupt;
    unsigned long timer_remaining_ms;
    void (*timer_cb)(void);
};

static struct dmp_clock_s clock = {
    .enabled = 0,
    .timer_remaining_ms = 0,
    .timer_cb = 0
};
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		clock.timestamp += clock.ms_per_interrupt;
		//printf("%d\n", clock.timestamp);
		if (clock.timer_remaining_ms) {
			clock.timer_remaining_ms -= clock.ms_per_interrupt;
			if (!clock.timer_remaining_ms)
				clock.timer_cb();
		}
	}
}
int clock_enable(){

	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	/* Enable clock for TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = 4200;
	/* Count up */
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_BaseStruct.TIM_Period = 10; /* 10kHz PWM */
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM4 */
	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	/* Start count on TIM4 */

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM4, ENABLE);
	clock.ticks_per_interrupt = 10;
    clock.ms_per_interrupt = 1;
    clock.timer_remaining_ms += (clock.timer_remaining_ms % clock.ms_per_interrupt);
    clock.enabled = 1;

	return 0;
}

int clock_init(){
	clock_enable();

    /* Start timestamp at zero. */
    clock.timestamp = 0;
    clock.timer_cb = 0;
    clock.timer_remaining_ms = 0;
    return 0;
}

int clock_disable(){
	TIM_DeInit(TIM4);
	return 0;
}

int get_clock_ms(unsigned long *count){
    if (!count)
        return 1;
    count[0] = clock.timestamp;
    return 0;
}

int delay_ms(unsigned long num_ms)

{
    uint32_t start_time = clock.timestamp;

    if (!clock.enabled)
        return -1;

    while (clock.timestamp - start_time < num_ms);
    return 0;
}

int register_timer_cb(void (*timer_cb)(void), unsigned long num_ms)
{
    if (!timer_cb || !num_ms) {
        clock.timer_cb = 0;
        clock.timer_remaining_ms = 0;
        return 0;
    }

    /* Timer count needs to be evenly divisible by clock.ms_per_interrupt to
     * avoid overflow.
     */
    clock.timer_remaining_ms = num_ms + (clock.timer_remaining_ms % clock.ms_per_interrupt);
    clock.timer_cb = timer_cb;

    return 0;
}

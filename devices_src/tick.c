/*
 * tick.c
 *
 *  Created on: 8. sep. 2018
 *      Author: mvols
 */

#include "../devices_header/tick.h"

volatile uint64_t msTicks;

void (*user_callback)(void) = NULL;
uint64_t callback_period = 0;
uint64_t callback_timestamp = 0;

void SysTick_Handler(void)
{
	msTicks++;

    if (user_callback)
    {
        if ((msTicks - callback_timestamp) > callback_period)
        {
            callback_timestamp = msTicks;
            user_callback();
        }
    }
}

void tick_init(void)
{
	SystemCoreClockUpdate();
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1) ;
}

void tick_deinit(void)
{

}

void tick_register_callback(void (*user_cb)(void), uint64_t period)
{
    user_callback = user_cb;
    callback_period = period;
    callback_timestamp = tick_get();
}

uint64_t tick_get(void)
{
	return msTicks;
}

void tick_delay(uint32_t ticks)
{
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < ticks) ;
}

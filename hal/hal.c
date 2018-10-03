/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/
#include "../lmic/hal.h"
#include "../lmic/lmic.h"
#include "debug.h"


#define N_SAMPLES 	4
#define BASE_2_N 	16		//-1 done inside if...

// HAL state
static struct
{
    //int irqlevel;
    uint64_t ticks;
} HAL;
//////////////////////////////////////////////////////////////
static	uint32_t	one_sec_top_ref=32768;
static	bool		letimer_running=false;
static 	int			last_letimer_count=65535;
static	uint16_t	average_n=0;
static 	uint32_t	avergae_sum=0;
static 	uint32_t	ref_count=0;

extern void debug_function(void);
//////////////////////////////////////////////////////////////

static time_manager_cmd_t 		time_manager_cmd=basic_sync;
static int 						time_count=0;
char							temp_buf[128];

void BURTC_IRQHandler(void)
{
	uint32_t	int_mask=BURTC_IntGet();
	if(int_mask & BURTC_IF_COMP0){

		time_count++;
		 if(time_count==ADVANCE_SYNCH_SECONDS){
			 time_manager_cmd=advance_sync;
			 	 //run time controller on GPS timestamp...
			 if(running_tstamp.valid==true){
				 diff_in_tstamp=(int)((uint32_t)(running_tstamp.gps_timestamp)-(uint32_t)(ref_tstamp.gps_timestamp));
				 if(diff_in_tstamp>=10){
					 ref_tstamp.gps_timestamp+=10;
				 }
					 time_count=diff_in_tstamp;
			 }
			 else {
				 time_count=0;
			 }
			 time_count=0;
				//wakeup
			 debug_function();

		 }
		 else if(time_count%(BASIC_SYNCH_SECONDS)==0 && time_count!=0 ){
			 time_manager_cmd=basic_sync;
		 	 	 //update clock...
			 if(one_sec_top_ref>32000 && one_sec_top_ref<33000){
				 BURTC_CompareSet(0,one_sec_top_ref);
			 }
			 	 //wakeup
			 debug_function();

		 }
		 else {
			 ;
		 }

	}
	BURTC_IntClear(int_mask);
}
void 		time_manager_init(void){

				/////////////GPS PPS and INT pins////////////
	GPIO_PinModeSet(GPS_SIG_PORT, GPS_TIME_PULSE, gpioModeInput, 0);
	GPIO_IntConfig(GPS_SIG_PORT,GPS_TIME_PULSE,true,false,true);
	GPIO_IntClear(_GPIO_IF_MASK);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    			/////////////BURTC and LETIMER////////////
    	//Setup and initialize BURTC
    BURTC_Init_TypeDef	burtc_init=BURTC_INIT_DEFAULT;
    burtc_init.enable=false;
    burtc_init.clkSel=burtcClkSelLFXO ;
    burtc_init.compare0Top=true;
    burtc_init.mode=burtcModeEM4;
    BURTC_Reset();
    BURTC_Init(&burtc_init);
    BURTC_CompareSet(0,one_sec_top_ref);
    BURTC_IntDisable(_BURTC_IF_MASK);
    BURTC_IntEnable(BURTC_IF_COMP0);
	NVIC_ClearPendingIRQ(BURTC_IRQn);
	NVIC_EnableIRQ(BURTC_IRQn);
		//Setup and initialize LETIMER
    LETIMER_Init_TypeDef	letimer_init=LETIMER_INIT_DEFAULT;
    letimer_init.enable=false;
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	LETIMER_Reset(LETIMER0);
	LETIMER_Init(LETIMER0,&letimer_init);
		//start BURTC
		BURTC_Enable(true);
    return;
}

unsigned long 		time_manager_unixTimestamp(int year, int month, int day,
              int hour, int min, int sec)
{
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};

  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;

  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;

  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */

  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}

time_manager_cmd_t 		time_manager_get_cmd(void){
	return time_manager_cmd;
}
//////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
// I/O
extern void radio_irq_handler(u1_t dio);

void GPIO_EVEN_IRQHandler()	//impar
 {
	u4_t int_mask = GPIO_IntGetEnabled();
	GPIO_IntClear(int_mask);
	if (int_mask & 1<<RADIO_IO_0){
		radio_irq_handler(0);
	}
	else if (int_mask & 1<<RADIO_IO_2){
		radio_irq_handler(2);
	}
	else if (int_mask & 1<<GPS_TIME_PULSE){
		if(letimer_running){
			LETIMER_Enable(LETIMER0,false);
			if(LETIMER_CounterGet(LETIMER0)>last_letimer_count){
				avergae_sum+=(uint32_t)(LETIMER_CounterGet(LETIMER0)-last_letimer_count);
			}
			else{
				avergae_sum+=(uint32_t)(last_letimer_count-LETIMER_CounterGet(LETIMER0));
			}
			last_letimer_count=LETIMER_CounterGet(LETIMER0);
			letimer_running=false;
			average_n++;
			if(average_n>BASE_2_N-1){
				one_sec_top_ref=avergae_sum>>N_SAMPLES;
				avergae_sum=0;
				average_n=0;
			}
		}
		else{
			LETIMER_Enable(LETIMER0,true);
			letimer_running=true;
		}
		ref_count++;
	}
	else{
		;
	}
	return;
 }

void GPIO_ODD_IRQHandler()	//par
 {
	u4_t int_mask = GPIO_IntGetEnabled();
	GPIO_IntClear(int_mask);
	if (int_mask & 1<<RADIO_IO_1){
		radio_irq_handler(1);
	}
	return;
 }

static void hal_io_init ()
{

	spi_cs_set(radio);

	GPIO_PinModeSet(RADIO_IO_0345_PORT, RADIO_IO_0, gpioModeInput, 0);	//DIO0=PayLoadReady
	GPIO_PinModeSet(RADIO_IO_12_PORT, RADIO_IO_1, gpioModeInput, 0);	//DIO1=FifoLevel
	GPIO_PinModeSet(RADIO_IO_12_PORT, RADIO_IO_2, gpioModeInput, 0);	//DIO2=SyncAddr
	GPIO_PinModeSet(RADIO_IO_0345_PORT, RADIO_IO_3, gpioModeInput, 0);	//DIO3=FifoEmpty
	GPIO_PinModeSet(RADIO_IO_0345_PORT, RADIO_IO_4, gpioModeInput, 0);	//DIO4=PreambleDetect/RSSI
	GPIO_PinModeSet(RADIO_IO_0345_PORT, RADIO_IO_5, gpioModeInput, 0);	//DIO5=ModeReady

	GPIO_IntConfig(RADIO_IO_0345_PORT, RADIO_IO_0, true, false, true);
	GPIO_IntConfig(RADIO_IO_12_PORT, RADIO_IO_1, true, false, true);
	GPIO_IntConfig(RADIO_IO_12_PORT, RADIO_IO_2, true, false, true);

	GPIO_IntClear(_GPIO_IF_MASK);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);

	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void hal_pin_rxtx (u1_t val)
{

}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val)
{
	if (val)
		spi_cs_set(radio);
	else
		spi_cs_clear(radio);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val)
{
	return;
}

static void hal_spi_init ()
{

	spi_init();
	spi_enable();
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out)
{
	u1_t	ret_val=0;
	ret_val=spi_read_write_byte(out);
	return ret_val;
}


// -----------------------------------------------------------------------------
// TIME
static uint8_t       rtcInitialized = 0;    /**< 1 if rtc is initialized */
static uint32_t      rtcFreq;               /**< RTC Frequence. 32.768 kHz */

/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback function if defined.
 ******************************************************************************/
void RTC_IRQHandler(void)
{
	//debug_str("\tRTC IRQ");
	if (RTC_IntGet() & RTC_IF_OF)
	{
		HAL.ticks ++;
	}

    if(RTC_IntGet() & RTC_IF_COMP0) // expired
    {
        // do nothing, only wake up cpu
    }
	RTC_IntClear(_RTC_IF_MASK); // clear IRQ flags
}


static void hal_time_init ()
{
	RTC_Init_TypeDef init;

	rtcInitialized = 1;

	/* Ensure LE modules are accessible */
	CMU_ClockEnable(cmuClock_CORELE, true);

	/* Enable LFACLK in CMU (will also enable oscillator if not enabled) */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	/* Use the prescaler to reduce power consumption. */
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

	rtcFreq = CMU_ClockFreqGet(cmuClock_RTC);

	/* Enable clock to RTC module */
	CMU_ClockEnable(cmuClock_RTC, true);

	init.enable   = false;
	init.debugRun = false;
	init.comp0Top = false;
	//init.comp0Top = true; /* Count to max before wrapping */
	RTC_Init(&init);

	/* Disable interrupt generation from RTC0 */
	RTC_IntDisable(_RTC_IF_MASK);
	RTC_IntEnable(RTC_IF_OF);	//Enable interrupt on overflow

	/* Enable interrupts */
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Enable RTC */
	RTC_Enable(true);
}

u4_t hal_ticks ()
{
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u4_t cnt = RTC_CounterGet();
	if (RTC_IntGet() & RTC_IF_OF)	// Overflow before we read CNT?
	{
        cnt = RTC_CounterGet();
        t ++;	// Include overflow in evaluation but leave update of state to ISR once interrupts enabled again
    }
    hal_enableIRQs();
	return (t<<24)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time)
{
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time)
{
	 while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time)
{
    u2_t dt;
	RTC_IntClear(RTC_IF_COMP0);		//clear any pending interrupts
    if((dt = deltaticks(time)) < 5) // event is now (a few ticks ahead)
    {
    	RTC_IntDisable(RTC_IF_COMP0);	// disable IE
        return 1;
    }
    else // rewind timer (fully or to exact time))
    {
    	RTC_CompareSet(0, RTC_CounterGet() + dt);   // set comparator
    	RTC_IntEnable(RTC_IF_COMP0);  // enable IE
        return 0;
    }
}



// -----------------------------------------------------------------------------
// IRQ
static uint8_t irqlevel = 0;
static bool int_flag=false;

CORE_DECLARE_IRQ_STATE;
void hal_disableIRQs ()
{
	if(int_flag==false){
		int_flag=true;
		CORE_ENTER_ATOMIC();
	}
	 irqlevel++;

	return;

}

void hal_enableIRQs ()
{


	if(--irqlevel == 0) {
		CORE_EXIT_ATOMIC();
		int_flag=false;
	}
	return;
}

void hal_sleep ()
{
	EMU_EnterEM1();
}

// -----------------------------------------------------------------------------

void hal_init ()
{

    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

	hal_io_init();	// configure radio I/O and interrupt handler

    hal_spi_init();	// configure radio SPI

    hal_time_init();	// configure timer and interrupt handler

    hal_enableIRQs();
 }

void hal_failed ()
{
	//rgb_on(true,false,false);
	CORE_EXIT_ATOMIC();
	debug_str("Failed");
	spi_cs_clear(radio);
	delay_ms(9);
	while(1){
	spi_write_byte(0x81);
	spi_write_byte(0x80);
	//delay_ms(1);
	spi_write_byte(0x01);
	if(spi_read_byte()==0x80){debug_str("OK");break;};
	delay_ms(9);
	}
	spi_cs_set(radio);
	NVIC_SystemReset();
	// HALT...
    hal_sleep();
    while(1);
}


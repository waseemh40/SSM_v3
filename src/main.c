//
//
//#include "../resource managers_header/lpwan_manager.h"
//#include "../resource managers_header/app_manager.h"
//#include "em_dma.h"
//#include "em_adc.h"
//
///*
//	 * Shared variables
//	 */
//#ifdef USE_RADIO
//uint8_t 		lora_buffer[512];
//uint8_t			lora_msg_length=0;
//#else
//nav_data_t	 	running_tstamp;
//nav_data_t	 	ref_tstamp;
//#endif
//
///* DMA control block, must be aligned to 256. */
//SL_ALIGN(256)
//DMA_DESCRIPTOR_TypeDef dmaControlBlock[2] SL_ATTRIBUTE_ALIGN(256);
//
///* DMA callback structure */
//static DMA_CB_TypeDef cb[2];
//
//#include "em_usb.h"
//#include "cdc.h"
//#include "descriptors.h"
//
//int SetupCmd(const USB_Setup_TypeDef *setup);
//void StateChangeEvent(USBD_State_TypeDef oldState,
//                      USBD_State_TypeDef newState);
//
//static const USBD_Callbacks_TypeDef callbacks =
//{
//  .usbReset        = NULL,
//  .usbStateChange  = StateChangeEvent,
//  .setupCmd        = SetupCmd,
//  .isSelfPowered   = NULL,
//  .sofInt          = NULL
//};
//
//const USBD_Init_TypeDef usbInitStruct =
//{
//  .deviceDescriptor    = &USBDESC_deviceDesc,
//  .configDescriptor    = USBDESC_configDesc,
//  .stringDescriptors   = USBDESC_strings,
//  .numberOfStrings     = sizeof(USBDESC_strings) / sizeof(void*),
//  .callbacks           = &callbacks,
//  .bufferingMultiplier = USBDESC_bufferingMultiplier,
//  .reserved            = 0
//};
//
//#include "../devices_header/display.h"
//#include "../devices_header/mode_switch.h"
//#include "../devices_header/tick.h"
//#include "../devices_header/ublox_gps.h"
//
//
//#include <string.h>
//
// /* reverse:  reverse string s in place */
// void reverse(char s[])
// {
//     int i, j;
//     char c;
//
//     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
//         c = s[i];
//         s[i] = s[j];
//         s[j] = c;
//     }
// }
//
// void itoa(int n, char s[])
//  {
//      int i, sign;
//
//      if ((sign = n) < 0)  /* record sign */
//          n = -n;          /* make n positive */
//      i = 0;
//      do {       /* generate digits in reverse order */
//          s[i++] = n % 10 + '0';   /* get next digit */
//      } while ((n /= 10) > 0);     /* delete it */
//      if (sign < 0)
//          s[i++] = '-';
//      s[i] = '\0';
//      reverse(s);
//  }
//
//uint8_t cdc_buffer[512];
//uint32_t cdc_length = 0;
//uint8_t mode = 0;
//char buffer[10] = "Mode: ";
//uint8_t fix = 0;
//char gps_buf[10] = "GPS: ";
//
//uint64_t prevTick = 0;
//uint64_t prevUpdate = 0;
//void user_function(void)
//{
//	static uint64_t state_change_timestamp = 0;
//	static uint32_t state = 0;
//	static nav_pvt_t* nav_pvt_data;
//	static nav_sat_t* nav_sat_data;
//
//	// Only perform action every 100ms
//	if (tick_get() - prevTick < 100)
//		return;
//
//	// Toggle LED
//	GPIO_PinOutToggle(gpioPortE, 9);
//
//	switch (state)
//	{
//	case 0: // Init
//		display_dummy();
//
//		if ((tick_get()-state_change_timestamp) > 60000)
//		{
//			state = 1;
//			state_change_timestamp = tick_get();
//		}
//		break;
//	case 1: // Request GPS
//	{
//		// Empty navigation data
//		gps_get_navigation_data(&nav_pvt_data);
//		gps_get_navsat_data(&nav_sat_data);
//		// Request new navigation data
//		gps_send_cmd(NAV, PVT, 0, NULL, false);
//		gps_send_cmd(NAV, SAT, 0, NULL, false);
//
//		state = 2;
//		state_change_timestamp = tick_get();
//
//		break;
//	}
//	case 2:	// Check for GPS response
//	{
//		gps_read_and_parse();
//		uint64_t timestamp = gps_get_navigation_data(&nav_pvt_data);
//
//		if (timestamp != 0)
//		{
//			state = 3;
//			state_change_timestamp = tick_get();
//		} else if ((tick_get() - state_change_timestamp) > 6000)
//		{
//			state = 0;
//			state_change_timestamp = tick_get();
//		}
//		break;
//	}
//	case 3: // Check for GPS sat response
//	{
//		gps_read_and_parse();
//		uint64_t timestamp = gps_get_navsat_data(&nav_sat_data);
//
//		if (timestamp != 0)
//		{
//			state = 4;
//			state_change_timestamp = tick_get();
//		} else if ((tick_get() - state_change_timestamp) > 6000)
//		{
//			state = 0;
//			state_change_timestamp = tick_get();
//		}
//		break;
//	}
//	case 4: // Update display
//	{
//		display_clear();
//		display_put_string(3, 3, "NTNU SSM", font_large);
//
//		mode = mode_switch_read();
//		itoa(mode, &buffer[6]);
//		display_put_string(3, 3+(16+3)*1, buffer, font_medium);
//
//		uint8_t max_sat_cno = 0;
//		for (int i = 0; i < nav_sat_data->numSvs; i++)
//			if (nav_sat_data->nav_sat_satellites[i].cno > max_sat_cno)
//				max_sat_cno = nav_sat_data->nav_sat_satellites[i].cno;
//
//		fix = nav_pvt_data->fixType;
//
//		switch (fix)
//		{
//		case 1:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: DeadReck", font_medium);
//			break;
//		case 2:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: 2D-fix", font_medium);
//			break;
//		case 3:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: 3D-fix", font_medium);
//			break;
//		case 4:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: GNSSDeadR", font_medium);
//			break;
//		case 5:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: Time Only", font_medium);
//			break;
//		case 0:
//		default:
//			display_put_string(3, 3+(16+3)*1 + (12+3)*1, "GPS: No Fix", font_medium);
//			break;
//		}
//
//
//		char lat_buf[100] = "Lat: ";
//		itoa(nav_pvt_data->lat, &lat_buf[5]);
//		int null_index = 0;
//		while (lat_buf[null_index++]);
//		lat_buf[null_index--] = 0;
//
//		int dot_index;
//		if (null_index > 8)
//		{
//			dot_index = null_index-7;
//			while (null_index != dot_index)
//			{
//				lat_buf[null_index] = lat_buf[null_index-1];
//				null_index--;
//			}
//			lat_buf[dot_index] = '.';
//		}
//
//		display_put_string(3, 3 + (16+3)*1 + (12+3)*2, lat_buf, font_small);
//
//
//		char lon_buf[100] = "Lon: ";
//		itoa(nav_pvt_data->lon, &lon_buf[5]);
//
//		null_index = 0;
//		while (lon_buf[null_index++]);
//		lon_buf[null_index--] = 0;
//
//		if (null_index > 8)
//		{
//			dot_index = null_index-7;
//			while (null_index != dot_index)
//			{
//				lon_buf[null_index] = lon_buf[null_index-1];
//				null_index--;
//			}
//			lon_buf[dot_index] = '.';
//		}
//
//		display_put_string(3, 3 + (16+3)*1 + (12+3)*2 + (8+3)*1, lon_buf, font_small);
//
//
//		char cno_buf[100] = "Best sat cno: ";
//		itoa(max_sat_cno, &cno_buf[14]);
//		display_put_string(3, 3 + (16+3)*1 + (12+3)*2 + (8+3)*2, cno_buf, font_small);
//
//		display_update();
//
//		state = 0;
//		state_change_timestamp = tick_get();
//
//		break;
//	}
//	default:
//		state = 0;
//		state_change_timestamp = tick_get();
//		break;
//	}
//	//uint32_t read_size = CDC_Read(&cdc_buffer[cdc_length], 512 - cdc_length - 1);
//
//
//	prevTick = tick_get();
// }
//
//void task_run_callback(void)
//{
//	schedule_user_job();
//}
//
//#include "../devices_header/tick.h"
//
///* Juse used to ensure the second timer is executing. */
//volatile uint32_t ulLETimerIncrements = 0;
//
//void vPortSetupTimerInterrupt(void)
//{
//  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
//
//  /* Ensure LE modules are accessible */
//  CMU_ClockEnable(cmuClock_CORELE, true);
//
//  /* Enable access to BURTC registers */
//  RMU_ResetControl(rmuResetBU, rmuResetModeClear);
//
//  /* Configure BURTC as system tick source */
//  burtcInit.mode        = burtcModeEM3;      /* BURTC is enabled to EM3 */
//  burtcInit.clkSel      = burtcClkSelLFXO; /* Select ULFRCO as clock source */
//  burtcInit.clkDiv      = burtcClkDiv_1;     /* Choose 2kHz ULFRCO clock frequency */
//  burtcInit.debugRun = true;
//  burtcInit.enable = 0;
//  //burtcInit.compare0Top = true;              /* Wrap on COMP0. */
//  /* Initialization of BURTC */
//  BURTC_Init(&burtcInit);
//
//}
//
//volatile uint8_t time_state = 0;
//volatile uint32_t tick_count = 0;
//volatile uint32_t tick_counts[500];
//volatile uint32_t tick_counts_cnt = 0;
///*void GPIO_EVEN_IRQHandler()
//{
//	u4_t int_mask = GPIO_IntGetEnabled();
//	GPIO_IntClear(int_mask);
//
//	GPIO_PinOutToggle(gpioPortC, 2);
//
//	switch (time_state)
//	{
//	case 0:
//		BURTC_CounterReset();
//		vPortSetupTimerInterrupt();
//		ulLETimerIncrements = 0;
//		tick_count = 0;
//		time_state = 1;
//		break;
//	case 1:
//		BURTC_Enable(true);
//		time_state = 2;
//		break;
//	case 2:
//	{
//		if (++tick_count == 120)
//		{
//			//BURTC_Enable(0);
//			volatile uint32_t counter = BURTC_CounterGet();
//
//			if (tick_counts_cnt < 500)
//				tick_counts[tick_counts_cnt++] = counter;
//
//			for (volatile int i = 0; i < 10; i++);
//			time_state = 0;
//
//			display_clear();
//			display_put_string(3, 3, "Crystal Test", font_medium);
//			char value_buf[100];
//			itoa(counter, value_buf);
//			display_put_string(3, 3+16+3, value_buf, font_large);
//			itoa(tick_counts_cnt, value_buf);
//			display_put_string(3, 3+16+3+16+3, value_buf, font_large);
//			display_update();
//		}
//		break;
//	}
//
//	}
//}*/
//
//static void setupRS485(void)
//{
//  /* Enable peripheral clocks */
//    CMU_ClockEnable(cmuClock_HFPER, true);
//    /* Configure GPIO pins */
//    CMU_ClockEnable(cmuClock_GPIO, true);
//
//    	CMU_ClockEnable(cmuClock_CORELE, true);
//    	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
//    	CMU_ClockEnable(cmuClock_LFB, true);
//
//    GPIO_PinModeSet((GPIO_Port_TypeDef)gpioPortE, 14, gpioModePushPull, 1); // TX
//    GPIO_PinModeSet((GPIO_Port_TypeDef)gpioPortE, 15, gpioModeInputPull, 1); // RX
//
//
//    LEUART_Init_TypeDef leuartInit = {
//    			.enable=	leuartDisable,
//    			.baudrate=	RS485_BAUDRATE,
//    			.databits=	leuartDatabits8,
//    			.parity=	leuartNoParity,
//    			.refFreq=   0,
//    			.stopbits= 	leuartStopbits1
//    	};
//
//    /* Enable CORE LE clock in order to access LE modules */
//    CMU_ClockEnable(cmuClock_CORELE, true);
//
//    /* Select LFXO for LEUARTs (and wait for it to stabilize) */
//    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_HFCLKLE);
//    CMU_ClockEnable(cmuClock_LEUART0, true);
//
//    /* Do not prescale clock */
//    CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
//
//    /* Configure LEUART */
//    leuartInit.enable = leuartDisable;
//
//    LEUART_Init(LEUART0, &leuartInit);
//
//    LEUART0->ROUTE = LEUART_ROUTE_LOCATION_LOC2
//  							  | LEUART_ROUTE_TXPEN
//  							  | LEUART_ROUTE_RXPEN;
//
//
//    /* Finally enable it */
//    LEUART_Enable(LEUART0, leuartEnable);
//}
//
//#include "../devices_header/analog.h"
//int main() {
//	CHIP_Init();
//
//	// Ensure LE modules are accessible.
//	CMU_HFRCOBandSet(cmuHFRCOBand_7MHz );
//	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
//	CMU_ClockEnable( cmuClock_CORELE, true );
//	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
//	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
//
//	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
//	//CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
//	//CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
//	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_8);
//
//	CMU_ClockEnable(cmuClock_GPIO, true);
//	GPIO_PinModeSet(gpioPortE, 9, gpioModePushPull, 1);
//	GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 1);
//	//setupLetimer();
//
//	GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 1);
//	GPIO_IntConfig(gpioPortD, 4, true, false, true);
//
//	GPIO_IntClear(_GPIO_IF_MASK);
//	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
//	NVIC_EnableIRQ( GPIO_EVEN_IRQn );
//
//	CMU_ClockEnable(cmuClock_GPIO, true);
//	CMU_ClockEnable(cmuClock_USART0, true);
//	CMU_ClockEnable(cmuClock_USART1, true);
//	CMU_ClockEnable(cmuClock_LEUART0, true);
//	CMU_ClockEnable(cmuClock_HFPER, true);
//
//	analog_init();
//	volatile int temperature = analog_read(TEMPERATURE);
//
//	tick_init();
//	display_init();
//	mode_switch_init();
//	delay_init();
//	display_clear();
//	display_put_string(3, 3, "App_mngr init...\nNew", font_small);
//	display_update();
//	if(app_manager_init()){
//		debug_str((const u1_t*)"\tApp Manager Init Successful\t\n");
//		display_clear();
//		display_put_string(3, 3, "App_mngr SUCCESS...", font_small);
//		display_update();
//	}
//	else{
//		debug_str((const u1_t*)"\tApp Manager Init Failed...\t\n");
//		display_clear();
//		display_put_string(3, 3, "App_mngr FAILED...", font_small);
//		display_update();
//	}
//	gps_init();
//
//	display_clear();
//	display_put_string(3, 3, "Crystal Test", font_medium);
//	display_update();
//
//	/*	while(1)
//	{
//		uint8_t mode = mode_switch_read();
//
//		display_clear();
//		display_put_string(3, 3, "Mode Test", font_large);
//		char value_buf[100];
//		itoa(mode, value_buf);
//		display_put_string(3, 3+16+3, value_buf, font_large);
//		display_update();
//	}*/
///*
//	GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0); // Driver Enable
//	GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1); // Receiver nEnable
//
//	setupRS485();
//
//	GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 1); // Driver Enable
//
//	//GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 0); // Receiver Enable
//
//	while(1)
//	{
//		GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 1); // Driver Enable
//		tick_delay(1);
//		LEUART_Tx(LEUART0, '?');
//		//tick_delay(1);
//		for (volatile int i = 0; i < 25000; i++);
//		GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 0); // Receiver Enable
//		GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0); // Driver Disable
//		//tick_delay(1);
//		volatile uint8_t buf[50];
//		volatile uint8_t data_count = 0;
//		uint64_t start_time = tick_get();
//		while ((tick_get()-start_time) < 100)
//		{
//			if (LEUART0->STATUS & LEUART_STATUS_RXDATAV)
//			{
//				uint8_t rs485_data = LEUART_Rx(LEUART0);
//				buf[data_count++] = rs485_data;
//			}
//		}
//		buf[data_count] = 0;
//		GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1); // Receiver Disable
//	}
//	//GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1); // Receiver Disable
//*/
//
//	CMU_HFRCOBandSet(cmuHFRCOBand_7MHz );
//	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
//	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
//
//	CMU_ClockEnable(cmuClock_GPIO, true);
//	CMU_ClockEnable( cmuClock_CORELE, true );
//	CMU_ClockEnable(cmuClock_USART0, true);
//	CMU_ClockEnable(cmuClock_USART1, true);
//	//CMU_ClockEnable(cmuClock_LEUART0, true);
//	CMU_ClockEnable(cmuClock_HFPER, true);
//	//CMU_ClockEnable(cmuClock_USB, true);
//	//CMU_ClockEnable(cmuClock_USBC, true);
//
//	GPIO_PinModeSet(gpioPortE, 9, gpioModePushPull, 1);
//	GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 1);
//
//	tick_init();
//	display_init();
//	mode_switch_init();
//	delay_init();
//	//gps_init();
//
//	GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0);
//	GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1);
//
//	display_clear();
//	display_put_string(3, 3, "NTNU SSM", font_large);
//	mode = mode_switch_read();
//	itoa(mode, &buffer[6]);
//	display_put_string(3, 3+16+3, buffer, font_medium);
//	display_update();
//
//	if(app_manager_init()){
//		debug_str((const u1_t*)"\tApp Manager Init Successful\t\n");
//	}
//	else{
//		debug_str((const u1_t*)"\tApp Manager Init Failed...\t\n");
//		 //rgb_on(true,false,false);
//		 while(1);
//	}
//
//	GPIO_PinModeSet(gpioPortE, 9, gpioModePushPull, 0);
//
//
//	//CDC_Init();
//	//USBD_Init(&usbInitStruct);
//
//    //tick_register_callback(task_run_callback, 100);
//	//register_user_callback(user_function);
//	while(1) {
//#ifdef USE_RADIO
//	  lpwan_init();
//#else
//		time_manager_cmd_t		time_manager_cmd=basic_sync;
//
//		rgb_shutdown();
//		debug_str((const u1_t*)"\t\tNo radio version started\n");
//		  while(ref_tstamp.valid!=true){		//wait for reference timestamp...
//			  ref_tstamp=gps_get_nav_data();
//			  delay_ms(5);
//		  }
//		ref_tstamp.gps_timestamp=time_manager_unixTimestamp(ref_tstamp.year,ref_tstamp.month,ref_tstamp.day,
//				  	  	  	  	  	  	  	  	  	  	  	  ref_tstamp.hour,ref_tstamp.min,ref_tstamp.sec);
//		RMU_ResetControl(rmuResetBU, rmuResetModeClear);
//		time_manager_init();
//		while(1){
//				//goto sleep
//			SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
//			EMU_EnterEM1();
//			//update Timestamps
//			running_tstamp=gps_get_nav_data();
//			running_tstamp.gps_timestamp=time_manager_unixTimestamp(running_tstamp.year,running_tstamp.month,running_tstamp.day,
//																	running_tstamp.hour,running_tstamp.min,running_tstamp.sec);
//			ref_tstamp.gps_timestamp+=BASIC_SYNCH_SECONDS;				//add 10secs
//	    	//update application manager
//			time_manager_cmd=time_manager_get_cmd();
//			app_manager_tbr_synch_msg(time_manager_cmd,ref_tstamp,running_tstamp);
//
//		}
//#endif
//  }
//}
//
///***************************************************************************//**
// * @brief
// *   Called whenever a USB setup command is received.
// *
// * @param[in] setup
// *   Pointer to an USB setup packet.
// *
// * @return
// *   An appropriate status/error code. See USB_Status_TypeDef.
// ******************************************************************************/
//int SetupCmd(const USB_Setup_TypeDef *setup)
//{
//  int retVal;
//
//  /* Call SetupCmd handlers for all functions within the composite device. */
//
//  retVal = CDC_SetupCmd(setup);
//
//  return retVal;
//}
//
///***************************************************************************//**
// * @brief
// *   Called whenever the USB device has changed its device state.
// *
// * @param[in] oldState
// *   The device USB state just leaved. See USBD_State_TypeDef.
// *
// * @param[in] newState
// *   New (the current) USB device state. See USBD_State_TypeDef.
// ******************************************************************************/
//void StateChangeEvent(USBD_State_TypeDef oldState,
//                      USBD_State_TypeDef newState)
//{
//  /* Call device StateChange event handlers for all relevant functions within
//     the composite device. */
//
//  CDC_StateChangeEvent(oldState, newState);
//}


///////////////////////////////////////////



#include "../resource managers_header/lpwan_manager.h"
#include "../resource managers_header/app_manager.h"
#include "../devices_header/display.h"
	/*
	 * Shared variables
	 */
#ifdef USE_RADIO
uint8_t 		lora_buffer[512];
uint8_t			lora_msg_length=0;
#else
nav_data_t	 	running_tstamp;
nav_data_t	 	ref_tstamp;
#endif

int main() {
	 /*
	  ********************* Chip initialization*************
	  */
			CHIP_Init();
			CMU_HFRCOBandSet(cmuHFRCOBand_7MHz );
			CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
			CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
			CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	 /*
	  *******************************************************
	  */
	CMU_ClockEnable(cmuClock_GPIO, true);
	display_init();
	display_clear();
	display_put_string(3,3,"iA will work. Init...",font_small);
	display_update();
	GPIO_PinModeSet(LED_RED_GREEN_PORT, LED_R, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_RED_GREEN_PORT, LED_G, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_GPS_RADIO_PORT, LED_GPS, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_GPS_RADIO_PORT, LED_RADIO, gpioModePushPull, 0);
	GPIO_PinOutSet(LED_RED_GREEN_PORT, LED_R);
	  rs232_init();
	  rs232_enable();
	  delay_init();
	  static 			unsigned char  		rs232_tx_buf[512];
	  int loop_var=0;
	  bool	flag_test=false;
	  delay_ms(10);
	  delay_ms(10);
		display_clear();
		display_put_string(3,3,"entering loop",font_small);
		display_update();
	  while(1){
		  flag_test=gps_init();
		  display_clear();
		  if(flag_test){
			  sprintf((char *)rs232_tx_buf,"GPS INIT Success\n");
			  display_put_string(3,3,rs232_tx_buf,font_small);
			  display_update();
			  break;
		  }
		  else {
			  sprintf((char *)rs232_tx_buf,"GPS INIT FAIL\n");
			  display_put_string(3,3,rs232_tx_buf,font_small);
			  display_update();
		  }
		  sprintf((char *)rs232_tx_buf,"Iter=%d, flag=%d\n",loop_var,(int)flag_test);
		  display_put_string(12,12,rs232_tx_buf,font_small);
		  display_update();
		  loop_var++;
		  delay_ms(10);
	  }

	if(app_manager_init()){
		debug_str((const u1_t*)"\tApp Manager Init Successful\t\n");
		display_clear();
		display_put_string(3,3,"Successful...",font_small);
		display_update();
	}
	else{
		debug_str((const u1_t*)"\tApp Manager Init Failed...\t\n");
		// rgb_on(true,false,false);
		display_clear();
		display_put_string(3,3,"Successful...",font_small);
		display_update();
		 return 0;
	}
	GPIO_PinOutClear(LED_RED_GREEN_PORT, LED_R);
	GPIO_PinOutSet(LED_RED_GREEN_PORT, LED_G);
	/*
			  rgb_init();
			  rs232_init();
			  rs232_enable();
			  delay_init();
*/
	//rgb_on(false,false,true);					//keep blue led on


#ifdef USE_RADIO
	  lpwan_init();
#else
		time_manager_cmd_t		time_manager_cmd=basic_sync;

		rgb_shutdown();
		debug_str((const u1_t*)"\t\tNo radio version started\n");
		  while(ref_tstamp.valid!=true){		//wait for reference timestamp...
			  ref_tstamp=gps_get_nav_data();
			  delay_ms(5);
		  }
		ref_tstamp.gps_timestamp=time_manager_unixTimestamp(ref_tstamp.year,ref_tstamp.month,ref_tstamp.day,
				  	  	  	  	  	  	  	  	  	  	  	  ref_tstamp.hour,ref_tstamp.min,ref_tstamp.sec);
		RMU_ResetControl(rmuResetBU, rmuResetModeClear);
		time_manager_init();
		while(1){
				//goto sleep
			SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
			EMU_EnterEM1();
			//update Timestamps
			running_tstamp=gps_get_nav_data();
			running_tstamp.gps_timestamp=time_manager_unixTimestamp(running_tstamp.year,running_tstamp.month,running_tstamp.day,
																	running_tstamp.hour,running_tstamp.min,running_tstamp.sec);
			ref_tstamp.gps_timestamp+=BASIC_SYNCH_SECONDS;				//add 10secs
	    	//update application manager
			time_manager_cmd=time_manager_get_cmd();
			app_manager_tbr_synch_msg(time_manager_cmd,ref_tstamp,running_tstamp);

		}
#endif
  }

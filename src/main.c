#include "../resource managers_header/lpwan_manager.h"
#include "../resource managers_header/app_manager.h"

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

static 			unsigned char  		display_buffer[512];

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
	init_led_switch();
	set_status_led(true,true);
	display_clear();
	sprintf(display_buffer,"Adddress=%2x\n",read_switch());
	display_put_string(6,3,display_buffer,font_medium);
	display_update();
	delay_ms(10);
	delay_ms(10);
	delay_ms(10);
	set_status_led(false,false);
	delay_ms(10);
	set_status_led(true,false);
	display_clear();
	display_put_string(3,3,"Init...\n\n",font_medium);
	display_update();
	if(app_manager_init()){
		debug_str((const u1_t*)"\tInit Successful\t\n");
		display_put_string(3,3,"\tSuccessful",font_medium);
		display_update();
	}
	else{
		debug_str((const u1_t*)"\tApp Manager Init Failed...\t\n");
		display_put_string(3,3,"\tFailed",font_medium);
		display_update();
		 return 0;
	}
	set_status_led(false,true);
	delay_ms(10);
	delay_ms(10);
	display_clear();
	display_put_string(3,3,"\tJoininig LoRa\n",font_medium);
	display_update();

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

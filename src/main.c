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
int				diff_in_tstamp=0;
#endif

static 			unsigned char  		display_buffer[512];
static 			unsigned char  		rs232_tx_buf[512];


int main() {
	 /*
	  ********************* Chip initialization*************
	  */
			CHIP_Init();
			//CMU_HFRCOBandSet(cmuHFRCOBand_7MHz );
			//CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
			CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
			//CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

			CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
			CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
			CMU_ClockDivSet(cmuClock_HF,cmuClkDiv_8);
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

#ifdef USE_RADIO
	lpwan_init();
#else
	delay_ms(10);
	delay_ms(10);
	display_clear();
	display_put_string(3,3,"Waiting GPS TS...",font_medium);
	display_update();
		time_manager_cmd_t		time_manager_cmd=basic_sync;
		uint8_t					gps_state=0;

		debug_str((const u1_t*)"\t\tNo radio version started\n");
		RMU_ResetControl(rmuResetBU, rmuResetModeClear);
		gps_poll_nav_status();
		  while(1){
			  ref_tstamp=gps_get_nav_data();
			  ref_tstamp.gps_timestamp=time_manager_unixTimestamp(ref_tstamp.year,ref_tstamp.month,ref_tstamp.day,
					  	  	  	  	  	  	  	  	  	  	  	  	 ref_tstamp.hour,ref_tstamp.min,ref_tstamp.sec);
			  if(ref_tstamp.valid==false){
				  gps_state++;
				  if(gps_state>10){
					  debug_str("GPS double polling scenario\n");
					  gps_poll_nav_status();
				  }
			  }
			  else {
				  if(ref_tstamp.fix==0x03 && ref_tstamp.gps_timestamp%10==0){
					  break;
				  }
				  else {
						gps_poll_nav_status();
				  }
			  }
		  }
		time_manager_init();
		while(1){
					//goto sleep
			SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
			EMU_EnterEM1();
					//add 10secs
			ref_tstamp.gps_timestamp+=BASIC_SYNCH_SECONDS;
					//get synch command type from time manager
			time_manager_cmd=time_manager_get_cmd();
					//update application manager
			app_manager_tbr_synch_msg(time_manager_cmd,ref_tstamp,running_tstamp,diff_in_tstamp);
						//update Timestamps
				gps_state=0;
				gps_poll_nav_status();
				while(gps_state<10){
					running_tstamp=gps_get_nav_data();
					if (running_tstamp.valid==true){
						break;
					}
						gps_state++;
				}
				running_tstamp.gps_timestamp=time_manager_unixTimestamp(running_tstamp.year,running_tstamp.month,running_tstamp.day,
																	running_tstamp.hour,running_tstamp.min,running_tstamp.sec);

				diff_in_tstamp= (int)(ref_tstamp.gps_timestamp-running_tstamp.gps_timestamp);
				sprintf(rs232_tx_buf,"CMD=%d\tTOP=%d\tTime Diff:Ref=%ld\tCur=%ld\tdiff=%d\tmin=%d\tsec=%d\tnano=%ld\ttAcc=%ld\tGPS_fix=%2x\tgps_state=%d\n",time_manager_cmd,BURTC->COMP0,(time_t)ref_tstamp.gps_timestamp,(time_t)running_tstamp.gps_timestamp,diff_in_tstamp,running_tstamp.min,running_tstamp.sec,running_tstamp.nano,running_tstamp.tAcc,running_tstamp.fix,gps_state);
				debug_str(rs232_tx_buf);
		}
#endif
  }

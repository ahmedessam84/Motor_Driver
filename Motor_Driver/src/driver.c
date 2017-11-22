/* 
 * EE497_19
 * Assignment 4: driver.c
 * 
 * Created: 10/27/2017 5:35:42 PM
 * Author: Ahmed Sobhy
 */ 

#include <asf.h>
#include "tc_megarf.h"
#include "pushbutton.h"
#include "led.h"
#include "driver.h"
#include <assert.h>

// Local storage for the call back function
static volatile drv_callback_t drv_callback=NULL;

static volatile drv_state_t drv_state = NOT_INITIALIZED;
static volatile void *vpTmr = NULL;
static volatile uint8_t *port;
static volatile uint8_t ui8Chnl;
static volatile uint8_t ui8Emergency_switch;
static volatile	uint8_t	ui8PWM_channelA;
static volatile	uint8_t	ui8PWM_channelB;
static volatile bool bEmergency_state;

static void reverse_stop(void);
static void forward_stop(void);
static void vTimer_init_PWM(volatile void *tc);
static void interrupt_init(void);


/**********************************************************************/
/*
 * @brief Driver Initialization Function
 * @param config Passed a 0 use LED configuration A  configure the GPIO
 * Passed a 1 use LED configuration B  configure the GPIO
 * @param callback Passed a pointer to a function to call in Main.
*/
/**********************************************************************/
drv_state_t driver_init(uint8_t config, drv_callback_t callback)
{
	// check input parameters
	assert( (config==0) || (config==1) );

	vpTmr = NULL;
	ui8Chnl = 0;
	ui8Emergency_switch = 0;
	ui8PWM_channelA = 0;
	ui8PWM_channelB = 0;
	
	// clear emergency state
	bEmergency_state = false;
	
	ioport_init();
	
	// switch statement to choose between configurations
	switch(config)
	{
		
		case 0: 
		{		
			// configuration A - Timer 3 is used
			vpTmr = TIMER3_BASE;
			port = &PORT_LED_A;
			ui8Emergency_switch = EMR_SW_PB_7;
			ui8PWM_channelA = OC3A_PIN;
			ui8PWM_channelB = OC3B_PIN;
			// enable timer3 clk
			sysclk_enable_module(POWER_RED_REG1, 1<<PRTIM3);
		}
		break;
		
		case 1: 
		{	
			// configuration B - Timer 4 is used
			vpTmr = TIMER4_BASE;
			port = &PORT_LED_B;
			ui8Emergency_switch = EMR_SW_PB_6;
			ui8PWM_channelA = OC4A_PIN;
			ui8PWM_channelB = OC4B_PIN;
			// enable timer 4 clk	
			sysclk_enable_module(POWER_RED_REG1, 1<<PRTIM4);
		}
		break;
		
		default: 
		{
			// error happened, return not initialized
			drv_state = NOT_INITIALIZED;
		}
		
	}
	
	// initializes the leds pins and sets them all to off
	vLedInit(port, (ui8PWM_channelA | ui8PWM_channelB) );
	
	// Initializes Emergency switch
	vPButtonInit(&PORTB, ui8Emergency_switch);
	
	// Initialize timer
	// Symmetric phase and frequency correct PWM mode best for motor control
	vTimer_init_PWM(vpTmr);
	
	// configure the interrupt for the emergency switch EMR_SW:
	interrupt_init();	
		
	// register the driver callback function passed by main
	drv_callback = callback;
		
	// change driver status to initialized & stopped after finishing initialization
	drv_state = STOPPED;
		
	return drv_state;
			
}

/**********************************************************************/
/*
 * @brief Motor Forward Function
 * @param speed  Passed a 0 the Forward LED goes slow
 * Passed a 1 the Forward LED goes fast
 * The Reverse LED is always turned off
*/
/**********************************************************************/
drv_state_t motor_forward(uint8_t speed) 
{
	assert( speed==0 || speed==1 );
		
	// clear emergency
	bEmergency_state = false;
	drv_callback(bEmergency_state);
		
	// forward is on channel A
	if (speed==1) 
	{// fast speed
			
		// change state
		drv_state = FORWARD_FAST;
				
		// frequency
		tc_write_ICR(vpTmr, FAST_RATE);
		// 50% duty cycle
		tc_write_out_comp(vpTmr, TC_COMPA, FAST_RATE/2);
			
	} 
	else if (speed == 0)
	{// slow speed
			
		// change state
		drv_state = FORWARD_SLOW;
				
		// frequency
		tc_write_ICR(vpTmr, SLOW_RATE);
		// 50% duty cycle
		tc_write_out_comp(vpTmr, TC_COMPA, SLOW_RATE/2);
			
	} 
	else {}
			
	return drv_state;
	 
}



/**********************************************************************/
/*
 * @brief Motor Reverse Function
 * @param speed   Passed a 0 the Reverse LED goes slow
 * Passed a 1 the Reverse LED goes fast
 * The Forward LED is always turned off
*/
/**********************************************************************/
drv_state_t motor_reverse(uint8_t speed) 
{
	assert( speed==0 || speed==1 );
		
	// clear emergency
	bEmergency_state = false;
	drv_callback(bEmergency_state);
			
	// forward is on channel A
	if (speed==1)
	{// fast speed
			
		drv_state = REVERSE_FAST;
				
		// fast frequency
		tc_write_ICR(vpTmr, FAST_RATE);
		// 50% duty cycle
		tc_write_out_comp(vpTmr, TC_COMPB, FAST_RATE/2);
			
	} 
	else if (speed==0)
	{// slow speed
			
		drv_state = REVERSE_SLOW;
				
		// slow frequency
		tc_write_ICR(vpTmr, SLOW_RATE);
		// 50% duty cycle
		tc_write_out_comp(vpTmr, TC_COMPB, SLOW_RATE/2);
			
	} 
	else {}
			
	return drv_state;
	
}

/**********************************************************************/
/*
 * @brief Motor Stop Function
 * Both Forward and Reverse LEDs turn off
*/
/**********************************************************************/
drv_state_t motor_stop() 
{	
	// clear emergency
	bEmergency_state = false;
	drv_callback(bEmergency_state);
	
	reverse_stop();
	forward_stop();
		
	drv_state = STOPPED;
		
	return drv_state;
		
}


drv_state_t motor_getStatus()
{
	return drv_state;
}

/**********************************************************************/
// @brief static file scope function to stop reverse motion 
/**********************************************************************/
static void reverse_stop() 
{
	// reverse stopped by setting duty cycle to 0%
	tc_write_out_comp(vpTmr, TC_COMPB, 0);
}

/**********************************************************************/
// @brief static file scope function to stop forward motion
/**********************************************************************/
static void forward_stop() 
{
	// forward stopped by setting duty cycle to 0%
	tc_write_out_comp(vpTmr, TC_COMPA, 0);
}

/**********************************************************************/
// @brief static file scope function to initialize interrupt on change
// for emergency switch
/**********************************************************************/
static void interrupt_init() 
{
	
	if( ui8Emergency_switch == EMR_SW_PB_7 ) 
	{
		// enable PCINT7 interrupt
		SET_REG_DATA(PCMSK0, 1<<PCINT7, 0xff);
	
	} 
	else if ( ui8Emergency_switch == EMR_SW_PB_6 ) 
	{
		// enable PCINT6 interrupt
		SET_REG_DATA(PCMSK0, 1<<PCINT6, 0xff);
	
	} 
	else {/*error*/}	
	
	// enable pin change interrupt
	SET_REG_DATA(PCICR, 1<<PCIE0, 0xff);

	// enable global interrupts
	sei();
	
}


/**********************************************************************/
/*
 * @brief Initialize timer for PWM function - static file scope function
 * @param *tc timer_base pointer
 * 
 */
/**********************************************************************/
static void vTimer_init_PWM(volatile void *tc)
{
	// Initialize timer
	// Symmetric phase and frequency correct PWM mode best for motor control
	
	tc_set_mode(tc, channel_A, PWM_Mode8, Set_OnUpCount_Mode );
	tc_set_mode(tc, channel_B, PWM_Mode8, Set_OnUpCount_Mode );
	
	tc_write_clock_source(tc, TC_CLKSEL_DIV1024_gc);
	
	tc_write_count(tc, 0xffff);
	
	tc_write_ICR(tc, SLOW_RATE);	
	
	// duty cycle control
	tc_write_out_comp(tc, TC_COMPA, 0);
	
	// duty cycle control
	tc_write_out_comp(tc, TC_COMPB, 0);
		
}


/**********************************************************************/
// @brief interrupt service routine toggles the emergency bit & calls
// the call back function - Pin change Interrupt.
/**********************************************************************/
 ISR(PCINT0_vect)
 {
	
	 // the interrupt flag is automatically cleared by the ATmega
	 // do the following only on falling edge:	 
	 if(IsPButtonPressed(&PORTB, ui8Emergency_switch))
	 {
		 
		 // toggle emergency state
		 bEmergency_state = !bEmergency_state;

		 // blink fast both leds if in emergency state
		 if( bEmergency_state )
		 {
			// frequency
			tc_write_ICR(vpTmr, FAST_RATE);
			// 50% duty cycle
			tc_write_out_comp(vpTmr, TC_COMPA, FAST_RATE/2);
			tc_write_out_comp(vpTmr, TC_COMPB, FAST_RATE/2);
			
			drv_state = EMERGENCY;
					 
		 }
		 else
		 {
			 // not in emergency, turn blinking leds off
			 reverse_stop();
			 forward_stop();
			 
			 drv_state = STOPPED;
			 
		 }
			 
		 if(drv_callback) // make sure the function pointer is not null
		 {
			drv_callback(bEmergency_state);
		 }
		 else{}
		 	 
	 }
	 
 }
 
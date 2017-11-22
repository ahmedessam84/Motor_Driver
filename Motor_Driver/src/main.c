/* 
 * EE497_19
 * Motor Driver
 * 
 * Created: 11/10/2017 5:35:42 PM
 * Author: Ahmed Sobhy
 */ 

/* How to connect Board:

	1- connect ISP 6 pin cable instead of the JTAG cable
	2- connect 10 pin cable from PortB to Switches
	3- for configuration A connect the other 10 pin cable to PortE and Leds
	4- for configuration B connect the 10 pin cable to PortH and Leds  

	*Please change "configuration" variable in main to switch between the
	 two configuration settings 

*/

/**********************************************************************/
// number of LED 3:
// 1 LED for Forward - PWM output pin -		  Pin3
// 1 LED for Reverse - PWM output pin -		  Pin4
// 1 LED for Emergency - General output pin - Pin5

/*

Switches are connected to PortB
FORWARD SLOW BTN				Pin0
FORWARD FAST BTN				Pin1
REVERSE SLOW BTN				Pin2
REVERSE FAST BTN				Pin3
STOP BTN						Pin4
EMERGENCY BUTTON				Pin7 or Pin6

*/

/**********************************************************************/
// configuration A:
/**********************************************************************/
// LEDs on portA
//PORTE
//OC3A_PIN			Pin3
//OC3B_PIN			Pin4
//Emergency switch	Pin7

/**********************************************************************/
// configuration B:
/**********************************************************************/
// LEDs on portH
//PORTH
//OC4A_PIN			Pin3
//OC4B_PIN			Pin4
//Emergency switch	Pin6

/**********************************************************************/
// Predefined PWM rates to control the speed of the motor
// Duty cycle is set to 50% when motor is in any of the running modes
// Duty cycle is set to 0% to stop the motor
// SLOW_RATE			10000
// FAST_RATE			2000
/**********************************************************************/

#include <asf.h>
#include "api.h"
#include "pushbutton.h"
#include "tc_megarf.h"
#include "led.h"
#include "bit_ops.h"

#define EMR_LED					1<<5

#define BTN_FWD_S				1<<0
#define BTN_FWD_F				1<<1
#define BTN_RVS_S				1<<2
#define BTN_RVS_F				1<<3
#define BTN_STOP				1<<4
#define BTN_MASK				(BTN_FWD_F | BTN_FWD_S | BTN_RVS_F | BTN_RVS_S | BTN_STOP)

void emergency_callback(bool);
volatile uint8_t *pEmergencyLedPort = NULL;

int main (void)
{
	uint8_t btn_pressed;
	uint8_t configuration;
	
	//
	// use this variable to change the configuration
	// set it to zero for config A and set it to one for config B
	//
	configuration = 0;
	//configuration = 1;
	
	board_init();
	sysclk_init();
	
	vPButtonInit(&PORTB, BTN_MASK);
	
	//
	// Emergency Led changes according to configuration because I am using
	// the 10 pin cable to connect the port pins to the Led pins for simplicity
	// and easy of testing
	//
	if(configuration)
	{
		// emergency led connected to Port H
		pEmergencyLedPort = &PORTH;
	}
	else
	{
		// emergency led connected to Port E
		pEmergencyLedPort = &PORTE;
	}
	
	vLedInit(pEmergencyLedPort, EMR_LED);
	
	
	//
	// driver initialization
	//
	driver_api_init(configuration, emergency_callback);
	
	while(1)
	{
		btn_pressed = getPButtonPressed(&PORTB, BTN_MASK);
		
		switch(btn_pressed)
		{	
			case BTN_FWD_F: motor_api_forward(1);
			break;
			
			case BTN_FWD_S: motor_api_forward(0); 
			break;
			
			case BTN_RVS_F: motor_api_reverse(1); 
			break;
			
			case BTN_RVS_S: motor_api_reverse(0);
			break; 
			
			case BTN_STOP: motor_api_stop();
			break;
					
		}
	}
}

// callback function passed to driver
void emergency_callback(bool emergency_state)
{
	if(emergency_state)
	{
		// turn on emergency led
		vLedOn(pEmergencyLedPort,EMR_LED);
	}
	else
	{
		// turn off emergency led
		vLedOff(pEmergencyLedPort,EMR_LED);
	}	
}

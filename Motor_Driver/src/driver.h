/* 
 * EE497_19
 * Assignment 4: driver.h
 * 
 * Created: 10/27/2017 5:35:42 PM
 * Author: Ahmed Sobhy
 */ 


#ifndef DRIVER_H_
#define DRIVER_H_

/**********************************************************************/
// number of LED 3:
// 1 LED for Forward - PWM output pin
// 1 LED for Reverse - PWM output pin
// 1 LED for Emergency - General output pin
/**********************************************************************/
// configuration A:
/**********************************************************************/
// LEDs on portA
#define PORT_LED_A			PORTE
#define OC3A_PIN			1<<3
#define OC3B_PIN			1<<4
// Emergency switch portB7
#define EMR_SW_PB_7			1<<7

/**********************************************************************/
// configuration B:
/**********************************************************************/
// LEDs on portH
#define PORT_LED_B			PORTH
#define OC4A_PIN			1<<3
#define OC4B_PIN			1<<4
// Emergency switch portB6
#define EMR_SW_PB_6			1<<6

/**********************************************************************/
// predefined PWM rates to control the speed of the motor
// Duty cycle is set to 50% when motor is in any of the running modes
// Duty cycle is set to 0% to stop the motor
#define SLOW_RATE			10000
#define FAST_RATE			2000
/**********************************************************************/

typedef enum drv_state { 
	NOT_INITIALIZED, 
	INITIALIZED,
	STOPPED, 
	FORWARD_FAST, 
	FORWARD_SLOW, 
	REVERSE_FAST, 
	REVERSE_SLOW, 
	EMERGENCY 
	} drv_state_t;
	

/**
 * \brief Interrupt event callback function type
 *
 * The interrupt handler can be configured to do a function callback,
 * the callback function must match the drv_callback_t type.
 *
 */

typedef void (*drv_callback_t)(bool);

drv_state_t driver_init(uint8_t, drv_callback_t );
drv_state_t motor_forward(uint8_t );
drv_state_t motor_reverse(uint8_t );
drv_state_t motor_stop(void);
drv_state_t motor_getStatus(void);






#endif /* DRIVER_H_ */
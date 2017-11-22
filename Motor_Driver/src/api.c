/*
 * api.c
 *
 * Created: 11/3/2017 11:49:07 PM
 *  Author: Ahmed Sobhy
 */ 

#include <asf.h>
#include "driver.h"
#include "api.h"

static volatile drv_state_t drvr_state;
/**********************************************************************/
/*
 * @brief Driver Initialization Function
 * @param config Passed a 0 use LED configuration A  configure the GPIO
 * Passed a 1 use LED configuration B  configure the GPIO
 * @param callback Passed a pointer to a function to call in Main.
*/
/**********************************************************************/
drv_state_t driver_api_init(uint8_t config, drv_callback_t callback)
{
	
	drvr_state = driver_init(config, callback);
	
	return drvr_state;
		
}

/**********************************************************************/
/*
 * @brief Motor Forward Function
 * @param speed  Passed a 0 the Forward LED goes slow
 * Passed a 1 the Forward LED goes fast
 * The Reverse LED is always turned off
*/
/**********************************************************************/
drv_state_t motor_api_forward(uint8_t speed)
{
	// enter critical section
	uint8_t sreg = cpu_irq_save();
	
	// driver must be initialized first
	if (drvr_state != NOT_INITIALIZED) 
	{
		// no need to execute code if motor is already on that mode
		if ( (speed==1) && (drvr_state != FORWARD_FAST) )
		{// fast speed
			
			// stop motor
			drvr_state = motor_stop();
			drvr_state = motor_forward(speed);
		
		}
		// no need to execute code if motor is already on that mode
		else if( (speed == 0) && (drvr_state != FORWARD_SLOW) )
		{
			// stop motor
			drvr_state = motor_stop();
			drvr_state = motor_forward(speed);
			
		} 
		else {}
			
	}
	else{}
	
	// exit critical section
	cpu_irq_restore(sreg);
		
	return drvr_state;
	
}
/**********************************************************************/
/*
 * @brief Motor Reverse Function
 * @param speed   Passed a 0 the Reverse LED goes slow
 * Passed a 1 the Reverse LED goes fast
 * The Forward LED is always turned off
*/
/**********************************************************************/
drv_state_t motor_api_reverse(uint8_t speed)
{
	// enter critical section
	uint8_t sreg = cpu_irq_save();
	
	// driver must be initialized first
	if (drvr_state != NOT_INITIALIZED)
	{
		// no need to execute code if motor is already on that mode
		if ( (speed==1) && (drvr_state != REVERSE_FAST) )
		{// fast speed
			
			// stop motor
			drvr_state = motor_stop();
			drvr_state = motor_reverse(speed);
			
		}
		// no need to execute code if motor is already on that mode
		else if( (speed == 0) && (drvr_state != REVERSE_SLOW) )
		{
			// stop motor
			drvr_state = motor_stop();
			drvr_state = motor_reverse(speed);
			
		} 
		else {}
	
	}
	else{}
	
	// exit critical section	
	cpu_irq_restore(sreg);
		
	return drvr_state;
	
}

/**********************************************************************/
/*
 * @brief Motor Stop Function
 * Both Forward and Reverse LEDs turn off
*/
/**********************************************************************/
drv_state_t motor_api_stop()
{
	// enter critical section
	uint8_t sreg = cpu_irq_save();
	
	if ( drvr_state != NOT_INITIALIZED )
	{
		drvr_state = motor_stop();	
	}	
	else{}
	
	// exit critical section
	cpu_irq_restore(sreg);
		
	return drvr_state;
			
}

/**********************************************************************/
/*
 * @brief returns status of motor
*/
/**********************************************************************/
drv_state_t motor_api_getStatus()
{

	return drvr_state;

}

/*
 * led.c
 *
 * Created: 10/2/2017 9:22:42 PM
 *  Author: Ahmed Sobhy
 */ 

#include <asf.h>
#include "led.h"
#include "bit_ops.h"
//*****************************************************************************
//
//! \brief	Initializes led pins
//!
//! \param	ui8Led represent the pins where the led is connected to
//!
//! \return Void
//
//*****************************************************************************
void vLedInit(volatile uint8_t *port_address, uint8_t ui8Led)
{
	
	// use port to write and pin to read
	volatile uint8_t *port_ptr = port_address;
	volatile uint8_t *ddr_ptr = port_ptr-1;
	volatile uint8_t *pin_ptr = port_ptr-2;
	
	// set direction
	SET_REG_DATA(*ddr_ptr, ui8Led, 0xff);
	// set pullup to high
	SET_REG_DATA(*port_ptr, ui8Led, 0xff);
	
}

//*****************************************************************************
//
//! \brief	Turns led on
//!
//! \param	ui8Led represents the leds to turn on
//!
//! \return Void
//
//*****************************************************************************
void vLedOn(volatile uint8_t *port_address, uint8_t ui8Led)
{
	// use port to write and pin to read
	volatile uint8_t *port_ptr = port_address;
	SET_REG_DATA(*port_ptr, ui8Led, 0x00);
}

//*****************************************************************************
//
//! \brief	Turns leds off
//!
//! \param	ui8Led represents the leds to turn off
//!
//! \return Void
//
//*****************************************************************************
void vLedOff(volatile uint8_t *port_address, uint8_t ui8Led)
{
	// use port to write and pin to read
	volatile uint8_t *port_ptr = port_address;
	SET_REG_DATA(*port_ptr, ui8Led, 0xff);
}

void vLedToggle(volatile uint8_t *port_address, uint8_t ui8Led)
{
	// use port to write and pin to read
	volatile uint8_t *port_ptr = port_address;
	volatile uint8_t *pin_ptr = port_ptr-2;
	
	// if led is off turn it on by setting it to zero
	if( GET_REG_DATA(*pin_ptr, ui8Led) == ui8Led )
	{
		// turn led on
		SET_REG_DATA(*port_ptr, ui8Led, 0x00);	
	}
	else
	{
		// turn led off
		SET_REG_DATA(*port_ptr, ui8Led, 0xff);
	}
	
}

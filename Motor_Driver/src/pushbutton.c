/*
 * pushbutton.c
 *
 * Created: 10/2/2017 9:22:26 PM
 * Author: Ahmed Sobhy
 */ 

#include <asf.h>
#include "pushbutton.h"
#include "bit_ops.h"
//*****************************************************************************
//
//! \brief	Initializes push button
//!
//! \param	ui8Buttons are buttons to initialize   
//!
//! \return void
//
//***************************************************************************** 
void vPButtonInit(volatile uint8_t *port_address, uint8_t ui8Buttons)
{	
	// use port to write and pin to read
	volatile uint8_t *port_ptr = port_address;
	volatile uint8_t *ddr_ptr = port_ptr-1;
	// set direction to input
	SET_REG_DATA(*ddr_ptr, ui8Buttons, 0x00);
	// pull up
	SET_REG_DATA(*port_ptr, ui8Buttons, 0xff);
	
	//ioport_set_port_dir()

}

//*****************************************************************************
//
//! \brief	function to get button pressed
//!
//! \param	buttons: button to test
//!
//! \return	button pressed
//
//*****************************************************************************
uint8_t getPButtonPressed(volatile uint8_t *port_address, uint8_t ui8Buttons)
{
	volatile uint8_t *pin_ptr = (port_address)-2;
	// if button is pressed input is logic zero 
	// if button is not pressed input is logic one 
	return (GET_REG_DATA((~(*pin_ptr)),ui8Buttons));	
}

bool IsPButtonPressed(volatile uint8_t *port_address, uint8_t ui8Button)
{
	return ( (getPButtonPressed(port_address,ui8Button) & ui8Button) == ui8Button ) ;
}
/*
 * btn.c
 *
 * Created: 11/17/2017 11:10:57 PM
 *  Author: ahmed
 */ 

#include "asf.h"
#include "btn.h"


void BTN_Init()
{
	ioport_init();
	// sets the direction of the button to input
	ioport_set_port_dir(BTN_PORT, BTN_MASK, IOPORT_DIR_INPUT);
}

btn_state_t BTN_Scan(uint8_t BtnId)
{
	unsigned char Current_State[NO_OF_BTNS];			//current state
	static uint8_t Past_State[NO_OF_BTNS]={1};			//past state
	volatile uint8_t Return_State;						//return state
	
	Current_State[BtnId]= ioport_get_port_level(BTN_PORT,1<<BtnId);
	
	switch(Current_State[BtnId]>>BtnId)
	{

		case 1: 
				if(Past_State[BtnId]==1)
				{
					Return_State = BTN_OFF;
				}
				else if(Past_State[BtnId]==0)
				{
					Return_State = BTN_FALLING;
					Past_State[BtnId] = Current_State[BtnId]>>BtnId;
				}
				break;
				
		case 0:
				if(Past_State[BtnId]==0)
				{
					Return_State = BTN_ON;
				}
				else if(Past_State[BtnId]==1)
				{
					Return_State = BTN_RISING;
					Past_State[BtnId]=Current_State[BtnId]>>BtnId;
				}
				break;

	}
	
	return Return_State;

}
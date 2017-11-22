/*
 * btn.h
 *
 * Created: 11/17/2017 11:11:11 PM
 *  Author: ahmed
 */ 


#ifndef BTN_H_
#define BTN_H_

#define NO_OF_BTNS				5
#define BTN_PORT				IOPORT_PORTB
#define BTN_FWD_S				1<<0
#define BTN_FWD_F				1<<1
#define BTN_RVS_S				1<<2
#define BTN_RVS_F				1<<3
#define BTN_STOP				1<<4
#define BTN_MASK				(BTN_FWD_F | BTN_FWD_S | BTN_RVS_F | BTN_RVS_S | BTN_STOP)

typedef enum btn_state{
	BTN_OFF, 
	BTN_FALLING, 
	BTN_ON, 
	BTN_RISING
	} btn_state_t; 

void BTN_Init();
btn_state_t BTN_Scan(uint8_t);

#endif /* BTN_H_ */
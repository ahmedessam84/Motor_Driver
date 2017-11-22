/*
 * pushbutton.h
 *
 * Created: 10/2/2017 9:44:55 PM
 *  Author: Ahmed Sobhy
 */ 


#ifndef PUSHBUTTON_H_
#define PUSHBUTTON_H_


void vPButtonInit(volatile uint8_t *, uint8_t );
uint8_t getPButtonPressed(volatile uint8_t *, uint8_t );
bool IsPButtonPressed(volatile uint8_t *, uint8_t );


#endif /* PUSHBUTTON_H_ */
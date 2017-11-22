/*
 * led.h
 *
 * Created: 10/2/2017 9:46:09 PM
 *  Author: Ahmed Sobhy
 */ 


#ifndef LED_H_
#define LED_H_


void vLedInit(volatile uint8_t *, uint8_t );
void vLedOn(volatile uint8_t *, uint8_t );
void vLedOff(volatile uint8_t *, uint8_t );
void vLedToggle(volatile uint8_t *, uint8_t );



#endif /* LED_H_ */
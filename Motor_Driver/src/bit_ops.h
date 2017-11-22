/*
 * bit_ops.h
 *
 * Created: 10/28/2017 9:57:59 AM
 *  Author: ahmed
 */ 


#ifndef BIT_OPS_H_
#define BIT_OPS_H_


#define SET_REG_DATA(REG,MASK,DATA)		REG = ((REG & (~(MASK))) | (DATA & MASK))
#define GET_REG_DATA(REG,MASK)			(REG & MASK)
#define BIT(X)							1<<X
#define SETBIT(REG,X)					REG |= BIT(X)
#define CLRBIT(REG,X)					REG &= ~BIT(X)
#define TGLBIT(REG,X)					REG ^= BIT(X)



#endif /* BIT_OPS_H_ */
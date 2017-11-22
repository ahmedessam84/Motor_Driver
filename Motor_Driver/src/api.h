/*
 * api.h
 *
 * Created: 11/3/2017 11:49:29 PM
 *  Author: ahmed
 */ 


#ifndef API_H_
#define API_H_

#include "driver.h"

drv_state_t driver_api_init(uint8_t config, drv_callback_t callback);
drv_state_t motor_api_forward(uint8_t speed);
drv_state_t motor_api_reverse(uint8_t speed);
drv_state_t motor_api_stop(void);
drv_state_t motor_api_getStatus(void);




#endif /* API_H_ */
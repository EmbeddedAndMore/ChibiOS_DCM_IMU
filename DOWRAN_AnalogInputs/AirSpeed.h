/*
 * AirSpeed.h
 *
 *  Created on: Feb 4, 2015
 *      Author: Mohamad Armoun
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_

#include "ch.h"
#include "hal.h"




extern bool_t AirSpeed_Connected;
extern bool_t AirSpeed_Enabled;
extern bool_t AirSpeed_use;



float Airspeed(void);


#endif /* AIRSPEED_H_ */

/*
 * AirSpeed.c
 *
 *  Created on: Feb 4, 2015
 *      Author: Mohamad Armoun
 */



#include "AirSpeed.h"
#include "Analogs.h"






bool_t AirSpeed_Connected = TRUE;
bool_t AirSpeed_Enabled = TRUE;
bool_t AirSpeed_use = FALSE;
bool_t healthy;
float offset = 0;
static float _airspeed = 0;
static float _raw_airspeed = 0;
// read the airspeed sensor





float Airspeed(void)
{
  if (!AirSpeed_Enabled) {
          return 0;
      }
  float pressure = 0;
  healthy =  get_difrential_pressure(&pressure) ;
  return ;
}

void AirSpeed_calibrate(void)
{
      float sum = 0;
      uint8_t count = 0;
      uint8_t i = 0;
      if (!AirSpeed_Enabled) {
          return;
      }

      // discard first reading
      Airspeed();
      for (i = 0; i < 10; i++) {
          chThdSleepMilliseconds(100);
          float p = Airspeed();
          if (healthy) {
              sum += p;
              count++;
          }
      }

      if (count == 0) {
          offset = 0;
          return;
      }

      float raw = sum/count;
      offset = raw;
      _airspeed = 0;
      _raw_airspeed = 0;
}

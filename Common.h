/*
 * Common.h
 *
 *  Created on: Apr 8, 2015
 *      Author: Mohamad Armoun
 */


#ifndef COMMON_H_
#define COMMON_H_


#define PI 3.14
#define DEG_TO_RAD(deg) ((deg) *((PI)/(180)))
#define RAD_TO_DEG(deg) ((deg) *((180)/(PI)))
#define GRAVITY_MSS 9.80665f

extern float _kp_yaw;
extern float _kp;










int32_t wrap_360_cd(int32_t error);
int32_t wrap_180_cd(int32_t error);
float wrap_360_cd_float(float angle);
float wrap_180_cd_float(float angle);
float wrap_PI(float angle_in_radians);
extern inline uint32_t hal_get_millis(void);
extern inline uint32_t hal_get_micros(void);

#endif /* COMMON_H_ */

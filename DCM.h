/*
 * DCM.h
 *
 *  Created on: Apr 3, 2015
 *      Author: Mohamad Armoun
 */

#ifndef DCM_H_
#define DCM_H_

#include "DP_Math.h"
#include "DCMmath.h"
// this is the speed in m/s above which we first get a yaw lock with
// the GPS
#define GPS_SPEED_MIN 3
#define SPIN_RATE_LIMIT 20

void dcm_init(void);
void dcm_update(void);
void dcm_matrix_update(float _G_Dt);
void dcm_reset(bool_t recover_eulers);
void reset_attitude(float _roll , float _pitch , float _yaw);
void check_matrix(void);
bool_t renorm(Vector3f a,Vector3f *result);
void normalize(void);
float yaw_error_compass(void);
float _P_gain(float spin_rate);
float _yaw_gain(void);
bool_t use_compass(void);
void drift_correction_yaw(void);
Vector3f ra_delayed(uint8_t instance, Vector3f ra);
void drift_correction(float deltat);
void estimate_wind(void);
void euler_angles(void);


extern float roll;
extern float pitch;
extern float yaw;

/**
 * temperory
 */
extern float _omega_I_sum_time;
extern int mohamadplus;
extern Vector3f _omega;
extern Matrix3f _dcm_matrix;


extern float eachLoopTime;
extern float wholeLoopTime;


#endif /* DCM_H_ */

/*
 * Common.c
 *
 *  Created on: Apr 8, 2015
 *      Author: Mohamad Armoun
 */

#include "ch.h"
#include "hal.h"
#include <math.h>




//____ Parameters
// @Param: YAW_P
// @DisplayName: Yaw P
// @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
// @Range: 0.1 0.4
// @Increment: .01
float _kp_yaw = 1.2f;//0.4


// @Param: RP_P
// @DisplayName: AHRS RP_P
// @Description: This controls how fast the accelerometers correct the attitude
// @Range: 0.1 0.4
// @Increment: .01
float _kp = 0.4f;



/*
  wrap an angle in centi-degrees to 0..35999
 */
int32_t wrap_360_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error >= 36000) error -= 36000;
    while (error < 0) error += 36000;
    return error;
}

/*
  wrap an angle in centi-degrees to -18000..18000
 */
int32_t wrap_180_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error > 18000) { error -= 36000; }
    while (error < -18000) { error += 36000; }
    return error;
}


/*
  wrap an angle in centi-degrees to 0..35999
 */
float wrap_360_cd_float(float angle)
{
    if (angle >= 72000.0f || angle < -36000.0f) {
        // for larger number use fmodulus
        angle = fmod(angle, 36000.0f);
    }
    if (angle >= 36000.0f) angle -= 36000.0f;
    if (angle < 0.0f) angle += 36000.0f;
    return angle;
}

/*
  wrap an angle in centi-degrees to -18000..18000
 */
float wrap_180_cd_float(float angle)
{
    if (angle > 54000.0f || angle < -54000.0f) {
        // for large numbers use modulus
        angle = fmod(angle,36000.0f);
    }
    if (angle > 18000.0f) { angle -= 36000.0f; }
    if (angle < -18000.0f) { angle += 36000.0f; }
    return angle;
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10*M_PI || angle_in_radians < -10*M_PI) {
        // for very large numbers use modulus
        angle_in_radians = fmodf(angle_in_radians, 2*M_PI);
    }
    while (angle_in_radians > M_PI) angle_in_radians -= 2*M_PI;
    while (angle_in_radians < -M_PI) angle_in_radians += 2*M_PI;
    return angle_in_radians;
}

inline uint32_t hal_get_millis(void)
{
    return floor(((double)halGetCounterValue()/(double)halGetCounterFrequency())*1000);
}

inline uint32_t hal_get_micros(void)
{
   return floor(((double)halGetCounterValue()/(double)halGetCounterFrequency())*1000000);
}

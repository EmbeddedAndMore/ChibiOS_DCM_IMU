/*
 * AHRS.c
 *
 *  Created on: Apr 11, 2015
 *      Author: Mohamad Armoun
 */
#include "AHRS.h"

#include "ms5611.h"
#include "MPU6050.h"
#include "hmc5883.h"
#include "DOWRAN_GPS/GPS_ublox.h"
#include "DOWRAN_AnalogInputs/AirSpeed.h"
#include "Common.h"
Vector3f _trim = {1,1,1};

// integer Euler angles (Degrees * 100)
int32_t roll_sensor;
int32_t pitch_sensor;
int32_t yaw_sensor; // dar hmc5883 hamtarif shode.


/*
 * Gyro Global Parameters
 */
float gyro_drift_limit = DEG_TO_RAD(0.5/60); // for mpu 6k


/*
 * GPS Global Parameters
 */

// @Param: GPS_MINSATS
// @DisplayName: AHRS GPS Minimum satellites
// @Description: Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
// @Range: 0 10
// @Increment: 1
// @User: Advanced
uint8_t _gps_minsats = 6;

// @Param: GPS_GAIN
// @DisplayName: AHRS GPS gain
// @Description: This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
// @Range: 0.0 1.0
// @Increment: .01
float gps_gain =  1.0f;

// @Param: GPS_USE
// @DisplayName: AHRS use GPS for navigation
// @Description: This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
// @Values: 0:Disabled,1:Enabled
// @User: Advanced
int8_t _gps_use  = 1 ; //AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, 1),

bool_t have_gps(void)
{
  return FIX_TYPE > 1;
}
float get_GPS_ground_speed(void)
{
  return ground_speed;
}

uint8_t get_GPS_FIX_TYPE(void)
{
  return FIX_TYPE;
}

int32_t get_GPS_ground_course_cd(void)
{
  return ground_course_cd;
}

Location get_GPS_Location(void)
{
    Location l ;
    l.lat = latitude;
    l.lng = longitude;
    l.alt = gps_altitude;
    return l;
}

Vector3f get_GPS_velocity(void)
{
  return NED_vlocity;
}

uint8_t get_GPS_num_sats(void)
{
  return satellites;
}

uint32_t get_GPS_last_fix_time_ms(void)
{
  return last_fix_time_ms;
}


/*
 * Analog inputs Parameters
 */
float  get_Airspeed (void)
{
  //return Airspeed();
}
bool_t Airspeed_using(void)
{
    //return AirSpeed_use;
}

bool_t ins_healthy(void)
{
  return gyro_accel_healthy || baro_healthy || comp_healthy;
}



/**
 * Compass global Parameters
 */

uint32_t Compass_last_update(void)
{
    return Comp_last_update;
}


//uint16_t    AOA      =                      get_AOA();
//uint16_t    Voltage  =                      get_Voltage();
//uint16_t    Current  =                      get_Current();

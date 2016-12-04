/*
 * AHRS.h
 *
 *  Created on: Apr 11, 2015
 *      Author: Mohamad Armoun
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "DP_Math_Vector2f.h"
#include "DP_Math.h"

extern Vector3f _trim;
extern int32_t roll_sensor;
extern int32_t pitch_sensor;
extern int32_t yaw_sensor;

/*
 * Gyro Global Parameters
 */
extern float gyro_drift_limit;

/*
 * GPS Global Parameters
 */
extern int8_t _gps_use;
//extern float       GPS_ground_speed;
//extern uint8_t     GPS_FIX_TYPE ;
//extern int32_t     GPS_ground_course_cd;


/*
 *  Analog input Parameters
 */
//extern uint16_t    Airspeed;
//extern uint16_t    AOA;
//extern uint16_t    Voltage;
//extern uint16_t    Current;

bool_t Airspeed_using(void);
float  get_Airspeed (void);



bool_t ins_healthy(void);

/*
 * GPS Parameters
 */
////////////////////////////////////////////////////////////////////////////////
/// @name   Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored               0: Absolute,    1: Relative
/// bit 1: Chnage Alt between WP            0: Gradually,   1: ASAP
/// bit 2: Direction of loiter command      0: Clockwise    1: Counter-Clockwise
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home                 0: No,          1: Yes
/// bit 5:
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

typedef struct Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relateive to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
}LOF;

typedef struct ST_Location {
    union {
        LOF flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
}Location;
extern uint8_t _gps_minsats;
extern float gps_gain ;

bool_t have_gps(void);
float get_GPS_ground_speed(void);
uint8_t get_GPS_FIX_TYPE(void);
int32_t get_GPS_ground_course_cd(void);
Vector3f get_GPS_velocity(void);
uint8_t get_GPS_num_sats(void);
uint32_t get_GPS_last_fix_time_ms(void);
Location get_GPS_Location(void);







typedef struct AHRS_FLAGS{

    uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
    uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
    uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
    uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
    uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    uint8_t armed                   : 1;    // 1 if we are armed for flight

}flags;

/**
 * Compass global Parameters
 */

uint32_t Compass_last_update(void);
#endif /* AHRS_H_ */

/*
 * DCM.c
 *
 *  Created on: Apr 3, 2015
 *      Author: Mohamad Armoun
 *      based on Arduplane
 */

#include "stdlib.h"
#include "ch.h"
#include "hal.h"
#include "DCM.h"
#include "DP_Math.h"
#include "DCMmath.h"
#include "MPU6050.h"
#include "hmc5883.h"
#include <math.h>
#include <DP_Math_Vector2f.h>
#include "Common.h"
#include "AHRS.h"
#include "DOWRAN_GPS/GPS_ublox.h"
#include "chprintf.h"

union {uint8_t byte[4]; float value;}_floatToByte;
uint8_t dcmMessage[30];


  flags _flags;

  // primary representation of attitude of board used for all inertial calculations
  Matrix3f _dcm_matrix;

  // primary representation of attitude of flight vehicle body
  Matrix3f _body_dcm_matrix;

  float _ki;
  float _ki_yaw;

  // primary representation of attitude
  Vector3f _omega_P = {0,0,0};                          // accel Omega proportional correction
  Vector3f _omega_yaw_P= {0,0,0};                      // proportional yaw correction
  Vector3f _omega_I= {0,0,0};                          // Omega Integrator correction
  Vector3f _omega_I_sum= {0,0,0};
  float _omega_I_sum_time;
  Vector3f _omega = {0,0,0};



  // state of accel drift correction
  Vector3f _ra_sum[INS_MAX_INSTANCES] = {0,0,0};
  Vector3f _last_velocity = {0,0,0};
  float _ra_deltat;
  uint32_t _ra_sum_start;

  // state to support status reporting
  float _renorm_val_sum;
  uint16_t _renorm_val_count;
  float _error_rp_sum;
  uint16_t _error_rp_count;
  float _error_rp_last;
  float _error_yaw_sum;
  uint16_t _error_yaw_count;
  float _error_yaw_last;

  // last time AHRS failed in milliseconds
  uint32_t _last_failure_ms;


  // the earths magnetic field
  float _last_declination;
  Vector2f _mag_earth = {1,0};

  // estimated wind in m/s
  Vector3f _wind = {0,0,0};

  // support for wind estimation
  Vector3f _last_fuse =  {0,0,0};
  Vector3f _last_vel = {0,0,0};
  uint32_t _last_wind_time;
  float _last_airspeed;
  uint32_t _last_consistent_heading;

  // time in millis when we last got a GPS heading
  uint32_t _gps_last_update;

  // the lat/lng where we last had GPS lock
  int32_t _last_lat;
  int32_t _last_lng;

  // position offset from last GPS lock
  float _position_offset_north;
  float _position_offset_east;

  // variables to cope with delaying the GA sum to match GPS lag
  Vector3f _ra_delay_buffer[INS_MAX_INSTANCES] ={0,0,0};

  // whether we have a position estimate
  bool _have_position;

  // whether we have GPS lock
  bool _have_gps_lock;

  uint32_t  _compass_last_update = 0;


  Matrix3f TempMat;

/*
 * Euler angel (Radian)
 */
float roll;
float pitch;
float yaw;


// helper trig variables
float _cos_roll, _cos_pitch, _cos_yaw;
float _sin_roll, _sin_pitch, _sin_yaw;

void dcm_init(void)
{
  _omega_I_sum_time = 0.0f;
  _renorm_val_sum = 0.0f;
  _renorm_val_count = 0;
  _error_rp_sum = 0.0f;
  _error_rp_count = 0.0f;
  _error_rp_last = 0.0f;
  _error_yaw_sum = 0.0f;
  _error_yaw_count = 0;
  _error_yaw_last = 0.0f;
  _gps_last_update = 0;
  _ra_deltat = 0.0f;
  _ra_sum_start = 0;
  _last_declination = 0.0f;
  _mag_earth.x = 0.0f; _mag_earth.y = 0.0f;
  _have_gps_lock = FALSE;
  _last_lat = 0;
  _last_lng = 0;
  _position_offset_north = 0.0f;
  _position_offset_east = 0.0f;
  _have_position = FALSE;
  _last_wind_time = 0;
  _last_airspeed = 0.0f;
  _last_consistent_heading = 0;
  _last_failure_ms = 0;


  upDate();
  roll = -atan(-ins_get_Accel().y/ins_get_Accel().z);
  pitch = atan(ins_get_Accel().x / sqrt(ins_get_Accel().y * ins_get_Accel().y + ins_get_Accel().z * ins_get_Accel().z));
  yaw = 0;

  matrix_identity(&_dcm_matrix);
  _ki = 0.0087;
  _ki_yaw = 0.00002; //0.01;


  from_euler(&_dcm_matrix,0,0,0);
  dcm_reset(TRUE);
  float temoFloat[3];
  getValuesF(&temoFloat[0] , &temoFloat[1] , &temoFloat[2]);

}


void dcm_update(void)
{


    float delta_t;

    // tell the IMU to grab some data
    //_ins.update();
    upDate();

    delta_t =get_delta_time();//wholeLoopTime;

    if (delta_t > 0.2f) {
         memset(&_ra_sum[0], 0, sizeof(_ra_sum));
         _ra_deltat = 0;
         return;
    }





    //chprintf((BaseSequentialStream *)&SD6,"b - %f \n",delta_t);
    // Integrate the DCM matrix using gyro inputs
    dcm_matrix_update(delta_t);


    // Normalize the DCM matrix
    normalize();

    // Perform drift correction
    drift_correction(delta_t);

    // paranoid check for bad values in the DCM matrix
    check_matrix();

    // Calculate pitch, roll, yaw for stabilization and navigation
    euler_angles();

    // update trig values including _cos_roll, cos_pitch
//    update_trig();


}


// update the DCM matrix using only the gyros
void dcm_matrix_update(float _G_Dt)
{
    Vector3f_zero(&_omega);
     _omega = ins_get_gyro();

    _omega = Vector3f_Sum(_omega, _omega_I);



    Mat_rotate(&_dcm_matrix,Vector3f_Mult_to_float((Vector3f_Sum(_omega,Vector3f_Sum(_omega_P,_omega_yaw_P))),_G_Dt));
    //chprintf((BaseSequentialStream *)&SD3,"%f   %f   %f  %f  %f   %f\n",_omega_yaw_P.x , _omega_yaw_P.y ,_omega_yaw_P.z ,_omega_P.x , _omega_P.y ,_omega_P .z);


}

void dcm_reset(bool_t recover_eulers)
{
    Vector3f_zero(&_omega_I);
    Vector3f_zero(&_omega_P);
    Vector3f_zero(&_omega_yaw_P);
    Vector3f_zero(&_omega);

    if(recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)){
      from_euler(&_dcm_matrix,roll,pitch,yaw);
    }else
    {
      from_euler(&_dcm_matrix,0,0,0);
    }

}

void reset_attitude(float _roll , float _pitch , float _yaw)
{
  from_euler(&_dcm_matrix,_roll, _pitch,_yaw);
}


/*
 *  check the DCM matrix for pathological values
 */
void check_matrix(void)
{
  if(matrix_isnan(_dcm_matrix))
  {
    dcm_reset(TRUE);
    return;
  }

  if (!(_dcm_matrix.c.x < 1.0f && _dcm_matrix.c.x > -1.0f))
  {

    normalize();

    if(matrix_isnan(_dcm_matrix) || fabsf(_dcm_matrix.c.x) > 10 )
    {
      dcm_reset(TRUE);
    }
  }

}

bool_t renorm(Vector3f a,Vector3f *result)
{
    float renorm_val;


    renorm_val = 1.0f / Vector3f_lenght(a);
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0f && renorm_val > 0.5f)){
      // this is larger than it should get - log it as a warning
      if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f))
      {
        return FALSE;
      }

    }

    *result = Vector3f_Mult_to_float(a,renorm_val);
    return TRUE;
}


void normalize(void)
{
    float error;
    Vector3f t0  = {0,0,0}, t1 = {0,0,0}, t2 = {0,0,0};


    error = Vector3f_dot_prod(_dcm_matrix.a , _dcm_matrix.b);// eg.18

    t0 = Vector3f_Sub(_dcm_matrix.a ,  Vector3f_Mult_to_float(_dcm_matrix.b , (0.5f * error)));              // eq.19
    t1 = Vector3f_Sub(_dcm_matrix.b , Vector3f_Mult_to_float(_dcm_matrix.a , (0.5f * error)));               // eq.19
    t2 = Vector3f_cross_prod(t0,t1);                                                                       // c= a x b // eq.20




    if (!renorm(t0, &_dcm_matrix.a) || !renorm(t1, &_dcm_matrix.b) || !renorm(t2, &_dcm_matrix.c)) {
            // Our solution is blowing up and we will force back
            // to last euler angles
            _last_failure_ms = chTimeNow();
            dcm_reset(TRUE);
        }
}


// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float yaw_error_compass(void)
{
    Vector3f mag = {0,0,0};
    getValuesF(&mag.x,&mag.y,&mag.z);

    Vector2f rb = mat_mulXY(_dcm_matrix,mag);

    Vector2f_normalize(&rb);

    if(Vector2f_isinf(rb))
    {
        // not a valid vector
        return 0.0;
    }


    // to be continue...
//    if(_last_declination != _Compass_declination)
//    {
//        _last_declination = _Compass_declination;
//        _mag_earth.x = cosf(_last_declination);
//        _mag_earth.y = sinf(_last_declination);
//    }


    // calculate the error term in earth frame
    // calculate the Z component of the cross product of rb and _mag_earth
    //chprintf((BaseSequentialStream *)&SD3,"%f\n",Vector2f_Cross_prod(rb , _mag_earth));
    return Vector2f_Cross_prod(rb , _mag_earth);

}


// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float _P_gain(float spin_rate)
{
    if (spin_rate < DEG_TO_RAD(50)) {
        return 1.0f;
    }
    if (spin_rate > DEG_TO_RAD(500)) {
        return 10.0f;
    }
    return spin_rate/DEG_TO_RAD(50);
}


// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold
// and/or filtering accelerations before getting magnitude
float _yaw_gain(void)
{
    float VdotEFmag = pythagorous2(_accel_ef[_active_accel_instance].x,
                                   _accel_ef[_active_accel_instance].y);
    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}


// return true if we have and should use GPS
//bool_t have_gps(void)
//{get_GPS_FIX_TYPE() < GPS_OK_FIX_3D
//    if (get_GPS_FIX_TYPE() <= NO_FIX || !_gps_use) {
//        return FALSE;
//    }
//    return TRUE;
//}



// return true if we should use the compass for yaw correction
bool_t use_compass(void)
{
    if (!USE_COMPASS || !USE_COMPASS_FOR_YAW) {
        // no compass available
        return false;
    }
    /*********/
    if (!_flags.fly_forward || !have_gps()) {
        // we don't have any alterative to the compass

        return TRUE;
    }
    if (get_GPS_ground_speed() < GPS_SPEED_MIN) {                       // in shart bar gharar nist
        // we are not going fast enough to use the GPS
        return TRUE;
    }


    /*********/
    // if the current yaw differs from the GPS yaw by more than 45
    // degrees and the estimated wind speed is less than 80% of the
    // ground speed, then switch to GPS navigation. This will help
    // prevent flyaways with very bad compass offsets
    int32_t error = abs((int)wrap_180_cd(yaw_sensor - get_GPS_ground_course_cd()));

    if (error > 4500 && Vector3f_lenght(_wind)< get_GPS_ground_speed()*0.8f) {
        if (chTimeNow() - _last_consistent_heading > 2000) {
            // start using the GPS for heading if the compass has been
            // inconsistent with the GPS for 2 seconds
            return false;
        }
    } else {
        _last_consistent_heading = chTimeNow();
    }

    // use the compass
    return true;
}


// yaw drift correction using the compass or GPS
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
//float mag_heading_x = 0;
//float mag_heading_y = 0;
//float errorCourse;
//Vector3f errorYaw;
//Vector3f Scaled_Omega_P;
//Vector3f Scaled_Omega_I;
void drift_correction_yaw(void)
{
    bool new_value = false;
    float yaw_error;
    float yaw_deltat;

    if (use_compass()) {

        if ( Compass_last_update() != _compass_last_update) {
            yaw_deltat = (Compass_last_update() - _compass_last_update) * 1.0e-6f;
            _compass_last_update = Compass_last_update();

            if (!_flags.have_initial_yaw && Compass_update()) {
                float heading = compass_calculate_heading(_dcm_matrix);
                from_euler(&_dcm_matrix,roll, pitch, heading);

                Vector3f_zero(&_omega_yaw_P);
                _flags.have_initial_yaw = true;

            }
            new_value = true;
            yaw_error = yaw_error_compass();

            //chprintf((BaseSequentialStream *)&SD3,"%f\n",yaw_error);

            _gps_last_update = chTimeNow();//get_GPS_last_fix_time_ms();
        }
    } else if (_flags.fly_forward && have_gps()) {

        /*
          we are using GPS for yaw
         */
        if (get_GPS_last_fix_time_ms() != _gps_last_update &&
            get_GPS_ground_speed() >= GPS_SPEED_MIN) {
            yaw_deltat = (get_GPS_last_fix_time_ms() - _gps_last_update) * 1.0e-3f;
            _gps_last_update = get_GPS_last_fix_time_ms();
            new_value = true;
            float gps_course_rad = DEG_TO_RAD(get_GPS_ground_course_cd() * 0.01f);
            float yaw_error_rad = wrap_PI(gps_course_rad - yaw);
            yaw_error = sinf(yaw_error_rad);



            if (!_flags.have_initial_yaw ||
                yaw_deltat > 20 ||
                (get_GPS_ground_speed() >= 3*GPS_SPEED_MIN && fabsf(yaw_error_rad) >= 1.047f)) {
                // reset DCM matrix based on current yaw
                from_euler(&_dcm_matrix,roll, pitch, gps_course_rad);
                Vector3f_zero(&_omega_yaw_P);
                _flags.have_initial_yaw = true;
                yaw_error = 0;
            }
        }
    }

    if (!new_value) {

        _omega_yaw_P = Vector3f_Mult_to_float(_omega_yaw_P,0.97f);

        return;
    }

    // convert the error vector to body frame
    float error_z = _dcm_matrix.c.z * yaw_error;

    // the spin rate changes the P gain, and disables the
    // integration at higher rates
    float spin_rate = Vector3f_lenght(_omega);



    _omega_yaw_P.z = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();
    if (_flags.fast_ground_gains) {
        _omega_yaw_P.z *= 8;
    }


    if (yaw_deltat < 2.0f && spin_rate < DEG_TO_RAD(SPIN_RATE_LIMIT)) {
        // also add to the I term
        _omega_I_sum.z += error_z * _ki_yaw * yaw_deltat;
    }

    _error_yaw_sum += fabsf(yaw_error);
    _error_yaw_count++;

//    float mag_x;
//    float mag_y;
//    float cos_roll;
//    float sin_roll;
//    float cos_pitch;
//    float sin_pitch;
//    float MAG_Heading;
//
//    cos_roll = cos(roll);
//    sin_roll = sin(roll);
//    cos_pitch = cos(pitch);
//    sin_pitch = sin(pitch);
//
//    float magnetom[3];
//    getValuesF(&magnetom[0] , &magnetom[1] ,&magnetom[2]);
//
//    // Tilt compensated magnetic field X
//    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
//    // Tilt compensated magnetic field Y
//    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
//    // Magnetic Heading
//    MAG_Heading = atan2(-mag_y, mag_x);
//
//    mag_heading_x = cos(MAG_Heading);
//    mag_heading_y = sin(MAG_Heading);
//
//    //errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
//    errorCourse = (_dcm_matrix.a.x * mag_heading_y) - (_dcm_matrix.b.x * mag_heading_x);
//    //Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depending the position.
//    errorYaw = Vector3f_Mult_to_float(_dcm_matrix.c , errorCourse);
//
//    //Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
//    Scaled_Omega_P = Vector3f_Mult_to_float(errorYaw , _kp_yaw);
//    //Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
//    _omega_P = Vector3f_Sum(_omega_P,Scaled_Omega_P);
//
//
//
//    //Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
//    Scaled_Omega_I = Vector3f_Mult_to_float(errorYaw,_ki_yaw);
//    //Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
//    _omega_I = Vector3f_Sum(_omega_I,Scaled_Omega_I);
}





/**
   return an accel vector delayed by AHRS_ACCEL_DELAY samples for a
   specific accelerometer instance
 */
Vector3f ra_delayed(uint8_t instance, Vector3f ra)
{
    // get the old element, and then fill it with the new element
    Vector3f ret = _ra_delay_buffer[instance];
    _ra_delay_buffer[instance] = ra;
    if (Vector3f_iszero(ret)) {
        // use the current vector if the previous vector is exactly
        // zero. This prevents an error on initialisation
        return ra;
    }
    return ret;
}


// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void drift_correction(float deltat)
{


    Vector3f velocity = {0,0,0};
    uint32_t last_correction_time;

    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();

    // rotate accelerometer values into the earth frame

    _accel_ef[0] = mul_Mat3f_to_Vec3f(_dcm_matrix , ins_get_Accel());


    // integrate the accel vector in the earth frame between GPS readings

    _ra_sum[0] = Vector3f_Sum(_ra_sum[0],Vector3f_Mult_to_float(_accel_ef[0] , deltat));



    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
      if(deltat > 0){
        _ra_deltat += deltat;
      }


    if (!have_gps() ||
        get_GPS_FIX_TYPE() < GPS_OK_FIX_3D ||
        get_GPS_num_sats() < _gps_minsats) {
        // no GPS, or not a good lock. From experience we need at
        // least 6 satellites to get a really reliable velocity number
        // from the GPS.
        //
        // As a fallback we use the fixed wing acceleration correction
        // if we have an airspeed estimate (which we only have if
        // _fly_forward is set), otherwise no correction
//        if (_ra_deltat < 0.02f) {
//            // not enough time has accumulated
//            return;
//
//        }

        float airspeed;
        if (Airspeed_using()) {
            airspeed = get_Airspeed();
        } else {
            airspeed = _last_airspeed;
        }
        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        velocity = Vector3f_Mult_to_float(matrix_colX(_dcm_matrix),airspeed);


        // add in wind estimate
        velocity = Vector3f_Sum(velocity,_wind);

        last_correction_time = chTimeNow();
        _have_gps_lock = FALSE;

    } else {
        if (get_GPS_last_fix_time_ms() == _ra_sum_start) {
            // we don't have a new GPS fix - nothing more to do

            return;
        }
        velocity = get_GPS_velocity();
        last_correction_time = get_GPS_last_fix_time_ms();
        if (_have_gps_lock == FALSE) {
            // if we didn't have GPS lock in the last drift
            // correction interval then set the velocities equal
            _last_velocity = velocity;
        }
        _have_gps_lock = TRUE;

        // keep last airspeed estimate for dead-reckoning purposes
        Vector3f airspeed = Vector3f_Sub(velocity , _wind);
        airspeed.z = 0;
        _last_airspeed = Vector3f_lenght(airspeed);

    }

    if (have_gps()) {
        // use GPS for positioning with any fix, even a 2D fix

        _last_lat = get_GPS_Location().lat;
        _last_lng = get_GPS_Location().lng;


        _position_offset_north = 0;
        _position_offset_east = 0;

        // once we have a single GPS lock, we can update using
        // dead-reckoning from then on
        _have_position = TRUE;
    } else {
        // update dead-reckoning position estimate
        _position_offset_north += velocity.x * _ra_deltat;
        _position_offset_east  += velocity.y * _ra_deltat;

    }

    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0) {
        _ra_sum_start = last_correction_time;
        _last_velocity = velocity;

        return;
    }

    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    Vector3f GA_e ={0, 0, -1.0f};

    bool using_gps_corrections = FALSE;
    float ra_scale = 1.0f/(_ra_deltat*GRAVITY_MSS);

    if (_flags.correct_centrifugal && (_have_gps_lock || _flags.fly_forward)) {                    // in shart bar gharar nists
        float v_scale = gps_gain * ra_scale;
        Vector3f vdelta = Vector3f_Mult_to_float(Vector3f_Sub(velocity , _last_velocity) , v_scale);
        GA_e = Vector3f_Sum(GA_e,vdelta);
        Vector3f_normalize(&GA_e);
        //palClearPad(GPIOE,12);
        if (Vector3f_isinf(GA_e)) {
            // wait for some non-zero acceleration information
            _last_failure_ms = chTimeNow();

            return;
        }
        using_gps_corrections = TRUE;
    }

    // calculate the error term in earth frame.
    // we do this for each available accelerometer then pick the
    // accelerometer that leads to the smallest error term. This takes
    // advantage of the different sample rates on different
    // accelerometers to dramatically reduce the impact of aliasing
    // due to harmonics of vibrations that match closely the sampling
    // rate of our accelerometers. On the Pixhawk we have the LSM303D
    // running at 800Hz and the MPU6000 running at 1kHz, by combining
    // the two the effects of aliasing are greatly reduced.
    Vector3f error[INS_MAX_INSTANCES];
    Vector3f GA_b[INS_MAX_INSTANCES];
    int8_t besti = -1;
    float best_error = 0;
    _ra_sum[0] = Vector3f_Mult_to_float(_ra_sum[0],ra_scale);


    // get the delayed ra_sum to match the GPS lag
    if (using_gps_corrections) {
        GA_b[0] = ra_delayed(0, _ra_sum[0]);
    } else {
        GA_b[0] = _ra_sum[0];

    }
    if (Vector3f_iszero(GA_b[0])) {
        // wait for some non-zero acceleration information
        //continue;
    }
    Vector3f_normalize(&GA_b[0]);
    if (Vector3f_isinf(GA_b[0])) {
        // wait for some non-zero acceleration information
        //continue;
    }
    error[0] = Vector3f_cross_prod(GA_b[0] ,GA_e) ;

    float error_length = Vector3f_lenght(error[0]);
    if (besti == -1 || error_length < best_error) {
        besti = 0;
        best_error = error_length;
    }


    if (besti == -1) {
        // no healthy accelerometers!
        _last_failure_ms = chTimeNow();

        return;
    }

    _active_accel_instance = besti;


#define YAW_INDEPENDENT_DRIFT_CORRECTION 0
#if YAW_INDEPENDENT_DRIFT_CORRECTION
    // step 2 calculate earth_error_Z
    float earth_error_Z = error.z;

    // equation 10
    float tilt = pythagorous2(GA_e.x, GA_e.y);

    // equation 11
    float theta = atan2f(GA_b[besti].y, GA_b[besti].x);

    // equation 12
    Vector3f GA_e2 = Vector3f(cosf(theta)*tilt, sinf(theta)*tilt, GA_e.z);

    // step 6
    error = GA_b[besti] % GA_e2;
    error.z = earth_error_Z;
#endif // YAW_INDEPENDENT_DRIFT_CORRECTION

    // to reduce the impact of two competing yaw controllers, we
    // reduce the impact of the gps/accelerometers on yaw when we are
    // flat, but still allow for yaw correction using the
    // accelerometers at high roll angles as long as we have a GPS

    if (use_compass()) {
        if (have_gps() && gps_gain == 1.0f) {
            error[besti].z *= sinf(fabsf(roll));
        } else {
            error[besti].z = 0;
        }
    }


    // if ins is unhealthy then stop attitude drift correction and
    // hope the gyros are OK for a while. Just slowly reduce _omega_P
    // to prevent previous bad accels from throwing us off
    if (!ins_healthy()) {
        Vector3f_zero(&error[besti]);
    } else {
        // convert the error term to body frame
        error[besti] = mul_transpose(_dcm_matrix,error[besti]);
    }

    if ( Vector3f_isnan(error[besti])  || Vector3f_isinf(error[besti])) {
        // don't allow bad values
        check_matrix();
        _last_failure_ms = chTimeNow();

        return;
    }

    _error_rp_sum += best_error;
    _error_rp_count++;

    // base the P gain on the spin rate
    float spin_rate =  Vector3f_lenght(_omega);

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P =Vector3f_Mult_to_float(error[besti] ,( _P_gain(spin_rate) * _kp));
    if (_flags.fast_ground_gains) {

        _omega_P =Vector3f_Mult_to_float(_omega_P, 8);
    }


    if (_flags.fly_forward && get_GPS_FIX_TYPE() >= GPS_OK_FIX_2D &&        // in shart bar gharar nist
        get_GPS_ground_speed() < GPS_SPEED_MIN &&
        ins_get_Accel().x >= 7 &&
        pitch_sensor > -3000 && pitch_sensor < 3000) {
            // assume we are in a launch acceleration, and reduce the
            // rp gain by 50% to reduce the impact of GPS lag on
            // takeoff attitude when using a catapult
            _omega_P =Vector3f_Mult_to_float(_omega_P, 0.5);

    }

    // accumulate some integrator error
    if (spin_rate < DEG_TO_RAD(SPIN_RATE_LIMIT)) {
        _omega_I_sum = Vector3f_Sum(_omega_I_sum ,Vector3f_Mult_to_float(error[besti] ,(_ki * _ra_deltat)));
        _omega_I_sum_time += _ra_deltat;

    }

    if (_omega_I_sum_time >= 5) {
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        float change_limit = gyro_drift_limit * _omega_I_sum_time;
        _omega_I_sum.x = constrain_float(_omega_I_sum.x, -change_limit, change_limit);
        _omega_I_sum.y = constrain_float(_omega_I_sum.y, -change_limit, change_limit);
        _omega_I_sum.z = constrain_float(_omega_I_sum.z, -change_limit, change_limit);
        _omega_I = Vector3f_Sum(_omega_I ,_omega_I_sum);
        Vector3f_zero(&_omega_I_sum);
        _omega_I_sum_time = 0;

    }

    // zero our accumulator ready for the next GPS step
    memset(&_ra_sum[0], 0, sizeof(_ra_sum));
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    _last_velocity = velocity;

    palClearPad(GPIOE,12);
    //drift_correction_yaw();

//    chprintf((BaseSequentialStream *)&SD3,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",_omega_P.x,_omega_P.y,_omega_P.z,_omega_I.x, _omega_I.y ,_omega_I.z
//                                                                             ,_omega_I_sum.x ,_omega_I_sum.y,_omega_I_sum.z);

}



// update our wind speed estimate
//void estimate_wind(void)
//{
//    if (!_flags.wind_estimation) {
//        return;
//    }
//    Vector3f velocity = _last_velocity;
//
//    // this is based on the wind speed estimation code from MatrixPilot by
//    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
//    // See http://gentlenav.googlecode.com/files/WindEstimation.pdf
//    Vector3f fuselageDirection = matrix_colX(_dcm_matrix);
//    Vector3f fuselageDirectionDiff = Vector3f_Sub(fuselageDirection , _last_fuse);
//    uint32_t now = hal_get_millis();
//
//    // scrap our data and start over if we're taking too long to get a direction change
//    if (now - _last_wind_time > 10000) {
//        _last_wind_time = now;
//        _last_fuse = fuselageDirection;
//        _last_vel = velocity;
//        return;
//    }
//
//    float diff_length = Vector3f_lenght(fuselageDirectionDiff);
//    if (diff_length > 0.2f) {
//        // when turning, use the attitude response to estimate
//        // wind speed
//        float V;
//        Vector3f velocityDiff = Vector3f_Sub(velocity , _last_vel);
//
//        // estimate airspeed it using equation 6
//        V = Vector3f_lenght(velocityDiff)/ diff_length;
//
//        _last_fuse = fuselageDirection;
//        _last_vel = velocity;
//
//        Vector3f fuselageDirectionSum = Vector3f_Sum(fuselageDirection , _last_fuse);
//        Vector3f velocitySum = Vector3f_Sum(velocity , _last_vel);
//
//        float theta = atan2f(velocityDiff.y, velocityDiff.x) - atan2f(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
//        float sintheta = sinf(theta);
//        float costheta = cosf(theta);
//
//        Vector3f wind;
//        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
//        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
//        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
//        wind = Vector3f_Mult_to_float(wind, 0.5f);
//
//        if (Vector3f_lenght(wind) < Vector3f_lenght(_wind)+ 20) {
//           // _wind = _wind * 0.95f + wind * 0.05f;
//            _wind =Vector3f_Sum(Vector3f_Mult_to_float(_wind,0.95f),Vector3f_Mult_to_float(wind,0.05f));
//        }
//
//        _last_wind_time = now;
//    } else if (now - _last_wind_time > 2000 && _airspeed && _airspeed->use()) {
//        // when flying straight use airspeed to get wind estimate if available
//        Vector3f airspeed = matrix_colX(_dcm_matrix)  * _airspeed->get_airspeed();
//        Vector3f wind = Vector3f_Sub(velocity , (Vector3f_Mult_to_float(airspeed,get_EAS2TAS())));
//        //_wind = _wind * 0.92f + wind * 0.08f;
//        _wind = Vector3f_Sum(Vector3f_Mult_to_float(_wind,0.92f),Vector3f_Mult_to_float(wind,0.08f));
//
//    }
//}

// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a
// positive vehicle rotation about that axis (ie a negative offset)
void euler_angles(void)
{
    _body_dcm_matrix = _dcm_matrix;
//    Mat_rotateXYinv(&_body_dcm_matrix,_trim);
    to_euler(_body_dcm_matrix,&roll, &pitch, &yaw);

//    yaw = atan2f(TempMat.b.x, TempMat.a.x);

    roll_sensor     = RAD_TO_DEG(roll)  * 100;
    pitch_sensor    = RAD_TO_DEG(pitch) * 100;
    yaw_sensor      = RAD_TO_DEG(yaw)   * 100;


    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}


/* reporting of DCM state for MAVLink */

//// average error_roll_pitch since last call
//float get_error_rp(void)
//{
//    if (_error_rp_count == 0) {
//        // this happens when telemetry is setup on two
//        // serial ports
//        return _error_rp_last;
//    }
//    _error_rp_last = _error_rp_sum / _error_rp_count;
//    _error_rp_sum = 0;
//    _error_rp_count = 0;
//    return _error_rp_last;
//}
//
//
//// average error_yaw since last call
//float get_error_yaw(void)
//{
//    if (_error_yaw_count == 0) {
//        // this happens when telemetry is setup on two
//        // serial ports
//        return _error_yaw_last;
//    }
//    _error_yaw_last = _error_yaw_sum / _error_yaw_count;
//    _error_yaw_sum = 0;
//    _error_yaw_count = 0;
//    return _error_yaw_last;
//}

// return our current position estimate using
// dead-reckoning or GPS
//bool_t get_position(Location loc)
//{
//    loc.lat = _last_lat;
//    loc.lng = _last_lng;
//    loc.alt = _baro.get_altitude() * 100 + _home.alt;
//    loc.flags.relative_alt = 0;
//    loc.flags.terrain_alt = 0;
//    location_offset(loc, _position_offset_north, _position_offset_east);
//    if (_flags.fly_forward && _have_position) {
//        location_update(loc, GPS_ground_course_cd * 0.01f, GPS_ground_speed * _gps.get_lag());
//    }
//    return _have_position;
//}


// return an airspeed estimate if available
//bool airspeed_estimate(float *airspeed_ret)
//{
//    bool ret = false;
//    if (_airspeed && _airspeed->use()) {
//        *airspeed_ret = _airspeed->get_airspeed();
//        return true;
//    }
//
//    if (!_flags.wind_estimation) {
//        return false;
//    }
//
//    // estimate it via GPS speed and wind
//    if (have_gps()) {
//        *airspeed_ret = _last_airspeed;
//        ret = true;
//    }
//
//    if (ret && _wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
//        // constrain the airspeed by the ground speed
//        // and AHRS_WIND_MAX
//        float gnd_speed = GPS_ground_speed;
//        float true_airspeed = *airspeed_ret * get_EAS2TAS();
//        true_airspeed = constrain_float(true_airspeed,
//                                        gnd_speed - _wind_max,
//                                        gnd_speed + _wind_max);
//        *airspeed_ret = true_airspeed / get_EAS2TAS();
//    }
//    return ret;
//}

//void set_home(Location loc)
//{
//    _home = loc;
//    _home.options = 0;
//}
//
///*
//  check if the AHRS subsystem is healthy
//*/
//bool    healthy(void)
//{
//    // consider ourselves healthy if there have been no failures for 5 seconds
//    return (_last_failure_ms == 0 || hal.scheduler->millis() - _last_failure_ms > 5000);
//}


// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void update_trig(void)
{

    Vector2f yaw_vector;
    Matrix3f temp = _body_dcm_matrix;

    // sin_yaw, cos_yaw
    yaw_vector.x = temp.a.x;
    yaw_vector.y = temp.b.x;
    Vector2f_normalize(&yaw_vector);
    _sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);
    _cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

    // cos_roll, cos_pitch
    float cx2 = temp.c.x * temp.c.x;
    if (cx2 >= 1.0f) {
        _cos_pitch = 0;
        _cos_roll = 1.0f;
    } else {
        _cos_pitch = safe_sqrt(1 - cx2);
        _cos_roll = temp.c.z / _cos_pitch;
    }
    _cos_pitch = constrain_float(_cos_pitch, 0, 1.0);
    _cos_roll = constrain_float(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

    // sin_roll, sin_pitch
    _sin_pitch = -temp.c.x;
    _sin_roll = temp.c.y / _cos_pitch;

}

// helper trig value accessors
float cos_roll()   { return _cos_roll; }
float cos_pitch()  { return _cos_pitch; }
float cos_yaw()    { return _cos_yaw; }
float sin_roll()   { return _sin_roll; }
float sin_pitch()  { return _sin_pitch; }
float sin_yaw()    { return _sin_yaw; }

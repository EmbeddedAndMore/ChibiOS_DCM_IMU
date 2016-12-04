/*
 * hmc5883.c
 *
 *  Created on: Apr 4, 2015
 *      Author: Mohamad Armoun
 */

#include "hmc5883.h"
#include "ch.h"
#include "hal.h"
#include <math.h>
//#include "compass_declination.h"
#include "DP_Math.h"
#include "Common.h"

uint8_t hmc_txbuf[6];
uint8_t hmc_rxbuf[6];
 i2cflags_t comp_errors;
 msg_t comp_status ;
 systime_t cmp_tmo = MS2ST(4);

float x_scale,y_scale,z_scale,x_max,y_max,z_max;
int16_t ix,iy,iz;
float  fc_x,fc_y,fc_z;

Vector3f _field[1] = {0,0,0};
float magData[3];
//---------------------------
float _Compass_declination ;
bool_t USE_COMPASS_FOR_YAW = TRUE;
bool_t USE_COMPASS = TRUE;
//int32_t yaw_sensor; // canti degree
uint32_t Comp_last_update;
bool_t comp_healthy = TRUE;
const int counts_per_milligauss[8]={
    1370,
    1090,
    820,
    660,
    440,
    390,
    330,
    230
  };
uint8_t HMC_readByte(uint8_t devAddr, uint8_t regAddr)
{

  hmc_txbuf[0] = regAddr;
  i2cAcquireBus(&I2CD2);
  comp_status = i2cMasterTransmit(&I2CD2,devAddr,hmc_txbuf,1,hmc_rxbuf,1);
  i2cReleaseBus(&I2CD2);
  comp_healthy = TRUE;
  if (comp_status != RDY_OK){
    comp_errors = i2cGetErrors(&I2CD2);
    comp_healthy = FALSE;
  }
  return hmc_rxbuf[0];
}
void HMC_writeByte(uint8_t devAddr , uint8_t regAddr , uint8_t Value)
{
  hmc_txbuf[0] = regAddr;
  hmc_txbuf[1] = Value;
  i2cAcquireBus(&I2CD2);
  comp_status = i2cMasterTransmit(&I2CD2,devAddr,hmc_txbuf,2,hmc_rxbuf,0);
  i2cReleaseBus(&I2CD2);
  comp_healthy = TRUE;
  if (comp_status != RDY_OK){
    comp_errors = i2cGetErrors(&I2CD2);
    comp_healthy = FALSE;
  }
}
void setMode(bool mode) {

   if (mode > 2) {
   return;
   }
   HMC_writeByte(HMC5883_ADDR,HMC58X3_R_MODE, mode);
   chThdSleepMilliseconds(5);
}


void getID(char id[3]){

  hmc_txbuf[0] = HMC58X3_R_IDA;
  i2cAcquireBus(&I2CD2);
  comp_status = i2cMasterTransmit(&I2CD2,HMC5883_ADDR,hmc_txbuf,1,hmc_rxbuf,3);
  i2cReleaseBus(&I2CD2);
  comp_healthy = TRUE;
  if (comp_status != RDY_OK){
    comp_errors = i2cGetErrors(&I2CD2);
    comp_healthy = FALSE;
  }
  id[0] = hmc_rxbuf[0];
  id[1] = hmc_rxbuf[1];
  id[2] = hmc_rxbuf[2];

}
void setGain(uint8_t gain){
  if(gain>7) return;
  HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFB, gain << 5);
}


uint8_t temp_cmp[6];
void getRaw(int16_t *x, int16_t *y , int16_t *z){

    int i = 0;
    for( ; i< 6 ; i++)
    {
      temp_cmp[i] = HMC_readByte(HMC5883_ADDR,HMC_XOUT_H + i);
    }

    *x = (temp_cmp[0] <<8 ) | temp_cmp[1];
    *z = (temp_cmp[2] <<8 ) | temp_cmp[3];
    *y = (temp_cmp[4] <<8 ) | temp_cmp[5];
}


bool_t calibrate(unsigned char gain,unsigned int n_samples)
{
    int16_t xyz[3];                     // 16 bit integer values for each axis.
    int xyz_total[3]={0,0,0};           // 32 bit totals so they won't overflow.
    bool bret=true;                     // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    char id[3];                         // Three identification registers should return 'H43'.
    int low_limit, high_limit;
    /*
        Make sure we are talking to the correct device.
        Hard to believe Honeywell didn't change the identifier.
    */
    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        getID(id);
        if (('H' == id[0]) && ('4' == id[1]) && ('3' == id[2]))
        {   /*
                Use the positive bias current to impose a known field on each axis.
               This field depends on the device and the axis.
            */
            HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFA, 0x11); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
            /*
                Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
                The new gain setting is effective from the second measurement and on.
            */
            setGain(gain);
            setMode(1);                         // Change to single measurement mode.
            getRaw(&xyz[0],&xyz[1],&xyz[2]);    // Get the raw values and ignore since this reading may use previous gain.
            unsigned int i=0;
            for (; i<n_samples; i++)
            {
                setMode(1);
                getRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged rather than taking the max.
                */
                xyz_total[0]+=xyz[0];
                xyz_total[1]+=xyz[1];
                xyz_total[2]+=xyz[2];
                chThdSleepMilliseconds(5);
                /*
                    Detect saturation.
                */

                if (-(1<<12) >= fmin(xyz[0],fmin(xyz[1],xyz[2])))
                {
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Apply the negative bias. (Same gain)
            */
            HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFA, 0x12); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.

            for (i=0; i<n_samples; i++)
            {
                setMode(1);
                getRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged.
                */
                xyz_total[0]-=xyz[0];
                xyz_total[1]-=xyz[1];
                xyz_total[2]-=xyz[2];

                chThdSleepMilliseconds(5);
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= fmin(xyz[0],fmin(xyz[1],xyz[2])))
                {
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Compare the values against the expected self test bias gauss.
                Notice, the same limits are applied to all axis.
            */
            low_limit =SELF_TEST_LOW_LIMIT *counts_per_milligauss[gain]*2*n_samples;
            high_limit=SELF_TEST_HIGH_LIMIT*counts_per_milligauss[gain]*2*n_samples;



            if ((true==bret) &&
                (low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
                (low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
                (low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2]) )
            {   /*
                    Successful calibration.
                    Normalize the scale factors so all axis return the same range of values for the bias field.
                    Factor of 2 is from summation of total of n_samples from both positive and negative bias.
                */

                x_scale=(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
                y_scale=(counts_per_milligauss[gain]*(HMC58X3_Y_SELF_TEST_GAUSS*2))/(xyz_total[1]/n_samples);
                z_scale=(counts_per_milligauss[gain]*(HMC58X3_Z_SELF_TEST_GAUSS*2))/(xyz_total[2]/n_samples);
            }else
            {
                bret=false;
            }
            HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default.
        }else
        {
            bret=false;
        }
    }else
    {   /*
            Bad input parameters.
        */
        bret=false;
    }
    return(bret);
}   //  calibrate().


void getValuesF(float *x,float *y,float *z){


  int i = 0;
  int16_t xr,yr,zr;
//  for( ; i< 6 ; i++)
//  {
//    temp_cmp[i] = HMC_readByte(HMC5883_ADDR,HMC_XOUT_H + i);
//  }
//
//  xr = (temp_cmp[0] <<8 ) | temp_cmp[1];
//  yr = (temp_cmp[2] <<8 ) | temp_cmp[3];
//  zr = (temp_cmp[4] <<8 ) | temp_cmp[5];


  getRaw(&xr, &yr, &zr);
  *x= -((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = -((float) zr) / z_scale;

  Comp_last_update = chTimeNow();
}

void getValues(int16_t *x,int16_t *y, int16_t *z){
  float fx,fy,fz;
  getValuesF(&fx,&fy,&fz);
  *x= (int16_t) (fx + 0.5);
  *y= (int16_t) (fy + 0.5);
  *z= (int16_t) (fz + 0.5);
}

void HMC5883_Init(bool setmode) {
  chThdSleepMilliseconds(20); // you need to wait at least 5ms after power on to initialize
  if (setmode) {
    setMode(0);

  }
  chThdSleepMilliseconds(10);
  HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFA,0x78);// 8 avarage,75 Hz outeput rate
  HMC_writeByte(HMC5883_ADDR,HMC58X3_R_CONFB ,0xA0);// set gain in 440 and -+ 4ga
  HMC_writeByte(HMC5883_ADDR,HMC58X3_R_MODE , 0x00);

  calibrate(1,32);
  setMode(0);
  chThdSleepMilliseconds(20);

}

//--------------------- based arduino code: compass.cpp-----------------
bool_t Compass_update(void)
{
    getValuesF(&_field[0].x,&_field[0].y,&_field[0].z);
    return TRUE;
}

float compass_calculate_heading(Matrix3f dcm_matrix)
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    Compass_update();

    // Tilt compensated magnetic field Y component:
    float headY = _field[0].y * dcm_matrix.c.z - _field[0].z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = _field[0].x * cos_pitch_sq - dcm_matrix.c.x * (_field[0].y * dcm_matrix.c.y + _field[0].z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_Compass_declination) > 0.0f )
    {
        heading = heading + _Compass_declination;
        if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * M_PI);
        else if (heading < -M_PI)
            heading += (2.0f * M_PI);
    }

    return heading;
}


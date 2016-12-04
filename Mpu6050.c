/*
 * Mpu6050.c
 *
 *  Created on: Dec 16, 2014
 *      Author: Aohamad Armoun
 */


#include "MPU6050.h"
#include "DP_Math.h"
#include "Common.h"
#include "chprintf.h"

#define byte unsigned char
//variables

 extern msg_t status ;
 extern systime_t tmo ;

 extern uint8_t rxbuf[6];
 extern uint8_t txbuf[6];
 extern i2cflags_t errors;

float read_mpu_delta_t;
float last_read_mpu_delta_t;
bool_t gyro_accel_healthy = TRUE;
// accelerometer values in the earth frame in m/s/s
Vector3f _accel_ef[1];
// which accelerometer instance is active
uint8_t _active_accel_instance;
uint8_t numberOfSample = 0;

Vector3f _gyro_offset = {0,0,0};
Vector3f _accel_offset = {0,0,0};
Vector3f _accel_scale = {1,1,1};

uint8_t readByte(uint8_t devAddr, uint8_t regAddr)
{

  txbuf[0] = regAddr;
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmit(&I2CD2,devAddr,txbuf,1,rxbuf,1);
  i2cReleaseBus(&I2CD2);

  gyro_accel_healthy = TRUE;
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD2);
    gyro_accel_healthy = FALSE;

  }
  return rxbuf[0];
}
void writeByte(uint8_t devAddr , uint8_t regAddr , uint8_t Value)
{
  txbuf[0] = regAddr;
  txbuf[1] = Value;
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmit(&I2CD2,devAddr,txbuf,2,rxbuf,0);
  i2cReleaseBus(&I2CD2);
  gyro_accel_healthy = TRUE;
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD2);
    gyro_accel_healthy = FALSE;
  }
}
void writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){

  uint8_t b = readByte(devAddr, regAddr);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask; // zero all non-important bits in data
  b &= ~(mask); // zero all important bits in existing byte
  b |= data; // combine data with existing byte
  writeByte(devAddr, regAddr, b);
}
void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
  uint8_t b = readByte(devAddr, regAddr);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  writeByte(devAddr, regAddr, b);

}


void mpu_setRate(uint8_t rate) {
    writeByte(mpu6050DevAddr, SMPLRT_DIV, rate);
}
void mpu_setDLPFMode(uint8_t mode) {
    writeByte(mpu6050DevAddr, config, mode);
}
void mpu_setClockSource(uint8_t source) {
  writeByte(mpu6050DevAddr,PWR_MGMT_1,source);
  //writeBits(mpu6050DevAddr,PWR_MGMT_1,2,3,source);
}
void mpu_setFullScaleGyroRange(uint8_t range) {
    writeBits(mpu6050DevAddr,Gyro_config,4,2,range);
}
void mpu_setFullScaleAccelRange(uint8_t range) {
    writeBits(mpu6050DevAddr,accel_config,4,2,range);
}
void mpu_setSleepEnable(bool enable){
    writeBit(mpu6050DevAddr,PWR_MGMT_1,6,enable);
}
void mpu_setI2CMasterModeEnabled(bool enabled) {
    writeBit(mpu6050DevAddr, USER_CTRL, 5, enabled);
}
void mpu_setI2CBypassEnabled(bool enabled) {
    writeBit(mpu6050DevAddr, INT_PIN_CFG, 1, enabled);
}

bool checkMpuWorking(uint8_t value)
{
  if(readByte(mpu6050DevAddr,PWR_MGMT_1) == value)
  {
    return true;
  }else{
    return false;
  }
}
unsigned char Samples=0;
int16_t Data[6]={0,0,0,0,0,0}; //Integral of Data
#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]));
inline void ReadMPU(uint8_t *value)
{


     uint32_t now = halGetCounterValue();
  // try not to use "for" loop because of performance
     value[0] = readByte(mpu6050DevAddr,GYRO_XOUT_H);
     value[1] = readByte(mpu6050DevAddr,GYRO_XOUT_L);
     value[2] = readByte(mpu6050DevAddr,GYRO_YOUT_H);
     value[3] = readByte(mpu6050DevAddr,GYRO_YOUT_L);
     value[4] = readByte(mpu6050DevAddr,GYRO_ZOUT_H);
     value[5] = readByte(mpu6050DevAddr,GYRO_ZOUT_L);
     value[6] = readByte(mpu6050DevAddr,ACCEL_XOUT_H);
     value[7] = readByte(mpu6050DevAddr,ACCEL_XOUT_L);
     value[8] = readByte(mpu6050DevAddr,ACCEL_YOUT_H);
     value[9] = readByte(mpu6050DevAddr,ACCEL_YOUT_L);
     value[10] = readByte(mpu6050DevAddr,ACCEL_ZOUT_H);
     value[11] = readByte(mpu6050DevAddr,ACCEL_ZOUT_L);


     Data[0]+=int16_val(value,1);
     Data[1]+=int16_val(value,0);
     Data[2]-=int16_val(value,2);
     Data[3]+=int16_val(value,4);
     Data[4]+=int16_val(value,3);
     Data[5]-=int16_val(value,5);
     Samples++;
     read_mpu_delta_t =(double)(((double)halGetCounterValue() - now)/ (double)halGetCounterFrequency());
}

void mpu6050_init(void){


  /*
   * gx_out = 2     gy_out = 3      gz_out = 4
   * accX_out = 5   accY_out= 6      accZ_out= 7
   */

  uint8_t tries;
    for(tries = 0; tries< 5;tries++)
    {
      // reset entire sensor
      writeByte(mpu6050DevAddr,PWR_MGMT_1,0x80);
      chThdSleepMilliseconds(100);


      // wakeUp sensor and set the clock select
      mpu_setClockSource(0x03);


      if(checkMpuWorking(0x03))
      {
        break;
      }

    }

    if(tries == 5)
    {
      //uartStartSend(&UARTD6,30,"Failed to boot MPU6050 5 times");//Try to notice user the faile of MPU6050
    }
    chThdSleepMilliseconds(20);


    // only used for wake-up in accelerometer only low power mode
    writeByte(mpu6050DevAddr,PWR_MGMT_2,0x00);
    chThdSleepMilliseconds(2);


  /*
   * samplerate = gyroscope output rate / 1+(SMPLRT_DIV)
   * DLPF : on => gyroscope output rate = 1 khz
   * DLPF : off => gyroscope output rate = 8 Khz
   * mpu_setRate(SMPLRT_DIV);
   *
   *
   * here : 1-  DLPF : on
   *        2-  SMPLRT_DIV = 0x04 =>samplerate = 200hz
   */
  mpu_setDLPFMode(BITS_DLPF_CFG_20HZ);
  chThdSleepMilliseconds(2);
  mpu_setRate(0x04);
  chThdSleepMilliseconds(2);


  /*
   *   0x00 : ±250 deg/s
   *   0x01 : ±500 deg/s
   *   0x02 : ±1000 deg/s
   *   0x03 : ±2000 deg/s
   */
  mpu_setFullScaleGyroRange(0x03);//0x03
  chThdSleepMilliseconds(2);

  /*
   * 0x00 : ±2g
   * 0x01 : ±4g
   * 0x02 : ±8g
   * 0x03 : ±16g
   */
  mpu_setFullScaleAccelRange(0x02);//0x01

  chThdSleepMilliseconds(2);
  mpu_setSleepEnable(0);


  mpu_setI2CMasterModeEnabled(0);// desable master mode


  mpu_setI2CBypassEnabled(1);

}



//_____________________________Mathematical unit_______________________________



void ZeroMem(void *T,unsigned char Length)
{
    char *C=T;
    unsigned int i=0;
    for(;i<Length;i++)
        C[i]=0;
}


unsigned int Delta=0;
void m_PollData(void)
{
//    if(halGetCounterValue()- Delta<MS2ST(5))
//        return ;
//    Delta = halGetCounterValue();
//    byte MPU60XX[14];

//    ReadGyroscope(MPU60XX);

//    chSysLock();
//    Data[0]+=int16_val(MPU60XX,0);
//    Data[1]+=int16_val(MPU60XX,1);
//    Data[2]+=int16_val(MPU60XX,2);
//    Data[3]+=int16_val(MPU60XX,3);
//    Data[4]+=int16_val(MPU60XX,4);
//    Data[5]+=int16_val(MPU60XX,5);
//    Samples++;
//    chSysUnlock();
//    if(Samples==0)
//    {
//        ZeroMem(Data,12);
//    }
}
//Gathered Data;
#define X 0
#define Y 1
#define Z 2

float Accel[3];
float Gyro[3];
float _Roll =0,
      _Pitch=0,
      _Yaw  =0;
uint8_t gyronputsX[3] = {15,10,18};
uint8_t gyronputsY[3] = {30,12,20};
uint8_t gyronputsZ[3] = {0,0,0};
Vector3f drift = {0,0,0};
void upDate()
{
  chSysLock();
  if(Samples>0)//Check to control system recieve sample from IMU
  {
     Gyro[X]=(float)(Data[0])*((float)GyroScale / (float)Samples) ;
     Gyro[Y]=(float)(Data[1])*((float)GyroScale / (float)Samples) ;
     Gyro[Z]=(float)(Data[2])*((float)GyroScale / (float)Samples) ;

     Gyro[X] -= _gyro_offset.x ;
     Gyro[Y] -= _gyro_offset.y ;
     Gyro[Z] -= _gyro_offset.z ;

     drift.x += Gyro[X];
     drift.y += Gyro[Y];
     drift.z += Gyro[Z];

     Accel[X]=(float)(Data[3])*((float)AccelScale/(float)Samples);
     Accel[Y]=(float)(Data[4])*((float)AccelScale/(float)Samples);
     Accel[Z]=(float)(Data[5])*((float)AccelScale/(float)Samples);



     Accel[X] -= _accel_offset.x;
     Accel[Y] -= _accel_offset.y;
     Accel[Z] -= _accel_offset.z;


   numberOfSample = Samples;
   Samples=0;
   memset(Data,0,12);
  }
  chSysUnlock();
  //CombineGyro_Accel();
}


float GetAngle(float v)
{
    float AccelR=sqrt(      //Later it should be done by atan2f
        (Accel[X]*Accel[X])+
        (Accel[Y]*Accel[Y])+
        (Accel[Z]*Accel[Z])
        );
    return ((acos(v / AccelR) * 57.295779513082320876798154814105) - 90.0f);
}
double f=0.2, g=0.1;
void CombineGyro_Accel()
{
  _Roll -=((float)Gyro[X]*0.05f);
  _Pitch+=((float)Gyro[Y]*0.05f);
  _Yaw  +=((float)Gyro[Z]*0.05f);
  if(_Roll!=_Roll)//Check for Nan Value
    _Roll=0;
  _Roll=(_Roll*0.98)+(GetAngle(Accel[Y])*0.02);

}

Vector3f ins_get_gyro(void)
{
    Vector3f tmp_vect = {Gyro[X],Gyro[Y],Gyro[Z]};
    return tmp_vect;
}
Vector3f ins_get_Accel(void)
{
    Vector3f tmp_vect = {Accel[X],Accel[Y],Accel[Z]};
    return tmp_vect;
}
Vector3f ins_get_Gyro_drift(void)
{
  return drift;
}
float get_delta_time(void)
{
  return 0.00126*numberOfSample;
}

void mpu6050_gyro_calibration(void)
{
    uint8_t  c = 0;
    uint16_t j = 0;
    Vector3f last_Average,best_Avg;
    float best_diff;
    bool_t converged;

    // start initialize gyro

    best_diff = 0;
    Vector3f_zero(&last_Average);
    converged = FALSE;

   for(c = 0 ; c<5 ; c++)
   {
       chThdSleepMilliseconds(10);
       upDate();
   }

   uint8_t num_converged = 0;

   for(j = 0 ; j <= 20 && num_converged < 1 ; j++)
   {
       Vector3f gyro_sum , gyro_avg , gyro_diff;
       float diff_norm;
       uint8_t i = 0;


       diff_norm = 0;

       Vector3f_zero(&gyro_sum);

       for(i = 0 ; i<50 ; i++)
       {
           upDate();

           gyro_sum = Vector3f_Sum(gyro_sum , ins_get_gyro());

           chThdSleepMilliseconds(10);
       }

       gyro_avg = Vector3f_div_to_float(gyro_sum , i);
       gyro_diff = Vector3f_Sub(last_Average , gyro_avg);
       diff_norm = Vector3f_lenght(gyro_diff);
       //chprintf((BaseSequentialStream*)&SD6,"%d- %f   %f   %f   %f\n",j,gyro_sum.x,gyro_sum.y,gyro_sum.z,diff_norm);
       if(j == 0)
       {
           best_diff = diff_norm;
           best_Avg = gyro_avg;

       }else if (Vector3f_lenght(gyro_diff) < DEG_TO_RAD(0.1))
       {
           last_Average = Vector3f_Sum(Vector3f_Mult_to_float(gyro_avg , 0.5f) , Vector3f_Mult_to_float(last_Average,0.5f));
           _gyro_offset = last_Average;
           //chprintf((BaseSequentialStream*)&SD6,"%f   %f   %f\n",gyro_avg.x,last_Average.x,_gyro_offset.x);
           converged = TRUE;
           num_converged++;

       }else if (diff_norm < best_diff)
       {
           best_diff = diff_norm;
           best_Avg =  Vector3f_Sum(Vector3f_Mult_to_float(gyro_avg , 0.5f) , Vector3f_Mult_to_float(last_Average,0.5f));
       }

       last_Average = gyro_avg;


       if(num_converged == 1)
       {
           return;
       }

   }

}


void mpu6050_accel_calibration1(void)
{
     uint8_t flashcount = 0;
     Vector3f prev;
     Vector3f accel_offset;
     float total_change = 0;
     float max_offset = 0;

     uint8_t i = 0;
     chThdSleepMilliseconds(100);


     Vector3f_zero(&_accel_offset);
     _accel_scale.x = _accel_scale.y = _accel_scale.z = 0;

     // clear accelerometer offsets and scaling
     accel_offset.x = accel_offset.y = accel_offset.z = 500;


     while (TRUE)
     {
         upDate();

         prev = accel_offset;
         accel_offset = ins_get_Accel();

         for(i = 0 ; i < 50 ; i++)
         {
             //chThdSleepMilliseconds(20);
             upDate();
             accel_offset = Vector3f_Sum(Vector3f_Mult_to_float(accel_offset , 0.9f),Vector3f_Mult_to_float(ins_get_Accel() , 0.1f));
             //chprintf((BaseSequentialStream *)&SD6," accel_calib1:%f   %f    %f  %d\n",ins_get_Accel().x,ins_get_Accel().y,ins_get_Accel().z,numberOfSample);

         }


         accel_offset.z += GRAVITY_MSS;
         total_change = fabsf(prev.x - accel_offset.x) +
                        fabsf(prev.y - accel_offset.y) +
                        fabsf(prev.z - accel_offset.z);

         max_offset = (accel_offset.x > accel_offset.y) ? accel_offset.x : accel_offset.y;
         max_offset = (max_offset > accel_offset.z) ? max_offset : accel_offset.z;


         uint8_t num_converged = 0;
         if(total_change <= AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE && max_offset <= AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET)
         {
             num_converged++;
         }

         if (num_converged == 1) break;

         chThdSleepMilliseconds(500);
     }


     _accel_offset =  accel_offset;

}
void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
bool_t _calibrate_accel(Vector3f accel_sample[6],Vector3f *accel_offsets, Vector3f *accel_scale);

void Mpu6050_Accel_Calibration(void)
{
      //uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
      Vector3f samples[6];
      Vector3f new_offsets;
      Vector3f new_scaling;
      Vector3f orig_offset;
      Vector3f orig_scale;
      uint8_t num_ok = 0;
      uint8_t i = 0;


      orig_offset.x = orig_offset.y = orig_offset.z = 0;
      orig_scale.x = orig_scale.y = orig_scale.z = 0;

      // clear accelerometer offsets and scaling
      _accel_offset.x = _accel_offset.y = _accel_offset.z = 0;
      _accel_scale.x = _accel_scale.y = _accel_scale.z = 1;

      // capture data from 6 positions
      for( i = 0 ; i < 6 ; i++)
      {
          switch(i)
          {

            case 0 :
              //chprintf((BaseSequentialStream*)&SD3, "Level\n");
              //uartStartSend(&UARTD6,6,"Level\n");
              chThdSleepMilliseconds(5000);
              break;
            case 1 :
              //chprintf((BaseSequentialStream*)&SD3, "on the LEFT side\n");
              //uartStartSend(&UARTD6,17,"on the LEFT side\n");
              chThdSleepMilliseconds(5000);
              break;
            case 2 :
              //chprintf((BaseSequentialStream*)&SD3, "on the RIGHT side\n");
              //uartStartSend(&UARTD6,18,"on the RIGHT side\n");
              chThdSleepMilliseconds(5000);
              break;
            case 3 :
              //chprintf((BaseSequentialStream*)&SD3, "Nose DOWN\n");
              //uartStartSend(&UARTD6,10,"Nose DOWN\n");
              chThdSleepMilliseconds(5000);
              break;
            case 4 :
              //chprintf((BaseSequentialStream*)&SD3, "Nose UP\n");
              //uartStartSend(&UARTD6,8,"Nose UP\n");
              chThdSleepMilliseconds(5000);
              break;
            default:    // default added to avoid compiler warning
            case 5 :
              //chprintf((BaseSequentialStream*)&SD3, "on the BACK\n");
              //uartStartSend(&UARTD6,12,"on the BACK\n");
              chThdSleepMilliseconds(5000);
              break;

          }


          // clear out any existing samples from ins
          upDate();

          samples[i].x = samples[i].y = samples[i].z = 0;
          uint8_t num_samples = 0;

          while (num_samples < 32)
          {
            // read samples from ins
            upDate();
            // capture sample

            chThdSleepMicroseconds(10);
            samples[i] = Vector3f_Sum(samples[i] , ins_get_Accel());
            //chprintf((BaseSequentialStream*)&SD3, "%f   %f   %f \n",ins_get_Accel().x,ins_get_Accel().y,ins_get_Accel().z);
           // chThdSleepMilliseconds(20);
            num_samples++;

          }

          samples[i] = Vector3f_div_to_float(samples[i] ,num_samples);
      }



      // run the calibration routine
      bool_t Success = _calibrate_accel(samples, &new_offsets, &new_scaling);

//      chprintf((BaseSequentialStream*)&SD3, "Success : %d\n",(uint8_t) Success);
//      chprintf((BaseSequentialStream*)&SD6, "new Scale : %f   %f    %f \n",new_scaling.x,new_scaling.y,new_scaling.z);

      if(Success)
      {
//        chprintf((BaseSequentialStream*)&SD6, "Calibration Successful \n");

        _accel_offset = new_offsets;
        _accel_scale = new_scaling;
        palSetPad(GPIOD,14);

      }

}

bool_t _calibrate_accel(Vector3f accel_sample[6],Vector3f *accel_offsets, Vector3f *accel_scale)
{
      int16_t i;
      int16_t num_iterations = 0;
      float eps = 0.000000001;
      float change = 100.0;
      float data[3];
      float beta[6];
      float delta[6];
      float ds[6];
      float JS[6][6];
      bool_t success = true;


      // reset
      beta[0] = beta[1] = beta[2] = 0;
      beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;

      while( num_iterations < 20 && change > eps )
      {
          num_iterations++;
          _calibrate_reset_matrices(ds, JS);

          for( i=0; i<6; i++ ) {
              data[0] = accel_sample[i].x;
              data[1] = accel_sample[i].y;
              data[2] = accel_sample[i].z;
              _calibrate_update_matrices(ds, JS, beta, data);
          }

          _calibrate_find_delta(ds, JS, delta);

          change =    delta[0]*delta[0] +
                      delta[0]*delta[0] +
                      delta[1]*delta[1] +
                      delta[2]*delta[2] +
                      delta[3]*delta[3] / (beta[3]*beta[3]) +
                      delta[4]*delta[4] / (beta[4]*beta[4]) +
                      delta[5]*delta[5] / (beta[5]*beta[5]);

          for( i=0; i<6; i++ ) {
              beta[i] -= delta[i];
          }

      }

      // copy results out

      accel_scale->x = beta[3] * GRAVITY_MSS;
      accel_scale->y = beta[4] * GRAVITY_MSS;
      accel_scale->z = beta[5] * GRAVITY_MSS;
      accel_offsets->x = beta[0] * accel_scale->x;
      accel_offsets->y = beta[1] * accel_scale->y;
      accel_offsets->z = beta[2] * accel_scale->z;

      // sanity check scale
      if(Vector3f_isnan(*accel_scale) || fabsf(accel_scale->x-1.0f) > 0.1f || fabsf(accel_scale->y-1.0f) > 0.1f || fabsf(accel_scale->z-1.0f) > 0.1f)
      {
          success = FALSE;
      }

      if( Vector3f_isnan(*accel_offsets) || fabsf(accel_offsets->x) > 3.5f || fabsf(accel_offsets->y) > 3.5f || fabsf(accel_offsets->z) > 3.5f ) {
          success = FALSE;
      }

      // return success or failure
      return success;

}


void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3])
{
      int16_t j, k;
      float dx, b;
      float residual = 1.0;
      float jacobian[6];

      for( j=0; j<3; j++ ) {
          b = beta[3+j];
          dx = (float)data[j] - beta[j];
          residual -= b*b*dx*dx;
          jacobian[j] = 2.0f*b*b*dx;
          jacobian[3+j] = -2.0f*b*dx*dx;
      }

      for( j=0; j<6; j++ ) {
          dS[j] += jacobian[j]*residual;
          for( k=0; k<6; k++ ) {
              JS[j][k] += jacobian[j]*jacobian[k];
          }
      }
}


void _calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}


void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

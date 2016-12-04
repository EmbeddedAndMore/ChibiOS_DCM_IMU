/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "MPU6050.h"
#include "chprintf.h"
#include "DCM.h"
#include "DCMmath.h"
#include "Common.h"
#include <math.h>
#include "MAVLink/common/mavlink.h"
#include "hmc5883.h"
#include "ms5611.h"
#include "DOWRAN_GPS/GPS_ublox.h"
#include "AHRS.h"


//#define RAD_TO_DEG 180/M_PI

union {uint8_t byte[4]; float value;}floatToByte;
union {uint8_t byte[4]; int value;}intToByte;
union {uint8_t byte[2]; int16_t value;}int16ToByte;
union {uint8_t byte[2]; uint16_t value;}uint16ToByte;
union {uint8_t byte[4]; uint32_t value;}uint32ToByte;


float lastLoopTime = 0;
float eachLoopTime = 0;
float wholeLoopTime = 0;



uint8_t rxbuf[6];
uint8_t txbuf[6];

bool_t RdyToSend = TRUE;

/*****************************  usart Driver  *******************************/
bool_t attitude_send_enabled = TRUE;
bool_t GPS_send_enabled = FALSE;
bool_t RawPress_send_enable = FALSE;
bool_t RawImu_send_enable = FALSE;
uint8_t startMessageCounet = 1;
static void rxchar(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
}
static void uart3_txend2(UARTDriver *uartp) {

  (void)uartp;
  chSysLockFromIsr();
  RdyToSend = TRUE;
  chSysUnlockFromIsr();
}

static UARTConfig uart_cfg_1 = {
  NULL,
  uart3_txend2,
  NULL,
  rxchar,
  NULL,
  115200,
  0,
  USART_CR2_LINEN,
  0
};

//static SerialConfig serial_conf = {
//     115200
//};
/************************************************************/




static const I2CConfig i2cfg2 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};
uint8_t myMPU[12];
uint8_t reading = 0;
static WORKING_AREA(Thread_Unit, 128);
static msg_t Thread1(void *arg) {
  (void)arg;

  while(TRUE)
  {
      //PollData();
      ReadMPU(myMPU);
      //reading++;
      chThdSleepMicroseconds(1);
      //chThdSleepMilliseconds(20);
  }
  return (msg_t)0;
}
inline int16_t byteToInt16(uint8_t msb,uint8_t lsb)
{
  int16_t temp = msb<<8 | lsb;
  return temp;
}

uint8_t GyroData[6];
uint8_t AccelData[6];

uint32_t timeNow = 0;

mavlink_message_t msg;
uint8_t buff[MAVLINK_MAX_PACKET_LEN];






inline void check_ready_to_send_params(void);
inline void Send_mavlink_rawImu(void);
inline void Send_mavlink_Attitude(void);
inline void Send_mavlink_GPS(void);
inline void Send_mavlink_rawPressure(void);

int main(void) {

  halInit();
  chSysInit();

  palSetPad(GPIOD,13);
  palSetPad(GPIOE,11);
  palSetPad(GPIOE,12);
  /*
   * Starts I2C
   */
  i2cStart(&I2CD2, &i2cfg2);


  /*
   * Start UART
   */
  //sdStart(&SD3,&serial_conf);
  uartStart(&UARTD3, &uart_cfg_1);



  mpu6050_init();
  chThdSleepMilliseconds(100);
  HMC5883_Init(TRUE);

  chThdSleepMilliseconds(10);
  ReadMPU(myMPU);
  dcm_init();
  chThdCreateStatic(Thread_Unit, sizeof(Thread_Unit), NORMALPRIO, Thread1, NULL);


  mpu6050_gyro_calibration();

  // mpu6050_accel_calibration1();
  // Mpu6050_Accel_Calibration();

   //chprintf((BaseSequentialStream *)&SD3," Accel_offset : %f   %f    %f   %d\n",_accel_offset.x,_accel_offset.y,_accel_offset.z , chCoreStatus());

   uint8_t i = 0;
   uint32_t myLoopTime = chTimeNow();
   uint32_t mytime = 0;

   float beginReadingLoop = 0.0;
   float endReadingLoop = 0.0;
  while(true){


   dcm_update();
   mytime = chTimeNow() - myLoopTime;
   myLoopTime = chTimeNow();







    if(chTimeNow() - timeNow >= 25){
      timeNow = chTimeNow();
      //chprintf((BaseSequentialStream *)&SD3,"%f   %f   %f   %f   %f   %f\n",RAD_TO_DEG(roll),RAD_TO_DEG(pitch),RAD_TO_DEG(yaw) ,ins_get_Accel().x,ins_get_Accel().y ,ins_get_Accel().z);
      check_ready_to_send_params();
    }
  }
}


inline void check_ready_to_send_params(void)
{
  if(RdyToSend && startMessageCounet == 1 && attitude_send_enabled)
   {
      RdyToSend = FALSE;
      startMessageCounet++;
      Send_mavlink_Attitude();
      attitude_send_enabled = FALSE;
      RawImu_send_enable = TRUE;
   }
   else if(RdyToSend && startMessageCounet == 2 && RawImu_send_enable)
   {
      RdyToSend = FALSE;
      startMessageCounet++;
      Send_mavlink_rawImu();
      RawImu_send_enable = FALSE;
      GPS_send_enabled = TRUE;
   }
   else if(RdyToSend && startMessageCounet == 3 && GPS_send_enabled)
   {
       RdyToSend = FALSE;
       startMessageCounet++;
       Send_mavlink_GPS();
       GPS_send_enabled = FALSE;
       RawPress_send_enable = TRUE;
   }
   else if(RdyToSend && startMessageCounet == 4 && RawPress_send_enable)
   {
       RdyToSend = FALSE;
       startMessageCounet = 1;
       Send_mavlink_rawPressure();
       RawPress_send_enable = FALSE;
       attitude_send_enabled = TRUE;
   }

}

inline void Send_mavlink_rawImu(void)
{
    mavlink_msg_attitude_quaternion_pack(1,1,&msg,0,ins_get_Accel().x,ins_get_Accel().y,ins_get_Accel().z,ins_get_gyro().x,ins_get_gyro().y,ins_get_gyro().z,0);
    uint16_t len = mavlink_msg_to_send_buffer(buff,&msg);
    uartStartSendI(&UARTD3,len,buff);
}

inline void Send_mavlink_Attitude(void)
{

      mavlink_msg_attitude_pack(1,1,&msg , chTimeNow() , RAD_TO_DEG(roll),RAD_TO_DEG(pitch),RAD_TO_DEG(yaw) , ins_get_Gyro_drift().x ,ins_get_Gyro_drift().y , ins_get_Gyro_drift().z);
      uint16_t len = mavlink_msg_to_send_buffer(buff,&msg);
      uartStartSend(&UARTD3,len,buff);
}

inline void Send_mavlink_GPS(void)
{
    mavlink_msg_gps_raw_int_pack(1,1,&msg,chCoreStatus(),0,latitude,longitude,(CurrentAlt-Home_Altitude)*1000,0,0,0,0,0);
    uint16_t len = mavlink_msg_to_send_buffer(buff,&msg);
    uartStartSendI(&UARTD3,len,buff);
}

inline void Send_mavlink_rawPressure(void)
{
    //mavlink_msg_raw_pressure_pack(1,1,&msg,0,0,CurrentPress,0,CurrentTemp);
  mavlink_msg_raw_pressure_pack(1,1,&msg,0,0,12,0,12);
    uint16_t len = mavlink_msg_to_send_buffer(buff,&msg);
    uartStartSendI(&UARTD3,len,buff);
}

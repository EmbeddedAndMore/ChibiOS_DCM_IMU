/*
 * hmc5883.h
 *
 *  Created on: Apr 4, 2015
 *      Author: Mohamad Armoun
 */
#include "ch.h"
#include "hal.h"
#include "DP_Math.h"
#include "DCMmath.h"


#ifndef HMC5883L_RegMap_H_
#define HMC5883L_RegMap_H_

#define HMC5883_ADDR 0x1E
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// registers configuration

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

// data register 
#define HMC_XOUT_H 3
#define HMC_XOUT_L 4
#define HMC_ZOUT_H 5
#define HMC_ZOUT_L 6
#define HMC_YOUT_H 7
#define HMC_YOUT_L 8

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.




#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12


extern float        _Compass_declination;
extern bool_t       USE_COMPASS_FOR_YAW;
extern bool_t       USE_COMPASS;
extern int32_t      yaw_sensor;
extern bool_t       comp_healthy;
extern uint32_t     Comp_last_update;

extern float magData[3];

 uint8_t HMC_readByte(uint8_t devAddr, uint8_t regAddr);
 void HMC_writeByte(uint8_t devAddr , uint8_t regAddr , uint8_t Value);
 void setMode(bool mode);
 void getID(char id[3]);
 void setGain(uint8_t gain);
 void getRaw(int16_t *x, int16_t *y , int16_t *z);
 bool_t calibrate(unsigned char gain,unsigned int n_samples);
void getValuesF(float *x,float *y,float *z);
void getValues(int16_t *x,int16_t *y, int16_t *z);
void HMC5883_Init(bool setmode);

bool_t Compass_update(void);
float compass_calculate_heading(Matrix3f dcm_matrix);

#endif // HMC58X3_h


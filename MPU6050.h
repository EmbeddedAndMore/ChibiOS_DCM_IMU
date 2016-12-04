#ifndef _MPU6050_RegMap_H_
#define _MPU6050_RegMap_H_

#include "hal.h"
#include "ch.h"
#include <math.h>
#include <string.h>
#include "DP_Math.h"
//#include "Common.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define mpu6050DevAddr MPU6050_DEFAULT_ADDRESS


#define SMPLRT_DIV 0x19
#define config 0x1A
#define Gyro_config 0x1B
#define accel_config 0x1C
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A 
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34

// interrupt

#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_S TA TUS 0x3A

// Accel data READ

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// Tempreture data Read

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

// Gyro data READ

#define GYRO_XOUT_H 0x43 
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48


// DLPF Setting

#define BITS_DLPF_CFG_256HZ_NOLPF2                      0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF                      0x07
#define BITS_DLPF_CFG_MASK                              0x07


// additional

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define EXT_SENS_DATA_00 0x49
#define USER_CTRL 0x6A
#define MPU6050_CLOCK_PLL_XGYRO         0x01

#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
//_______________________________Parameters_____________________________________
//#define
//#define GRAVITY_MSS  9.80665f
#define GyroScale   (0.0174532f / 16.4f)//16.384//2^16/±8
#define AccelScale  (9.80665 /4096)//4096//2^16/±2000
#define INS_MAX_INSTANCES 1
//______________________________________________________________________________
extern float read_mpu_delta_t;
extern Vector3f _accel_ef[1];
extern uint8_t _active_accel_instance;
extern bool_t gyro_accel_healthy;
extern Vector3f _gyro_offset;
extern Vector3f _accel_offset;
extern Vector3f _accel_scale;
extern uint8_t numberOfSample;

extern float Accel[3];
extern float Gyro[3];

// functions

uint8_t readByte(uint8_t devAddr, uint8_t regAddr);
void writeByte(uint8_t devAddr , uint8_t regAddr , uint8_t Value);
void writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
uint8_t readByte(uint8_t devAddr, uint8_t regAddr);

void mpu_setRate(uint8_t rate);
void mpu_setDLPFMode(uint8_t mode);
void mpu_setClockSource(uint8_t source);
void mpu_setFullScaleGyroRange(uint8_t range);
void mpu_setFullScaleAccelRange(uint8_t range);
void mpu_setSleepEnable(bool enable);
void mpu_setI2CMasterModeEnabled(bool enabled);
void mpu_setI2CBypassEnabled(bool enabled);
void mpu6050_init(void);
void mpu6050_gyro_calibration(void);
void Mpu6050_Accel_Calibration(void);
void mpu6050_accel_calibration1(void);
//------------------------------------------------------------------------------
inline void ReadMPU(uint8_t *value);
void m_PollData(void);
void ZeroMem(void *T,unsigned char Length);
void upDate(void);//Update Data
float GetAngle(float v);
void CombineGyro_Accel(void);
float get_delta_time(void);

Vector3f ins_get_gyro(void);
Vector3f ins_get_Accel(void);
Vector3f ins_get_Gyro_drift(void);


//------------------------------------------------------------------------------
#endif

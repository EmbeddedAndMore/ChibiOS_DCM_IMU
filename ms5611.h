/*
 * ms5611-01ba03.h
 *
 *  Created on: Jan 24, 2015
 *      Author: Mohamad Armoun
 */


#ifndef _MS5611_RegMap_H_
#define _MS5611_RegMap_H_

#include "hal.h"
#include "ch.h"

#define Exponent    0.1902225f
#define SEA_PRESS    1013.25f
#define MOVAVG_SIZE     32


#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device

#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2

/*     pressure         */
#define MS561101BA03_D1_256     0x40
#define MS561101BA03_D1_512     0x42
#define MS561101BA03_D1_1024     0x44
#define MS561101BA03_D1_2048     0x46
#define MS561101BA03_D1_4096     0x48


/*    temperature       */
#define MS561101BA03_D2_256     0x50
#define MS561101BA03_D2_512     0x52
#define MS561101BA03_D2_1024     0x54
#define MS561101BA03_D2_2048     0x56
#define MS561101BA03_D2_4096     0x58



void baroReset(void);
void baroReadPROM(void);
void startConversion(uint8_t command);
uint32_t getRawPressure(void);
uint32_t getRawTemperature(void);
float getPressure(uint32_t rawPressure , int32_t deltaTemp);
float getTemperature(int32_t deltaTemp);
int32_t getDeltaTemp(uint32_t rawTemperture);
float getAltitude(float press, float temp);
void pushAvg(float val);
float getAvg(float * buff, int size);
void ms5611_init(void);






extern float Ground_temp;
extern bool_t baro_healthy;
extern float Home_Altitude ;
extern float CurrentAlt;
//extern float movavg_buff[MOVAVG_SIZE];
//extern int movavg_i;


#endif

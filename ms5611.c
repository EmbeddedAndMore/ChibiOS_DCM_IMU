/*
 * ms5611.c
 *
 *  Created on: Jan 24, 2015
 *      Author: Mohamad Armoun
 */


#include "ms5611.h"
#include <math.h>
#include "DP_Math.h"
#include "DOWRAN_AnalogInputs/AirSpeed.h"

 i2cflags_t errors = 0;
 msg_t status = RDY_OK ;
 systime_t tmo = MS2ST(4);

 uint32_t D1;    //raw pressure
 uint32_t D2;    //raw temperature

uint16_t Coeffi[6];

float Home_Altitude = 0;
float Ground_temp = 0;
float Ground_press = 0;
float CurrentAlt = 0;
static float current_alt = 0;
static float last_altitude_EAS2TAS = 0;
static float EAS2TAS;
bool_t baro_healthy = TRUE;

uint8_t baro_rxbuf[6];
uint8_t baro_txbuf[6];
/**
 *  Reset Barometer
 *  it happens once after power up.
 */
void baroReset(void){
    baro_txbuf[0] = 0x1E;
    i2cAcquireBus(&I2CD2);
    status = i2cMasterTransmitTimeout(&I2CD2,MS561101BA_ADDR_CSB_LOW,baro_txbuf,1,baro_rxbuf,0,tmo);
    i2cReleaseBus(&I2CD2);
    baro_healthy = TRUE;
    if (status != RDY_OK){
      errors = i2cGetErrors(&I2CD2);
      baro_healthy = FALSE;
    }
}

/**
 * this function reads Coefficients that is saved in PROM
 * there is 6 Value(16 bit each one) in the PROM
 *
 */
void baroReadPROM(void){
  int i = 0;
  for(i=0 ;i < 6 ; i++)
  {
      baro_txbuf[0] = 0xA2 + (i * 2);
      i2cAcquireBus(&I2CD2);
      status = i2cMasterTransmitTimeout(&I2CD2,MS561101BA_ADDR_CSB_LOW,baro_txbuf,1,baro_rxbuf,2,tmo);
      i2cReleaseBus(&I2CD2);

      Coeffi[i] = (baro_rxbuf[0]<<8) | baro_rxbuf[1];
      baro_healthy = TRUE;
      if (status != RDY_OK){
        errors = i2cGetErrors(&I2CD2);
        baro_healthy = FALSE;
      }

  }
}

/**
 * this commands to barometer start conversion
 * after conversion we should hold on as i describe :
 * OSR : 4096 => 8.22 ms
 *     : 2048 => 4.13 ms
 *     : 1024 => 2.08 ms
 *     : 512 => 1.6 ms
 *     : 256 => 0.54 ms
 */
void startConversion(uint8_t command)
{

      baro_txbuf[0] = command;
      i2cAcquireBus(&I2CD2);
      status = i2cMasterTransmitTimeout(&I2CD2,MS561101BA_ADDR_CSB_LOW,baro_txbuf,1,baro_rxbuf,0,tmo);
      i2cReleaseBus(&I2CD2);
      baro_healthy = TRUE;
      if (status != RDY_OK){
        errors = i2cGetErrors(&I2CD2);
        baro_healthy = FALSE;
      }
}



/**
 * this should start 8 ms after startConversion()
 *
 */
uint32_t getRawPressure(void)
{
  union {uint32_t val; uint8_t raw[4];} baro_data = {0};
  baro_txbuf[0] = 0x00;
  i2cAcquireBus(&I2CD2);
   status = i2cMasterTransmitTimeout(&I2CD2,MS561101BA_ADDR_CSB_LOW,baro_txbuf,1,baro_rxbuf,3,tmo);
   i2cReleaseBus(&I2CD2);
   baro_healthy = TRUE;
   if (status != RDY_OK){
     errors = i2cGetErrors(&I2CD2);
     baro_healthy = FALSE;
   }

  baro_data.raw[2] = baro_rxbuf[0];
  baro_data.raw[1] = baro_rxbuf[1];
  baro_data.raw[0] = baro_rxbuf[2];
  D1 = baro_data.val;
  return baro_data.val;
}


/**
 * this should start 8 ms after startConversion()
 *
 */
uint32_t getRawTemperature(void)
{
  union {uint32_t val; uint8_t raw[4];} Temp_data = {0};
    baro_txbuf[0] = 0x00;
   i2cAcquireBus(&I2CD2);
   status = i2cMasterTransmitTimeout(&I2CD2,MS561101BA_ADDR_CSB_LOW,baro_txbuf,1,baro_rxbuf,3,tmo);
   i2cReleaseBus(&I2CD2);
   baro_healthy = TRUE;
   if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD2);
    baro_healthy = FALSE;
   }

  Temp_data.raw[2] = baro_rxbuf[0];
  Temp_data.raw[1] = baro_rxbuf[1];
  Temp_data.raw[0] = baro_rxbuf[2];
  D2 = Temp_data.val;
  return Temp_data.val;
}



/**
 *  GET pressure using raw pressure and raw temperature
 *  NOTE : it should be placed after getRawTemperature() and getRawPressure()
 */
float getPressure(uint32_t rawPressure , int32_t deltaTemp){
  uint32_t rawPress = rawPressure;
  int32_t dT = deltaTemp;
  float temperature = getTemperature(dT);

  int64_t off  = ((uint32_t)Coeffi[1] <<16) + (((int64_t)dT * Coeffi[3]) >> 7);
  int64_t sens = ((uint32_t)Coeffi[0] <<15) + (((int64_t)dT * Coeffi[2]) >> 8);

  if(temperature < 20)
  {

    // low temperature
    float T2 = (dT * dT) >> 31;
    int32_t off2 = 5*pow((temperature-2000),2);
    off2 = off2 >> 1 ;
    int32_t sens2 = 5*(pow((temperature-2000),2));
    sens2 = sens2 >> 2;

    if(temperature < -15)
    {
      off2 = off2 + 7 *(pow((temperature+1500),2));
      sens2 = sens2 +((int32_t)(11 * pow((temperature+1500),2))>>2);
    }

    D2 = temperature - T2 ;
    off = off - off2;
    sens = sens - sens2;
  }
  return  ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}



/**
 *  get Temprature
 */
float getTemperature(int32_t deltaTemp){
  int64_t dT = deltaTemp;
  if(dT != 0) {
    return (2000 + ((dT * Coeffi[5]) >> 23)) / 100.0;
  }
  return 0;
}



/**
 * Calculate deltaTemp
 */
inline int32_t getDeltaTemp(uint32_t rawTemperture){
  uint32_t rawTemp = rawTemperture;
  if(rawTemp != 0) {
       return (int32_t)(rawTemp - ((uint32_t)Coeffi[4] << 8));
  }
  else
    return 0;
}


/*
 * calc the Altitude from sea level
 */
float getAltitude(float press, float temp) {

  return ((pow((SEA_PRESS / press), Exponent) - 1.0) * (temp + 273.15)) / 0.0065;
}




float getAvg(float * buff, int size) {
  float sum = 0.0;
  int i=0;
  for(; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}
/*
 * this function had been called as board power on.
 * when the board get powered , it's considered that the current altitude is it's
 * home altitude .
 * we calculate home altitude and then the AGL altitude will be calculates in get_AGL_Altitude()
 */
void Calc_homeAlt(void)
{
    float prs = 0;
    int32_t temp_dt = 0;
    uint8_t i = 0;
    for( ; i< 10 ; i++)
    {
        startConversion(MS561101BA03_D2_4096);
        chThdSleepMicroseconds(9040);
        temp_dt = getDeltaTemp(getRawTemperature());
        startConversion(MS561101BA03_D1_4096);
        chThdSleepMicroseconds(9040);
        prs +=getPressure(getRawPressure(),temp_dt) ;
    }
    Ground_temp =getTemperature(temp_dt);
    Ground_press = prs/10;
    Home_Altitude = getAltitude(Ground_press,Ground_temp);

}


float get_AGL_Altitude(float press, float temp)
{
     current_alt = getAltitude(press,temp) - Home_Altitude;
     return current_alt;
}


float get_EAS2TAS(void)
{
    if ((fabsf(current_alt - last_altitude_EAS2TAS) < 100.0f) && (EAS2TAS != 0.0f)) {
           // not enough change to require re-calculating
           return EAS2TAS;
       }
    float tempK = ((float)Ground_temp) + 273.15f - 0.0065f * current_alt;
    EAS2TAS = safe_sqrt(1.225f / ((float)get_Airspeed() / (287.26f * tempK)));
    last_altitude_EAS2TAS = current_alt;
    return EAS2TAS;
}

void ms5611_init(void)
{
  baroReset();
  chThdSleepMilliseconds(10);
  baroReadPROM();
  chThdSleepMilliseconds(100);

   chThdSleepMilliseconds(10);


  int i=0;
  for(i = 0 ; i<MOVAVG_SIZE; i++) {
    startConversion(MS561101BA03_D2_4096);
    chThdSleepMicroseconds(9040);
    getRawTemperature();
    startConversion(MS561101BA03_D1_4096);
    chThdSleepMicroseconds(9040);
    getRawPressure();
//   movavg_buff[i] = getPressure(D1,getDeltaTemp(D2));
  }


  Calc_homeAlt();
}


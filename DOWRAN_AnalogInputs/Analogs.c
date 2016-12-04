/*
 * Analogs.c
 *
 *  Created on: Apr 16, 2015
 *      Author: Mohamad Armoun
 */

#include "ch.h"
#include "hal.h"
#include "Analogs.h"
#define VOLTS_TO_PASCAL 1241 // 4096/3.3

uint16_t sumAir = 0 , sumAX = 0;
static adcsample_t AdcSamples[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];
/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;
  if (AdcSamples == buffer) {
    nx += n;
  }
  else {
    ny += n;
  }
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}



/*
 * ADC conversion group.
 * Mode:        Continuous, 10 samples of 6 channels, SW triggered.
 * Channels:    IN10, IN11, IN12, IN13, IN2 , IN3
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN13(ADC_SAMPLE_144) | ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_144) | ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144) ,
  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_144) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_144),                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_GRP2_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN13) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN12) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN10) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2)
};




void initADC(void)
{


    palSetGroupMode(GPIOC,PAL_PORT_BIT(0) | PAL_PORT_BIT(1) | PAL_PORT_BIT(2) | PAL_PORT_BIT(3),
                    0, PAL_MODE_INPUT_ANALOG);
    palSetGroupMode(GPIOA,PAL_PORT_BIT(2) | PAL_PORT_BIT(3),
                      0, PAL_MODE_INPUT_ANALOG);

  /*
   * Activates the ADC1 driver
   */
   adcStart(&ADCD1, NULL);

   adcStartConversion(&ADCD1, &adcgrpcfg1, AdcSamples, ADC_GRP2_BUF_DEPTH);
}


bool_t get_difrential_pressure(float *pressure)
{
    *pressure = (float)AdcSamples[0] ;
    return TRUE;
}

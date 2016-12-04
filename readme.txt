*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F407.                            **
*****************************************************************************

This repo contains DCM (directional cosine matrix) based IMU which developed over ChibiOS RTOS (http://www.chibios.org/) and It's charming HAL.

the motivation for this job raised when i was studing ArdoPilot code (https://github.com/ArduPilot/ardupilot) 

i used GY-86 (MPU6050 , MS5611 , HMC5883) and also stm32f407VGT on my customized board (tested with stm32f407 discovery board)

in order to get data from IMU i used Mavlink (http://qgroundcontrol.org/mavlink/start) which is ready simple and handy to use.

the code is still naive and need to be overviewd and debuged (specialy in sensor calibration). any contribution would be appreciated.

Thanks,
MA.

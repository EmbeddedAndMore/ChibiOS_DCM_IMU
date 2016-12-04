/*
 * GPS.h
 *
 *  Created on: Feb 4, 2015
 *      Author: Mohamad Armoun
 */

#ifndef GPS_H_
#define GPS_H_
#include "ch.h"
#include "hal.h"
#include "DP_Math.h"

   typedef enum  {
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_AUTO  = 1,
        GPS_TYPE_UBLOX = 2,
        GPS_TYPE_MTK   = 3,
        GPS_TYPE_MTK19 = 4,
        GPS_TYPE_NMEA  = 5,
        GPS_TYPE_SIRF  = 6,
        GPS_TYPE_HIL   = 7,
        GPS_TYPE_SBP   = 8
    }GPS_Type;


    /// GPS status codes
   enum GPS_Status {
        NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
    };

    // GPS navigation engine settings. Not all GPS receivers support
    // this
   typedef  enum  {
         GPS_ENGINE_NONE        = -1,
         GPS_ENGINE_PORTABLE    = 0,
         GPS_ENGINE_STATIONARY  = 2,
         GPS_ENGINE_PEDESTRIAN  = 3,
         GPS_ENGINE_AUTOMOTIVE  = 4,
         GPS_ENGINE_SEA         = 5,
         GPS_ENGINE_AIRBORNE_1G = 6,
         GPS_ENGINE_AIRBORNE_2G = 7,
         GPS_ENGINE_AIRBORNE_4G = 8
     }GPS_Engine_Setting;



//============================================================================
//=
//============================================================================
#define UBLOX_SET_BINARY "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,38400,0*26\r\n"
#define UBX "$PUBX,41,1,0003,0001,38400,0*26\r\n"

#define     PREAMBLE1 0xB5
#define     PREAMBLE2 0x62
#define     CLASS_NAV  0x01
#define     CLASS_ACK  0x05
#define     CLASS_CFG  0x06
#define     MSG_ACK_NACK  0x00
#define     MSG_ACK_ACK  0x01
#define     MSG_POSLLH  0x2
#define     MSG_STATUS  0x3
#define     MSG_SOL  0x6
#define     MSG_VELNED  0x12
#define     MSG_CFG_PRT  0x00
#define     MSG_CFG_RATE  0x08
#define     MSG_CFG_SET_RATE  0x01
#define     MSG_CFG_NAV_SETTINGS  0x24

#define     NAV_STATUS_FIX_VALID  1

typedef struct ubx_header {

  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
}ubloxHeadr;


typedef struct ubx_cfg_nav_rate {
  uint16_t measure_rate_ms;
  uint16_t nav_rate;
  uint16_t timeref;
 }ubx_nav_rate;

typedef struct ubx_cfg_msg_rate {
  uint8_t msg_class;
  uint8_t msg_id;
  uint8_t rate;
}ubx_msg_rate;


//=============================================================================
//=
//=============================================================================
typedef struct  {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t res1;
        uint32_t res2;
        uint32_t res3;
        uint32_t res4;
    }ubx_cfg_nav_settings;

typedef struct  {
        uint32_t time;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    }ubx_nav_posllh;


 typedef struct  {
        uint32_t time;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    }ubx_nav_status;


typedef struct  {
        uint32_t time;
        int32_t time_nsec;
        uint16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    }ubx_nav_solution;


typedef struct  {
        uint32_t time;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    }ubx_nav_velned;


// Lea6 uses a 60 byte message
typedef struct  {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[17];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    }ubx_mon_hw_60 ;

    // Neo7 uses a 68 byte message
typedef struct  {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[25];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    }ubx_mon_hw_68;


typedef struct  {
        int8_t ofsI;
        uint8_t magI;
        int8_t ofsQ;
        uint8_t magQ;
        uint8_t cfgSource;
        uint8_t reserved0[3];
        uint32_t lowLevCfg;
        uint32_t reserved1[2];
        uint32_t postStatus;
        uint32_t reserved2;
    }ubx_mon_hw2;








//******************************************************************************



void configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
void Gps_Config_Navigation_rate(uint16_t rate_ms);
void Gps_Configuration(void);
void gps_uart_init(UARTConfig uart_cfg);
inline void gps_test(void);

void Ublox_Read(void);
void Ublox_Parse_Gps(void);

void GPS_Start_Reading(void);
void GPS_Stop_Reading(void);
//******************************************************************************
// Packet checksum accumulators
extern uint8_t         _ck_a;
extern uint8_t         _ck_b;

      // State machine state
extern uint8_t         step;
extern uint8_t         msg_id;
extern uint16_t        payload_length;
extern uint16_t        payload_counter;


extern uint8_t         _class;
extern uint32_t        last_pos_time;
extern int32_t         latitude;
extern int32_t         longitude;
extern int32_t         gps_altitude;

extern uint8_t          FIX_TYPE;
extern float            ground_speed;
extern int32_t          ground_course_cd;
extern bool             have_vertical_velocity;
extern float            velocity_x,velocity_y,velocity_z;
extern Vector3f         GPS_vlocity;
extern Vector3f         NED_vlocity;
extern uint8_t          satellites;
extern uint32_t         last_fix_time_ms;

#endif /* GPS_H_ */

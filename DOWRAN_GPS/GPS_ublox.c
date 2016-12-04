/*
 * GPS.c
 *
 *  Created on: Feb 5, 2015
 *      Author: Mohamad Armoun
 */


#include "GPS_ublox.h"
#include "ch.h"
#include "hal.h"
#include "Common.h"



union PACKED {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_solution solution;
        ubx_nav_velned velned;
        ubx_cfg_nav_settings nav_settings;
        ubx_mon_hw_60 mon_hw_60;
        ubx_mon_hw_68 mon_hw_68;
        ubx_mon_hw2 mon_hw2;
        uint8_t bytes[1];
    } _buffer;




unsigned char buffer_POSLLH[] =   { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47 };//1 hrz
unsigned char buffer_STATUS[] =   { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49 };//1 hrz
unsigned char buffer_SOL[] =      { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F };//1 hrz
unsigned char buffer_VELNED[] =   { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67 };//1 hrz


unsigned char buffer_NAV_RAte[]= { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x00, 0x00,0xDD,0x68 };


uint8_t RecBuffer[164];

// Packet checksum accumulators
      uint8_t         _ck_a;
      uint8_t         _ck_b;

      // State machine state
      uint8_t         step = 0;
      uint8_t         msg_id;
      uint16_t        payload_length;
      uint16_t        payload_counter;


      uint8_t         _class;
      uint32_t        last_pos_time;
      int32_t           latitude;
      int32_t           longitude;
      int32_t           gps_altitude;

    // GPS fix type
    uint8_t            FIX_TYPE;
    float              ground_speed;
    int32_t            ground_course_cd;
    bool               have_vertical_velocity = 1;
    float              velocity_x,velocity_y,velocity_z;
    uint8_t            satellites;
    uint32_t              last_fix_time_ms;
    Vector3f           NED_vlocity;
//============================================================================
//=       uart x configuration for GPS
//============================================================================
    static void GPS_txend1(UARTDriver *uartp) {

      (void)uartp;
    }

    /*
     * This callback is invoked when a transmission has physically completed.
     */
    static void GPS_txend2(UARTDriver *uartp) {

      (void)uartp;
      chSysLockFromIsr();
      //Ublox_Read();
      chSysUnlockFromIsr();
    }
    static void GPS_rxerr(UARTDriver *uartp, uartflags_t e) {

      (void)uartp;
      (void)e;
    }
    static void GPS_rxchar(UARTDriver *uartp, uint16_t c) {

      (void)uartp;
      (void)c;
    }
    static void GPS_rxend(UARTDriver *uartp) {

      (void)uartp;

      /* Flashing the LED each time a character is received.*/

      chSysLockFromIsr();
      uint8_t data;
              palSetPad(GPIOD,14);

               int i = 0;
               for(i = 0 ; i < 164 ; i++)
               {
                   data = RecBuffer[i];
                reset:
                   switch(step)
                   {
                       case 1 :
                           if(PREAMBLE2 == data)
                           {
                             step++;
                             break;
                           }
                           step = 0;
                       case 0 :
                           if(PREAMBLE1 == data){
                              step++;
                           }
                            break;
                       case 2 :
                           step++;
                           _class = data;
                           _ck_b = _ck_a = data;
                           break;
                       case 3:
                           step++;
                           _ck_b += (_ck_a += data);
                           msg_id = data;
                           break;
                       case 4:
                           step++;
                           _ck_b += (_ck_a += data);                   // checksum byte
                           payload_length = data;
                           break;
                       case 5:
                           step++;
                           _ck_b += (_ck_a += data);

                           payload_length += (uint16_t)(data<<8);

                           if(payload_length > 512)
                           {
                               payload_length = 0;
                               step = 0;
                               goto reset;
                           }
                           payload_counter = 0;                        // prepare to receive payload
                           break;
                       case 6:
                           _ck_b += (_ck_a += data);                   // checksum byte
                           if (payload_counter < sizeof(_buffer)) {
                               _buffer.bytes[payload_counter] = data;

                           }
                           if (++payload_counter == payload_length)
                               step++;
                           break;
                       case 7:
                           step++;
                           if (_ck_a != data) {
                               step = 0;
                               goto reset;
                           }
                           break;
                       case 8 :
                           step = 0;
                           if (_ck_b != data) {
                               break;
                           }

                           if (_class == CLASS_ACK) {}

                                      if (_class == CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS) {
                              //              Debug("Got engine settings %u\n", (unsigned)_buffer.nav_settings.dynModel);
                              //              if (gps._navfilter != AP_GPS::GPS_ENGINE_NONE &&
                              //                  _buffer.nav_settings.dynModel != gps._navfilter) {
                              //                  // we've received the current nav settings, change the engine
                              //                  // settings and send them back
                              //                  Debug("Changing engine setting from %u to %u\n",
                              //                        (unsigned)_buffer.nav_settings.dynModel, (unsigned)gps._navfilter);
                              //                  _buffer.nav_settings.dynModel = gps._navfilter;
                              //                  _buffer.nav_settings.mask = 1; // only change dynamic model
                              //                  _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                              //                                &_buffer.nav_settings,
                              //                                sizeof(_buffer.nav_settings));
                              //              }
                              //              return false;
                                      }

                                      switch(msg_id)
                                      {
                                      case MSG_POSLLH:
                                                      last_pos_time  =    _buffer.posllh.time;
                                                      longitude      =    _buffer.posllh.longitude;
                                                      latitude       =    _buffer.posllh.latitude;
                                                      gps_altitude       =    _buffer.posllh.altitude_msl / 10;

                                                      break;
                                      case MSG_STATUS:
                                                      if(_buffer.status.fix_status & NAV_STATUS_FIX_VALID)
                                                      {
                                                          if(_buffer.status.fix_type == GPS_OK_FIX_3D)
                                                          {
                                                            FIX_TYPE = GPS_OK_FIX_3D;
                                                            last_fix_time_ms = chTimeNow();
                                                          }else if(_buffer.status.fix_type == GPS_OK_FIX_2D)
                                                          {
                                                            FIX_TYPE = GPS_OK_FIX_2D;
                                                            last_fix_time_ms = chTimeNow();
                                                          }else
                                                          {
                                                            FIX_TYPE = NO_FIX;
                                                          }

                                                      }else
                                                      {
                                                        FIX_TYPE = NO_FIX;
                                                      }
                                                      break;

                                      case MSG_SOL:
                                                      if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID)
                                                      {
                                                        if(_buffer.solution.fix_type == GPS_OK_FIX_3D)
                                                        {
                                                            FIX_TYPE = GPS_OK_FIX_3D;
                                                        }else if(_buffer.solution.fix_type == GPS_OK_FIX_2D)
                                                        {
                                                            FIX_TYPE = GPS_OK_FIX_2D;
                                                        }else
                                                        {
                                                          FIX_TYPE = NO_FIX;
                                                        }
                                                      }else
                                                      {
                                                        FIX_TYPE = NO_FIX;
                                                      }
                                                      satellites = _buffer.solution.satellites;

                                                      break;

                                      case MSG_VELNED:
                                                      ground_speed =          _buffer.velned.speed_2d*0.01f; //m/s
                                                      ground_course_cd =      _buffer.velned.heading_2d / 1000;       // Heading 2D deg * 100000 rescaled to deg * 100
                                                      have_vertical_velocity = TRUE;
                                                      NED_vlocity.x =            _buffer.velned.ned_north * 0.01f;
                                                      NED_vlocity.y =            _buffer.velned.ned_east * 0.01f;
                                                      NED_vlocity.z =            _buffer.velned.ned_down * 0.01f;
                                                      break;
                                      }
                       }
                   }


      chSysUnlockFromIsr();
    }

// static UARTConfig uart_cfg_GPS_1 = {
//   GPS_txend1,
//   GPS_txend2,
//   GPS_rxerr,
//   GPS_rxchar,
//   GPS_rxend,
//   57600,
//   0,
//   USART_CR2_LINEN,
//   0
// };
 static UARTConfig uart_cfg_GPS_2 = {
   GPS_txend1,
   GPS_txend2,
   GPS_rxend,
   GPS_rxchar,
   GPS_rxerr,
   38400,
   0,
   USART_CR2_LINEN,
   0
 };
// static UARTConfig uart_cfg_GPS_3 = {
//   GPS_txend1,
//   GPS_txend2,
//   GPS_rxerr,
//   GPS_rxchar,
//   GPS_rxend,
//   19200,
//   0,
//   USART_CR2_LINEN,
//   0
// };
 static UARTConfig uart_cfg_GPS_4 = {
   GPS_txend1,
   GPS_txend2,
   NULL,
   GPS_rxchar,
   GPS_rxerr,
   9600,
   0,
   USART_CR2_LINEN,
   0
 };







 static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);
 static void send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);




// UBlox auto configuration

void gps_uart_init(UARTConfig uart_cfg)
{
    uartStart(&UARTD2, &uart_cfg);
}

/*
 *  configure a UBlox GPS for the given message rate
 */
void Gps_Configuration(void)
{

      uartStart(&UARTD2, &uart_cfg_GPS_4);

      uartStartSend(&UARTD2,14,buffer_NAV_RAte);
      chThdSleepMilliseconds(15);
      uartStartSend(&UARTD2,11,buffer_POSLLH);
      chThdSleepMilliseconds(15);
      uartStartSend(&UARTD2,11,buffer_STATUS);
      chThdSleepMilliseconds(15);
      uartStartSend(&UARTD2,11,buffer_SOL);
      chThdSleepMilliseconds(15);
      uartStartSend(&UARTD2,11,buffer_VELNED);
      chThdSleepMilliseconds(15);
      uartStartSend(&UARTD2,33,UBX);
      chThdSleepMilliseconds(40);



      uartStop(&UARTD2);
      chThdSleepMilliseconds(200);

      uartStart(&UARTD2, &uart_cfg_GPS_2);


}

void GPS_Start_Reading(void)
{
    uartStartReceive(&UARTD2,164,RecBuffer);
}

void GPS_Stop_Reading(void)
{
    uartStopReceive(&UARTD2);
}

/*
 *  configure a UBlox GPS navigation solution rate of 200ms
 */
void Gps_Config_Navigation_rate(uint16_t rate_ms)
{
    ubx_nav_rate msg;
    msg.measure_rate_ms = rate_ms;
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}


/*
 *  configure a UBlox GPS for the given message rate for a specific
 *  message class and msg_id
 */
void configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
      ubx_msg_rate msg;
      msg.msg_class = msg_class;
      msg.msg_id    = msg_id;
      msg.rate          = rate;
      send_message(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
}



/*
 *  send a ublox message
 */
static void send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
   ubloxHeadr header;
   uint8_t ck_a=0, ck_b=0;
   header.preamble1 = PREAMBLE1;
   header.preamble2 = PREAMBLE2;
   header.msg_class = msg_class;
   header.msg_id    = msg_id;
   header.length    = size;


   update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, &ck_a, &ck_b) ;
   update_checksum((uint8_t *)msg, size, &ck_a, &ck_b);



   uartStartSend(&UARTD2,sizeof(header),&header);
   chThdSleepMicroseconds(2000);

   if(size > 0){
     uartStartSend(&UARTD2,size,msg);
     chThdSleepMicroseconds(2000);
   }

   uartStartSend(&UARTD2,1,&ck_a);
   chThdSleepMicroseconds(500);


   uartStartSend(&UARTD2,1,&ck_b);
   chThdSleepMicroseconds(500);


}
/*
 *  update checksum for a set of bytes
 */
static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
  while (len--) {
          *ck_a += *data;
          *ck_b += *ck_a;
          data++;
      }
}
//
//void Ublox_Read(void)
//{
//
//
//          uint8_t data;
//
//          int i = 0;
//          for(i = 0 ; i < 164 ; i++)
//          {
//              data = RecBuffer[i];
//           reset:
//              switch(step)
//              {
//                  case 1 :
//                      if(PREAMBLE2 == data)
//                      {
//                        step++;
//                        break;
//                      }
//                      step = 0;
//                  case 0 :
//                      if(PREAMBLE1 == data){
//                         step++;
//                      }
//                       break;
//                  case 2 :
//                      step++;
//                      _class = data;
//                      _ck_b = _ck_a = data;
//                      break;
//                  case 3:
//                      step++;
//                      _ck_b += (_ck_a += data);
//                      msg_id = data;
//                      break;
//                  case 4:
//                      step++;
//                      _ck_b += (_ck_a += data);                   // checksum byte
//                      payload_length = data;                             // payload length low byte
//                      break;
//                  case 5:
//                      step++;
//                      _ck_b += (_ck_a += data);                   // checksum byte
//
//                      payload_length += (uint16_t)(data<<8);
//
//                      if(payload_length > 512)
//                      {
//                          payload_length = 0;
//                          step = 0;
//                          goto reset;
//                      }
//                      payload_counter = 0;                               // prepare to receive payload
//                      break;
//                  case 6:
//                      _ck_b += (_ck_a += data);                   // checksum byte
//                      if (payload_counter < sizeof(_buffer)) {
//                          _buffer.bytes[payload_counter] = data;
//                      }
//                      if (++payload_counter == payload_length)
//                          step++;
//                      break;
//                  case 7:
//                      step++;
//                      if (_ck_a != data) {
//                          step = 0;
//                          goto reset;
//                      }
//                      break;
//                  case 8 :
//                      step = 0;
//                      if (_ck_b != data) {
//                          break;                                                  // bad checksum
//                      }
//
//                      Ublox_Parse_Gps();
//                  }
//              }
//
//
//}
//
//void Ublox_Parse_Gps(void)
//{
//           if (_class == CLASS_ACK) {}
//
//           if (_class == CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS) {
//   //              Debug("Got engine settings %u\n", (unsigned)_buffer.nav_settings.dynModel);
//   //              if (gps._navfilter != AP_GPS::GPS_ENGINE_NONE &&
//   //                  _buffer.nav_settings.dynModel != gps._navfilter) {
//   //                  // we've received the current nav settings, change the engine
//   //                  // settings and send them back
//   //                  Debug("Changing engine setting from %u to %u\n",
//   //                        (unsigned)_buffer.nav_settings.dynModel, (unsigned)gps._navfilter);
//   //                  _buffer.nav_settings.dynModel = gps._navfilter;
//   //                  _buffer.nav_settings.mask = 1; // only change dynamic model
//   //                  _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
//   //                                &_buffer.nav_settings,
//   //                                sizeof(_buffer.nav_settings));
//   //              }
//   //              return false;
//           }
//
//           switch(msg_id)
//           {
//           case MSG_POSLLH:
//                           last_pos_time  =    _buffer.posllh.time;
//                           longitude      =    _buffer.posllh.longitude;
//                           latitude       =    _buffer.posllh.latitude;
//                           altitude       =    _buffer.posllh.altitude_msl / 10;
//                           palSetPad(GPIOD,12);
//                           break;
//           case MSG_STATUS:
//                           if(_buffer.status.fix_status & NAV_STATUS_FIX_VALID)
//                           {
//                               if(_buffer.status.fix_type == GPS_OK_FIX_3D)
//                               {
//                                 GPS_FIX_TYPE = GPS_OK_FIX_3D;
//                               }else if(_buffer.status.fix_type == GPS_OK_FIX_2D)
//                               {
//                                 GPS_FIX_TYPE = GPS_OK_FIX_2D;
//                               }else
//                               {
//                                 GPS_FIX_TYPE = NO_FIX;
//                               }
//
//                           }else
//                           {
//                             GPS_FIX_TYPE = NO_FIX;
//                           }
//                           break;
//
//           case MSG_SOL:
//                           if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID)
//                           {
//                             if(_buffer.solution.fix_type == GPS_OK_FIX_3D)
//                             {
//                                 GPS_FIX_TYPE = GPS_OK_FIX_3D;
//                             }else if(_buffer.solution.fix_type == GPS_OK_FIX_2D)
//                             {
//                                 GPS_FIX_TYPE = GPS_OK_FIX_2D;
//                             }else
//                             {
//                               GPS_FIX_TYPE = NO_FIX;
//                             }
//                           }else
//                           {
//                             GPS_FIX_TYPE = NO_FIX;
//                           }
//                           break;
//
//           case MSG_VELNED:
//                           ground_speed =          _buffer.velned.speed_2d*0.01f; //m/s
//                           ground_course_cd =      _buffer.velned.heading_2d / 1000;       // Heading 2D deg * 100000 rescaled to deg * 100
//                           have_vertical_velocity = TRUE;
//                           velocity_x =            _buffer.velned.ned_north * 0.01f;
//                           velocity_y =            _buffer.velned.ned_east * 0.01f;
//                           velocity_z =            _buffer.velned.ned_down * 0.01f;
//                           break;
//           }
//}





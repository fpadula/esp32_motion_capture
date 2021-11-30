#ifndef _COMMCONSTS_H_
#define _COMMCONSTS_H_

#define IMU_MSG_TYPE_DATA '0'
#define IMU_MSG_TYPE_OK '1'
#define IMU_MSG_TYPE_ERROR '2'
#define IMU_MSG_TYPE_INFO '3'
#define IMU_BDCST_RESP '4'

#define IMU_CAL_ACCGYRO '0'
#define IMU_CAL_MAG '1'
#define IMU_CHANGEPORT '2'
#define IMU_CHANGEIP '3'
#define IMU_SAVE_CFG '4'
#define IMU_LOAD_CFG '5'
#define IMU_DISCOVER '6'

#define DISCOVERY_PORT 55055

#define MAXLINE 1024

typedef uint16_t msg_size_type;

/*****************************************
Message structure:
    Message type | Message size in bytes | Message data

Field sizes:
    Message type: char (1 byte)
    Message size in bytes: uint16_t (2 bytes)
    Message data: byte (char) array of 'Message size' size)
*****************************************/

#endif
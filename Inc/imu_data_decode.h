#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>
#define MAX_LENGTH 16

extern uint8_t bitmap;

#define BIT_VALID_ID   (0x01)
#define BIT_VALID_ACC  (0x02)
#define BIT_VALID_GYR  (0x04)
#define BIT_VALID_MAG  (0x08)
#define BIT_VALID_EUL  (0x10)
#define BIT_VALID_QUAT (0x20)
#define BIT_VALID_TIME (0x40)
#define BIT_VALID_ALL  (BIT_VALID_ID | BIT_VALID_ACC | BIT_VALID_GYR | BIT_VALID_MAG | BIT_VALID_EUL | BIT_VALID_QUAT | BIT_VALID_TIME)

__packed typedef  struct  id0x91_t {
        uint8_t     tag;                /* 0x91 */
        uint8_t     id;
        uint8_t     rev[6];             /* reserved */
        uint32_t    ts;                 /* timestamp */
        float       acc[3];
        float       gyr[3];
        float       mag[3];
        float       eul[3];             /* eular angles:R/P/Y */
        float       quat[4];            /* quaternion */

} id0x91_t;

__packed typedef  struct  id0x62_t {
	uint8_t tag;
	uint8_t gw_id;
	uint8_t n;
	id0x91_t id0x91[MAX_LENGTH];
} id0x62_t;
	 
extern id0x91_t id0x91;
extern id0x62_t id0x62;
typedef enum 
{
    kItemID =                   0x90,   /* user programed ID   */
    kItemAccRaw =               0xA0,   /* raw acc             */
    kItemGyrRaw =               0xB0,   /* raw gyro            */  
    kItemMagRaw =               0xC0,   /* raw mag             */
    kItemRotationEul =          0xD0,   /* eular angle         */
    kItemRotationQuat =         0xD1,   /* att q               */
    kItemPressure =             0xF0,   /* pressure            */
    kItemEnd =                  0x00,   
	KItemIMUSOL =               0x91,   /* IMUSOL  */
	KItemGWSOL =                0x62,   /* RFSOL  */
}ItemID_t;

int imu_data_decode_init(void);
int get_imu_data(id0x91_t *data);
int get_gw_data(id0x62_t *data);

#endif

 

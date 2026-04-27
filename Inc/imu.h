#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"
typedef struct
{

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;


void imu_init(void);
void imu_get(void);
extern imu_t imu;
#endif

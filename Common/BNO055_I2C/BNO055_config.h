/*
 * BNO055_config.h
 *
 *  Created on: Nov 9, 2024
 *      Author: tim
 */

#ifndef BNO055_CONFIG_H_
#define BNO055_CONFIG_H_

#include <BNO055_I2C/BNO055.h>

#define BNO_CALIB_OFF // Switch toggle of Calibration mode

#define BNO_ACC_OFF_X 14
#define BNO_ACC_OFF_Y -38
#define BNO_ACC_OFF_Z -21

#define BNO_MAG_OFF_X 627
#define BNO_MAG_OFF_Y -307
#define BNO_MAG_OFF_Z 171

#define BNO_GYRO_OFF_X -2
#define BNO_GYRO_OFF_Y -4
#define BNO_GYRO_OFF_Z 0

#define BNO_ACC_RAD 1000
#define BNO_MAG_RAD 388



extern BNO055_Offsets BNO055_off;
#ifdef BNO_CALIB_ON
extern BNO055_Calibration_Status BNO055_stat;
#endif

#endif /* BNO055_CONFIG_H_ */

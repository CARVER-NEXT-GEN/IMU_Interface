/*
 * BNO086_SPI.h
 *
 *  Created on: Oct 22, 2024
 *      Author: natth
 */

#ifndef INC_BNO086_SPI_H_
#define INC_BNO086_SPI_H_

#include "main.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#if __CORTEX_M == 4 // Only for CM7 core
//////////////////////////////////////////////////////////////////////////
// Definition for SPI2 (APB1 PCLK = 42MHz)
#define BNO086_SPI_CHANNEL      SPI1
#define BNO086_INT_PIN          INT_Pin
#define BNO086_INT_PORT         INT_GPIO_Port


// Chip Select, Wake, and Reset Control Macros (Modified for HAL)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(BNO086)     HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin, GPIO_PIN_RESET)
#define CHIP_DESELECT(BNO086)   HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin, GPIO_PIN_SET)

#define WAKE_HIGH()             HAL_GPIO_WritePin(WAK_GPIO_Port, WAK_Pin, GPIO_PIN_SET)
#define WAKE_LOW()              HAL_GPIO_WritePin(WAK_GPIO_Port, WAK_Pin, GPIO_PIN_RESET)

//#define RESET_HIGH()            HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET)
//#define RESET_LOW()             HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

#endif

typedef enum{
    UNIT_RAD,
    UNIT_DEG
} RPY_UNIT;

typedef struct{
    float i;
    float j;
    float k;
    float w;
} quat_t;

typedef struct{
    float x;
    float y;
    float z;
} accel_t;

typedef struct{
    float x;
    float y;
    float z;
} linear_accel_t;

typedef struct{
    float x;
    float y;
    float z;
} gyro_t;

typedef struct{
    float x;
    float y;
    float z;
} mag_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
} euler_angle_t;

typedef struct{
    quat_t quaternion;
    accel_t acceleration;
    linear_accel_t linear_acceleration;
    gyro_t angular_velocity;
    mag_t magnetometer;
    euler_angle_t euler_angle;
} BNO086_t;

typedef enum {
	IDLE,
	UNRELIABLE,
	LOW,
	MEDIUM,
	HIGH
} StatusBit;

typedef enum {
	STORED_IDLE,
	STORED_SUCCESSFULLY,
	STORED_FAILED
} StatusCalibrationData;

typedef struct{
	StatusBit accuracyQuat;
	StatusBit accuracyAccel;
	StatusBit accuracyGyro;
	StatusBit accuracyMag;
	StatusCalibrationData CalibrationData;
} CalibrateStatus;

void READIMU_HSEM(BNO086_t *bno); // Only declaration for CM7

//Registers
enum Registers
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
};


//All the ways we can configure or talk to the BNO086, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

int BNO086_Initialization(BNO086_t *bno);
unsigned char SPI2_SendByte(unsigned char data);

int BNO086_dataAvailable(void);
void BNO086_parseCommandReport(void);
void BNO086_parseInputReport(void);

float BNO086_getQuatI();
float BNO086_getQuatJ();
float BNO086_getQuatK();
float BNO086_getQuatReal();
float BNO086_getQuatRadianAccuracy();
uint8_t BNO086_getQuatAccuracy();
float BNO086_getAccelX();
float BNO086_getAccelY();
float BNO086_getAccelZ();
uint8_t BNO086_getAccelAccuracy();
float BNO086_getLinAccelX();
float BNO086_getLinAccelY();
float BNO086_getLinAccelZ();
uint8_t BNO086_getLinAccelAccuracy();
float BNO086_getGyroX();
float BNO086_getGyroY();
float BNO086_getGyroZ();
uint8_t BNO086_getGyroAccuracy();
float BNO086_getMagX();
float BNO086_getMagY();
float BNO086_getMagZ();
uint8_t BNO086_getMagAccuracy();
uint16_t BNO086_getStepCount();
uint8_t BNO086_getStabilityClassifier();
uint8_t BNO086_getActivityClassifier();
uint32_t BNO086_getTimeStamp();
int16_t BNO086_getQ1(uint16_t recordID);
int16_t BNO086_getQ2(uint16_t recordID);
int16_t BNO086_getQ3(uint16_t recordID);
float BNO086_getResolution(uint16_t recordID);
float BNO086_getRange(uint16_t recordID);

uint32_t BNO086_readFRSword(uint16_t recordID, uint8_t wordNumber);
void BNO086_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
int BNO086_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);
void BNO086_softReset(void);
uint8_t BNO086_resetReason();

float BNO086_qToFloat(int16_t fixedPointValue, uint8_t qPoint);

void BNO086_enableRotationVector(uint16_t timeBetweenReports);
void BNO086_enableGameRotationVector(uint16_t timeBetweenReports);
void BNO086_enableAccelerometer(uint16_t timeBetweenReports);
void BNO086_enableLinearAccelerometer(uint16_t timeBetweenReports);
void BNO086_enableGyro(uint16_t timeBetweenReports);
void BNO086_enableMagnetometer(uint16_t timeBetweenReports);
void BNO086_enableStepCounter(uint16_t timeBetweenReports);
void BNO086_enableStabilityClassifier(uint16_t timeBetweenReports);

void BNO086_calibrateAccelerometer();
void BNO086_calibrateGyro();
void BNO086_calibrateMagnetometer();
void BNO086_calibratePlanarAccelerometer();
void BNO086_calibrateAll();
void BNO086_endCalibration();
int BNO086_calibrationComplete();

void BNO086_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);
void BNO086_sendCommand(uint8_t command);
void BNO086_sendCalibrateCommand(uint8_t thingToCalibrate);
void BNO086_requestCalibrationStatus();
void BNO086_saveCalibration();

int BNO086_waitForSPI(void);
int BNO086_receivePacket(void);
int BNO086_sendPacket(uint8_t channelNumber, uint8_t dataLength);

float getRoll(uint8_t unit);
float getPitch(uint8_t unit);
float getYaw(uint8_t unit);

void BNO086_getData(BNO086_t *bno, RPY_UNIT rpy_unit);
void BNO086_SAVE_HSEM(BNO086_t *bno, uint8_t status);
void BNO086_READ_HSEM(BNO086_t *bno);



#endif /* INC_BNO086_SPI_H_ */

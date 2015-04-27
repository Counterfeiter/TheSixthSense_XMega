/*
 * lsm303.h
 *
 * Created: 19.04.2015 12:36:35
 *  Author: Sebastian Foerster
 */ 


#ifndef LSM303_H_
#define LSM303_H_

#include <asf.h>

typedef struct vector_s
{
	float x;
	float y;
	float z;
} vector;

vector accelData;    // Last read accelerometer data will be available here
vector magData;        // Last read magnetometer data will be available here

vector m_min;
vector m_max;

extern uint8_t LSM303_read(void);
extern uint8_t LSM303_init(void);
extern float heading(void);


#endif /* LSM303_H_ */
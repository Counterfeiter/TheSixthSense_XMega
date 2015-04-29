/*
 * lsm303.h
 *
 * Created: 19.04.2015 12:36:35
 *  Author: Sebastian Foerster
 */ 


#ifndef LSM303_H_
#define LSM303_H_

#include <asf.h>

typedef struct vector_sf
{
	float x;
	float y;
	float z;
} vector_f;

typedef struct vector_s16
{
	int16_t x;
	int16_t y;
	int16_t z;
} vector_16;

typedef struct vector_s32
{
	int32_t x;
	int32_t y;
	int32_t z;
} vector_32;

vector_f m_min;
vector_f m_max;

extern uint8_t LSM303_init(void);
extern uint8_t LSM303_read_mag(vector_f *magData);
extern uint8_t LSM303_read_accel(vector_f *accelData);
extern void LSM303_writestartupsettings(void);
extern float heading(const vector_f *magData,const vector_f *accelData);


#endif /* LSM303_H_ */
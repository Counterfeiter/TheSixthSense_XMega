/*
 * lsm303.h
 *
 * Created: 19.04.2015 12:36:35
 *  Author: Sebastian Foerster
 
 TheSixthSense C (Atmel Studio 6.2) Source Code with ASF lib from Atmel
 Copyright (C) Atmel Corporation
 Copyright (C) Sebastian Foerster

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 
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

struct mag_cal_data {
	vector_f m_min;
	vector_f m_max;
};

struct mag_cal_data mag_cali;

extern uint8_t	LSM303_init(void);
extern uint8_t	LSM303_new_accel_data(void);
extern uint8_t	LSM303_new_mag_data(void);
extern uint8_t	LSM303_read_mag(vector_f *magData);
extern uint8_t	LSM303_read_accel_f(vector_f *accelData);
extern uint8_t	LSM303_read_accel_32(vector_32 *accelData);
extern void		LSM303_set_sleep(void);
extern void		LSM303_writestartupsettings(void);
extern float	heading(const vector_f *magData,const vector_f *accelData);


#endif /* LSM303_H_ */
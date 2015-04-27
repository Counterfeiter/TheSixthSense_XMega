/*
 * lsm303.c
 *
 * Created: 19.04.2015 12:36:47
 *  Author: Sebastian Foerster
 */ 
#include "LSM303.h"
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

#define LSM303_ADDRESS							0b00011110     
#define LSM303_ID								0b01001001

#define LSM303_REGISTER_TEMP_OUT_L				0x05 
#define LSM303_REGISTER_TEMP_OUT_H				0x06
#define LSM303_REGISTER_STATUS_M				0x07
#define LSM303_REGISTER_OUT_X_L_M				0x08
#define LSM303_REGISTER_WHO_AM_I				0x0F
#define LSM303_REGISTER_INT_CTRL_M				0x12
#define LSM303_REGISTER_INT_SRC_M				0x13
#define LSM303_REGISTER_INT_THS_L_M 			0x14
#define LSM303_REGISTER_INT_THS_H_M				0x15
#define LSM303_REGISTER_OFFSET_X_L_M 			0x16
#define LSM303_REGISTER_OFFSET_X_H_M			0x17
#define LSM303_REGISTER_OFFSET_Y_L_M			0x18
#define LSM303_REGISTER_OFFSET_Y_H_M			0x19
#define LSM303_REGISTER_OFFSET_Z_L_M			0x1A
#define LSM303_REGISTER_OFFSET_Z_H_M			0x1B
#define LSM303_REGISTER_REFERENCE_X				0x1C
#define LSM303_REGISTER_REFERENCE_Y				0x1D
#define LSM303_REGISTER_REFERENCE_Z				0x1E
#define LSM303_REGISTER_CTRL0					0x1F
#define LSM303_REGISTER_CTRL1					0x20
#define LSM303_REGISTER_CTRL2					0x21
#define LSM303_REGISTER_CTRL3					0x22
#define LSM303_REGISTER_CTRL4					0x23
#define LSM303_REGISTER_CTRL5					0x24
#define LSM303_REGISTER_CTRL6					0x25
#define LSM303_REGISTER_CTRL7					0x26
#define LSM303_REGISTER_STATUS_A				0x27
#define LSM303_REGISTER_OUT_X_L_A				0x28
#define LSM303_REGISTER_FIFO_CTRL				0x2E
#define LSM303_REGISTER_FIFO_SRC				0x2F

#define LSM303_REGISTER_IG_CFG1					0x30
#define LSM303_REGISTER_IG_SRC1					0x31
#define LSM303_REGISTER_IG_THS1					0x32
#define LSM303_REGISTER_IG_DUR1					0x33
#define LSM303_REGISTER_IG_CFG2					0x34
#define LSM303_REGISTER_IG_SRC2					0x35
#define LSM303_REGISTER_IG_THS2					0x36
#define LSM303_REGISTER_IG_DUR2					0x37
#define LSM303_REGISTER_CLICK_CFG				0x38
#define LSM303_REGISTER_CLICK_SRC				0x39
#define LSM303_REGISTER_CLICK_THS				0x3A
#define LSM303_REGISTER_TIME_LIMIT				0x3B
#define LSM303_REGISTER_TIME_LATENCY			0x3C
#define LSM303_REGISTER_TIME_WINDOW				0x3D
#define LSM303_REGISTER_ACT_THS					0x3E
#define LSM303_REGISTER_ACT_DUR					0x3F

#define TWI_MASTER_PORT		PORTC
#define TWI_MASTER			TWIC
#define TWI_SPEED			400000

vector m_min = {-2279, -2937, -2880};
vector m_max = { 1684, 2507, 2443};

//default values -> no calibration
//vector m_min = {-32767.0, -32767.0, -32767.0};
//vector m_max = {+32767.0, +32767.0, +32767.0};

void LSM303_write8(uint8_t reg, uint8_t value);
uint8_t LSM303_read8(uint8_t reg);
void vector_normalize(vector *a);
void vector_cross(const vector *a, const vector *b, vector *out);
float vector_dot(const vector *a, const vector *b);

uint8_t LSM303_init(void)
{
	// TWI master options /////////ASF I2C init
	twi_options_t m_options;
	m_options.speed = TWI_SPEED;
	m_options.chip  = 0x50;
	m_options.speed_reg = TWI_BAUD(sysclk_get_cpu_hz(), TWI_SPEED);


	// Initialize TWI_MASTER
	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &m_options);
	twi_master_enable(&TWI_MASTER);
	
	// Enable the accelerometer
	LSM303_write8(LSM303_REGISTER_CTRL2, 0x00);
		
	// Enable the accelerometer
	//LSM303_write8(LSM303_REGISTER_CTRL1, 0b00011111);
	LSM303_write8(LSM303_REGISTER_CTRL1, 0x57);

	// Enable the magnetometer
	//LSM303_write8(LSM303_REGISTER_CTRL5, 0b01100000);
	LSM303_write8(LSM303_REGISTER_CTRL5, 0x64);
	LSM303_write8(LSM303_REGISTER_CTRL6, 0x20);
	LSM303_write8(LSM303_REGISTER_CTRL7, 0x00);
	
	if(LSM303_ID ==  LSM303_read8(LSM303_REGISTER_WHO_AM_I)) {
		return true;
	}
	
	return false;
}

uint8_t LSM303_read(void) 
{
	
	uint8_t buffer[7];
	  	
	// Package to send
	twi_package_t packet;
	//address or command
	packet.addr_length	=	1;
	packet.addr[0]		=	LSM303_REGISTER_OUT_X_L_A | 0x80;
	packet.chip			=	LSM303_ADDRESS;
	packet.buffer		=	(void *)buffer;
	packet.length		=	6;
	// Wait if bus is busy
	packet.no_wait     =	false;
	  	
	  	
	if(twi_master_read(&TWI_MASTER, &packet)) {
		return 0x00;
	}

	int16_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	int16_t ylo = buffer[2];
	int16_t yhi = buffer[3]; 
	int16_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	accelData.x = (float)((xlo | (xhi << 8)) >> 4);
	accelData.y = (float)((ylo | (yhi << 8)) >> 4);
	accelData.z = (float)((zlo | (zhi << 8)) >> 4);

	// Read the magnetometer
	packet.addr[0] = LSM303_REGISTER_OUT_X_L_M | 0x80;

	if(twi_master_read(&TWI_MASTER, &packet)) {
		return 0x00;
	}
	
	xlo = buffer[0];
	xhi = buffer[1];
	ylo = buffer[2];
	yhi = buffer[3]; 
	zlo = buffer[4];
	zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	magData.x = (float)(xlo | (xhi << 8));
	magData.y = (float)(ylo | (yhi << 8));
	magData.z = (float)(zlo | (zhi << 8));
	
	return 1;
}


void LSM303_write8(uint8_t reg, uint8_t value)
{
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = value;
    
    // Package to send
    twi_package_t packet;
    //address or command
    packet.addr_length =	0;
    packet.chip        = LSM303_ADDRESS;
    packet.buffer      = (void *)buffer;
    packet.length      = 2;
    // Wait if bus is busy
    packet.no_wait     = false;
    
    if(twi_master_write(&TWI_MASTER, &packet) != STATUS_OK) {

    }
}

uint8_t LSM303_read8(uint8_t reg)
{
  	uint8_t buffer[3];
  	
  	// Package to send
  	twi_package_t packet;
  	//address or command
  	packet.addr_length	=	1;
  	packet.addr[0]		=	reg;
  	packet.chip			=	LSM303_ADDRESS;
  	packet.buffer		=	(void *)buffer;
  	packet.length		=	1;
  	// Wait if bus is busy
  	packet.no_wait     =	false;
  	
  	
  	if(twi_master_read(&TWI_MASTER, &packet)) {
	  	return 0x00;
  	}

  	return buffer[0];
}

float heading(void)
{
	vector from = { 1.0, 0.0, 0.0};
    vector temp_m = {magData.x, magData.y, magData.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // compute E and N
    vector E;
    vector N;
    vector_cross(&temp_m, &accelData, &E);
    vector_normalize(&E);
    vector_cross(&accelData, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading_f = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
    heading_f += 180.0;
    return heading_f;
}

void vector_normalize(vector *a)
{
	float mag = sqrt(vector_dot(a, a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

void vector_cross(const vector *a , const vector *b, vector *out)
{
	out->x = (a->y * b->z) - (a->z * b->y);
	out->y = (a->z * b->x) - (a->x * b->z);
	out->z = (a->x * b->y) - (a->y * b->x);
}

float vector_dot(const vector *a, const vector *b)
{
	return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}
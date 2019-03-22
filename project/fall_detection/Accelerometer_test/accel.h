/*
* Project Name: eYRC_BB_2403_Task1_Sensing
* File Name: accel.h
*
* Created: 04-Dec-16 7:56:40 PM
* Author : Heethesh Vhavle
*
* Team: eYRC-BB#2403
* Theme: Balance Bot
*
* Library for ADXL345 Accelerometer
*/

#ifndef ACCEL_H_
#define ACCEL_H_

// Register Map
#define ADXL345_ADDRESS			0x53 << 1
#define	ADXL345_DEVID			0x00
#define ADXL345_KNOWN_ID		0xE5
#define ADXL345_OFSX			0x1E
#define ADXL345_OFSY			0x1F
#define ADXL345_OFSZ			0x20
#define ADXL345_BW_RATE			0x2C
#define ADXL345_POWER_CTL		0x2D
#define ADXL345_DATA_FORMAT		0x31
#define ADXL345_DATAX0			0x32

// Function Declarations
void accel_init();
void calibrate_accel(char x_offset, char y_offset, char z_offset);
float convert_accelerometer(unsigned int value);
float read_accelerometer();

#endif